/*
 * RobotArmClient.cs -- C# client for the Robot Arm WebSocket server
 *
 * Mirrors the Python RobotArmClient API.
 * Uses only built-in .NET libraries (System.Net.WebSockets, System.Text.Json).
 *
 * Usage:
 *   await using var arm = new RobotArmClient("192.168.1.112");
 *   await arm.ConnectAsync();
 *   var status = await arm.GetStatusAsync();
 *   await arm.MoveJointAsync(1, 45.0);
 */

using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;

/// <summary>Thrown on server errors, timeouts, and not-connected state.</summary>
public sealed class RobotArmError : Exception
{
    public RobotArmError(string message) : base(message) { }
}

/// <summary>WebSocket client for the robot arm Pi server.</summary>
public sealed class RobotArmClient : IAsyncDisposable
{
    private readonly string _host;
    private readonly int _port;
    private ClientWebSocket _ws = new();
    private CancellationTokenSource _cts = new();
    private Task _receiveTask = Task.CompletedTask;
    private int _nextId;

    private readonly Dictionary<int, TaskCompletionSource<JsonElement>> _pending = new();
    private readonly object _pendingLock = new();

    public RobotArmClient(string host, int port = 8080)
    {
        _host = host;
        _port = port;
    }

    public bool IsConnected => _ws.State == WebSocketState.Open;

    // -------------------------------------------------------------------------
    // Connection
    // -------------------------------------------------------------------------

    public async Task ConnectAsync(CancellationToken ct = default)
    {
        var uri = new Uri($"ws://{_host}:{_port}");
        await _ws.ConnectAsync(uri, ct);
        _receiveTask = Task.Run(() => ReceiveLoopAsync(_cts.Token));
    }

    public async ValueTask DisposeAsync()
    {
        _cts.Cancel();
        FailAllPending("Connection closed");
        try
        {
            if (_ws.State == WebSocketState.Open)
                await _ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "bye", CancellationToken.None);
        }
        catch { }
        try { await _receiveTask; } catch { }
        _ws.Dispose();
        _cts.Dispose();
    }

    // -------------------------------------------------------------------------
    // Core request / response
    // -------------------------------------------------------------------------

    /// <summary>
    /// Send a command and wait for the matching response (matched by requestId).
    /// Throws RobotArmError on failure, timeout, or server error response.
    /// </summary>
    public async Task<JsonElement> RequestAsync(string command,
                                                JsonObject? parameters = null,
                                                int timeoutMs = 5000)
    {
        if (!IsConnected)
            throw new RobotArmError("Not connected to robot arm server");

        int id = Interlocked.Increment(ref _nextId);
        var tcs = new TaskCompletionSource<JsonElement>(
            TaskCreationOptions.RunContinuationsAsynchronously);

        lock (_pendingLock) _pending[id] = tcs;

        // Build JSON message
        var msg = new JsonObject { ["command"] = command, ["requestId"] = id };
        if (parameters is not null)
            foreach (var kv in parameters)
                msg[kv.Key] = kv.Value?.DeepClone();

        var bytes = Encoding.UTF8.GetBytes(msg.ToJsonString());
        try
        {
            await _ws.SendAsync(bytes, WebSocketMessageType.Text,
                                endOfMessage: true, CancellationToken.None);
        }
        catch
        {
            lock (_pendingLock) _pending.Remove(id);
            throw new RobotArmError($"Failed to send command: {command}");
        }

        // Timeout cancellation
        using var timeoutCts = new CancellationTokenSource(timeoutMs);
        timeoutCts.Token.Register(() =>
        {
            lock (_pendingLock) _pending.Remove(id);
            tcs.TrySetException(new RobotArmError($"Request timed out: {command}"));
        });

        var result = await tcs.Task;

        // Server-side error response
        if (result.TryGetProperty("type", out var type) && type.GetString() == "error")
        {
            var serverMsg = result.TryGetProperty("message", out var m)
                ? m.GetString() : null;
            throw new RobotArmError(serverMsg ?? "Unknown server error");
        }

        return result;
    }

    // -------------------------------------------------------------------------
    // Receive loop (background task)
    // -------------------------------------------------------------------------

    private async Task ReceiveLoopAsync(CancellationToken ct)
    {
        var buffer = new byte[65536];
        var sb = new StringBuilder();

        while (!ct.IsCancellationRequested && _ws.State == WebSocketState.Open)
        {
            try
            {
                sb.Clear();
                WebSocketReceiveResult result;
                do
                {
                    result = await _ws.ReceiveAsync(buffer, ct);
                    if (result.MessageType == WebSocketMessageType.Close) return;
                    sb.Append(Encoding.UTF8.GetString(buffer, 0, result.Count));
                }
                while (!result.EndOfMessage);

                HandleMessage(sb.ToString());
            }
            catch (OperationCanceledException) { break; }
            catch
            {
                FailAllPending("Connection lost");
                break;
            }
        }
    }

    private void HandleMessage(string raw)
    {
        JsonDocument doc;
        try { doc = JsonDocument.Parse(raw); }
        catch { return; }

        var root = doc.RootElement;
        if (!root.TryGetProperty("requestId", out var idEl) ||
            !idEl.TryGetInt32(out int id)) return;

        TaskCompletionSource<JsonElement>? tcs;
        lock (_pendingLock)
        {
            if (!_pending.TryGetValue(id, out tcs)) return;
            _pending.Remove(id);
        }

        tcs.TrySetResult(root.Clone()); // clone so doc can be disposed
        doc.Dispose();
    }

    private void FailAllPending(string reason)
    {
        List<TaskCompletionSource<JsonElement>> snapshot;
        lock (_pendingLock)
        {
            snapshot = [.._pending.Values];
            _pending.Clear();
        }
        var ex = new RobotArmError(reason);
        foreach (var tcs in snapshot) tcs.TrySetException(ex);
    }

    // =========================================================================
    // Joint commands
    // =========================================================================

    public Task<JsonElement> GetJointConfigsAsync(int timeoutMs = 5000)
        => RequestAsync("getJointConfigs", timeoutMs: timeoutMs);

    public Task<JsonElement> GetStatusAsync(int timeoutMs = 5000)
        => RequestAsync("getStatus", timeoutMs: timeoutMs);

    public Task<JsonElement> MoveJointAsync(int joint, double angle,
                                            int speed = 1500, int timeoutMs = 5000)
        => RequestAsync("moveJoint",
            new() { ["joint"] = joint, ["angle"] = angle, ["speed"] = speed },
            timeoutMs);

    public Task<JsonElement> StopJointAsync(int joint, int timeoutMs = 5000)
        => RequestAsync("stopJoint",
            new() { ["joint"] = joint }, timeoutMs);

    public Task<JsonElement> StopAllJointsAsync(int timeoutMs = 5000)
        => RequestAsync("stopAllJoints", timeoutMs: timeoutMs);

    public Task<JsonElement> SetSpeedAsync(int joint, int speed, int timeoutMs = 5000)
        => RequestAsync("setSpeed",
            new() { ["joint"] = joint, ["speed"] = speed }, timeoutMs);

    public Task<JsonElement> SetSpeedAllAsync(int speed, int timeoutMs = 5000)
        => RequestAsync("setSpeedAll",
            new() { ["speed"] = speed }, timeoutMs);

    public Task<JsonElement> SetTorqueAllAsync(bool enabled, int timeoutMs = 5000)
        => RequestAsync("setTorqueAll",
            new() { ["enabled"] = enabled }, timeoutMs);

    public Task<JsonElement> RescanServosAsync(int timeoutMs = 10000)
        => RequestAsync("rescanServos", timeoutMs: timeoutMs);

    // =========================================================================
    // End-tool commands
    // =========================================================================

    public Task<JsonElement> ToolPingAsync(int timeoutMs = 5000)
        => RequestAsync("toolPing", timeoutMs: timeoutMs);

    public Task<JsonElement> ToolReadCurrentsAsync(int timeoutMs = 5000)
        => RequestAsync("toolReadCurrents", timeoutMs: timeoutMs);

    public Task<JsonElement> ToolReadAdcAsync(int timeoutMs = 5000)
        => RequestAsync("toolReadAdc", timeoutMs: timeoutMs);

    public Task<JsonElement> ToolSetPwmAsync(int pwm1Duty, int pwm2Duty,
                                             bool enable1 = true, bool enable2 = true,
                                             int timeoutMs = 5000)
        => RequestAsync("toolSetPwm",
            new() { ["pwm1Duty"] = pwm1Duty, ["pwm2Duty"] = pwm2Duty,
                    ["enable1"] = enable1,   ["enable2"] = enable2 },
            timeoutMs);

    public Task<JsonElement> ToolSetServoEnabledAsync(bool enabled, int timeoutMs = 5000)
        => RequestAsync("toolSetServoEnabled",
            new() { ["enabled"] = enabled }, timeoutMs);

    public Task<JsonElement> ToolSetServoAngleAsync(double angle, int timeoutMs = 5000)
        => RequestAsync("toolSetServoAngle",
            new() { ["angle"] = angle }, timeoutMs);

    public Task<JsonElement> ToolSetServoPositionAsync(int position, int timeoutMs = 5000)
        => RequestAsync("toolSetServoPosition",
            new() { ["position"] = position }, timeoutMs);

    public Task<JsonElement> ToolGetServoStateAsync(int timeoutMs = 5000)
        => RequestAsync("toolGetServoState", timeoutMs: timeoutMs);

    // =========================================================================
    // Kinematics
    // =========================================================================

    public Task<JsonElement> KinematicsLoadUrdfAsync(string urdfXml, int timeoutMs = 8000)
        => RequestAsync("kinematicsLoadURDF",
            new() { ["urdfXml"] = urdfXml }, timeoutMs);

    public Task<JsonElement> KinematicsForwardAsync(IReadOnlyList<double> jointAngles,
                                                    int timeoutMs = 5000)
    {
        var arr = new JsonArray(jointAngles.Select(a => (JsonNode?)JsonValue.Create(a)).ToArray());
        return RequestAsync("kinematicsForwardKinematics",
            new() { ["jointAngles"] = arr }, timeoutMs);
    }

    public Task<JsonElement> KinematicsInverseAsync(double x, double y, double z,
                                                    IReadOnlyList<double>? initialAngles = null,
                                                    int timeoutMs = 10000)
    {
        var p = new JsonObject
        {
            ["targetPose"] = new JsonObject { ["x"] = x, ["y"] = y, ["z"] = z }
        };
        if (initialAngles is not null)
            p["initialAngles"] = new JsonArray(
                initialAngles.Select(a => (JsonNode?)JsonValue.Create(a)).ToArray());
        return RequestAsync("kinematicsInverseKinematics", p, timeoutMs);
    }
}
