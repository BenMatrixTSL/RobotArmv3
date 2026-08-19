/*
 * ExampleBasic.cs -- Connect to the Raspberry Pi robot arm server and read status.
 *
 * Build and run:
 *   dotnet run                           -- uses default IP
 *   dotnet run -- 192.168.1.50           -- override the Pi IP
 *
 * Or build a self-contained exe:
 *   dotnet publish -c Release -r win-x64 --self-contained
 */

using System.Text.Json;

// ---- Configuration ----------------------------------------------------------
string host = args.Length > 0 ? args[0] : "192.168.1.112";
const int port = 8080;
// -----------------------------------------------------------------------------

await using var arm = new RobotArmClient(host, port);

try
{
    Console.WriteLine($"Connecting to ws://{host}:{port} ...");
    await arm.ConnectAsync();
    Console.WriteLine("Connected.\n");

    // How many servos are on the bus?
    var configs = await arm.GetJointConfigsAsync();
    Console.WriteLine($"Servos found: {configs.GetInt("count")} of {configs.GetInt("total")}");

    // Read each joint angle
    var status = await arm.GetStatusAsync();
    Console.WriteLine("\nJoint angles:");
    foreach (var joint in status.GetArray("joints"))
    {
        int  jNum      = joint.GetInt("joint");
        double angle   = joint.GetDouble("angleDegrees");
        bool available = joint.GetBool("available");
        if (available)
            Console.WriteLine($"  Joint {jNum}: {angle:F4} degrees");
        else
            Console.WriteLine($"  Joint {jNum}: (not available)");
    }

    // End-tool currents (skipped if tool not connected)
    Console.WriteLine("\nEnd tool currents (mA):");
    try
    {
        var cur = await arm.ToolReadCurrentsAsync();
        Console.WriteLine($"  PWM1:  {cur.GetInt("pwm1CurrentRaw")}");
        Console.WriteLine($"  PWM2:  {cur.GetInt("pwm2CurrentRaw")}");
        Console.WriteLine($"  Servo: {cur.GetInt("servoCurrentRaw")}");
    }
    catch (RobotArmError e) { PrintToolHint(e); }

    // End-tool ADC
    Console.WriteLine("\nEnd tool ADC:");
    try
    {
        var adc = await arm.ToolReadAdcAsync();
        Console.WriteLine($"  ADC0 raw: {adc.GetInt("adc0Raw")}  ({adc.GetInt("adc0mV")} mV)");
        Console.WriteLine($"  ADC1 raw: {adc.GetInt("adc1Raw")}  ({adc.GetInt("adc1mV")} mV)");
    }
    catch (RobotArmError e) { PrintToolHint(e); }

    /*
     * --- Move a single joint (use with care -- arm must have clearance) ---
     *
     * await arm.MoveJointAsync(1, 0.0);           // joint 1, 0 degrees, default speed
     * await arm.MoveJointAsync(2, 45.0, speed: 800); // joint 2, 45 degrees, slower
     * await arm.StopJointAsync(1);                // stop joint 1 immediately
     * await arm.StopAllJointsAsync();             // emergency stop all joints
     */

    /*
     * --- Forward kinematics: get XYZ from current joint angles ---
     *
     * Asks the server to compute where the end-effector tip is in Cartesian
     * space (mm) for the given joint angles.
     * Requires a URDF loaded on the server first (see URDF example below).
     *
     * var fk = await arm.KinematicsForwardAsync([0.0, 45.0, -30.0, 0.0, 0.0, 0.0]);
     * var pos = fk.GetProperty("position");
     * Console.WriteLine($"XYZ: {pos.GetDouble("x"):F1}, " +
     *                         $"{pos.GetDouble("y"):F1}, " +
     *                         $"{pos.GetDouble("z"):F1} mm");
     *
     * // Or use the angles already read from GetStatusAsync():
     * var angles = status.GetArray("joints")
     *     .Where(j => j.GetBool("available"))
     *     .Select(j => j.GetDouble("angleDegrees"))
     *     .ToList();
     * var fk2 = await arm.KinematicsForwardAsync(angles);
     */

    /*
     * --- Inverse kinematics: move end-effector to an XYZ target (mm) ---
     *
     * The server solves for joint angles that put the tip at the target,
     * then move each joint to the solved angle.
     * Requires a URDF loaded on the server first.
     *
     * var ik = await arm.KinematicsInverseAsync(x: 150.0, y: 0.0, z: 200.0);
     * if (ik.TryGetProperty("jointAngles", out var solved))
     * {
     *     var solvedAngles = solved.EnumerateArray()
     *         .Select(a => a.GetDouble()).ToList();
     *
     *     Console.WriteLine("IK solution (degrees):");
     *     for (int i = 0; i < solvedAngles.Count; i++)
     *         Console.WriteLine($"  Joint {i + 1}: {solvedAngles[i]:F2}");
     *
     *     // Move each joint to the solved angle
     *     for (int i = 0; i < solvedAngles.Count; i++)
     *         await arm.MoveJointAsync(i + 1, solvedAngles[i]);
     * }
     *
     * // Optionally supply an initial-angle guess to steer the solver:
     * double[] guess = [0.0, 30.0, -20.0, 0.0, 0.0, 0.0];
     * var ik2 = await arm.KinematicsInverseAsync(150.0, 0.0, 200.0, initialAngles: guess);
     */

    /*
     * --- Load a URDF into the server-side kinematics solver ---
     *
     * IK/FK calls require this at startup. The server holds it in memory
     * until restarted.
     *
     * string urdf = await File.ReadAllTextAsync("robot.urdf");
     * await arm.KinematicsLoadUrdfAsync(urdf);
     */
}
catch (RobotArmError e)
{
    Console.Error.WriteLine($"Robot arm error: {e.Message}");
}
catch (Exception e)
{
    Console.Error.WriteLine($"Unexpected error: {e.Message}");
}

Console.WriteLine("\nDisconnected.");

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static void PrintToolHint(RobotArmError e)
{
    Console.WriteLine($"  (skipped -- {e.Message})");
    if (e.Message.Contains("Unknown command") && e.Message.Contains("toolRead"))
    {
        Console.WriteLine();
        Console.WriteLine("  >>> The Pi is running an OLD copy of server.js.");
        Console.WriteLine("  >>> Update raspberry-pi-control-st3215 on the Pi,");
        Console.WriteLine("  >>> then restart:  node server.js");
    }
}

// Convenience extension methods for cleaner JsonElement access
static class JsonElementExtensions
{
    public static int    GetInt   (this JsonElement el, string name) =>
        el.TryGetProperty(name, out var v) ? v.GetInt32()    : 0;
    public static double GetDouble(this JsonElement el, string name) =>
        el.TryGetProperty(name, out var v) ? v.GetDouble()   : 0.0;
    public static bool   GetBool  (this JsonElement el, string name) =>
        el.TryGetProperty(name, out var v) && v.GetBoolean();
    public static JsonElement.ArrayEnumerator GetArray(this JsonElement el, string name) =>
        el.TryGetProperty(name, out var v) ? v.EnumerateArray()
                                           : default(JsonElement).EnumerateArray();
}
