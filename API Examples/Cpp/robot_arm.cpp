/*
 * robot_arm.cpp - Robot Arm WebSocket client implementation
 */

#include "robot_arm.h"

#include <chrono>
#include <sstream>

/* -------------------------------------------------------------------------
 * Constructor / destructor
 * -------------------------------------------------------------------------*/
RobotArmClient::RobotArmClient(const std::string& host, int port)
    : m_host(host), m_port(port)
{
    m_ws.onMessage([this](const std::string& raw) {
        handleMessage(raw);
    });
    m_ws.onError([this](const std::string& reason) {
        failAllPending("Connection error: " + reason);
    });
}

RobotArmClient::~RobotArmClient() {
    disconnect();
}

/* -------------------------------------------------------------------------
 * connect / disconnect
 * -------------------------------------------------------------------------*/
void RobotArmClient::connect(int timeoutMs) {
    if (m_ws.isConnected()) return;
    if (!m_ws.connect(m_host, m_port, "/", timeoutMs))
        throw RobotArmError("Failed to connect to " + m_host + ":" + std::to_string(m_port));
}

void RobotArmClient::disconnect() {
    failAllPending("Disconnected");
    m_ws.disconnect();
}

/* -------------------------------------------------------------------------
 * request() — core send-and-wait implementation
 * -------------------------------------------------------------------------*/
json RobotArmClient::request(const std::string& command, json params, int timeoutMs) {
    if (!m_ws.isConnected())
        throw RobotArmError("Not connected to robot arm server");

    int id = m_nextId.fetch_add(1);

    json msg = params;
    msg["command"]   = command;
    msg["requestId"] = id;

    auto pending = std::make_shared<Pending>();

    {
        std::lock_guard<std::mutex> lock(m_pendingMtx);
        m_pending[id] = pending;
    }

    if (!m_ws.send(msg.dump())) {
        std::lock_guard<std::mutex> lock(m_pendingMtx);
        m_pending.erase(id);
        throw RobotArmError("Failed to send command: " + command);
    }

    /* Wait for the matching response */
    std::unique_lock<std::mutex> lock(pending->mtx);
    bool signalled = pending->cv.wait_for(
        lock,
        std::chrono::milliseconds(timeoutMs),
        [&] { return pending->done; }
    );

    if (!signalled) {
        std::lock_guard<std::mutex> pl(m_pendingMtx);
        m_pending.erase(id);
        throw RobotArmError("Request timed out: " + command);
    }

    if (!pending->error.empty())
        throw RobotArmError(pending->error);

    if (pending->result.contains("type") &&
        pending->result["type"] == "error") {
        std::string msg = pending->result.value("message", "Unknown server error");
        throw RobotArmError(msg);
    }

    return pending->result;
}

/* -------------------------------------------------------------------------
 * handleMessage() — called from receive thread for each inbound JSON frame
 * -------------------------------------------------------------------------*/
void RobotArmClient::handleMessage(const std::string& raw) {
    json data;
    try {
        data = json::parse(raw);
    } catch (...) {
        return;
    }

    if (!data.contains("requestId") || !data["requestId"].is_number_integer())
        return; /* unsolicited (e.g. welcome message) — ignore */

    int id = data["requestId"].get<int>();

    std::shared_ptr<Pending> pending;
    {
        std::lock_guard<std::mutex> lock(m_pendingMtx);
        auto it = m_pending.find(id);
        if (it == m_pending.end()) return;
        pending = it->second;
        m_pending.erase(it);
    }

    {
        std::lock_guard<std::mutex> lock(pending->mtx);
        pending->result = std::move(data);
        pending->done   = true;
    }
    pending->cv.notify_one();
}

/* -------------------------------------------------------------------------
 * failAllPending() — unblock all waiting requests with an error
 * -------------------------------------------------------------------------*/
void RobotArmClient::failAllPending(const std::string& reason) {
    std::map<int, std::shared_ptr<Pending>> snapshot;
    {
        std::lock_guard<std::mutex> lock(m_pendingMtx);
        snapshot.swap(m_pending);
    }
    for (auto& [id, pending] : snapshot) {
        {
            std::lock_guard<std::mutex> lock(pending->mtx);
            pending->error = reason;
            pending->done  = true;
        }
        pending->cv.notify_one();
    }
}

/* =========================================================================
 * Joint commands
 * =========================================================================*/
json RobotArmClient::getJointConfigs(int t) {
    return request("getJointConfigs", {}, t);
}

json RobotArmClient::getStatus(int t) {
    return request("getStatus", {}, t);
}

json RobotArmClient::moveJoint(int joint, double angle, int speed, int t) {
    return request("moveJoint",
                   {{"joint", joint}, {"angle", angle}, {"speed", speed}}, t);
}

json RobotArmClient::stopJoint(int joint, int t) {
    return request("stopJoint", {{"joint", joint}}, t);
}

json RobotArmClient::stopAllJoints(int t) {
    return request("stopAllJoints", {}, t);
}

json RobotArmClient::setSpeed(int joint, int speed, int t) {
    return request("setSpeed", {{"joint", joint}, {"speed", speed}}, t);
}

json RobotArmClient::setSpeedAll(int speed, int t) {
    return request("setSpeedAll", {{"speed", speed}}, t);
}

json RobotArmClient::setTorqueAll(bool enabled, int t) {
    return request("setTorqueAll", {{"enabled", enabled}}, t);
}

json RobotArmClient::rescanServos(int t) {
    return request("rescanServos", {}, t);
}

/* =========================================================================
 * End-tool commands
 * =========================================================================*/
json RobotArmClient::toolPing(int t) {
    return request("toolPing", {}, t);
}

json RobotArmClient::toolReadCurrents(int t) {
    return request("toolReadCurrents", {}, t);
}

json RobotArmClient::toolReadAdc(int t) {
    return request("toolReadAdc", {}, t);
}

json RobotArmClient::toolSetPwm(int pwm1, int pwm2, bool en1, bool en2, int t) {
    return request("toolSetPwm",
                   {{"pwm1Duty", pwm1}, {"pwm2Duty", pwm2},
                    {"enable1", en1},   {"enable2", en2}}, t);
}

json RobotArmClient::toolSetServoEnabled(bool enabled, int t) {
    return request("toolSetServoEnabled", {{"enabled", enabled}}, t);
}

json RobotArmClient::toolSetServoAngle(double angle, int t) {
    return request("toolSetServoAngle", {{"angle", angle}}, t);
}

json RobotArmClient::toolSetServoPosition(int position, int t) {
    return request("toolSetServoPosition", {{"position", position}}, t);
}

json RobotArmClient::toolGetServoState(int t) {
    return request("toolGetServoState", {}, t);
}

/* =========================================================================
 * Kinematics
 * =========================================================================*/
json RobotArmClient::kinematicsLoadUrdf(const std::string& urdfXml, int t) {
    return request("kinematicsLoadURDF", {{"urdfXml", urdfXml}}, t);
}

json RobotArmClient::kinematicsForward(const std::vector<double>& angles, int t) {
    return request("kinematicsForwardKinematics", {{"jointAngles", angles}}, t);
}

json RobotArmClient::kinematicsInverse(double x, double y, double z,
                                       const std::vector<double>* initial, int t)
{
    json params = {{"targetPose", {{"x", x}, {"y", y}, {"z", z}}}};
    if (initial && !initial->empty())
        params["initialAngles"] = *initial;
    return request("kinematicsInverseKinematics", params, t);
}
