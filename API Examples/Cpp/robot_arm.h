/*
 * robot_arm.h - C++ client for the Robot Arm WebSocket server
 *
 * Mirrors the Python RobotArmClient API exactly.
 * Requires nlohmann/json (single-header, drop json.hpp into this directory).
 *
 * Usage:
 *   RobotArmClient arm("192.168.1.112");
 *   arm.connect();
 *   auto status = arm.getStatus();
 *   arm.moveJoint(1, 45.0);
 *   arm.disconnect();
 */
#pragma once

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "ws_client.h"

using json = nlohmann::json;

/* -------------------------------------------------------------------------
 * RobotArmError — thrown on server errors, timeouts, and not-connected state
 * -------------------------------------------------------------------------*/
class RobotArmError : public std::runtime_error {
public:
    explicit RobotArmError(const std::string& msg) : std::runtime_error(msg) {}
};

/* -------------------------------------------------------------------------
 * RobotArmClient
 * -------------------------------------------------------------------------*/
class RobotArmClient {
public:
    explicit RobotArmClient(const std::string& host, int port = 8080);
    ~RobotArmClient();

    /* Connection */
    void connect(int timeoutMs = 10000);
    void disconnect();

    /*
     * Core request/response helper.
     * command  — server command name, e.g. "getStatus"
     * params   — extra JSON fields merged into the message
     * timeoutMs — how long to wait for a reply
     * Throws RobotArmError on any failure.
     */
    json request(const std::string& command,
                 json params   = json::object(),
                 int  timeoutMs = 5000);

    /* ---- Joint commands ---- */
    json getJointConfigs(int timeoutMs = 5000);
    json getStatus(int timeoutMs = 5000);
    json moveJoint(int joint, double angle, int speed = 1500, int timeoutMs = 5000);
    json stopJoint(int joint, int timeoutMs = 5000);
    json stopAllJoints(int timeoutMs = 5000);
    json setSpeed(int joint, int speed, int timeoutMs = 5000);
    json setSpeedAll(int speed, int timeoutMs = 5000);
    json setTorqueAll(bool enabled, int timeoutMs = 5000);
    json rescanServos(int timeoutMs = 10000);

    /* ---- End-tool commands ---- */
    json toolPing(int timeoutMs = 5000);
    json toolReadCurrents(int timeoutMs = 5000);
    json toolReadAdc(int timeoutMs = 5000);
    json toolSetPwm(int pwm1Duty, int pwm2Duty,
                    bool enable1 = true, bool enable2 = true,
                    int timeoutMs = 5000);
    json toolSetServoEnabled(bool enabled, int timeoutMs = 5000);
    json toolSetServoAngle(double angle, int timeoutMs = 5000);
    json toolSetServoPosition(int position, int timeoutMs = 5000);
    json toolGetServoState(int timeoutMs = 5000);

    /* ---- Kinematics ---- */
    json kinematicsLoadUrdf(const std::string& urdfXml, int timeoutMs = 8000);
    json kinematicsForward(const std::vector<double>& jointAngles, int timeoutMs = 5000);
    json kinematicsInverse(double x, double y, double z,
                           const std::vector<double>* initialAngles = nullptr,
                           int timeoutMs = 10000);

private:
    void handleMessage(const std::string& raw);
    void failAllPending(const std::string& reason);

    struct Pending {
        std::mutex              mtx;
        std::condition_variable cv;
        json                    result;
        std::string             error;
        bool                    done { false };
    };

    std::string  m_host;
    int          m_port;
    WebSocketClient              m_ws;
    std::atomic<int>             m_nextId { 1 };
    std::map<int, std::shared_ptr<Pending>> m_pending;
    std::mutex                   m_pendingMtx;
};
