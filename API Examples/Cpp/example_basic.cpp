/*
 * example_basic.cpp - Connect to the Raspberry Pi robot arm server and read status.
 *
 * Before building:
 *   1. Download nlohmann/json single header:
 *      https://github.com/nlohmann/json/releases  →  json.hpp
 *      Place it in a "nlohmann/" subdirectory next to this file.
 *   2. Build with CMake (see CMakeLists.txt), or manually:
 *      Windows: cl /std:c++17 example_basic.cpp robot_arm.cpp ws_client.cpp /Fe:example_basic.exe
 *      Linux:   g++ -std=c++17 example_basic.cpp robot_arm.cpp ws_client.cpp -lpthread -o example_basic
 *   3. Start the Node.js server on the Pi:  node server.js
 *   4. Set PI_HOST below to your Pi's IP address.
 */

#include <iostream>
#include <string>
#include "robot_arm.h"

static const std::string DEFAULT_HOST = "192.168.1.112";
static const int         PI_PORT      = 8080;

static void printToolCommandHint(const RobotArmError& e) {
    std::string msg = e.what();
    std::cout << "  (skipped -- " << msg << ")\n";
    if (msg.find("Unknown command") != std::string::npos &&
        msg.find("toolRead") != std::string::npos) {
        std::cout << "\n"
                  << "  >>> The Pi is running an OLD copy of server.js.\n"
                  << "  >>> Update raspberry-pi-control-st3215 on the Pi,\n"
                  << "  >>> then restart:  node server.js\n";
    }
}

int main(int argc, char* argv[]) {
    std::string host = (argc > 1) ? argv[1] : DEFAULT_HOST;

    RobotArmClient arm(host, PI_PORT);

    try {
        std::cout << "Connecting to ws://" << host << ":" << PI_PORT << " ...\n";
        arm.connect();
        std::cout << "Connected.\n\n";

        /* How many servos are on the bus? */
        auto configs = arm.getJointConfigs();
        std::cout << "Servos found: "
                  << configs.value("count", 0) << " of "
                  << configs.value("total", 0) << "\n";

        /* Read each joint angle */
        auto status = arm.getStatus();
        std::cout << "\nJoint angles:\n";
        for (auto& joint : status.value("joints", json::array())) {
            int  jNum      = joint.value("joint",        0);
            double angle   = joint.value("angleDegrees", 0.0);
            bool available = joint.value("available",    false);
            if (available)
                std::cout << "  Joint " << jNum << ": " << angle << " degrees\n";
            else
                std::cout << "  Joint " << jNum << ": (not available)\n";
        }

        /* End-tool currents (optional -- skipped if tool not connected) */
        std::cout << "\nEnd tool currents (mA):\n";
        try {
            auto cur = arm.toolReadCurrents();
            std::cout << "  PWM1:  " << cur.value("pwm1CurrentRaw",  0) << "\n"
                      << "  PWM2:  " << cur.value("pwm2CurrentRaw",  0) << "\n"
                      << "  Servo: " << cur.value("servoCurrentRaw", 0) << "\n";
        } catch (const RobotArmError& e) {
            printToolCommandHint(e);
        }

        /* End-tool ADC */
        std::cout << "\nEnd tool ADC:\n";
        try {
            auto adc = arm.toolReadAdc();
            std::cout << "  ADC0 raw: " << adc.value("adc0Raw", 0)
                      << "  (" << adc.value("adc0mV", 0) << " mV)\n"
                      << "  ADC1 raw: " << adc.value("adc1Raw", 0)
                      << "  (" << adc.value("adc1mV", 0) << " mV)\n";
        } catch (const RobotArmError& e) {
            printToolCommandHint(e);
        }

        /*
         * --- Move a single joint (use with care -- arm must have clearance) ---
         *
         * arm.moveJoint(1, 0.0);          // joint 1, 0 degrees, default speed
         * arm.moveJoint(2, 45.0, 800);    // joint 2, 45 degrees, slower speed
         * arm.stopJoint(1);               // stop joint 1 immediately
         * arm.stopAllJoints();            // emergency stop all joints
         */

        /*
         * --- Forward kinematics: get XYZ position from current joint angles ---
         *
         * Reads the current joint angles then asks the server to compute
         * where the end-effector tip is in Cartesian space (mm).
         * Requires a URDF to be loaded on the server first (see load example below).
         *
         * auto fk = arm.kinematicsForward({0.0, 45.0, -30.0, 0.0, 0.0, 0.0});
         * auto& pos = fk["position"];
         * std::cout << "End-effector XYZ: "
         *           << pos.value("x", 0.0) << ", "
         *           << pos.value("y", 0.0) << ", "
         *           << pos.value("z", 0.0) << " mm\n";
         *
         * Or use the current joint angles read earlier:
         *
         * std::vector<double> angles;
         * for (auto& j : status.value("joints", json::array()))
         *     if (j.value("available", false))
         *         angles.push_back(j.value("angleDegrees", 0.0));
         * auto fk = arm.kinematicsForward(angles);
         */

        /*
         * --- Inverse kinematics: move end-effector to an XYZ position (mm) ---
         *
         * The server solves for joint angles that put the tip at the target
         * position, then you send the resulting angles as move commands.
         * Requires a URDF to be loaded on the server first (see below).
         *
         * // Solve IK for target position
         * auto ik = arm.kinematicsInverse(150.0, 0.0, 200.0);  // x, y, z in mm
         * if (ik.contains("jointAngles")) {
         *     auto solved = ik["jointAngles"].get<std::vector<double>>();
         *     std::cout << "IK solution (degrees):";
         *     for (size_t i = 0; i < solved.size(); ++i)
         *         std::cout << "  Joint " << (i + 1) << ": " << solved[i];
         *     std::cout << "\n";
         *
         *     // Move each joint to the solved angle
         *     for (size_t i = 0; i < solved.size(); ++i)
         *         arm.moveJoint((int)(i + 1), solved[i]);
         * }
         *
         * // Optionally supply an initial-angle guess to help the solver
         * // (useful when there are multiple IK solutions):
         * std::vector<double> guess = {0.0, 30.0, -20.0, 0.0, 0.0, 0.0};
         * auto ik2 = arm.kinematicsInverse(150.0, 0.0, 200.0, &guess);
         */

        /*
         * --- Load a URDF into the server-side kinematics solver ---
         *
         * The URDF describes the arm geometry; IK/FK calls fail without it.
         * Read the file contents into a string and pass to the server once
         * at startup. The server keeps it in memory until restarted.
         *
         * #include <fstream>
         * #include <sstream>
         * std::ifstream f("robot.urdf");
         * std::ostringstream ss; ss << f.rdbuf();
         * arm.kinematicsLoadUrdf(ss.str());
         */

    } catch (const RobotArmError& e) {
        std::cerr << "Robot arm error: " << e.what() << "\n";
    } catch (const std::exception& e) {
        std::cerr << "Unexpected error: " << e.what() << "\n";
    }

    arm.disconnect();
    std::cout << "\nDisconnected.\n";
    return 0;
}
