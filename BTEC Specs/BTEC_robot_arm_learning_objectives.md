## Robot Arm – BTEC‑Aligned Learning Objectives

This document summarises how the robot arm can be used to meet learning objectives drawn from the Pearson BTEC National (Level 3) and Higher National (Level 4/5) Engineering specifications. The specs do not explicitly name “robotics” or “robot arms”; instead, the arm is a teaching platform that helps deliver outcomes in multiple units (mechanical, electrical/electronic, control, automation, safety, and project work).

---

## 1. Mechanical & Mechatronic Principles

### Level 3 National

- **Linkages, levers and mechanisms**
  - Describe the robot arm as a multi‑link mechanism with joints, links and degrees of freedom.
  - Relate joint rotation (angles) to linear motion of the end effector.
  - Interpret simple engineering diagrams or URDF‑style descriptions of the arm (link lengths, joint types, axes).

- **Statics and forces**
  - Identify loads at each joint for different arm postures (especially shoulder and elbow).
  - Explain how joint torque limits and load readings constrain the safe working envelope (reach, payload).
  - Describe the effect of changing payload mass and reach on joint loading and stability.

- **Materials and structure**
  - Explain why specific materials are used for links, joints, and base (stiffness vs. weight vs. cost).
  - Identify potential failure modes (bending, backlash, fatigue) and how design reduces risk.

### Level 4/5 Higher National

- **Kinematics of mechanisms**
  - Use or derive forward kinematics (joint angles → XYZ position of end effector).
  - Work with homogeneous transformation matrices and coordinate frames for each link.
  - Explain joint limits and singularities and how they affect reachable workspace and motion.

- **Mechatronic system integration**
  - Describe the arm as a mechatronic system combining mechanics, actuators, sensors, controller and software.
  - Analyse how mechanical design (e.g. backlash) and control strategy influence accuracy and repeatability.

---

## 2. Electrical, Electronic & Control Systems

### Level 3 National

- **Basic electrical / electronic systems**
  - Identify key hardware: power supply, Raspberry Pi (or controller), serial bus, ST3215 servos.
  - Interpret simple wiring diagrams for power, ground, and data daisy‑chains.

- **Open‑loop and closed‑loop control**
  - Distinguish between the front‑end “commanded position” (open‑loop from the app’s perspective) and the servo’s internal closed‑loop control.
  - Explain what feedback is available from the servos: present position, speed, load, voltage, temperature, moving flag, torque on/off.

### Level 4/5 Higher National

- **Control and communication**
  - Describe the serial protocol: baud rate, packet layout (headers, ID, length, instruction, parameters, checksum).
  - Identify major contributors to latency: serial speed, per‑servo status polling, timeouts, and command queueing.
  - Justify design choices for timeouts, polling intervals, and command queue limits.

- **Feedback and monitoring**
  - Use live feedback (position, speed, load, voltage, temperature) to monitor system health.
  - Detect abnormal conditions (e.g. under‑voltage, high temperature, unusually high load).
  - Use moving/torque flags to reason about servo state and safety.

---

## 3. Software, Programming & Automation

### Level 3 National

- **Programming for motion control**
  - Use the Electron app / Blockly interface to:
    - Command individual joint movements.
    - Store, recall, and edit joint positions.
    - Build and run simple motion sequences (e.g. pick‑and‑place).
  - Understand the concept of a control loop: read status → decide → send commands → repeat.

- **Human–machine interface (HMI)**
  - Operate the pendant / UI safely:
    - Enable and disable torque.
    - Set speed and acceleration limits.
    - Jog joints and run/stop programs.
  - Explain how UI actions (sliders, text inputs, blocks) become WebSocket commands and then servo commands.

### Level 4/5 Higher National

- **Application architecture and integration**
  - Describe the software architecture:
    - Electron front‑end (UI and visualisation),
    - Node.js/WebSocket server on the Pi,
    - ST3215 servo communication layer on the shared serial bus.
  - Modify or extend control logic in code:
    - Joint motion commands with speed and acceleration.
    - Coordinated multi‑joint moves with shared timing.
    - Status polling and UI updates.

- **Path planning and motion profiles**
  - Explain differences between joint‑space control (commanding joint angles) and Cartesian‑space goals (target XYZ).
  - Analyse simple motion profiles (using speed and acceleration settings) and their effect on smoothness and mechanical stress.
  - Evaluate trade‑offs between speed, accuracy, and wear on mechanical components.

- **Data logging and diagnostics**
  - Capture logs of joint angles, loads, voltages, and errors over time.
  - Diagnose issues such as:
    - Serial timeouts,
    - Invalid position data,
    - Power/voltage dips,
    - Overload conditions on joints.
  - Use log data to refine polling intervals and timeouts for reliability.

---

## 4. Automation, Manufacturing & Workflows

### Level 3 National

- **Industrial context for robot arms**
  - Relate the educational robot arm to industrial uses:
    - Pick‑and‑place,
    - Assembly,
    - Welding and painting,
    - Inspection and sorting.
  - Compare the arm’s capabilities (reach, payload, repeatability, speed) to typical industrial manipulators.

- **Basic automated sequences**
  - Design and implement simple automated tasks:
    - Move between taught points,
    - Use dwell/wait steps,
    - Return to a safe “home” pose.
  - Explain how such sequences fit into wider automated processes.

### Level 4/5 Higher National

- **Cell/workstation design**
  - Design a small automated “cell” around the robot arm:
    - Safe working envelope and guarding,
    - Part presentation (jigs, fixtures, trays),
    - Operator access and ergonomics.
  - Estimate throughput and discuss bottlenecks and changeover times.

- **System integration concepts**
  - Outline how the robot could interface with:
    - Conveyors,
    - Vision systems,
    - PLCs and other controllers,
    - Safety systems.
  - Describe, at a conceptual level, how signals and data would be exchanged.

---

## 5. Safety, Risk & Compliance

### Level 3 National

- **Safe operation**
  - Identify key hazards:
    - Pinch and crush points,
    - Unexpected or rapid motion,
    - Electrical hazards.
  - Follow safe operating procedures:
    - Enabling/disabling torque,
    - Keeping clear of the arm’s envelope,
    - Using emergency stop or software stop where available.

- **Basic risk assessment**
  - Perform a simple risk assessment for a robot‑arm activity:
    - Identify hazards,
    - Assess likelihood and severity,
    - Propose basic control measures (speed limits, supervision, workspace layout).

### Level 4/5 Higher National

- **Standards and regulations awareness**
  - Recognise that industrial robot systems are subject to machinery and safety standards (e.g. ISO 10218, relevant directives).
  - Map typical safety requirements (safe stops, emergency stops, guarding, modes of operation) to the educational robot setup.

- **Designing for safety**
  - Justify safety‑related features in the hardware and software:
    - Joint limits,
    - Speed and acceleration caps,
    - Torque on/off control,
    - Conservative defaults on communication errors.
  - Propose safe behaviours on:
    - Loss of communication,
    - Power failure,
    - Sensor or feedback anomalies.

---

## 6. Project Work, Documentation & Professional Skills

### Across Level 3 and Level 4/5

- **Project planning and management**
  - Plan a robot‑arm project:
    - Define aims and requirements,
    - Break work into tasks (mechanical, electrical, software, testing),
    - Produce and follow a simple schedule.

- **Technical documentation**
  - Produce and maintain:
    - System block diagrams and architecture descriptions,
    - Wiring diagrams and interface descriptions,
    - Kinematic/URDF description of the arm,
    - Test plans, procedures, and results.

- **Evaluation and reflection**
  - Evaluate how well the robot arm solution meets:
    - Functional requirements (does it complete the task?),
    - Performance requirements (speed, repeatability, accuracy),
    - Safety and reliability expectations.
  - Reflect on limitations and propose realistic improvements.

- **Communication skills**
  - Present the robot arm system and project outcomes to different audiences:
    - Technical (peers, assessors),
    - Non‑technical (students, visitors, management).
  - Explain:
    - What the arm does and why it is useful,
    - How it is operated safely,
    - How it supports progression to employment or further study in engineering.

