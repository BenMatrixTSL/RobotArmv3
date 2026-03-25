# Module 3: Basic Electrical & Electronic Systems
## Multiple Choice Questions

**Instructions:** Choose the best answer for each question. Circle or highlight your choice.

---

## Section A: Electrical Components

**1. What is the main purpose of the power supply in a robot arm system?**
   a) To control the robot's movements
   b) To provide electrical power to all components
   c) To program the robot
   d) To make the robot look good

**2. What is the Raspberry Pi's role in the robot arm system?**
   a) To provide power
   b) To act as the controller/brain of the system
   c) To move the joints
   d) To hold the robot together

**3. What is a serial bus used for?**
   a) To provide power
   b) To allow communication between components
   c) To hold the robot together
   d) To make the robot faster

**4. What are ST3215 servos?**
   a) The power supply
   b) The motors that move the robot arm joints
   c) The controller
   d) The wiring

---

## Section B: Wiring Basics

**5. What are the three main types of power connections?**
   a) Red, blue, green
   b) Positive, negative, ground
   c) Up, down, sideways
   d) Fast, slow, medium

**6. What is "daisy-chain" wiring?**
   a) Connecting components in a series, one after another
   b) Connecting all components to one central point
   c) Using flower-shaped connectors
   d) Random connections

**7. In a daisy-chain, how are servos typically connected?**
   a) Each servo connects directly to the Raspberry Pi
   b) Servos connect to each other in a chain, sharing the data line
   c) Servos don't need to be connected
   d) Only the first servo is connected

**8. What is the difference between ground and negative?**
   a) There is no difference
   b) Ground is a reference point, negative is the return path
   c) Ground is positive, negative is ground
   d) They are the same thing

---

## Section C: Control Concepts

**9. What is open-loop control?**
   a) Control that uses feedback to adjust
   b) Control that does not use feedback
   c) Control that only works in circles
   d) Control that is always open

**10. What is closed-loop control?**
   a) Control that uses feedback to adjust
   b) Control that does not use feedback
   c) Control that is always closed
   d) Control that only works in straight lines

**11. From the app's perspective, commanding a position is:**
   a) Closed-loop control
   b) Open-loop control (command sent without checking if it was received)
   c) Both open and closed loop
   d) Neither open nor closed loop

**12. Inside the servo, the control is:**
   a) Open-loop only
   b) Closed-loop (uses position feedback to reach target)
   c) No control at all
   d) Random control

---

## Section D: Servo Feedback

**13. What feedback does a servo provide about its position?**
   a) Present position
   b) Future position
   c) Past position only
   d) No position feedback

**14. What does the "load" feedback tell you?**
   a) How much the servo weighs
   b) How much force/torque the servo is experiencing
   c) How fast the servo is moving
   d) The servo's temperature

**15. What does the "voltage" feedback tell you?**
   a) How fast the servo is moving
   b) The electrical power level supplied to the servo
   c) The servo's position
   d) The servo's temperature

**16. What does the "temperature" feedback tell you?**
   a) The room temperature
   b) How hot the servo is getting
   c) The servo's speed
   d) The servo's position

**17. What does the "moving flag" indicate?**
   a) Whether the servo is currently moving
   b) The servo's color
   c) The servo's position
   d) The servo's speed setting

**18. What does "torque on/off" indicate?**
   a) Whether the servo motor is enabled or disabled
   b) The servo's speed
   c) The servo's position
   d) The servo's temperature

---

## Section E: Practical Application

**19. Why might there be a difference between commanded position and actual position?**
   a) The servo is broken
   b) Mechanical play, load, or the servo hasn't finished moving yet
   c) There should never be a difference
   d) The software is wrong

**20. Why is feedback important for safe operation?**
   a) It makes the robot look better
   b) It allows monitoring of system health and detection of problems
   c) It makes the robot faster
   d) It's not important

---

## Answer Key

1. **b)** To provide electrical power to all components
2. **b)** To act as the controller/brain of the system
3. **b)** To allow communication between components
4. **b)** The motors that move the robot arm joints
5. **b)** Positive, negative, ground
6. **a)** Connecting components in a series, one after another
7. **b)** Servos connect to each other in a chain, sharing the data line
8. **b)** Ground is a reference point, negative is the return path
9. **b)** Control that does not use feedback
10. **a)** Control that uses feedback to adjust
11. **b)** Open-loop control (command sent without checking if it was received)
12. **b)** Closed-loop (uses position feedback to reach target)
13. **a)** Present position
14. **b)** How much force/torque the servo is experiencing
15. **b)** The electrical power level supplied to the servo
16. **b)** How hot the servo is getting
17. **a)** Whether the servo is currently moving
18. **a)** Whether the servo motor is enabled or disabled
19. **b)** Mechanical play, load, or the servo hasn't finished moving yet
20. **b)** It allows monitoring of system health and detection of problems

---

## Scoring Guide

- **18-20 correct:** Excellent understanding
- **15-17 correct:** Good understanding
- **12-14 correct:** Satisfactory understanding
- **Below 12 correct:** Needs review of module content
