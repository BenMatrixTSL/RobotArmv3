# Module 4: Basic Software & Human-Machine Interface
## Multiple Choice Questions

**Instructions:** Choose the best answer for each question. Circle or highlight your choice.

---

## Section A: Software Interface

**1. What is the Electron app used for?**
   a) To power the robot
   b) To control and program the robot arm
   c) To build the robot
   d) To paint the robot

**2. Before operating the robot arm, what should you do first?**
   a) Start moving it immediately
   b) Enable torque and set speed/acceleration limits
   c) Remove all safety guards
   d) Increase speed to maximum

**3. What is the purpose of setting speed limits?**
   a) To make the robot slower
   b) To ensure safe operation and prevent damage
   c) To save electricity
   d) To make it look better

**4. What is the purpose of setting acceleration limits?**
   a) To prevent sudden movements and reduce mechanical stress
   b) To make the robot faster
   c) To save space
   d) To reduce cost

---

## Section B: Joint Control

**5. What does "jogging" a joint mean?**
   a) Running with the robot
   b) Manually moving a single joint to a specific position
   c) Stopping the robot
   d) Programming the robot

**6. What are joint limits?**
   a) The maximum speed a joint can move
   b) The minimum and maximum angles a joint can reach
   c) The number of joints
   d) The weight of joints

**7. What happens if you try to move a joint beyond its limits?**
   a) The robot breaks
   b) The movement is prevented or limited by the software
   c) Nothing happens
   d) The robot moves faster

**8. Why is it important to know joint limits?**
   a) To avoid damaging the robot
   b) To make it move faster
   c) To save electricity
   d) It's not important

---

## Section C: Position Teaching

**9. What does "teaching" a position mean?**
   a) Learning about positions
   b) Moving the robot to a position and storing it for later use
   c) Deleting positions
   d) Moving positions

**10. How do you store a position?**
   a) By moving the robot and clicking a save/store button
   b) By turning off the robot
   c) By moving it randomly
   d) By programming it

**11. How do you recall a stored position?**
   a) By selecting it from a list and commanding the robot to move there
   b) By moving the robot manually
   c) By restarting the software
   d) By deleting it

**12. Can you edit a stored position?**
   a) No, positions cannot be changed
   b) Yes, by modifying the stored values
   c) Only by deleting and recreating
   d) Only by moving the robot

---

## Section D: Motion Sequences

**13. What is a motion sequence?**
   a) A series of stored positions executed in order
   b) A single position
   c) The robot stopping
   d) The robot moving randomly

**14. What is a "dwell" or "wait" step in a sequence?**
   a) A pause for a specified time
   b) Moving faster
   c) Stopping permanently
   d) Moving backward

**15. What is a "home" position?**
   a) A safe starting/return position
   b) The robot's base
   c) The end effector
   d) The controller

**16. How do you start a sequence?**
   a) By clicking a "run" or "start" button
   b) By turning off the robot
   c) By moving it manually
   d) By deleting it

**17. How do you stop a sequence safely?**
   a) By using a stop button or emergency stop
   b) By unplugging the robot
   c) By hitting the robot
   d) By waiting for it to finish

---

## Section E: Control Loop

**18. What is a control loop?**
   a) A circle the robot moves in
   b) A repeating cycle: read status → decide → send commands → repeat
   c) A single command
   d) The robot stopping

**19. What is the first step in a control loop?**
   a) Send commands
   b) Read status from servos
   c) Decide what to do
   d) Stop the robot

**20. Why is timing important in a control loop?**
   a) To make it look good
   b) To ensure smooth operation and responsiveness
   c) To save electricity
   d) It's not important

---

## Section F: Command Flow

**21. When you move a slider in the UI, what happens next?**
   a) The robot moves immediately
   b) The command is sent via WebSocket to the server
   c) Nothing happens
   d) The robot stops

**22. What is WebSocket used for?**
   a) To provide power
   b) To enable real-time communication between UI and server
   c) To hold the robot together
   d) To make it faster

**23. How does a UI action become a servo command?**
   a) UI → WebSocket → Server → Servo command
   b) Direct connection
   c) Through the power supply
   d) It doesn't

**24. What happens after a command is sent to a servo?**
   a) Nothing
   b) Feedback is returned showing the servo's status
   c) The robot stops
   d) The command is deleted

---

## Answer Key

1. **b)** To control and program the robot arm
2. **b)** Enable torque and set speed/acceleration limits
3. **b)** To ensure safe operation and prevent damage
4. **a)** To prevent sudden movements and reduce mechanical stress
5. **b)** Manually moving a single joint to a specific position
6. **b)** The minimum and maximum angles a joint can reach
7. **b)** The movement is prevented or limited by the software
8. **a)** To avoid damaging the robot
9. **b)** Moving the robot to a position and storing it for later use
10. **a)** By moving the robot and clicking a save/store button
11. **a)** By selecting it from a list and commanding the robot to move there
12. **b)** Yes, by modifying the stored values
13. **a)** A series of stored positions executed in order
14. **a)** A pause for a specified time
15. **a)** A safe starting/return position
16. **a)** By clicking a "run" or "start" button
17. **a)** By using a stop button or emergency stop
18. **b)** A repeating cycle: read status → decide → send commands → repeat
19. **b)** Read status from servos
20. **b)** To ensure smooth operation and responsiveness
21. **b)** The command is sent via WebSocket to the server
22. **b)** To enable real-time communication between UI and server
23. **a)** UI → WebSocket → Server → Servo command
24. **b)** Feedback is returned showing the servo's status

---

## Scoring Guide

- **22-24 correct:** Excellent understanding
- **18-21 correct:** Good understanding
- **14-17 correct:** Satisfactory understanding
- **Below 14 correct:** Needs review of module content
