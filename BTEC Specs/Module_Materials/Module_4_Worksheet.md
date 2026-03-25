# Module 4: Basic Software & Human-Machine Interface
## Student Worksheet

**Name:** _________________________  
**Date:** _________________________  
**Group/Class:** _________________________

---

## Part 1: Introduction to Control Software

### Question 1: Software Interface Overview
Complete the following about the Electron app interface:

a) What are the main sections/areas of the interface?
   1. _________________________
   2. _________________________
   3. _________________________
   4. _________________________

b) How do you navigate between different functions?
   _________________________________________________________________

### Question 2: Basic Operations
Answer the following:

a) Where is the torque enable/disable control located?
   _________________________________________________________________

b) Where can you set speed limits?
   _________________________________________________________________

c) Where can you set acceleration limits?
   _________________________________________________________________

d) Why are these limits important?
   _________________________________________________________________
   _________________________________________________________________

---

## Part 2: Joint Control

### Question 3: Jogging Joints
Complete the following exercises:

**Exercise A:** Jog Joint 1 (Base) to the left
- Commanded position: _______
- Actual position (from feedback): _______

**Exercise B:** Jog Joint 2 (Shoulder) upward
- Commanded position: _______
- Actual position (from feedback): _______

**Exercise C:** Jog Joint 3 (Elbow) to bend
- Commanded position: _______
- Actual position (from feedback): _______

### Question 4: Joint Limits
Answer the following:

a) What are the joint limits for each joint?

| Joint | Minimum Limit | Maximum Limit |
|-------|---------------|---------------|
| Joint 1 | | |
| Joint 2 | | |
| Joint 3 | | |
| Joint 4 | | |

b) What happens if you try to move beyond these limits?
   _________________________________________________________________

---

## Part 3: Position Teaching

### Question 5: Storing Positions
Teach and store 5 different positions:

| Position Name | Joint 1 | Joint 2 | Joint 3 | Joint 4 | Description |
|---------------|---------|---------|---------|---------|-------------|
| Home | | | | | |
| Position 1 | | | | | |
| Position 2 | | | | | |
| Position 3 | | | | | |
| Position 4 | | | | | |

### Question 6: Recalling and Editing Positions
Complete the following:

a) How do you recall a stored position?
   _________________________________________________________________

b) How do you edit a stored position?
   _________________________________________________________________

c) Practice: Recall "Home" position and verify the robot moves there.
   **Did it work correctly?** Yes / No
   **Notes:** _________________________________________________________________

---

## Part 4: Motion Sequences

### Question 7: Building a Sequence
Create a simple pick-and-place sequence:

**Step 1:** Move to position: _________________________  
**Step 2:** Wait/dwell time: _______ seconds  
**Step 3:** Move to position: _________________________  
**Step 4:** Wait/dwell time: _______ seconds  
**Step 5:** Return to home position

**Sequence description:**
_________________________________________________________________
_________________________________________________________________

### Question 8: Running Sequences
Answer the following:

a) How do you start a sequence?
   _________________________________________________________________

b) How do you stop a sequence safely?
   _________________________________________________________________

c) What should you check before running a sequence?
   _________________________________________________________________
   _________________________________________________________________

---

## Part 5: Understanding the Control Loop

### Question 9: Control Loop Concept
Draw a flowchart showing the control loop:

**Steps:**
1. Read status from servos
2. Decide what to do
3. Send commands to servos
4. Repeat

[Space for flowchart]

### Question 10: Control Loop Timing
Answer the following:

a) How often does the control loop run? (approximately)
   _________________________________________________________________

b) Why is timing important in the control loop?
   _________________________________________________________________
   _________________________________________________________________

c) What happens if the loop runs too slowly?
   _________________________________________________________________

---

## Part 6: Command Flow

### Question 11: From UI to Servo
Trace how a command flows from the user interface to the servo:

**Step 1:** User action (e.g., moving a slider)
   **What happens:** _________________________________________________________________

**Step 2:** WebSocket communication
   **What happens:** _________________________________________________________________

**Step 3:** Servo command generation
   **What happens:** _________________________________________________________________

**Step 4:** Feedback return
   **What happens:** _________________________________________________________________

### Question 12: Command Flow Diagram
Draw a diagram showing the complete command flow:

[Space for diagram]

---

## Part 7: Practical Exercise

### Exercise 1: Complete Operation Sequence
Complete this full sequence:

1. [ ] Enable torque safely
2. [ ] Set speed limit to 50%
3. [ ] Set acceleration limit to 30%
4. [ ] Jog each joint individually
5. [ ] Teach 3 positions
6. [ ] Create a simple sequence using those positions
7. [ ] Run the sequence
8. [ ] Stop the sequence safely
9. [ ] Disable torque
10. [ ] Return to home position

**Time taken:** _______ minutes  
**Any problems encountered:** _________________________________________________________________
_________________________________________________________________

### Exercise 2: Control Loop Observation
While running a sequence, observe the control loop:

- How often does the position update? _______
- Can you see the feedback being read? Yes / No
- Can you see commands being sent? Yes / No

**Observations:**
_________________________________________________________________
_________________________________________________________________

---

## Part 8: Reflection

### Question 13: Understanding Software and HMI
Answer the following:

a) Why is it important to understand how the software works?
   _________________________________________________________________
   _________________________________________________________________

b) How does understanding the control loop help you use the robot arm?
   _________________________________________________________________
   _________________________________________________________________

c) What was the most challenging part of using the software?
   _________________________________________________________________
   _________________________________________________________________

---

## Self-Assessment Checklist

Before submitting, check that you have:

- [ ] Familiarized yourself with the software interface
- [ ] Set speed and acceleration limits safely
- [ ] Jogged all joints individually
- [ ] Taught and stored at least 5 positions
- [ ] Recalled and edited positions
- [ ] Created a motion sequence
- [ ] Run and stopped a sequence safely
- [ ] Understood the control loop concept
- [ ] Traced command flow from UI to servo
- [ ] Completed practical exercises
- [ ] Written reflection answers

**Student Signature:** _________________________  
**Date:** _________________________  
**Instructor Signature:** _________________________  
**Date:** _________________________
