# Module 4: Basic Software & Human-Machine Interface
## Theory & Teaching Notes

**Module Duration:** 2-3 hours  
**Level:** Foundation (Level 3)

---

## Learning Objectives

By the end of this module, students will be able to:
- Use the control software interface safely
- Command individual joint movements
- Store and recall positions
- Create and run motion sequences
- Understand the control loop concept
- Explain how commands flow from UI to servos

---

## 1. Introduction to Control Software

### What is the Control Software?

The **control software** is the program that lets you operate the robot arm. It's like the "remote control" for the robot.

### Electron App

**Electron** is a framework that lets developers create desktop applications using web technologies.

**Why Electron?**
- Works on different computers (Windows, Mac, Linux)
- Easy to update
- Can have a nice user interface
- Connects to the robot easily

### Main Screen Layout

The software typically has:

1. **Joint Control Area**
   - Sliders or buttons for each joint
   - Shows current position
   - Allows manual control

2. **Position Library**
   - List of saved positions
   - Can store, recall, edit positions

3. **Sequence Builder**
   - Create motion sequences
   - Add waypoints
   - Set timing

4. **Status Display**
   - Shows feedback from servos
   - Displays errors
   - Shows system status

5. **Control Buttons**
   - Enable/disable torque
   - Start/stop sequences
   - Emergency stop

---

## 2. Basic Operations

### Enabling and Disabling Torque

**Torque** = The rotational force that makes joints move

**Disabling Torque:**
- Prevents accidental movement
- Allows safe manual positioning
- Should be done when:
  - Setting up
  - Teaching positions
  - Making adjustments
  - Not in use

**Enabling Torque:**
- Allows robot to move
- Should only be done when:
  - Ready to operate
  - Workspace is clear
  - Know how to stop
  - Supervised

**How to do it safely:**
1. Check workspace is clear
2. Check you know how to stop
3. Enable torque slowly
4. Test movement carefully
5. Monitor constantly

### Setting Speed Limits

**Speed limit** = Maximum speed the robot can move

**Why set limits:**
- Safety (slower = safer)
- Control (easier to monitor)
- Accuracy (slower = more accurate)
- Reduce wear

**How to set:**
- Use slider or input box
- Start with low speed (e.g., 30%)
- Increase gradually if safe
- Never exceed safe limits

**Typical range:** 10% to 100%

### Setting Acceleration Limits

**Acceleration limit** = How quickly speed can change

**Why set limits:**
- Prevents sudden movements
- Reduces mechanical stress
- Improves smoothness
- Better control

**How to set:**
- Use slider or input box
- Start with low acceleration
- Increase gradually
- Balance smoothness vs. speed

**Typical range:** 10% to 100%

---

## 3. Joint Control

### Jogging Joints

**Jogging** = Manually moving one joint at a time

**How to jog:**
1. Select the joint you want to move
2. Use slider or buttons to move it
3. Watch the position change
4. Stop when at desired position

**Why jog:**
- Precise positioning
- Teaching positions
- Testing movement
- Safe, controlled movement

### Reading Joint Positions

**Position** = Current angle of the joint

**Displayed as:**
- Degrees (°)
- Sometimes as percentage
- Or as encoder counts

**Why it matters:**
- Know where robot is
- Teach accurate positions
- Verify movements
- Troubleshoot problems

### Joint Limits

**Joint limits** = Minimum and maximum angles

**Why they exist:**
- Prevent damage
- Stay in safe range
- Avoid collisions
- Protect mechanisms

**What happens if you exceed limits:**
- Software prevents movement
- Or movement stops at limit
- Warning message
- Safety feature

---

## 4. Position Teaching

### What is Teaching?

**Teaching** = Moving robot to a position and saving it

**Why teach positions:**
- Create position library
- Use in sequences
- Return to same place
- Repeat operations

### Storing Positions

**How to store:**
1. Move robot to desired position (by jogging)
2. Click "Store" or "Save Position"
3. Give it a name
4. Position is saved

**Good position names:**
- Descriptive: "Pick Position", "Place Position"
- Clear: "Home", "Position 1"
- Meaningful: "Above Box", "Over Conveyor"

### Recalling Positions

**How to recall:**
1. Select position from list
2. Click "Move To" or "Recall"
3. Robot moves to that position

**Why recall:**
- Repeat movements
- Use in sequences
- Return to known positions
- Consistency

### Editing Positions

**How to edit:**
1. Select position from list
2. Click "Edit"
3. Modify joint angles
4. Save changes

**When to edit:**
- Fine-tune positions
- Adjust for changes
- Correct errors
- Optimize movements

---

## 5. Motion Sequences

### What is a Sequence?

A **sequence** is a series of positions executed in order.

**Example sequence:**
1. Move to Position A (pick position)
2. Wait 1 second
3. Move to Position B (place position)
4. Wait 1 second
5. Return to Home

### Building a Sequence

**Steps to build:**

1. **Plan the sequence**
   - What positions are needed?
   - What order?
   - Any waits needed?

2. **Teach positions**
   - Move to each position
   - Store each one
   - Name them clearly

3. **Add to sequence**
   - Select position
   - Add to sequence
   - Set timing if needed

4. **Add dwell/wait**
   - Insert wait steps
   - Set wait time
   - Allows gripper to operate, etc.

5. **Test sequence**
   - Run slowly first
   - Check each step
   - Adjust if needed

### Dwell/Wait Steps

**Dwell** = A pause in the sequence

**Why use dwell:**
- Allow gripper to close/open
- Wait for part to settle
- Ensure operation completes
- Safety pause

**How to add:**
- Insert "Wait" step
- Set time (e.g., 1 second)
- Position in sequence

### Home Position

**Home position** = A safe starting/return position

**Characteristics:**
- Safe (no collisions)
- Easy to reach
- Known position
- Good starting point

**Why important:**
- Consistent starting point
- Safe return position
- Easy to find
- Reference point

---

## 6. Running Sequences

### Starting a Sequence

**How to start:**
1. Check workspace is clear
2. Check robot is ready
3. Select sequence
4. Click "Run" or "Start"
5. Monitor movement

**Before starting:**
- Workspace clear?
- Speed limits set?
- Emergency stop ready?
- Supervised?

### Stopping a Sequence

**Normal stop:**
- Click "Stop" button
- Robot stops at next waypoint
- Or stops immediately (depends on software)

**Emergency stop:**
- Press emergency stop button
- Immediate stop
- Use if unsafe

**After stopping:**
- Check robot position
- Check for problems
- Don't restart immediately
- Understand why stopped

---

## 7. Understanding the Control Loop

### What is a Control Loop?

A **control loop** is a repeating cycle that controls the robot.

### The Control Loop Steps

**Step 1: Read Status**
- Read feedback from servos
- Get current positions
- Check for errors
- Monitor system state

**Step 2: Decide**
- What should happen next?
- Compare desired vs. actual
- Calculate needed movement
- Check safety conditions

**Step 3: Send Commands**
- Send movement commands
- Update positions
- Control speed/acceleration
- Manage sequences

**Step 4: Repeat**
- Go back to Step 1
- Continue cycle
- Keep controlling

### Visual Representation

```
┌─────────────┐
│ Read Status │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│   Decide    │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│Send Commands│
└──────┬──────┘
       │
       └──────┐
              │
              ▼
         (Repeat)
```

### Why It Matters

**The control loop:**
- Keeps robot under control
- Responds to changes
- Maintains accuracy
- Enables automation

**Timing:**
- Loop runs many times per second
- Fast enough for smooth control
- Slow enough to process everything

---

## 8. Command Flow: From UI to Servo

### How Commands Travel

**Step 1: User Action**
- You move a slider
- Or click a button
- Or run a sequence

**Step 2: UI Processing**
- Software processes your action
- Converts to command
- Prepares to send

**Step 3: WebSocket Communication**
- Command sent via WebSocket
- Real-time communication
- Goes to server (Raspberry Pi)

**Step 4: Server Processing**
- Server receives command
- Processes it
- Converts to servo command

**Step 5: Serial Communication**
- Command sent via serial bus
- Goes to specific servo
- Servo receives command

**Step 6: Servo Execution**
- Servo processes command
- Moves to position
- Uses internal control loop

**Step 7: Feedback Return**
- Servo sends feedback
- Back through serial bus
- To server
- To UI
- Displayed to you

### Visual Flow

```
You → UI → WebSocket → Server → Serial → Servo
                                              │
                                              ▼
You ← UI ← WebSocket ← Server ← Serial ← Feedback
```

### Why Understand This?

**Understanding command flow helps you:**
- Troubleshoot problems
- Understand delays
- Know where errors occur
- Optimize performance

---

## 9. Practical Tips

### Safe Operation

1. **Always check before starting**
   - Workspace clear?
   - Speed limits set?
   - Emergency stop ready?

2. **Start slow**
   - Low speed first
   - Test movements
   - Increase gradually

3. **Monitor constantly**
   - Watch robot movement
   - Check feedback
   - Be ready to stop

4. **Use limits**
   - Speed limits
   - Acceleration limits
   - Joint limits

### Efficient Operation

1. **Organize positions**
   - Use clear names
   - Group related positions
   - Document purposes

2. **Test sequences**
   - Test slowly first
   - Check each step
   - Verify timing

3. **Save your work**
   - Save positions
   - Save sequences
   - Backup files

---

## 10. Summary

### Key Concepts

1. **Control software** lets you operate the robot
2. **Torque enable/disable** controls when robot can move
3. **Speed and acceleration limits** ensure safe operation
4. **Jogging** allows manual joint control
5. **Teaching** saves positions for later use
6. **Sequences** automate movements
7. **Control loop** continuously controls the robot
8. **Command flow** goes from UI through server to servos

### Understanding Your Software

You should now understand:
- How to use the interface safely
- How to control joints
- How to teach positions
- How to create sequences
- How commands flow through the system

### Next Steps

After learning this theory, you will:
- Practice using the software
- Teach positions
- Create sequences
- Understand the control loop
- Complete the Module 4 worksheet

---

## Questions for Review

Before moving to the worksheet, make sure you can answer:

1. What is the control software used for?
2. Why should you set speed limits?
3. What is jogging?
4. How do you teach a position?
5. What is a motion sequence?
6. What are the steps in a control loop?
7. How does a command flow from UI to servo?

---

**Remember:** Practice makes perfect. Start slowly, be safe, and gradually build your skills with the software.
