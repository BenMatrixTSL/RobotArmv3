# Module 9: Software Architecture & Advanced Programming
## Theory & Teaching Notes

**Module Duration:** 3-4 hours  
**Level:** Advanced (Level 4/5)

---

## Learning Objectives

By the end of this module, students will be able to:
- Describe the software architecture
- Understand how components interact
- Modify and extend control logic
- Understand motion control modes
- Analyze motion profiles
- Evaluate trade-offs between speed, accuracy, and wear

---

## 1. Software Architecture Overview

### Three-Layer Architecture

The robot arm software has three main layers:

#### Layer 1: Electron Front-End
**What it is:** User interface application

**What it does:**
- Displays robot status
- Accepts user input
- Shows visualizations
- Provides controls

**Technology:** Electron (web technologies in desktop app)

**Location:** Runs on your computer

#### Layer 2: Node.js/WebSocket Server
**What it is:** Communication and processing server

**What it does:**
- Receives commands from front-end
- Processes commands
- Sends commands to servos
- Receives feedback from servos
- Sends feedback to front-end

**Technology:** Node.js (JavaScript server)

**Location:** Runs on Raspberry Pi

#### Layer 3: ST3215 Servo Communication Layer
**What it is:** Low-level servo control

**What it does:**
- Communicates with servos via serial bus
- Sends servo commands
- Receives servo feedback
- Handles protocol details

**Technology:** Serial communication

**Location:** Runs on Raspberry Pi

### How Components Interact

**Data Flow:**

```
User → Electron Front-End → WebSocket → Node.js Server → Serial → Servos
                                                                      │
                                                                      ▼
User ← Electron Front-End ← WebSocket ← Node.js Server ← Serial ← Feedback
```

**Key points:**
- Front-end handles user interaction
- Server processes and coordinates
- Communication layer handles low-level details
- Feedback flows back through all layers

---

## 2. Extending Control Logic

### Understanding Code Structure

**Before modifying code, understand:**
- How code is organized
- Where different functions are
- How data flows
- What each part does

**Typical structure:**
- **UI handlers:** Handle user input
- **Command processors:** Process commands
- **Servo controllers:** Control servos
- **Feedback handlers:** Process feedback
- **Utilities:** Helper functions

### Adding New Joint Motion Commands

**Steps to add new command:**

1. **Define the command**
   - What should it do?
   - What parameters does it need?
   - What should it return?

2. **Implement the command**
   - Write the code
   - Handle parameters
   - Send to servo
   - Handle response

3. **Add to UI**
   - Add button or control
   - Connect to command
   - Display results

4. **Test**
   - Test with different parameters
   - Test error cases
   - Verify it works

### Coordinated Multi-Joint Moves

**What it is:** Moving multiple joints together with shared timing

**Why useful:**
- Smooth movement
- Coordinated motion
- Better control
- More natural movement

**How to implement:**

1. **Calculate target positions** for all joints
2. **Plan timing** (how long to take)
3. **Send commands** to all joints simultaneously
4. **Monitor progress** of all joints
5. **Wait for completion** of all joints

**Example:** Moving shoulder, elbow, and wrist together to reach a position smoothly.

### Status Polling Implementation

**Status polling** = Regularly checking servo status

**How to implement:**

1. **Set up timer** (e.g., every 30ms)
2. **On timer:** Request status from servos
3. **Process responses:** Update internal state
4. **Send to UI:** Update display
5. **Repeat**

**Considerations:**
- Polling frequency (how often?)
- Which servos to poll?
- Error handling
- Performance impact

### UI Update Mechanisms

**How UI gets updated:**

1. **Server receives feedback** from servos
2. **Server processes feedback**
3. **Server sends update** to front-end via WebSocket
4. **Front-end receives update**
5. **Front-end updates display**

**Update types:**
- Position updates
- Status updates
- Error notifications
- System state changes

---

## 3. Motion Control Modes

### Joint-Space Control

**What it is:** Commanding joint angles directly

**Example:**
- Joint 1 = 45°
- Joint 2 = 30°
- Joint 3 = -20°

**Advantages:**
- Direct control
- Simple to implement
- Fast execution
- No conversion needed

**Disadvantages:**
- Harder to plan paths
- Don't know end effector position directly
- May cause collisions

**When to use:**
- Simple movements
- Known safe positions
- When speed is priority

### Cartesian-Space Control

**What it is:** Commanding end effector position (X, Y, Z)

**Example:**
- X = 100mm
- Y = 200mm
- Z = 150mm

**Advantages:**
- Intuitive (know where end effector goes)
- Easier path planning
- Better for applications
- Natural for humans

**Disadvantages:**
- Requires inverse kinematics
- More complex
- May have multiple solutions
- Slower execution

**When to use:**
- Precise positioning needed
- Path planning important
- Application-specific tasks

### Converting Between Spaces

**Forward kinematics:** Joint space → Cartesian space
- Given joint angles, calculate end effector position

**Inverse kinematics:** Cartesian space → Joint space
- Given end effector position, calculate joint angles

**Why conversion matters:**
- Different control modes need different inputs
- Applications may need Cartesian coordinates
- Robot needs joint angles to move

---

## 4. Motion Profiles

### What is a Motion Profile?

**Motion profile** = How speed changes over time during movement

**Think of it like:** Acceleration and deceleration curve

### Speed Settings and Effects

**Speed setting** = Maximum speed during movement

**Effects:**

**High speed:**
- ✅ Faster completion
- ✅ Higher productivity
- ❌ Less smooth
- ❌ More stress
- ❌ Less accurate

**Low speed:**
- ✅ Smoother movement
- ✅ Less stress
- ✅ More accurate
- ❌ Slower completion
- ❌ Lower productivity

**Trade-off:** Speed vs. smoothness vs. accuracy

### Acceleration Settings and Effects

**Acceleration** = How quickly speed changes

**Effects:**

**High acceleration:**
- ✅ Faster to reach speed
- ✅ Quicker movements
- ❌ Jerky movement
- ❌ High stress
- ❌ Less smooth

**Low acceleration:**
- ✅ Smooth movement
- ✅ Low stress
- ✅ Better control
- ❌ Slower to reach speed
- ❌ Slower overall

**Trade-off:** Acceleration vs. smoothness vs. stress

### Smoothness Considerations

**Smooth movement:**
- Gradual speed changes
- No sudden stops/starts
- Continuous motion
- Better for mechanisms

**Jerk movement:**
- Sudden speed changes
- Abrupt stops/starts
- Discontinuous motion
- Hard on mechanisms

**Why smoothness matters:**
- Reduces wear
- Better accuracy
- Quieter operation
- Longer lifespan

### Mechanical Stress Implications

**High stress causes:**
- Faster wear
- Potential damage
- Reduced lifespan
- More maintenance

**Factors affecting stress:**
- Speed (higher = more stress)
- Acceleration (higher = more stress)
- Payload (heavier = more stress)
- Movement frequency (more = more stress)

**Reducing stress:**
- Lower speeds
- Lower accelerations
- Smooth profiles
- Appropriate payloads

---

## 5. Trade-off Analysis

### Speed vs. Accuracy

**Relationship:** Higher speed typically reduces accuracy

**Why:**
- Less time to reach exact position
- Momentum effects
- Control system limitations
- Mechanical play more noticeable

**Trade-off:**
- **High speed:** Fast but less accurate
- **Low speed:** Slow but more accurate
- **Medium speed:** Balance

**Application considerations:**
- Need accuracy? → Lower speed
- Need speed? → Accept lower accuracy
- Find balance for your application

### Speed vs. Wear

**Relationship:** Higher speed increases wear

**Why:**
- More movement cycles
- Higher forces
- More friction
- More stress

**Trade-off:**
- **High speed:** Fast but more wear
- **Low speed:** Slow but less wear
- **Medium speed:** Balance

**Application considerations:**
- Long-term use? → Lower speed
- Short-term use? → Can use higher speed
- Maintenance schedule? → Consider wear

### Accuracy vs. Wear

**Relationship:** More accuracy may require slower speeds (less wear)

**Why:**
- Accurate positioning takes time
- Slower = less stress = less wear
- But may need more cycles for accuracy

**Trade-off:**
- **High accuracy:** May require slower speeds
- **Lower accuracy:** Can use higher speeds
- **Balance:** Acceptable accuracy with reasonable wear

### Finding Optimal Settings

**Process:**

1. **Define requirements**
   - What accuracy is needed?
   - What speed is needed?
   - What lifespan is expected?

2. **Test different settings**
   - Try various speeds
   - Measure accuracy
   - Monitor wear

3. **Analyze results**
   - Which settings meet requirements?
   - What are the trade-offs?
   - What's the best balance?

4. **Choose optimal settings**
   - Balance all factors
   - Meet requirements
   - Optimize for application

### Application-Specific Considerations

**Different applications need different settings:**

**Precision assembly:**
- Priority: Accuracy
- Speed: Lower acceptable
- Wear: Less important

**High-speed pick-and-place:**
- Priority: Speed
- Accuracy: Acceptable range
- Wear: Monitor closely

**Long-term operation:**
- Priority: Wear reduction
- Speed: Moderate
- Accuracy: Adequate

---

## 6. Summary

### Key Concepts

1. **Software architecture** has three layers: Front-end, Server, Communication
2. **Control logic** can be extended and modified
3. **Motion control** has two modes: Joint-space and Cartesian-space
4. **Motion profiles** affect smoothness and stress
5. **Trade-offs** exist between speed, accuracy, and wear
6. **Optimal settings** depend on application requirements

### Understanding Software Architecture

You should now understand:
- How software components work together
- How to extend control logic
- Different motion control modes
- How motion profiles affect performance
- How to evaluate trade-offs
- How to find optimal settings

### Next Steps

After learning this theory, you will:
- Understand software architecture
- Modify control logic
- Implement coordinated moves
- Analyze motion profiles
- Evaluate trade-offs
- Complete the Module 9 worksheet

---

## Questions for Review

Before moving to the worksheet, make sure you can answer:

1. What are the three layers of software architecture?
2. How do components interact?
3. What is the difference between joint-space and Cartesian-space control?
4. How do speed settings affect movement?
5. How do acceleration settings affect movement?
6. What are the trade-offs between speed and accuracy?
7. How do you find optimal settings?

---

**Remember:** Understanding software architecture helps you modify and optimize the system. Consider trade-offs carefully for your specific application!
