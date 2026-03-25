# Module 3: Basic Electrical & Electronic Systems
## Theory & Teaching Notes

**Module Duration:** 2-3 hours  
**Level:** Foundation (Level 3)

---

## Learning Objectives

By the end of this module, students will be able to:
- Identify key electrical components in robot arm systems
- Understand power and data connections
- Distinguish between open-loop and closed-loop control
- Understand servo feedback
- Interpret simple wiring diagrams

---

## 1. Electrical Components Overview

### The Robot Arm Electrical System

A robot arm needs electricity to work. The electrical system includes:

1. **Power Supply** - Provides electricity
2. **Controller** (Raspberry Pi) - The brain
3. **Communication Bus** - How components talk
4. **Servos** - The muscles that move joints

---

## 2. Power Supply

### What is a Power Supply?

A **power supply** converts electricity from the wall outlet (AC) into the type of electricity the robot needs (DC).

### Power Supply Specifications

**Voltage:** Usually 5V, 12V, or 24V DC
- **What it means:** The "pressure" of electricity
- **Why it matters:** Components need the right voltage

**Current:** Measured in Amperes (A) or milliamperes (mA)
- **What it means:** The "flow" of electricity
- **Why it matters:** Must provide enough current for all components

**Power:** Voltage × Current = Power (Watts)
- **Example:** 12V × 5A = 60W

### Why Stable Power Matters

**Unstable power causes:**
- Servos to move erratically
- Controller to reset
- Communication errors
- Damage to components

**Good power supply:**
- Provides steady voltage
- Handles load changes
- Has safety features
- Reliable

---

## 3. Raspberry Pi (Controller)

### What is a Raspberry Pi?

**Raspberry Pi** is a small, affordable computer that acts as the "brain" of the robot arm.

### What It Does

1. **Runs software** - Controls the robot
2. **Processes commands** - Understands what you want
3. **Communicates** - Talks to servos and user interface
4. **Monitors** - Watches feedback from servos

### Why Raspberry Pi?

- **Small** - Fits in the robot
- **Affordable** - Low cost
- **Powerful** - Can run complex software
- **Flexible** - Can be programmed

---

## 4. Serial Bus Communication

### What is Serial Communication?

**Serial communication** sends data one bit at a time, one after another (like a line of people).

**Why serial?**
- Simple wiring
- Reliable
- Works over distances
- Standard method

### Baud Rate

**Baud rate** = Speed of data transmission (bits per second)

**Common rates:** 9600, 115200, etc.

**Higher baud rate = faster communication**
- But needs better wiring
- More sensitive to interference

### Daisy-Chain Wiring

**Daisy-chain** = Components connected in a line, one after another

**How it works:**
```
Power Supply → Servo 1 → Servo 2 → Servo 3 → Servo 4
```

**Advantages:**
- Simple wiring
- Fewer connections
- Easier to add/remove servos

**Disadvantages:**
- If one servo fails, others might be affected
- Need to address each servo individually

---

## 5. ST3215 Servos

### What are Servos?

**Servos** are motors with built-in control that can move to specific positions.

### ST3215 Servo Features

- **Position control** - Move to exact angles
- **Feedback** - Report their position
- **Load sensing** - Know how much force they're using
- **Temperature monitoring** - Know if they're overheating

### How Servos Work

1. **Receive command** - "Move to 45°"
2. **Move motor** - Rotates toward position
3. **Check position** - Uses sensor to see where it is
4. **Adjust** - Moves until at correct position
5. **Report** - Sends back current position

---

## 6. Wiring Basics

### Power Connections

**Three main types:**

1. **Positive (VCC, +)**
   - The "supply" wire
   - Usually red
   - Carries power TO components

2. **Negative (VSS, -)**
   - The "return" wire
   - Usually black
   - Completes the circuit

3. **Ground (GND)**
   - Safety reference point
   - Usually green or bare
   - Prevents electrical shock

**Important:** 
- Positive and negative complete the power circuit
- Ground is for safety
- Don't confuse them!

### Data Connections

**Data wires** carry information (not power):
- Usually separate from power wires
- Can be twisted pairs (reduces interference)
- Connect in daisy-chain

### Reading Wiring Diagrams

**Wiring diagrams** show:
- What connects to what
- Wire colors
- Connection points
- Power flow

**Basic symbols:**
- Lines = wires
- Circles = connection points
- Labels = component names
- Colors = wire colors

---

## 7. Control Concepts

### Open-Loop Control

**Open-loop** = Control without checking if it worked

**How it works:**
1. Send command: "Move to 45°"
2. Hope it gets there
3. Don't check if it actually did

**Example:** Telling someone to turn left, but not checking if they did.

**Characteristics:**
- Simple
- Fast
- But... no feedback
- Can be inaccurate

**In robot arm:** From app's perspective, you send a command and hope the servo receives it.

### Closed-Loop Control

**Closed-loop** = Control WITH checking and adjusting

**How it works:**
1. Send command: "Move to 45°"
2. Check actual position
3. Compare to desired position
4. Adjust if needed
5. Repeat until correct

**Example:** Telling someone to turn left, watching them, and correcting if they don't turn enough.

**Characteristics:**
- More complex
- Slower
- But... accurate
- Self-correcting

**In robot arm:** Inside the servo, it continuously checks position and adjusts.

### Both Types in Robot Arm

**The system uses BOTH:**

1. **App → Servo** = Open-loop (command sent, hope it arrives)
2. **Inside Servo** = Closed-loop (servo checks and adjusts position)

**Why this design?**
- App doesn't need to constantly check (simpler)
- Servo handles accuracy internally (better control)

---

## 8. Servo Feedback

### What is Feedback?

**Feedback** = Information about what's actually happening

**Why it matters:** You can't control what you can't measure.

### Types of Feedback from ST3215 Servos

#### 1. Present Position
- **What it is:** Current angle of the servo
- **Why useful:** Know where servo actually is
- **Example:** "Servo is at 42.5°"

#### 2. Speed
- **What it is:** How fast servo is moving
- **Why useful:** Monitor movement
- **Example:** "Moving at 50°/second"

#### 3. Load
- **What it is:** How much force/torque servo is using
- **Why useful:** Detect overload, know if stuck
- **Example:** "Using 80% of maximum torque"

#### 4. Voltage
- **What it is:** Electrical power level
- **Why useful:** Detect power problems
- **Example:** "Receiving 11.8V" (should be 12V)

#### 5. Temperature
- **What it is:** How hot the servo is
- **Why useful:** Prevent overheating damage
- **Example:** "Temperature is 45°C"

#### 6. Moving Flag
- **What it is:** Is servo currently moving? (Yes/No)
- **Why useful:** Know when movement is complete
- **Example:** "Moving = True"

#### 7. Torque On/Off Flag
- **What it is:** Is motor enabled? (Yes/No)
- **Why useful:** Know if servo can move
- **Example:** "Torque = On"

### Using Feedback

**Feedback helps you:**
- Monitor system health
- Detect problems early
- Verify commands worked
- Optimize performance
- Ensure safety

**Example:** If load is very high, something might be stuck or overloaded.

---

## 9. Commanded vs. Actual Position

### Why They Might Differ

**Commanded position** = Where you told it to go  
**Actual position** = Where it actually is

**They might differ because:**

1. **Servo hasn't finished moving yet**
   - Still moving to position
   - Check "moving flag"

2. **Mechanical play (backlash)**
   - Gears have small gaps
   - Causes small errors

3. **Load**
   - Heavy load might prevent reaching exact position
   - Servo might not have enough torque

4. **Wear**
   - Over time, accuracy decreases
   - Needs calibration

**Normal difference:** Usually very small (< 1°)

**Large difference:** Indicates a problem

---

## 10. Practical Understanding

### Identifying Components

When looking at a robot arm system, you should identify:

1. **Power supply**
   - Where is it?
   - What voltage/current?
   - Is it connected properly?

2. **Raspberry Pi**
   - Where is it?
   - Is it powered?
   - Is it communicating?

3. **Serial bus**
   - How are servos connected?
   - Is wiring secure?
   - Any damage?

4. **Servos**
   - How many?
   - Are they all connected?
   - Are they receiving power?

### Reading Feedback

You should be able to:
- Read position feedback
- Monitor load
- Check voltage
- Watch temperature
- Use flags to understand state

---

## 11. Summary

### Key Concepts

1. **Power supply** provides electricity to the system
2. **Raspberry Pi** is the controller/brain
3. **Serial bus** allows communication
4. **Servos** are the motors that move joints
5. **Open-loop** = command without checking
6. **Closed-loop** = command with checking and adjusting
7. **Feedback** tells you what's actually happening
8. **Wiring** connects everything together

### Understanding Your System

You should now understand:
- What each electrical component does
- How power flows
- How data flows
- How control works
- What feedback tells you

### Next Steps

After learning this theory, you will:
- Identify components in your system
- Trace wiring connections
- Read servo feedback
- Compare commanded vs. actual positions
- Complete the Module 3 worksheet

---

## Questions for Review

Before moving to the worksheet, make sure you can answer:

1. What does the power supply do?
2. What is the Raspberry Pi's role?
3. What is serial communication?
4. What is the difference between open-loop and closed-loop control?
5. What feedback does a servo provide?
6. Why might commanded and actual positions differ?
7. What is daisy-chain wiring?

---

**Remember:** Understanding the electrical system helps you troubleshoot problems and operate the robot safely. Always be careful with electricity!
