# Module 7: Forward Kinematics & Coordinate Systems
## Theory & Teaching Notes

**Module Duration:** 3-4 hours  
**Level:** Advanced (Level 4/5)

---

## Learning Objectives

By the end of this module, students will be able to:
- Understand forward kinematics concepts
- Work with coordinate systems and frames
- Calculate end effector position from joint angles
- Understand transformation matrices
- Identify joint limits and singularities
- Map the reachable workspace

---

## 1. Introduction to Kinematics

### What is Kinematics?

**Kinematics** = The study of motion without considering forces

**In robotics:** How joint movements relate to end effector position

### Forward vs. Inverse Kinematics

**Forward Kinematics:**
- **Input:** Joint angles
- **Output:** End effector position (X, Y, Z)
- **Question:** "Where is the end effector?"

**Inverse Kinematics:**
- **Input:** Desired end effector position (X, Y, Z)
- **Output:** Joint angles needed
- **Question:** "What angles get me there?"

**This module focuses on FORWARD kinematics.**

---

## 2. Coordinate Systems

### Cartesian Coordinates

**Cartesian coordinates** describe position using X, Y, Z values.

**Think of it like:** A 3D grid system

**Example:**
- X = 100mm (left/right)
- Y = 200mm (forward/back)
- Z = 150mm (up/down)

**Position:** (100, 200, 150)

### Joint Space vs. Cartesian Space

**Joint Space:**
- Describes position using joint angles
- Example: Joint 1 = 45°, Joint 2 = 30°, Joint 3 = -20°
- What the robot "thinks" in
- Direct control

**Cartesian Space:**
- Describes position using X, Y, Z coordinates
- Example: X = 100mm, Y = 200mm, Z = 150mm
- What humans "think" in
- More intuitive

**Conversion:** Forward kinematics converts joint space → Cartesian space

### Coordinate Frames

**Coordinate frame** = A reference system for position and orientation

**Each link has its own frame:**
- Base frame (fixed)
- Link 1 frame
- Link 2 frame
- End effector frame

**Why frames matter:**
- Each frame moves with its link
- Positions relative to frames
- Transformations between frames

---

## 3. Forward Kinematics

### Basic Concept

**Forward kinematics** calculates end effector position from joint angles.

**Simple 2D example:**

If you have:
- Link length = 100mm
- Joint angle = 45°

Then:
- X position = 100 × cos(45°) = 70.7mm
- Y position = 100 × sin(45°) = 70.7mm

**For 3D robot arms:** More complex, but same idea.

### The Process

**Step 1:** Start with joint angles
- Joint 1 = θ₁
- Joint 2 = θ₂
- Joint 3 = θ₃
- etc.

**Step 2:** Calculate each link's position
- Use trigonometry
- Consider link lengths
- Account for previous links

**Step 3:** Combine to get end effector position
- Add up all contributions
- Final X, Y, Z position

### Why It Matters

**Forward kinematics helps you:**
- Know where end effector is
- Plan movements
- Verify positions
- Understand workspace

---

## 4. Transformation Matrices

### What is a Transformation Matrix?

A **transformation matrix** is a mathematical tool that describes:
- Position (where)
- Orientation (which way)

**Think of it like:** A way to describe "move X units, rotate Y degrees"

### Homogeneous Transformation Matrices

**Homogeneous** = Includes both position and orientation

**Format:** 4×4 matrix

**What it contains:**
- Rotation information (3×3)
- Position information (3×1)
- Scaling (usually 1)

### Combining Transformations

**Key idea:** Transformations can be combined (multiplied)

**Example:**
- Transform 1: Move from base to joint 1
- Transform 2: Move from joint 1 to joint 2
- Transform 3: Move from joint 2 to end effector

**Combined:** Transform 1 × Transform 2 × Transform 3 = Final position

**Why useful:** Can calculate complex positions step by step

---

## 5. Joint Limits

### What are Joint Limits?

**Joint limits** = Minimum and maximum angles a joint can reach

**Example:**
- Joint 1: -180° to +180°
- Joint 2: -90° to +90°
- Joint 3: -120° to +120°

### Why Limits Exist

**Physical limits:**
- Mechanism design
- Prevents damage
- Avoids collisions
- Safety

**Software limits:**
- Additional safety
- Prevents unsafe positions
- Protects equipment

### How Limits Affect Workspace

**Joint limits define:**
- Where robot can reach
- Where robot cannot reach
- Boundaries of workspace

**Example:** If joint 2 can only go -90° to +90°, robot can't reach behind itself.

---

## 6. Singularities

### What is a Singularity?

A **singularity** is a configuration where the robot loses a degree of freedom.

**Think of it like:** A position where the robot "can't decide" which way to move.

### Common Singularity Types

#### 1. Wrist Singularity
- When wrist joints align
- Robot loses ability to rotate end effector
- Common in 6-DOF arms

#### 2. Elbow Singularity
- When elbow is fully extended or retracted
- Limited movement options
- Can cause rapid joint movement

#### 3. Shoulder Singularity
- When arm is fully extended
- Limited movement options
- High load on joints

### Why Singularities Matter

**Problems:**
- Can't move in certain directions
- May require rapid joint movement
- Can cause instability
- Difficult to control

**Solutions:**
- Avoid singularity configurations
- Plan paths around them
- Use different postures
- Software can detect and avoid

---

## 7. Workspace Mapping

### What is Workspace?

**Workspace** = All positions the robot can reach

**Types:**

1. **Reachable Workspace**
   - All positions robot can reach
   - May have "holes" (unreachable areas)

2. **Dexterous Workspace**
   - Positions robot can reach with any orientation
   - Smaller than reachable workspace

### What Limits Workspace?

**Factors:**
- Joint limits
- Link lengths
- Singularities
- Collisions
- Payload limits

### Mapping the Workspace

**How to map:**

1. **Test reach limits**
   - Move to maximum in each direction
   - Document positions
   - Map boundaries

2. **Identify dead zones**
   - Areas robot can't reach
   - Document why
   - Understand limitations

3. **Create visualization**
   - 2D or 3D map
   - Show reachable areas
   - Mark boundaries

---

## 8. Practical Applications

### Using Forward Kinematics

**When you need it:**
- Verifying positions
- Planning movements
- Understanding workspace
- Debugging problems

**How to use:**
- Calculate expected position
- Compare to actual position
- Identify discrepancies
- Understand why

### Software Tools

**Many software tools can:**
- Calculate forward kinematics automatically
- Visualize workspace
- Show joint limits
- Identify singularities

**Use these tools to:**
- Understand your robot
- Plan movements
- Verify calculations
- Learn concepts

---

## 9. Summary

### Key Concepts

1. **Forward kinematics** calculates position from joint angles
2. **Coordinate systems** describe position (joint space vs. Cartesian)
3. **Transformation matrices** describe position and orientation
4. **Joint limits** restrict where robot can reach
5. **Singularities** are problematic configurations
6. **Workspace** is all reachable positions

### Understanding Kinematics

You should now understand:
- How joint angles relate to position
- Why coordinate systems matter
- How to calculate positions
- What limits workspace
- How to avoid problems

### Next Steps

After learning this theory, you will:
- Calculate forward kinematics
- Work with coordinate systems
- Understand transformation matrices
- Identify joint limits and singularities
- Map workspace
- Complete the Module 7 worksheet

---

## Questions for Review

Before moving to the worksheet, make sure you can answer:

1. What is forward kinematics?
2. What is the difference between joint space and Cartesian space?
3. What is a transformation matrix?
4. What are joint limits?
5. What is a singularity?
6. What is workspace?
7. What limits the workspace?

---

**Remember:** Forward kinematics helps you understand where your robot is and where it can go. Practice calculations and use software tools to visualize concepts!
