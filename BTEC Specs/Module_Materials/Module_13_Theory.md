# Module 13: Safety Standards & Design for Safety
## Theory & Teaching Notes

**Module Duration:** 2-3 hours  
**Level:** Advanced (Level 4/5)

---

## Learning Objectives

By the end of this module, students will be able to:
- Understand safety standards for robots
- Map safety requirements to educational systems
- Justify safety features in design
- Design fail-safe behaviors
- Understand safety implications of failures

---

## 1. Safety Standards Overview

### Why Standards Exist

**Safety standards** exist to:
- Protect people
- Prevent accidents
- Ensure consistent safety
- Provide guidelines
- Legal requirements

**Think of it like:** Rules that ensure safety is taken seriously and implemented correctly.

### ISO 10218

**What it is:** International standard for industrial robot safety

**Covers:**
- Robot design requirements
- Safety requirements
- Installation requirements
- Operation requirements
- Maintenance requirements

**Key principles:**
- Risk assessment
- Safety measures
- Verification
- Documentation

### Other Relevant Standards

**Examples:**
- Machinery Directive (Europe)
- OSHA standards (USA)
- National standards
- Industry-specific standards

**Why multiple standards:**
- Different regions
- Different industries
- Different applications
- All aim for safety

### Applicability to Educational Systems

**Educational robots:**
- May not need full industrial compliance
- But should follow principles
- Teach proper practices
- Prepare for industry

**Key point:** Understand standards even if not fully compliant

---

## 2. Safety Requirements Mapping

### Safe Stops

**What they are:** Controlled stops that maintain safety

**Types:**

1. **Category 0 Stop**
   - Immediate removal of power
   - Uncontrolled stop
   - Emergency situations

2. **Category 1 Stop**
   - Controlled stop, then power removal
   - Maintains control during stop
   - Safer than Category 0

3. **Category 2 Stop**
   - Controlled stop, power maintained
   - Can resume operation
   - Normal stops

**Mapping to educational robot:**
- Emergency stop = Category 0
- Normal stop = Category 2
- Software stop = Category 2

### Emergency Stops

**What they are:** Immediate stops in dangerous situations

**Requirements:**
- Easily accessible
- Clearly marked
- Immediately effective
- Cannot be bypassed

**Implementation:**
- Physical button
- Software function
- Both (redundant)

**Mapping to educational robot:**
- Emergency stop button
- Software emergency stop
- Both should work

### Guarding

**What it is:** Physical barriers preventing access

**Requirements:**
- Proper height
- Strong enough
- Secure mounting
- Interlocked (stops robot when opened)

**Types:**
- Fixed guards
- Interlocked guards
- Light curtains
- Pressure mats

**Mapping to educational robot:**
- Physical barriers
- Interlocked gates
- Clear working envelope
- Supervision

### Modes of Operation

**Types:**

1. **Teaching Mode**
   - Slow speed
   - Direct control
   - Operator present
   - Higher risk

2. **Automatic Mode**
   - Normal speed
   - Programmed operation
   - Operator may not be present
   - Requires guarding

**Mapping to educational robot:**
- Manual control = Teaching mode
- Sequence execution = Automatic mode
- Different safety requirements

---

## 3. Safety Features Analysis

### Joint Limits

**What they are:** Software limits on joint movement

**Why a safety feature:**
- Prevents unsafe positions
- Prevents collisions
- Prevents damage
- Protects mechanisms

**Justification:**
- Prevents accidents
- Protects equipment
- Ensures safe operation
- Required by standards

**Implementation:**
- Software checks
- Hardware limits (backup)
- Both (redundant)

### Speed and Acceleration Caps

**What they are:** Maximum allowed speed and acceleration

**Why a safety feature:**
- Prevents dangerous speeds
- Reduces impact force
- Improves control
- Protects operators

**Justification:**
- Limits impact energy
- Improves reaction time
- Reduces injury severity
- Required for safety

**Implementation:**
- Software limits
- Cannot be exceeded
- Default safe values
- Can be adjusted (with care)

### Torque On/Off Control

**What it is:** Ability to enable/disable motor torque

**Why a safety feature:**
- Prevents accidental movement
- Allows safe manual positioning
- Enables safe setup
- Required for maintenance

**Justification:**
- Essential for safety
- Prevents unexpected movement
- Enables safe operation
- Standard requirement

**Implementation:**
- Software control
- Physical switch (backup)
- Clear indication
- Easy to use

### Conservative Defaults

**What they are:** Safe default settings

**Examples:**
- Low speed defaults
- Low acceleration defaults
- Torque disabled by default
- Safe position defaults

**Why a safety feature:**
- Safe if misconfigured
- Prevents accidents
- Protects operators
- Good practice

**Justification:**
- Fail-safe principle
- Prevents accidents
- Protects users
- Industry best practice

---

## 4. Failure Mode Analysis

### Loss of Communication

**What happens:** Robot loses communication with controller

**Risks:**
- Robot may continue last command
- Robot may stop unexpectedly
- No feedback available
- Unpredictable behavior

**Safe behavior:**
- **Stop robot immediately**
- Disable torque
- Alert operator
- Require reset

**Justification:**
- Prevents unpredictable behavior
- Ensures safety
- Fail-safe design
- Standard requirement

### Power Failure

**What happens:** Power is lost

**Risks:**
- Robot may fall (gravity)
- Uncontrolled movement
- Loss of control
- Safety hazard

**Safe behavior:**
- **Brake or hold position** (if possible)
- **Fail to safe state**
- Prevent free fall
- Safe shutdown

**Justification:**
- Prevents uncontrolled movement
- Protects operators
- Prevents damage
- Critical safety requirement

### Sensor Anomalies

**What happens:** Sensors report incorrect or invalid data

**Risks:**
- Incorrect position known
- Wrong decisions made
- Unsafe operation
- Accidents

**Safe behavior:**
- **Detect anomalies**
- **Stop operation**
- **Alert operator**
- **Require verification**

**Justification:**
- Prevents unsafe operation
- Ensures reliability
- Protects operators
- Good practice

### Feedback Anomalies

**What happens:** Feedback data is invalid or missing

**Risks:**
- Don't know actual state
- Can't make good decisions
- Unsafe operation possible

**Safe behavior:**
- **Detect invalid data**
- **Use conservative assumptions**
- **Stop if uncertain**
- **Alert operator**

**Justification:**
- Prevents unsafe assumptions
- Ensures safety
- Fail-safe design
- Good practice

---

## 5. Fail-Safe Design Principles

### What is Fail-Safe?

**Fail-safe** = System fails to safest possible state

**Principle:** When something goes wrong, system becomes safer, not more dangerous.

### Examples

**Good fail-safe:**
- Communication loss → Stop robot
- Power failure → Hold position or safe stop
- Error detected → Stop and alert

**Bad (not fail-safe):**
- Communication loss → Continue last command
- Power failure → Free fall
- Error detected → Ignore and continue

### Designing Fail-Safe Behaviors

**Process:**

1. **Identify failure modes**
   - What can go wrong?
   - List all possibilities

2. **Determine safe state**
   - What is safest state?
   - For each failure mode

3. **Design behavior**
   - How to reach safe state?
   - How to detect failure?
   - How to respond?

4. **Verify**
   - Test failure modes
   - Verify safe behavior
   - Document

### Conservative Approach

**Principle:** When in doubt, choose safer option

**Examples:**
- Unknown state → Assume unsafe
- Uncertain data → Don't trust it
- Error detected → Stop and verify

---

## 6. Safety Justification

### Documenting Safety Features

**Why document:**
- Shows safety was considered
- Explains decisions
- Provides evidence
- Required by standards

**What to document:**
- Safety features implemented
- Why each feature exists
- How it improves safety
- How it meets requirements

### Risk Reduction Analysis

**Process:**

1. **Identify hazards**
   - What are the dangers?
   - List all hazards

2. **Assess risk**
   - How likely?
   - How severe?
   - Risk level?

3. **Implement controls**
   - How to reduce risk?
   - Safety features
   - Procedures

4. **Verify reduction**
   - Risk reduced?
   - Acceptable level?
   - Document

### Compliance Considerations

**Standards compliance:**
- Which standards apply?
- How does design meet them?
- What evidence exists?
- Documentation needed?

**For educational systems:**
- Understand standards
- Apply principles
- Document approach
- Prepare for industry

---

## 7. Summary

### Key Concepts

1. **Safety standards** provide guidelines for safe design
2. **Safety requirements** include stops, guards, modes
3. **Safety features** justify design choices
4. **Failure modes** must be analyzed
5. **Fail-safe design** ensures safe failure
6. **Safety justification** documents decisions
7. **Risk reduction** is the goal

### Understanding Safety Design

You should now understand:
- Why safety standards exist
- What safety requirements are
- How to justify safety features
- How to design fail-safe behaviors
- How to document safety

### Next Steps

After learning this theory, you will:
- Understand safety standards
- Map requirements to systems
- Analyze safety features
- Design fail-safe behaviors
- Justify safety decisions
- Complete the Module 13 worksheet

---

## Questions for Review

Before moving to the worksheet, make sure you can answer:

1. Why do safety standards exist?
2. What is ISO 10218?
3. What are the types of safe stops?
4. Why are joint limits a safety feature?
5. What is fail-safe design?
6. What should happen on communication loss?
7. How do you justify safety features?

---

**Remember:** Safety is not optional. Design for safety from the start, and always consider what happens when things go wrong!
