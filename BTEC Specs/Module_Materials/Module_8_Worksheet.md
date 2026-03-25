# Module 8: Advanced Control & Communication
## Student Worksheet

**Name:** _________________________  
**Date:** _________________________  
**Group/Class:** _________________________

---

## Part 1: Serial Communication Protocol

### Question 1: Protocol Basics
Answer the following:

a) What is serial communication?
   _________________________________________________________________

b) What is baud rate?
   _________________________________________________________________

c) What baud rate does your system use? _______

### Question 2: Packet Structure
Draw and label a serial packet structure showing:
- Header
- ID
- Length
- Instruction
- Parameters
- Checksum

[Space for diagram]

### Question 3: Error Detection
How does the checksum work for error detection?

_________________________________________________________________
_________________________________________________________________

---

## Part 2: System Latency

### Question 4: Latency Sources
Identify sources of latency in the system:

| Source | Contribution to Latency | How to Reduce? |
|--------|------------------------|----------------|
| Serial speed | | |
| Status polling | | |
| Timeouts | | |
| Command queueing | | |

### Question 5: Measuring Latency
Measure the latency in your system:

**Test:** Send command and measure time to response  
**Result:** _______ milliseconds

**Is this acceptable?** Yes / No  
**Why?** _________________________________________________________________

---

## Part 3: Design Decisions

### Question 6: Timeout Values
Answer the following:

a) What timeout values are used in your system?
   _________________________________________________________________

b) Why were these values chosen?
   _________________________________________________________________

c) What happens if timeout is too short?
   _________________________________________________________________

d) What happens if timeout is too long?
   _________________________________________________________________

### Question 7: Polling Intervals
Answer the following:

a) How often is status polled?
   _________________________________________________________________

b) What are the trade-offs of different polling intervals?
   _________________________________________________________________
   _________________________________________________________________

---

## Part 4: Advanced Monitoring

### Question 8: Real-Time Feedback
Monitor all feedback channels simultaneously for 30 seconds:

| Time | Position | Speed | Load | Voltage | Temperature | Moving? |
|------|----------|-------|------|---------|-------------|---------|
| 0s | | | | | | |
| 5s | | | | | | |
| 10s | | | | | | |
| 15s | | | | | | |
| 30s | | | | | | |

**Observations:** _________________________________________________________________

### Question 9: Abnormal Condition Detection
Design detection logic for abnormal conditions:

**Under-voltage Detection:**
- Threshold: _______ V
- Action: _________________________________________________________________

**Over-temperature Detection:**
- Threshold: _______ °C
- Action: _________________________________________________________________

**Overload Detection:**
- Threshold: _______ %
- Action: _________________________________________________________________

---

## Part 5: Safety Flags

### Question 10: Using Flags
Explain how to use moving and torque flags:

**Moving Flag:**
- What it indicates: _________________________________________________________________
- How to use it: _________________________________________________________________

**Torque Flag:**
- What it indicates: _________________________________________________________________
- How to use it: _________________________________________________________________

### Question 11: State Reasoning
How can flags help reason about servo state and safety?

_________________________________________________________________
_________________________________________________________________
_________________________________________________________________

---

## Part 6: Practical Exercise

### Exercise 1: Protocol Analysis
Analyze a serial packet:

**Packet:** _________________________________________________________________  
**Breakdown:**
- Header: _______
- ID: _______
- Length: _______
- Instruction: _______
- Parameters: _______
- Checksum: _______

**Is checksum correct?** Yes / No

### Exercise 2: Monitoring Dashboard
Create a simple monitoring dashboard showing:
- All feedback values
- Status flags
- Warning indicators

[Space for dashboard design]

---

## Part 7: Reflection

### Question 12: Understanding Communication
Answer the following:

a) Why is understanding the communication protocol important?
   _________________________________________________________________
   _________________________________________________________________

b) How does latency affect robot operation?
   _________________________________________________________________
   _________________________________________________________________

---

## Self-Assessment Checklist

- [ ] Understood serial protocol
- [ ] Identified latency sources
- [ ] Understood design decisions
- [ ] Monitored feedback channels
- [ ] Designed detection logic
- [ ] Used safety flags
- [ ] Analyzed protocol packets
- [ ] Written reflection

**Student Signature:** _________________________  
**Date:** _________________________  
**Instructor Signature:** _________________________  
**Date:** _________________________
