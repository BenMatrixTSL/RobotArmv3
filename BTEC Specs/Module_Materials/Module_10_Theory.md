# Module 10: Data Logging & Diagnostics
## Theory & Teaching Notes

**Module Duration:** 2-3 hours  
**Level:** Advanced (Level 4/5)

---

## Learning Objectives

By the end of this module, students will be able to:
- Understand why data logging is important
- Set up data logging systems
- Capture system data over time
- Analyze log files to diagnose issues
- Use logs to optimize system parameters
- Identify common problems from logs

---

## 1. Data Logging Basics

### Why Log Data?

**Data logging** = Recording system information over time

**Why it's important:**

1. **Diagnosis**
   - Understand what happened
   - Identify problems
   - Find root causes

2. **Optimization**
   - See system behavior
   - Identify bottlenecks
   - Improve performance

3. **Documentation**
   - Record system operation
   - Track changes over time
   - Provide evidence

4. **Prevention**
   - Detect trends
   - Predict problems
   - Prevent failures

### What Data to Capture

**Essential data:**

1. **Joint angles**
   - Current position of each joint
   - Over time
   - Shows movement patterns

2. **Load readings**
   - Torque/load on each joint
   - Over time
   - Shows stress patterns

3. **Voltages**
   - Power supply voltage
   - Over time
   - Shows power stability

4. **Errors**
   - When errors occur
   - What type of error
   - Error frequency

5. **Timestamps**
   - When each event occurred
   - Allows correlation
   - Essential for analysis

**Additional useful data:**
- Speed readings
- Temperature readings
- Communication status
- Command history

### Logging Frequency Considerations

**How often to log?**

**Factors to consider:**

1. **System activity**
   - Active movement? → Log more frequently
   - Idle? → Log less frequently

2. **Storage capacity**
   - More frequent = More data = More storage needed
   - Balance frequency with storage

3. **Analysis needs**
   - Need detailed data? → Log frequently
   - General trends? → Log less frequently

**Typical frequencies:**
- **High frequency:** 10-50ms (detailed analysis)
- **Medium frequency:** 100-500ms (general monitoring)
- **Low frequency:** 1-5 seconds (trends only)

### Storage Formats

**Common formats:**

1. **CSV (Comma-Separated Values)**
   - Simple text format
   - Easy to read
   - Works with Excel
   - Good for beginners

2. **JSON (JavaScript Object Notation)**
   - Structured format
   - Easy to parse programmatically
   - Good for software analysis

3. **Binary formats**
   - Compact
   - Fast to write/read
   - Requires special tools

**Choose based on:**
- How you'll analyze data
- What tools you have
- Storage requirements

---

## 2. Capturing System Data

### Joint Angles Over Time

**What to capture:**
- Angle of each joint
- At regular intervals
- With timestamps

**Why useful:**
- See movement patterns
- Identify unusual movements
- Verify sequences
- Debug positioning

**Example log:**
```
Time, Joint1, Joint2, Joint3, Joint4
0.000, 0.0, 0.0, 0.0, 0.0
0.100, 5.2, 2.1, -1.5, 0.0
0.200, 10.5, 4.3, -3.0, 0.0
```

### Load Readings Over Time

**What to capture:**
- Load percentage for each joint
- At regular intervals
- With timestamps

**Why useful:**
- Identify overload conditions
- See stress patterns
- Optimize movements
- Prevent damage

**Example log:**
```
Time, ShoulderLoad, ElbowLoad, WristLoad
0.000, 25, 15, 10
0.100, 45, 30, 12
0.200, 60, 45, 15
```

### Voltage Readings Over Time

**What to capture:**
- Power supply voltage
- At regular intervals
- With timestamps

**Why useful:**
- Detect power problems
- Identify voltage drops
- Monitor power stability
- Prevent failures

**Example log:**
```
Time, Voltage
0.000, 12.1
0.100, 12.0
0.200, 11.9
0.300, 11.8  <- Voltage drop detected!
```

### Error Events

**What to capture:**
- When error occurred
- Error type
- Error details
- System state at error

**Why useful:**
- Understand failures
- Identify patterns
- Debug problems
- Improve reliability

**Example log:**
```
Time, ErrorType, Details
1.250, TIMEOUT, Servo 2 did not respond
2.500, OVERLOAD, Shoulder load exceeded 90%
3.750, INVALID_POSITION, Commanded position out of range
```

### Timestamping Data

**Why timestamps matter:**
- Correlate events
- Understand timing
- See sequences
- Analyze patterns

**Format:**
- Absolute time (e.g., 2024-01-15 14:30:25.123)
- Relative time (e.g., 0.000, 0.100, 0.200)
- Both (absolute + relative)

---

## 3. Diagnostic Techniques

### Analyzing Log Files

**Steps to analyze:**

1. **Load the log file**
   - Open in appropriate tool
   - Verify data loaded correctly
   - Check format

2. **Visualize the data**
   - Create graphs
   - Plot over time
   - Compare different signals

3. **Look for patterns**
   - Trends
   - Cycles
   - Anomalies
   - Correlations

4. **Identify issues**
   - Unusual values
   - Errors
   - Problems
   - Failures

### Identifying Patterns

**Common patterns:**

1. **Cyclic patterns**
   - Regular repetition
   - Indicates normal operation
   - Example: Pick-and-place cycle

2. **Trends**
   - Gradual changes
   - May indicate problems
   - Example: Gradual voltage drop

3. **Spikes**
   - Sudden changes
   - May indicate problems
   - Example: Sudden load spike

4. **Correlations**
   - Related changes
   - Help understand causes
   - Example: Load increases when speed increases

### Finding Anomalies

**What to look for:**

1. **Unexpected values**
   - Values outside normal range
   - Impossible values
   - Invalid data

2. **Missing data**
   - Gaps in logging
   - Missing timestamps
   - Incomplete records

3. **Errors**
   - Error messages
   - Failed operations
   - Timeouts

4. **Unusual patterns**
   - Different from normal
   - Unexpected behavior
   - Problems

### Correlating Events

**Why correlation matters:**
- Understand relationships
- Find root causes
- See sequences of events

**How to correlate:**
- Look at timestamps
- Compare different signals
- See what happened before/after
- Identify sequences

**Example:**
- Voltage drops → Servo errors → System stops
- High load → Temperature increases → Overheat protection

---

## 4. Common Issues

### Serial Timeouts

**What it is:** Communication takes too long, timeout occurs

**Causes:**
- Slow serial speed
- Busy system
- Servo problems
- Wiring issues

**Detection in logs:**
- Timeout errors
- Missing responses
- Delayed responses

**Solutions:**
- Increase timeout (if valid but slow)
- Check wiring
- Reduce system load
- Increase serial speed (if wiring allows)

### Invalid Position Data

**What it is:** Position values that don't make sense

**Causes:**
- Communication errors
- Sensor problems
- Servo malfunction
- Data corruption

**Detection in logs:**
- Positions outside valid range
- Sudden jumps in position
- Impossible values

**Solutions:**
- Check communication
- Verify sensors
- Check servo health
- Add validation

### Power/Voltage Dips

**What it is:** Voltage drops below normal

**Causes:**
- Power supply problems
- High current draw
- Poor connections
- Insufficient power supply

**Detection in logs:**
- Voltage readings below threshold
- Sudden drops
- Gradual decreases

**Solutions:**
- Check power supply
- Reduce load
- Check connections
- Upgrade power supply if needed

### Overload Conditions

**What it is:** Load exceeds safe limits

**Causes:**
- Too heavy payload
- Extended reach
- Obstructions
- Servo problems

**Detection in logs:**
- Load readings above threshold
- Sustained high loads
- Load spikes

**Solutions:**
- Reduce payload
- Reduce reach
- Check for obstructions
- Verify servo operation

---

## 5. System Optimization

### Using Logs to Optimize Polling Intervals

**Process:**

1. **Log current polling interval**
   - Record what interval you're using
   - Log response times
   - Log system load

2. **Try different intervals**
   - Test faster polling
   - Test slower polling
   - Measure results

3. **Analyze logs**
   - Compare response times
   - Compare system load
   - Find optimal balance

4. **Choose optimal interval**
   - Balance responsiveness and load
   - Meet requirements
   - Optimize performance

**Example:**
- Current: 50ms polling
- Test: 30ms (faster) → Better responsiveness but higher load
- Test: 100ms (slower) → Lower load but slower response
- Optimal: 40ms (balance)

### Adjusting Timeouts Based on Data

**Process:**

1. **Log timeout occurrences**
   - When do timeouts occur?
   - What's typical response time?
   - What's worst-case response time?

2. **Analyze data**
   - Typical response: 20ms
   - Worst case: 80ms
   - Current timeout: 50ms (too short!)

3. **Adjust timeout**
   - Set to 120ms (gives margin)
   - Test and verify
   - Monitor results

### Improving Reliability

**Using logs to improve reliability:**

1. **Identify failure patterns**
   - When do failures occur?
   - What causes them?
   - Can they be prevented?

2. **Make improvements**
   - Fix root causes
   - Add safeguards
   - Improve error handling

3. **Verify improvements**
   - Log after changes
   - Compare before/after
   - Confirm improvement

---

## 6. Summary

### Key Concepts

1. **Data logging** records system information over time
2. **Essential data** includes positions, loads, voltages, errors
3. **Logging frequency** balances detail with storage
4. **Log analysis** helps diagnose problems
5. **Common issues** include timeouts, invalid data, voltage dips, overloads
6. **Logs help optimize** polling intervals, timeouts, and reliability

### Understanding Data Logging

You should now understand:
- Why logging is important
- What data to capture
- How to analyze logs
- How to identify problems
- How to optimize using logs

### Next Steps

After learning this theory, you will:
- Set up data logging
- Capture system data
- Analyze log files
- Diagnose problems
- Optimize parameters
- Complete the Module 10 worksheet

---

## Questions for Review

Before moving to the worksheet, make sure you can answer:

1. Why is data logging important?
2. What data should you capture?
3. How do you choose logging frequency?
4. How do you analyze log files?
5. What are common issues to look for?
6. How do logs help optimize the system?
7. How do you correlate events in logs?

---

**Remember:** Data logging is your window into system behavior. Use it to understand, diagnose, and optimize your robot arm system!
