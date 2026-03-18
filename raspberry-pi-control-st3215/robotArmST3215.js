/**
 * Robot Arm ST3215 Servo Communication Module
 * 
 * This module provides a simple interface to communicate with ST3215 serial bus
 * servo motors. Each servo is identified by a unique ID (1-6).
 * 
 * The ST3215 servos communicate over serial (UART) at 1Mbps by default.
 * This module acts as the master controller on a Raspberry Pi.
 * 
 * Usage:
 *   const RobotArm = require('./robotArmST3215');
 *   const servo1 = new RobotArm.ServoController(1, '/dev/ttyUSB0', 1);
 *   await servo1.open();
 *   await servo1.moveToAngle(90.0);
 */

const { SerialPort } = require('serialport');

// Debug flag - set to true to enable verbose debug messages
const DEBUG = false;

// ST3215 Protocol Constants (from Python library)
const PKT_HEADER_0 = 0;
const PKT_HEADER_1 = 1;
const PKT_ID = 2;
const PKT_LENGTH = 3;
const PKT_INSTRUCTION = 4;
const PKT_ERROR = 4;
const PKT_PARAMETER0 = 5;

// Protocol Instructions
const INST_PING = 1;
const INST_READ = 2;
const INST_WRITE = 3;

// Communication Results
const COMM_SUCCESS = 0;
const COMM_RX_TIMEOUT = -6;
const COMM_RX_CORRUPT = -7;

// Register Addresses (from values.py)
const STS_TORQUE_ENABLE = 40;
const STS_ACC = 41;
const STS_GOAL_POSITION_L = 42;
const STS_GOAL_POSITION_H = 43;
const STS_GOAL_SPEED_L = 46;
const STS_GOAL_SPEED_H = 47;
const STS_PRESENT_POSITION_L = 56;
const STS_PRESENT_POSITION_H = 57;
const STS_PRESENT_SPEED_L = 58;
const STS_PRESENT_SPEED_H = 59;
const STS_PRESENT_LOAD_L = 60;
const STS_PRESENT_VOLTAGE = 62;
const STS_PRESENT_TEMPERATURE = 63;
const STS_MOVING = 66;

// Position limits
const MIN_POSITION = 0;
const MAX_POSITION = 4095;
const MAX_SPEED = 3400;

// Default baud rate for ST3215
const DEFAULT_BAUDRATE = 1000000;

// Angle to steps conversion constants
// Mapping: 0° = 2048 steps (center), -90° = 1024 steps, +90° = 3072 steps
const CENTER_POSITION = 2048;  // Center position in steps (0 degrees)
const STEPS_PER_DEGREE = 2048 / 180;  // Steps per degree (11.377...)
const MIN_ANGLE = -180;  // Minimum angle in degrees
const MAX_ANGLE = 180;   // Maximum angle in degrees

/**
 * ServoController class - controls one ST3215 servo motor
 * 
 * Each servo has a unique ID. This class handles all communication
 * with that servo over the serial bus.
 */
class ServoController {
    /**
     * Creates a new servo controller
     * 
     * @param {number} servoId - The servo ID (1-6) - just for logging
     * @param {string|SerialPort} serialPortPathOrInstance - Serial port path or shared SerialPort instance
     * @param {number} servoIdNumber - ST3215 servo ID number (1-6, or 0-252)
     * @param {number} baudRate - Serial baud rate (default: 1000000)
     */
    constructor(servoId, serialPortPathOrInstance, servoIdNumber, baudRate = DEFAULT_BAUDRATE) {
        this.servoId = servoId;  // For logging (1-6)
        this.servoIdNumber = servoIdNumber;  // Actual ST3215 ID (1-6)
        this.baudRate = baudRate;
        
        // Check if we received a SerialPort instance or a path string
        if (serialPortPathOrInstance && typeof serialPortPathOrInstance.write === 'function') {
            // It's a SerialPort instance - use it directly
            this.serialPort = serialPortPathOrInstance;
            this.serialPortPath = null;
            this.isOpen = true;  // Assume it's already open
            this.ownsPort = false;  // We don't own this port
        } else {
            // It's a path string - we'll create our own port
            this.serialPortPath = serialPortPathOrInstance;
            this.serialPort = null;
            this.isOpen = false;
            this.ownsPort = true;  // We own this port
        }
        
        this.pendingResponse = null;
        this.responseResolve = null;
        this.responseReject = null;
        this.responseTimeout = null;
        this.currentSpeed = 1500; // Default speed, will be updated when setSpeed is called
    }

    /**
     * Opens the serial port connection
     * Call this before using any other functions
     * If a shared SerialPort instance was provided, this does nothing
     */
    async open() {
        // If we're using a shared port, it's already open
        // Don't set up data handler here - it's handled by server.js
        if (!this.ownsPort && this.serialPort) {
            return;
        }
        
        // We own the port, so open it
        try {
            // Create serial port with ST3215 default settings
            this.serialPort = new SerialPort({
                path: this.serialPortPath,
                baudRate: this.baudRate,
                dataBits: 8,
                parity: 'none',
                stopBits: 1,
                autoOpen: false
            });

            // Handle incoming data
            this.serialPort.on('data', (data) => {
                this.handleIncomingData(data);
            });

            // Handle errors
            this.serialPort.on('error', (error) => {
                console.error(`Servo ${this.servoId}: Serial port error:`, error.message);
            });

            // Open the port
            return new Promise((resolve, reject) => {
                this.serialPort.open((error) => {
                    if (error) {
                        console.error(`Servo ${this.servoId}: Failed to open serial port:`, error.message);
                        reject(error);
                    } else {
                        this.isOpen = true;
                        console.log(`Servo ${this.servoId}: Serial port opened successfully (ST3215 ID: ${this.servoIdNumber})`);
                        resolve();
                    }
                });
            });
        } catch (error) {
            console.error(`Servo ${this.servoId}: Failed to open serial port:`, error.message);
            throw error;
        }
    }

    /**
     * Closes the serial port connection
     * Call this when you're done using the controller
     * If using a shared port, this does nothing (don't close shared ports)
     */
    async close() {
        // Don't close shared ports - only close ports we own
        if (!this.ownsPort) {
            return;
        }
        
        if (this.serialPort && this.isOpen) {
            return new Promise((resolve, reject) => {
                this.serialPort.close((error) => {
                    if (error) {
                        reject(error);
                    } else {
                        this.isOpen = false;
                        console.log(`Servo ${this.servoId}: Serial port closed`);
                        resolve();
                    }
                });
            });
        }
    }

    /**
     * Handle incoming serial data
     * @private
     */
    handleIncomingData(data) {
        // Always process data, but only resolve if we're waiting
        const hasPendingResponse = !!this.responseResolve;
        
        if (!hasPendingResponse) {
            // Not waiting for response, ignore this data
            return;
        }
        
        // Accumulate data if we have pending response
        // Limit buffer size to prevent memory buildup (max 256 bytes should be enough for any packet)
        const MAX_BUFFER_SIZE = 256;
        if (this.pendingResponse) {
            const newSize = this.pendingResponse.length + data.length;
            if (newSize > MAX_BUFFER_SIZE) {
                // Buffer too large - likely corrupted or wrong packet, clear it
                console.warn(`Servo ${this.servoId}: Buffer too large (${newSize} bytes), clearing`);
                this.pendingResponse = null;
                if (this.responseResolve) {
                    // Clear timeout
                    if (this.responseTimeout) {
                        clearTimeout(this.responseTimeout);
                        this.responseTimeout = null;
                    }
                    // Reject the promise if we have a reject callback
                    if (this.responseReject) {
                        const reject = this.responseReject;
                        this.responseResolve = null;
                        this.responseReject = null;
                        reject(new Error('Response buffer overflow'));
                    } else {
                        // Just clear if no reject callback (shouldn't happen, but be safe)
                        this.responseResolve = null;
                    }
                }
                return;
            }
            this.pendingResponse = Buffer.concat([this.pendingResponse, data]);
        } else {
            this.pendingResponse = data;
        }
        
        // Try to parse the complete packet
        const result = this.parseResponsePacket(this.pendingResponse);
        
        if (result.complete) {
            // Check if this packet is for our servo ID
            if (result.id === this.servoIdNumber) {
                // This packet is for us - resolve the promise
                if (this.responseResolve) {
                    // Clear timeout
                    if (this.responseTimeout) {
                        clearTimeout(this.responseTimeout);
                        this.responseTimeout = null;
                    }
                    
                    // Resolve the promise
                    const resolve = this.responseResolve;
                    this.responseResolve = null;
                    this.responseReject = null;
                    this.pendingResponse = null;
                    
                    resolve(result);
                }
            } else {
                // This packet is for a different servo - clear our pending response
                // and let other servos handle it
                this.pendingResponse = null;
            }
        } else if (result.error) {
            // Only log actual errors, not incomplete packets
            if (result.error !== 'Invalid header' && result.error !== 'Header at wrong position') {
                if (DEBUG) console.log(`[DEBUG Servo ${this.servoId}] Packet parse error: ${result.error}`);
            }
        }
        // If not complete, keep accumulating data in pendingResponse
    }

    /**
     * Parse response packet from servo
     * @private
     */
    parseResponsePacket(buffer) {
        // Minimum packet size: Header(2) + ID(1) + Length(1) + Error(1) + Checksum(1) = 6 bytes
        if (buffer.length < 6) {
            return { complete: false };
        }

        // Check header bytes (0xFF 0xFF)
        if (buffer[0] !== 0xFF || buffer[1] !== 0xFF) {
            // Try to find header in the buffer (packet might start later)
            const headerIndex = buffer.indexOf(0xFF);
            if (headerIndex >= 0 && headerIndex < buffer.length - 1 && buffer[headerIndex + 1] === 0xFF) {
                // Found header at different position - this shouldn't happen but handle it
                return { complete: false, error: 'Header at wrong position' };
            }
            return { complete: false, error: 'Invalid header' };
        }

        const id = buffer[PKT_ID];
        const length = buffer[PKT_LENGTH];
        // According to ST3215 protocol (from Python code):
        // wait_length = rxpacket[PKT_LENGTH] + PKT_LENGTH + 1
        // PKT_LENGTH = 3 (the index of the length field)
        // So: expectedLength = length + 3 + 1 = length + 4
        // This means: Header(2) + ID(1) + Length(1) + [Data of 'length' bytes] + Checksum(1)
        // Where Data = Error(1) + Parameters
        const expectedLength = length + 4; // Header(2) + ID(1) + Length(1) + Data(length) + Checksum(1)

        // Check if we have enough data
        if (buffer.length < expectedLength) {
            return { complete: false };
        }

        // Verify checksum
        // Checksum is calculated from ID to the byte before checksum
        let checksum = 0;
        for (let i = PKT_ID; i < expectedLength - 1; i++) {
            checksum += buffer[i];
        }
        checksum = (~checksum) & 0xFF;

        if (checksum !== buffer[expectedLength - 1]) {
            if (DEBUG) {
                console.log(`[DEBUG Servo ${this.servoId}] Checksum mismatch: calculated=0x${checksum.toString(16)}, received=0x${buffer[expectedLength - 1].toString(16)}`);
                // For debugging, let's see what we calculated
            const calcBytes = [];
            for (let i = PKT_ID; i < expectedLength - 1; i++) {
                calcBytes.push('0x' + buffer[i].toString(16));
            }
                console.log(`[DEBUG Servo ${this.servoId}] Checksum bytes: ${calcBytes.join(' ')}`);
            }
            return { complete: false, error: 'Checksum mismatch' };
        }

        // Extract data
        // Error byte is at PKT_ERROR (which is same as PKT_INSTRUCTION = 4)
        // The length field in response packets includes: Error(1) + Parameters + Checksum(1)
        // So Parameters = length - 2 (excluding Error and Checksum)
        // Parameters start at PKT_PARAMETER0 (5) and are (length - 2) bytes long
        const error = buffer[PKT_ERROR];
        // Calculate how many parameter bytes we have
        // length includes Error(1) + Parameters + Checksum(1), so Parameters = length - 2
        const parameterLength = length - 2;
        // Extract parameters (data bytes) - these come after the Error byte
        // For write responses, parameterLength = 0 (no data returned, just Error + Checksum)
        // For read responses, parameterLength = number of bytes requested
        let parameters;
        if (parameterLength > 0) {
            // We have data bytes to extract
            const endIndex = PKT_PARAMETER0 + parameterLength;
            if (endIndex <= buffer.length) {
                parameters = buffer.slice(PKT_PARAMETER0, endIndex);
            } else {
                // Not enough data in buffer - this shouldn't happen if packet is complete
                if (DEBUG) console.log(`[DEBUG Servo ${this.servoId}] Buffer too short: need ${endIndex} bytes but have ${buffer.length}, length=${length}, parameterLength=${parameterLength}`);
                parameters = Buffer.alloc(0);
            }
        } else {
            // No data bytes (write response with just Error + Checksum)
            parameters = Buffer.alloc(0);
        }

        return {
            complete: true,
            id: id,
            error: error,
            parameters: parameters,
            commResult: error === 0 ? COMM_SUCCESS : COMM_RX_CORRUPT,
            responseLength: length  // Store the response packet length to distinguish write vs read responses
        };
    }

    /**
     * Send a packet to the servo
     * @private
     */
    async sendPacket(instruction, parameters = []) {
        if (!this.isOpen) {
            throw new Error(`Servo ${this.servoId}: Serial port is not open. Call open() first.`);
        }

        // Build packet: Header(2) + ID(1) + Length(1) + Instruction(1) + Parameters + Checksum(1)
        const length = 2 + parameters.length; // Instruction + Parameters + Checksum
        const packet = Buffer.alloc(4 + length);
        
        packet[PKT_HEADER_0] = 0xFF;
        packet[PKT_HEADER_1] = 0xFF;
        packet[PKT_ID] = this.servoIdNumber;
        packet[PKT_LENGTH] = length;
        packet[PKT_INSTRUCTION] = instruction;
        
        // Add parameters
        for (let i = 0; i < parameters.length; i++) {
            packet[PKT_PARAMETER0 + i] = parameters[i];
        }
        
        // Calculate checksum
        let checksum = 0;
        for (let i = PKT_ID; i < PKT_PARAMETER0 + parameters.length; i++) {
            checksum += packet[i];
        }
        checksum = (~checksum) & 0xFF;
        packet[PKT_PARAMETER0 + parameters.length] = checksum;

        // Debug: log full packet for write commands
        if (instruction === INST_WRITE) {
            const packetHex = Array.from(packet).map(b => '0x' + b.toString(16).padStart(2,'0')).join(' ');
            if (DEBUG) {
                console.log(`[DEBUG Servo ${this.servoId}] Full write packet (${packet.length} bytes): ${packetHex}`);
                console.log(`[DEBUG Servo ${this.servoId}] Packet structure: Header=[${packet[0]},${packet[1]}], ID=${packet[2]}, Length=${packet[3]}, Inst=${packet[4]}, Params=[${Array.from(packet.slice(5, 5+parameters.length)).map(b => '0x' + b.toString(16).padStart(2,'0')).join(', ')}], Checksum=0x${packet[packet.length-1].toString(16).padStart(2,'0')}`);
            }
        }

        // Write packet to serial port
        // If using shared port, we need to check if there's a write queue
        const writeFn = () => {
            return new Promise((resolve, reject) => {
                this.serialPort.write(packet, (error) => {
                    if (error) {
                        if (DEBUG) console.error(`[DEBUG Servo ${this.servoId}] Write error:`, error);
                        reject(error);
                    } else {
                        // Short wait to ensure the packet has been handed off to the driver.
                        // 1 ms at 1 Mbps is plenty for our packet sizes.
                        setTimeout(resolve, 1);
                    }
                });
            });
        };
        
        // Check if we're using a shared port (has writeQueue function)
        // For now, we'll always queue writes to ensure serialization
        // This will be set up by the server.js when using shared ports
        if (this.serialPort._writeQueue) {
            return this.serialPort._writeQueue(writeFn);
        } else {
            // Not using shared port, write directly
            return writeFn();
        }
    }

    /**
     * Read data from servo (send read instruction and wait for response)
     * @private
     */
    async readData(address, length) {
        // Prepare read parameters: address (1 byte) + length (1 byte)
        // This matches the ST3215 Python library:
        //   txpacket[PKT_PARAMETER0 + 0] = address
        //   txpacket[PKT_PARAMETER0 + 1] = length
        const parameters = [
            address & 0xFF,          // Address (single byte)
            length & 0xFF            // Number of bytes to read
        ];

        // Clear any pending response
        this.pendingResponse = null;
        if (this.responseTimeout) {
            clearTimeout(this.responseTimeout);
            this.responseTimeout = null;
        }
        this.responseResolve = null;

        // Set up response handler BEFORE sending packet to avoid race condition
        const readPromise = new Promise((resolve, reject) => {
            // Store reject callback for timeout/error handling
            this.responseReject = reject;
            this.responseResolve = (result) => {
                if (this.responseTimeout) {
                    clearTimeout(this.responseTimeout);
                    this.responseTimeout = null;
                }
                
                // Check communication result and error
                if (result.commResult !== COMM_SUCCESS) {
                    this.responseResolve = null;
                    this.pendingResponse = null;
                    reject(new Error(`Communication error: ${result.commResult}`));
                } else if (result.error !== 0) {
                    this.responseResolve = null;
                    this.pendingResponse = null;
                    reject(new Error(`Servo error: ${result.error}`));
                } else {
                    // Return the parameter data (which contains the register values)
                    // For read commands, parameters should be a Buffer with the register data
                    if (!result.parameters) {
                        if (DEBUG) console.error(`[DEBUG Servo ${this.servoId}] No parameters in read response, result:`, result);
                        this.responseResolve = null;
                        this.pendingResponse = null;
                        reject(new Error('No data in read response'));
                        return;
                    }
                    // Convert to Buffer if it's not already (it should be a Buffer slice)
                    const data = Buffer.isBuffer(result.parameters) ? result.parameters : Buffer.from(result.parameters);
                    if (data.length === 0) {
                        // Empty parameters - check if this is a write response (length=2) being matched to read
                        if (result.responseLength === 2) {
                            // This is a write response (Error + Checksum only), not a read response
                            // Ignore it and wait for the actual read response
                            // Keep quiet to avoid spamming logs during normal operation.
                            return; // Don't resolve or reject, wait for actual read response
                        } else {
                            // This is a read response but with no data - error
                            if (DEBUG) console.error(`[DEBUG Servo ${this.servoId}] Read response has no data, responseLength=${result.responseLength}`);
                            this.responseResolve = null;
                            this.pendingResponse = null;
                            reject(new Error('Empty read response data'));
                            return;
                        }
                    }
                    this.responseResolve = null;
                    this.pendingResponse = null;
                    resolve(data);
                }
            };
            
            // Set timeout (10ms) - increased for shared port scenarios
            this.responseTimeout = setTimeout(() => {
                this.responseResolve = null;
                this.pendingResponse = null;
                reject(new Error('Read timeout'));
            }, 10);
        });

        // Now send read instruction
        await this.sendPacket(INST_READ, parameters);

        // Wait for response
        return await readPromise;
    }

    /**
     * Write data to servo
     * @private
     */
    async writeData(address, data) {
        // Prepare write parameters: address (1 byte) + data bytes
        // Python: txpacket[PKT_PARAMETER0] = address (single byte)
        //         txpacket[PKT_PARAMETER0 + 1: PKT_PARAMETER0 + 1 + length] = data
        const parameters = [
            address & 0xFF,           // Address (single byte)
            ...data                    // Data bytes follow immediately after address
        ];
        
        // Debug: log the full packet structure
        if (DEBUG) console.log(`[DEBUG Servo ${this.servoId}] writeData: address=0x${(address & 0xFF).toString(16).padStart(2,'0')}, data=[${data.map(b => '0x' + b.toString(16).padStart(2,'0')).join(', ')}], parameters=[${parameters.map(b => '0x' + b.toString(16).padStart(2,'0')).join(', ')}]`);

        // Clear any pending response
        this.pendingResponse = null;
        if (this.responseTimeout) {
            clearTimeout(this.responseTimeout);
            this.responseTimeout = null;
        }
        this.responseResolve = null;

        // Set up response handler BEFORE sending packet to avoid race condition
        const writePromise = new Promise((resolve, reject) => {
            // Store reject callback for timeout/error handling
            this.responseReject = reject;
            this.responseResolve = (result) => {
                if (this.responseTimeout) {
                    clearTimeout(this.responseTimeout);
                    this.responseTimeout = null;
                }
                
                // Check communication result and error
                if (result.commResult !== COMM_SUCCESS) {
                    this.responseResolve = null;
                    this.pendingResponse = null;
                    reject(new Error(`Communication error: ${result.commResult}`));
                } else if (result.error !== 0) {
                    this.responseResolve = null;
                    this.pendingResponse = null;
                    reject(new Error(`Servo error: ${result.error}`));
                } else {
                    this.responseResolve = null;
                    this.pendingResponse = null;
                    resolve(result);
                }
            };
            
            // Set timeout (200ms) - increased for shared port scenarios
            this.responseTimeout = setTimeout(() => {
                if (DEBUG) console.log(`[DEBUG Servo ${this.servoId}] Write timeout - no response received`);
                this.responseResolve = null;
                this.responseReject = null;
                this.pendingResponse = null;
                reject(new Error('Write timeout'));
            }, 200);
        });

        // Now send write instruction
        await this.sendPacket(INST_WRITE, parameters);

        // Wait for response and return true on success
        const result = await writePromise;
        return true;
    }

    /**
     * Helper: Convert 16-bit value to low/high bytes (little-endian)
     * @private
     */
    stsLobyte(value) {
        return value & 0xFF;
    }

    stsHibyte(value) {
        return (value >> 8) & 0xFF;
    }

    /**
     * Helper: Convert two bytes to 16-bit value (little-endian)
     * @private
     */
    makeWord(low, high) {
        return (low & 0xFF) | ((high & 0xFF) << 8);
    }

    /**
     * Ping the servo to check if it's responding
     * @returns {boolean} True if servo responds
     */
    async ping() {
        try {
            // Clear any pending response
            this.pendingResponse = null;
            if (this.responseTimeout) {
                clearTimeout(this.responseTimeout);
                this.responseTimeout = null;
            }
            this.responseResolve = null;
            
            // Set up response handler BEFORE sending packet to avoid race condition
            const pingPromise = new Promise((resolve) => {
                this.responseResolve = (result) => {
                    if (this.responseTimeout) {
                        clearTimeout(this.responseTimeout);
                        this.responseTimeout = null;
                    }
                    
                    // For ping, we just need to check if we got a response with the correct ID
                    // The error byte should be 0, and the ID should match
                    const success = result.id === this.servoIdNumber && result.error === 0;
                    this.responseResolve = null;
                    this.pendingResponse = null;
                    resolve(success);
                };
                
                this.responseTimeout = setTimeout(() => {
                    if (DEBUG) console.log(`[DEBUG Servo ${this.servoId}] Ping timeout - no response received`);
                    this.responseResolve = null;
                    this.pendingResponse = null;
                    resolve(false);
                }, 200);
            });
            
            // Now send ping packet
            await this.sendPacket(INST_PING, []);
            
            // Wait for response
            return await pingPromise;
        } catch (error) {
            console.error(`Servo ${this.servoId}: Ping failed:`, error.message);
            this.responseResolve = null;
            this.pendingResponse = null;
            if (this.responseTimeout) {
                clearTimeout(this.responseTimeout);
                this.responseTimeout = null;
            }
            return false;
        }
    }

    /**
     * Enable torque (start servo)
     * @returns {boolean} True if successful
     */
    async startServo() {
        try {
            await this.writeData(STS_TORQUE_ENABLE, [1]);
            if (DEBUG) console.log(`Servo ${this.servoId}: Torque enabled`);
            return true;
        } catch (error) {
            console.error(`Servo ${this.servoId}: Failed to enable torque:`, error.message);
            return false;
        }
    }

    /**
     * Disable torque (stop servo)
     * @returns {boolean} True if successful
     */
    async stopServo() {
        try {
            await this.writeData(STS_TORQUE_ENABLE, [0]);
            if (DEBUG) console.log(`Servo ${this.servoId}: Torque disabled`);
            return true;
        } catch (error) {
            console.error(`Servo ${this.servoId}: Failed to disable torque:`, error.message);
            return false;
        }
    }

    /**
     * Set servo acceleration
     * @param {number} acc - Acceleration value (0-254, unit: 100 step/s²)
     */
    async setAcceleration(acc) {
        if (acc < 0) acc = 0;
        if (acc > 254) acc = 254;
        
        try {
            await this.writeData(STS_ACC, [acc]);
            if (DEBUG) console.log(`Servo ${this.servoId}: Acceleration set to ${acc} (${acc * 100} step/s²)`);
        } catch (error) {
            console.error(`Servo ${this.servoId}: Failed to set acceleration:`, error.message);
            throw error;
        }
    }

    /**
     * Set servo speed
     * @param {number} speed - Speed value (0-3400 step/s)
     */
    async setSpeed(speed) {
        if (speed < 0) speed = 0;
        if (speed > MAX_SPEED) speed = MAX_SPEED;
        
        try {
            // ST3215 expects bytes in [low_byte, high_byte] order (little-endian, matching Python library)
            const data = [
                this.stsLobyte(speed),  // Low byte first (matches Python)
                this.stsHibyte(speed)   // High byte second (matches Python)
            ];
            await this.writeData(STS_GOAL_SPEED_L, data);
            // Store the current speed so it can be used by moveToAngle if no speed is specified
            this.currentSpeed = speed;
            if (DEBUG) console.log(`Servo ${this.servoId}: Speed set to ${speed} step/s`);
        } catch (error) {
            console.error(`Servo ${this.servoId}: Failed to set speed:`, error.message);
            throw error;
        }
    }

    /**
     * Convert angle in degrees to step position
     * 
     * Mapping:
     *   - 0 degrees = 2048 steps (center)
     *   - -90 degrees = 1024 steps
     *   - +90 degrees = 3072 steps
     * 
     * @param {number} angleDegrees - Angle in degrees (-90 to +90)
     * @returns {number} Step position (1024 to 3072)
     */
    angleToSteps(angleDegrees) {
        // Clamp angle to valid range
        if (angleDegrees < MIN_ANGLE) angleDegrees = MIN_ANGLE;
        if (angleDegrees > MAX_ANGLE) angleDegrees = MAX_ANGLE;
        
        // Convert: steps = center + (angle * steps_per_degree)
        const steps = Math.round(CENTER_POSITION + (angleDegrees * STEPS_PER_DEGREE));
        
        // Clamp to valid step range (should be 1024-3072, but add safety check)
        if (steps < MIN_POSITION) return MIN_POSITION;
        if (steps > MAX_POSITION) return MAX_POSITION;
        
        return steps;
    }

    /**
     * Convert step position to angle in degrees
     * 
     * Inverse of angleToSteps:
     *   - 2048 steps = 0 degrees (center)
     *   - 1024 steps = -90 degrees
     *   - 3072 steps = +90 degrees
     * 
     * @param {number} steps - Step position (1024 to 3072)
     * @returns {number} Angle in degrees (-90 to +90)
     */
    stepsToAngle(steps) {
        // Clamp steps to valid range
        if (steps < MIN_POSITION) steps = MIN_POSITION;
        if (steps > MAX_POSITION) steps = MAX_POSITION;
        
        // Convert: angle = (steps - center) / steps_per_degree
        const angle = (steps - CENTER_POSITION) / STEPS_PER_DEGREE;
        
        // Clamp to valid angle range
        if (angle < MIN_ANGLE) return MIN_ANGLE;
        if (angle > MAX_ANGLE) return MAX_ANGLE;
        
        return angle;
    }

    /**
     * Move servo to a specific angle in degrees
     * 
     * Uses the angle-to-steps conversion:
     *   - 0° = 2048 steps (center)
     *   - -90° = 1024 steps
     *   - +90° = 3072 steps
     * 
     * @param {number} angleDegrees - Target angle in degrees (-90 to +90)
     * @param {number} speed - Optional speed in step/s (if not provided, uses last set speed or default 1500)
     */
    async moveToAngle(angleDegrees, speed = null) {
        // Convert angle to steps using the conversion function
        const position = this.angleToSteps(angleDegrees);
        
        try {
            // Use provided speed, or the current stored speed
            // Check explicitly for null/undefined, but allow 0 as a valid speed value
            const speedToUse = (speed !== null && speed !== undefined) ? speed : this.currentSpeed;
            
            // Always set speed to ensure it's applied (either the provided one or the stored one)
            await this.setSpeed(speedToUse);
            
            // Don't read position before move - it can cause response matching issues
            // Just proceed with the move
            if (DEBUG) console.log(`Servo ${this.servoId}: Moving to angle ${angleDegrees}° (position ${position}) at speed ${speedToUse} step/s`);

            const data = [
                this.stsLobyte(position),   // Low byte first
                this.stsHibyte(position)    // High byte second
            ];
            await this.writeData(STS_GOAL_POSITION_L, data);
            if (DEBUG) console.log(`Servo ${this.servoId}: Position command sent (bytes: 0x${data[0].toString(16).padStart(2,'0')} 0x${data[1].toString(16).padStart(2,'0')})`);
        } catch (error) {
            console.error(`Servo ${this.servoId}: Failed to move to angle:`, error.message);
            throw error;
        }
    }

    /**
     * Move servo to a specific position (0-4095)
     * 
     * @param {number} position - Target position (0-4095)
     */
    async moveToPosition(position) {
        // Clamp to valid range
        if (position < MIN_POSITION) position = MIN_POSITION;
        if (position > MAX_POSITION) position = MAX_POSITION;
        
        try {
            // ST3215 expects bytes in [low_byte, high_byte] order (little-endian, matching Python library)
            const data = [
                this.stsLobyte(position),  // Low byte first (matches Python)
                this.stsHibyte(position)   // High byte second (matches Python)
            ];
            await this.writeData(STS_GOAL_POSITION_L, data);
            if (DEBUG) console.log(`Servo ${this.servoId}: Moving to position ${position}`);
        } catch (error) {
            console.error(`Servo ${this.servoId}: Failed to move to position:`, error.message);
            throw error;
        }
    }

    /**
     * Read current position and convert to angle
     * Uses the steps-to-angle conversion function
     * 
     * @returns {number} Current angle in degrees (-90 to +90), or -1 if error
     */
    async getAngle() {
        try {
            const data = await this.readData(STS_PRESENT_POSITION_L, 2);
            const position = this.makeWord(data[0], data[1]);
            const angle = this.stepsToAngle(position);
            return angle;
        } catch (error) {
            console.error(`Servo ${this.servoId}: Failed to read angle:`, error.message);
            return -1;
        }
    }

    /**
     * Read current position (0-4095)
     * @returns {number} Current position, or -1 if error
     */
    async getPosition() {
        try {
            const data = await this.readData(STS_PRESENT_POSITION_L, 2);
            const position = this.makeWord(data[0], data[1]);
            return position;
        } catch (error) {
            console.error(`Servo ${this.servoId}: Failed to read position:`, error.message);
            return -1;
        }
    }

    /**
     * Check if servo is moving
     * @returns {boolean} True if moving, false if not, null if error
     */
    async isMoving() {
        try {
            const data = await this.readData(STS_MOVING, 1);
            return data[0] !== 0;
        } catch (error) {
            console.error(`Servo ${this.servoId}: Failed to read moving status:`, error.message);
            return null;
        }
    }

    /**
     * Read all servo statistics
     * @returns {Object} Status object with all information
     */
    async readStatus() {
        try {
            // Read position
            const positionData = await this.readData(STS_PRESENT_POSITION_L, 2);
            if (!positionData || positionData.length < 2) {
                // Log more details for debugging
                if (DEBUG) {
                    console.log(`[DEBUG Servo ${this.servoId}] Invalid position data:`, {
                        hasData: !!positionData,
                        length: positionData ? positionData.length : 0,
                        data: positionData ? Array.from(positionData).map(b => '0x' + b.toString(16).padStart(2, '0')).join(' ') : 'null'
                    });
                }
                throw new Error('Invalid position data received');
            }
            const position = this.makeWord(positionData[0], positionData[1]);
            // Convert position to angle using the conversion function
            const angleDegrees = this.stepsToAngle(position);
            
            // Debug: log position and angle conversion
            // Log when angle is unexpected or when position is near center
            if (Math.abs(angleDegrees - (-90)) < 1.0 || (position >= 2040 && position <= 2150)) {
                const byte0 = positionData[0] !== undefined ? '0x' + positionData[0].toString(16).padStart(2,'0') : 'undefined';
                const byte1 = positionData[1] !== undefined ? '0x' + positionData[1].toString(16).padStart(2,'0') : 'undefined';
                const expectedAngle = ((position - CENTER_POSITION) / STEPS_PER_DEGREE).toFixed(2);
                if (DEBUG) console.log(`[DEBUG Servo ${this.servoId}] Position: ${position} (bytes: ${byte0} ${byte1}), angle: ${angleDegrees.toFixed(2)}°, expected: ${expectedAngle}°`);
            }
            
            // Read speed
            const speedData = await this.readData(STS_PRESENT_SPEED_L, 2);
            if (!speedData || speedData.length < 2) {
                throw new Error('Invalid speed data received');
            }
            const speed = this.makeWord(speedData[0], speedData[1]);
            
            // Read load
            const loadData = await this.readData(STS_PRESENT_LOAD_L, 1);
            if (!loadData || loadData.length < 1) {
                throw new Error('Invalid load data received');
            }
            const load = loadData[0] * 0.1; // Convert to percentage
            
            // Read voltage
            const voltageData = await this.readData(STS_PRESENT_VOLTAGE, 1);
            if (!voltageData || voltageData.length < 1) {
                throw new Error('Invalid voltage data received');
            }
            const voltage = voltageData[0] * 0.1; // Convert to Volts
            
            // Read temperature
            const tempData = await this.readData(STS_PRESENT_TEMPERATURE, 1);
            if (!tempData || tempData.length < 1) {
                throw new Error('Invalid temperature data received');
            }
            const temperature = tempData[0]; // Celsius
            
            // Read moving status
            const movingData = await this.readData(STS_MOVING, 1);
            if (!movingData || movingData.length < 1) {
                throw new Error('Invalid moving status data received');
            }
            const isMoving = movingData[0] !== 0;
            
            // Read torque enable status
            const torqueData = await this.readData(STS_TORQUE_ENABLE, 1);
            if (!torqueData || torqueData.length < 1) {
                throw new Error('Invalid torque status data received');
            }
            const torqueEnabled = torqueData[0] !== 0;
            
            return {
                angleDegrees: angleDegrees,
                position: position,
                speed: speed,
                load: load,
                voltage: voltage,
                temperature: temperature,
                isMoving: isMoving,
                torqueEnabled: torqueEnabled
            };
        } catch (error) {
            console.error(`Servo ${this.servoId}: Failed to read status:`, error.message);
            throw error;
        }
    }

    /**
     * Read a minimal status set (fast path for frequent polling).
     * Uses ONE bulk read starting at STS_TORQUE_ENABLE so we only hit the bus once.
     * Extracts:
     *  - torqueEnabled (STS_TORQUE_ENABLE)
     *  - position -> angleDegrees (STS_PRESENT_POSITION_L/H)
     *  - isMoving (STS_MOVING)
     *
     * @returns {Object} { angleDegrees, position, isMoving, torqueEnabled }
     */
    async readQuickStatus() {
        try {
            // Read from STS_TORQUE_ENABLE up to (and including) STS_MOVING in one block.
            const startAddr = STS_TORQUE_ENABLE;
            const length = (STS_MOVING - STS_TORQUE_ENABLE) + 1; // inclusive range
            const data = await this.readData(startAddr, length);
            if (!data || data.length < length) {
                throw new Error(`Invalid quick status data received (len=${data ? data.length : 0}, expected>=${length})`);
            }

            // Torque enabled at STS_TORQUE_ENABLE
            const torqueEnabled = data[0] !== 0;

            // Position bytes at STS_PRESENT_POSITION_L/H
            const posLowIndex = STS_PRESENT_POSITION_L - startAddr;
            const posHighIndex = STS_PRESENT_POSITION_H - startAddr;
            const position = this.makeWord(data[posLowIndex], data[posHighIndex]);
            const angleDegrees = this.stepsToAngle(position);

            // Moving flag at STS_MOVING
            const movingIndex = STS_MOVING - startAddr;
            const isMoving = data[movingIndex] !== 0;

            return {
                angleDegrees,
                position,
                isMoving,
                torqueEnabled
            };
        } catch (error) {
            console.error(`Servo ${this.servoId}: Failed to read quick status:`, error.message);
            throw error;
        }
    }
}

// Export the module
module.exports = {
    ServoController: ServoController,
    // Export conversion constants and functions for external use
    CENTER_POSITION: CENTER_POSITION,
    STEPS_PER_DEGREE: STEPS_PER_DEGREE,
    MIN_ANGLE: MIN_ANGLE,
    MAX_ANGLE: MAX_ANGLE
};

