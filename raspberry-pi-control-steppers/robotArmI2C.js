/**
 * Robot Arm I2C Communication Module
 * 
 * This module provides a simple interface to communicate with PIC16F18326
 * controllers over I2C. Each PIC controller handles one robot arm joint.
 * 
 * The PIC controllers are I2C slaves with address 0x22 (by default).
 * This module acts as the I2C master on a Raspberry Pi.
 * 
 * Usage:
 *   const RobotArm = require('./robotArmI2C');
 *   const joint1 = new RobotArm.JointController(1, '/dev/i2c-3', 0x22);
 *   await joint1.moveToAngle(90.5);
 */

const i2c = require('i2c-bus');

/**
 * JointController class - controls one robot arm joint via I2C
 * 
 * Each joint has one PIC controller. This class handles all communication
 * with that PIC controller.
 */
class JointController {
    /**
     * Creates a new joint controller
     * 
     * @param {number} jointNumber - The joint number (1, 2, 3, etc.) - just for logging
     * @param {string} i2cBusPath - I2C bus path, e.g. '/dev/i2c-1' or '/dev/i2c-3'
     * @param {number} i2cAddress - I2C address of the PIC controller (default: 0x22)
     */
    constructor(jointNumber, i2cBusPath, i2cAddress = 0x22) {
        this.jointNumber = jointNumber;
        this.i2cBusPath = i2cBusPath;
        this.i2cAddress = i2cAddress;
        this.i2cBus = null;
        this.isOpen = false;
        
        // Extract the bus number from the path (e.g., '/dev/i2c-3' -> 3)
        const busMatch = i2cBusPath.match(/i2c-(\d+)/);
        if (busMatch) {
            this.i2cBusNumber = parseInt(busMatch[1], 10);
        } else {
            // Default to bus 1 if we can't parse the path
            this.i2cBusNumber = 1;
        }
    }

    /**
     * Opens the I2C bus connection
     * Call this before using any other functions
     */
    async open() {
        try {
            // Open the I2C bus for communication
            // Uses the bus number extracted from the path (e.g., /dev/i2c-3 -> bus 3)
            this.i2cBus = await i2c.openPromisified(this.i2cBusNumber);
            this.isOpen = true;
            console.log(`Joint ${this.jointNumber}: I2C bus ${this.i2cBusNumber} opened successfully at address 0x${this.i2cAddress.toString(16)}`);
        } catch (error) {
            console.error(`Joint ${this.jointNumber}: Failed to open I2C bus ${this.i2cBusNumber}:`, error.message);
            throw error;
        }
    }

    /**
     * Closes the I2C bus connection
     * Call this when you're done using the controller
     */
    async close() {
        if (this.i2cBus && this.isOpen) {
            await this.i2cBus.close();
            this.isOpen = false;
            console.log(`Joint ${this.jointNumber}: I2C bus closed`);
        }
    }

    /**
     * Sends a command to the PIC controller
     * This is a private helper function - other functions use this to send commands
     * 
     * @param {Array} commandBytes - Array of bytes to send (first byte is command, rest is data)
     * @private
     */
    async sendCommand(commandBytes) {
        if (!this.isOpen) {
            throw new Error(`Joint ${this.jointNumber}: I2C bus is not open. Call open() first.`);
        }

        try {
            // Create a buffer from the command bytes
            const buffer = Buffer.from(commandBytes);
            
            // Write the bytes directly to the PIC controller
            // i2cWrite(address, length, buffer) - writes raw bytes to I2C device
            await this.i2cBus.i2cWrite(this.i2cAddress, buffer.length, buffer);
            
            // Small delay to let the PIC process the command
            await new Promise(resolve => setTimeout(resolve, 10));
        } catch (error) {
            console.error(`Joint ${this.jointNumber}: Failed to send command:`, error.message);
            throw error;
        }
    }

    /**
     * Reads data from the PIC controller
     * This is a private helper function
     * 
     * @param {number} byteCount - How many bytes to read
     * @returns {Buffer} The data read from the PIC
     * @private
     */
    async readData(byteCount) {
        if (!this.isOpen) {
            throw new Error(`Joint ${this.jointNumber}: I2C bus is not open. Call open() first.`);
        }

        try {
            // Create a buffer to hold the data we'll read
            const buffer = Buffer.alloc(byteCount);
            
            // Read bytes directly from the PIC controller
            // i2cRead(address, length, buffer) - reads raw bytes from I2C device
            await this.i2cBus.i2cRead(this.i2cAddress, byteCount, buffer);
            
            return buffer;
        } catch (error) {
            console.error(`Joint ${this.jointNumber}: Failed to read data:`, error.message);
            throw error;
        }
    }

    /**
     * Moves the motor to a specific angle in degrees
     * 
     * Command 0x01: Move to target angle
     * Format: 0x01 <LSB> <MSB>
     * Angle is sent as a 16-bit value in hundredths of a degree
     * Example: 90.00 degrees = 9000 = 0x2328 = [0x28, 0x23]
     * 
     * @param {number} angleDegrees - Target angle in degrees (e.g., 90.5)
     */
    async moveToAngle(angleDegrees) {
        // Convert angle to hundredths of degrees (multiply by 100)
        const angleX100 = Math.round(angleDegrees * 100);
        
        // Clamp to valid range: -160 to 160 degrees (-16000 to 16000 in hundredths)
        let angleValue = angleX100;
        if (angleValue < -16000) angleValue = -16000;
        if (angleValue > 16000) angleValue = 16000;
        
        // Convert to signed 16-bit integer representation
        // JavaScript bitwise operations work with 32-bit signed integers
        // The value is already clamped to -16000 to 16000, so it fits in 16-bit signed range
        const angleInt16 = angleValue;
        
        // Split into low byte (LSB) and high byte (MSB)
        // For negative numbers, JavaScript's bitwise operations handle two's complement correctly
        const lsb = angleInt16 & 0xFF;           // Get the lower 8 bits
        const msb = (angleInt16 >> 8) & 0xFF;    // Get the upper 8 bits (arithmetic right shift sign-extends)
        
        // Send command: 0x01 followed by LSB, then MSB
        await this.sendCommand([0x01, lsb, msb]);
        
        console.log(`Joint ${this.jointNumber}: Moving to angle ${angleDegrees}° (${angleX100} hundredths)`);
    }

    /**
     * Immediately stops the motor motion
     * 
     * Command 0x02: Stop motion
     */
    async stopMotion() {
        await this.sendCommand([0x02]);
        console.log(`Joint ${this.jointNumber}: Stop command sent`);
    }

    /**
     * Sets the servo angle in degrees (0-180)
     * 
     * Command 0x03: Set servo angle
     * Format: 0x03 <angle>
     * 
     * @param {number} angleDegrees - Servo angle from 0 to 180 degrees
     */
    async setServoAngle(angleDegrees) {
        // Make sure angle is in valid range
        let angle = Math.round(angleDegrees);
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;
        
        await this.sendCommand([0x03, angle]);
        console.log(`Joint ${this.jointNumber}: Servo set to ${angle}°`);
    }

    /**
     * Sets the servo output as a relay (on/off) instead of PWM
     * 
     * Command 0x08: Set relay state
     * Format: 0x08 <state>
     * 
     * @param {boolean} state - true = on, false = off
     */
    async setRelayState(state) {
        const stateByte = state ? 1 : 0;
        await this.sendCommand([0x08, stateByte]);
        console.log(`Joint ${this.jointNumber}: Relay set to ${state ? 'ON' : 'OFF'}`);
    }

    /**
     * Sets the AS5600 sensor I2C address (stored in EEPROM)
     * 
     * Command 0x04: Set AS5600 I2C address
     * Format: 0x04 <addr>
     * 
     * @param {number} address - New I2C address for the AS5600 sensor
     */
    async setSensorAddress(address) {
        const addr = address & 0xFF; // Ensure it's a single byte
        await this.sendCommand([0x04, addr]);
        console.log(`Joint ${this.jointNumber}: Sensor address set to 0x${addr.toString(16)}`);
    }

    /**
     * Sets the AS5600 sensor offset in hundredths of degrees (stored in EEPROM)
     * 
     * Command 0x05: Set AS5600 offset
     * Format: 0x05 <LSB> <MSB>
     * 
     * @param {number} offsetDegrees - Offset in degrees (e.g., 1.5 degrees = 150 hundredths)
     */
    async setSensorOffset(offsetDegrees) {
        const offsetX100 = Math.round(offsetDegrees * 100);
        
        // Convert to 16-bit signed value
        let offsetValue = offsetX100;
        if (offsetValue < -32768) offsetValue = -32768;
        if (offsetValue > 32767) offsetValue = 32767;
        
        // Split into low byte (LSB) and high byte (MSB)
        const lsb = offsetValue & 0xFF;
        const msb = (offsetValue >> 8) & 0xFF;
        
        await this.sendCommand([0x05, lsb, msb]);
        console.log(`Joint ${this.jointNumber}: Sensor offset set to ${offsetDegrees}°`);
    }

    /**
     * Sets a kinematics parameter (Denavit-Hartenberg parameter)
     * 
     * Commands:
     *   0x09: Set kinematics d (link offset)
     *   0x0A: Set kinematics a (link length)
     *   0x0B: Set kinematics theta (joint angle)
     *   0x0C: Set kinematics alpha (link twist)
     * 
     * Format: <command> <B0> <B1> <B2> <B3>
     * Values are in hundredths of mm (for d, a) or hundredths of degree (for theta, alpha)
     * Stored as 32-bit signed integers, LSB first
     * 
     * @param {string} parameter - 'd', 'a', 'theta', or 'alpha'
     * @param {number} value - The value (will be multiplied by 100 for storage)
     * @param {string} unit - 'mm' for d/a, 'degrees' for theta/alpha
     */
    async setKinematicsParameter(parameter, value, unit) {
        // Determine which command byte to use
        let commandByte;
        if (parameter === 'd') commandByte = 0x09;
        else if (parameter === 'a') commandByte = 0x0A;
        else if (parameter === 'theta') commandByte = 0x0B;
        else if (parameter === 'alpha') commandByte = 0x0C;
        else {
            throw new Error(`Invalid kinematics parameter: ${parameter}. Use 'd', 'a', 'theta', or 'alpha'.`);
        }

        // Convert value to hundredths based on unit
        let valueX100;
        if (unit === 'mm') {
            valueX100 = Math.round(value * 100); // Convert mm to hundredths of mm
        } else if (unit === 'degrees') {
            valueX100 = Math.round(value * 100); // Convert degrees to hundredths of degrees
        } else {
            throw new Error(`Invalid unit: ${unit}. Use 'mm' or 'degrees'.`);
        }

        // Convert to 32-bit signed integer (4 bytes, LSB first)
        let value32 = valueX100;
        if (value32 < -2147483648) value32 = -2147483648;
        if (value32 > 2147483647) value32 = 2147483647;

        // Split into 4 bytes (little-endian: LSB first)
        const b0 = (value32) & 0xFF;
        const b1 = (value32 >> 8) & 0xFF;
        const b2 = (value32 >> 16) & 0xFF;
        const b3 = (value32 >> 24) & 0xFF;

        await this.sendCommand([commandByte, b0, b1, b2, b3]);
        console.log(`Joint ${this.jointNumber}: Kinematics ${parameter} set to ${value} ${unit}`);
    }

    /**
     * Reads the status packet from the PIC controller
     * 
     * Status packet (8 bytes):
     *   0: is_moving flag (1 = moving, 0 = idle)
     *   1: stall_detected flag (1 = stall, 0 = no stall)
     *   2-5: current_step_position (32-bit signed, LSB first)
     *   6-7: AS5600 angle in hundredths of degree (16-bit, LSB first)
     * 
     * @returns {Object} Status object with all the information
     */
    async readStatus() {
        try {
            // Read 8 bytes of status data
            const data = await this.readData(8);
            
            // Parse the status packet
            const isMoving = data[0] === 1;
            const stallDetected = data[1] === 1;
            
            // Read 32-bit step position (LSB first)
            const stepPosition = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
            // Convert from unsigned to signed 32-bit
            const stepPositionSigned = stepPosition | (stepPosition & 0x80000000 ? 0xFFFFFFFF << 31 : 0);
            
            // Read 16-bit signed angle (LSB first) - range is -18000 to +18000 (hundredths of degrees)
            // Combine bytes: LSB (data[6]) and MSB (data[7])
            // Buffer.readUInt16LE() reads little-endian unsigned 16-bit, which is what we need
            const angleX100 = data.readUInt16LE(6); // Read as unsigned 16-bit little-endian
            
            // Convert unsigned 16-bit to signed 16-bit using two's complement
            // In two's complement, values >= 32768 (0x8000) represent negative numbers
            // For a signed 16-bit value: -32768 to +32767
            // Formula: if value >= 32768, then signed = value - 65536
            const angleX100Signed = angleX100 >= 32768 ? angleX100 - 65536 : angleX100;
            
            // Convert from hundredths of degrees to degrees
            const angleDegrees = angleX100Signed / 100.0;
            
            // Debug logging for negative angles to help troubleshoot
            if (angleX100 >= 32768) {
                const lsb = data[6];
                const msb = data[7];
                console.log(`[DEBUG Joint ${this.jointNumber}] Negative angle detected: lsb=0x${lsb.toString(16).padStart(2,'0')} (${lsb}), msb=0x${msb.toString(16).padStart(2,'0')} (${msb}), raw=${angleX100}, signed=${angleX100Signed}, degrees=${angleDegrees.toFixed(2)}`);
            }
            
            return {
                isMoving: isMoving,
                stallDetected: stallDetected,
                stepPosition: stepPositionSigned,
                angleDegrees: angleDegrees,
                angleX100: angleX100
            };
        } catch (error) {
            console.error(`Joint ${this.jointNumber}: Failed to read status:`, error.message);
            throw error;
        }
    }

    /**
     * Requests and reads the settings packet from the PIC controller
     * 
     * Settings packet (20 bytes):
     *   0: 0xAA marker
     *   1: Stored AS5600 I2C address
     *   2-3: Stored offset in hundredths of degree (16-bit signed, LSB first)
     *   4-7: Kinematics d in hundredths of mm (32-bit signed, LSB first)
     *   8-11: Kinematics a in hundredths of mm (32-bit signed, LSB first)
     *   12-15: Kinematics theta in hundredths of degree (32-bit signed, LSB first)
     *   16-19: Kinematics alpha in hundredths of degree (32-bit signed, LSB first)
     * 
     * @returns {Object} Settings object with all the information
     */
    async readSettings() {
        try {
            // First, request the settings packet by sending command 0x06
            await this.sendCommand([0x06]);
            
            // Wait a bit for the PIC to prepare the packet
            await new Promise(resolve => setTimeout(resolve, 50));
            
            // Read 20 bytes of settings data
            const data = await this.readData(20);
            
            // Check for marker byte
            if (data[0] !== 0xAA) {
                throw new Error('Invalid settings packet marker. Expected 0xAA.');
            }
            
            // Parse the settings packet
            const sensorAddress = data[1];
            
            // Read 16-bit signed offset (LSB first)
            let offsetX100 = data[2] | (data[3] << 8);
            if (offsetX100 & 0x8000) {
                offsetX100 = offsetX100 - 65536; // Convert to signed
            }
            const offsetDegrees = offsetX100 / 100.0;
            
            // Helper function to read 32-bit signed value (LSB first)
            const readInt32 = (startIndex) => {
                let value = data[startIndex] | (data[startIndex + 1] << 8) | 
                           (data[startIndex + 2] << 16) | (data[startIndex + 3] << 24);
                // Convert to signed 32-bit
                if (value & 0x80000000) {
                    value = value - 0x100000000;
                }
                return value;
            };
            
            const kinematicsD = readInt32(4) / 100.0;  // Convert from hundredths of mm to mm
            const kinematicsA = readInt32(8) / 100.0;  // Convert from hundredths of mm to mm
            const kinematicsTheta = readInt32(12) / 100.0;  // Convert from hundredths of degree to degree
            const kinematicsAlpha = readInt32(16) / 100.0;  // Convert from hundredths of degree to degree
            
            return {
                sensorAddress: sensorAddress,
                sensorOffsetDegrees: offsetDegrees,
                kinematics: {
                    d: kinematicsD,
                    a: kinematicsA,
                    theta: kinematicsTheta,
                    alpha: kinematicsAlpha
                }
            };
        } catch (error) {
            console.error(`Joint ${this.jointNumber}: Failed to read settings:`, error.message);
            throw error;
        }
    }

    /**
     * Requests and reads the ADC packet from the PIC controller
     * 
     * ADC packet (5 bytes):
     *   0: 0xAB marker
     *   1-2: Latest RC3 ADC raw counts (16-bit, LSB first)
     *   3-4: RC3 voltage estimate in millivolts (16-bit, LSB first)
     * 
     * @returns {Object} ADC data object
     */
    async readADC() {
        try {
            // First, request the ADC packet by sending command 0x07
            await this.sendCommand([0x07]);
            
            // Wait a bit for the PIC to prepare the packet
            await new Promise(resolve => setTimeout(resolve, 50));
            
            // Read 5 bytes of ADC data
            const data = await this.readData(5);
            
            // Check for marker byte
            if (data[0] !== 0xAB) {
                throw new Error('Invalid ADC packet marker. Expected 0xAB.');
            }
            
            // Read 16-bit raw ADC value (LSB first)
            const rawADC = data[1] | (data[2] << 8);
            
            // Read 16-bit voltage in millivolts (LSB first)
            const voltageMV = data[3] | (data[4] << 8);
            const voltageV = voltageMV / 1000.0;
            
            return {
                rawADC: rawADC,
                voltageMillivolts: voltageMV,
                voltageVolts: voltageV
            };
        } catch (error) {
            console.error(`Joint ${this.jointNumber}: Failed to read ADC:`, error.message);
            throw error;
        }
    }
}

// Export the JointController class so other files can use it
module.exports = {
    JointController: JointController
};


