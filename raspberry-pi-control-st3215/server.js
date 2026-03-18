/**
 * Robot Arm WebSocket Server (ST3215 Version)
 * 
 * This server runs on the Raspberry Pi and connects the Electron desktop app
 * to the ST3215 serial bus servo motors. It receives commands from the Electron app
 * and translates them to serial commands for the ST3215 servos.
 * 
 * Usage:
 *   node server.js
 * 
 * This server listens on port 8080 by default and communicates with:
 * - Electron desktop app (via WebSocket)
 * - ST3215 servos (via Serial/UART)
 */

const WebSocket = require('ws');
const RobotArm = require('./robotArmST3215');

// Configuration
const PORT = 8080;
const JOINT_COUNT = 6; // Number of robot arm joints (ST3215 servos)
const SERIAL_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB0158625-if00'; // Serial port path (adjust as needed)
const SERIAL_BAUDRATE = 1000000; // ST3215 default baud rate

// Debug flag - set to true to enable verbose debug messages
const DEBUG = false;
// Simple performance logging flag (set to true to see timing info)
const PERF_DEBUG = false;

// Array to store servo controllers
const servos = [];

/**
 * Shared serial port instance (all servos use the same port)
 */
let sharedSerialPort = null;

/**
 * Array to store all servo controllers for data routing
 * This is used by the shared serial port data handler
 */
let allServoControllers = [];

/**
 * Write queue for serializing writes to the shared serial port
 * This ensures only one write happens at a time
 */
let writeQueue = [];
let isWriting = false;

/**
 * Global command queue for serializing ALL commands across all clients and servos
 * This ensures only one command is processed at a time, preventing conflicts
 */
let commandQueue = [];
let isProcessingCommand = false;
const MAX_COMMAND_QUEUE_SIZE = 100; // Maximum queue size (status polls + moves)

/**
 * Queue a write operation to the shared serial port
 * @param {Function} writeFn - Function that performs the write
 * @returns {Promise} Promise that resolves when write completes
 */
async function queueWrite(writeFn) {
    return new Promise((resolve, reject) => {
        writeQueue.push({ writeFn, resolve, reject });
        processWriteQueue();
    });
}

/**
 * Process the write queue (one write at a time)
 */
async function processWriteQueue() {
    if (isWriting || writeQueue.length === 0) {
        return;
    }
    
    isWriting = true;
    const { writeFn, resolve, reject } = writeQueue.shift();
    
    try {
        await writeFn();
        resolve();
    } catch (error) {
        reject(error);
    } finally {
        isWriting = false;
        // Process next item in queue
        processWriteQueue();
    }
}

/**
 * Queue a command to be processed (ensures only one command at a time across all clients)
 * @param {Function} commandFn - Function that executes the command
 * @returns {Promise} Promise that resolves when command completes
 */
async function queueCommand(commandFn) {
    return new Promise((resolve, reject) => {
        // Prevent queue from growing too large (could indicate a problem)
        if (commandQueue.length >= MAX_COMMAND_QUEUE_SIZE) {
            console.warn(`Command queue full (${commandQueue.length} items), rejecting new command`);
            reject(new Error('Command queue is full, server may be overloaded'));
            return;
        }
        commandQueue.push({ commandFn, resolve, reject });
        processCommandQueue();
    });
}

/**
 * Process the command queue (one command at a time)
 */
async function processCommandQueue() {
    if (isProcessingCommand || commandQueue.length === 0) {
        return;
    }
    
    isProcessingCommand = true;
    const { commandFn, resolve, reject } = commandQueue.shift();
    
    try {
        const result = await commandFn();
        // Reduced delay after command completes - only 5ms instead of 10ms
        // This helps prevent response conflicts while reducing accumulated lag
        await new Promise(resolve => setTimeout(resolve, 5));
        resolve(result);
    } catch (error) {
        reject(error);
    } finally {
        isProcessingCommand = false;
        // Process next command in queue
        processCommandQueue();
    }
}

/**
 * Initialize all servo controllers
 */
async function initializeServos() {
    console.log('Initializing ST3215 servo controllers...');
    
    // ST3215 servo IDs (1-6 for 6 servos)
    // Each servo must have a unique ID configured
    const servoIds = [1, 2, 3, 4, 5, 6];
    
    if (servoIds.length < JOINT_COUNT) {
        console.error(`Error: Need ${JOINT_COUNT} servo IDs but only ${servoIds.length} provided`);
        console.error('Please configure more servo IDs in the servoIds array');
        process.exit(1);
    }
    
    // Create a single shared serial port for all servos (they're daisy-chained)
    const { SerialPort } = require('serialport');
    
    try {
        console.log('Opening shared serial port...');
        sharedSerialPort = new SerialPort({
            path: SERIAL_PORT,
            baudRate: SERIAL_BAUDRATE,
            dataBits: 8,
            parity: 'none',
            stopBits: 1,
            autoOpen: false
        });
        
        // Open the shared port
        await new Promise((resolve, reject) => {
            sharedSerialPort.open((error) => {
                if (error) {
                    console.error('Failed to open shared serial port:', error.message);
                    reject(error);
                } else {
                    console.log('✓ Shared serial port opened successfully');
                    resolve();
                }
            });
        });
        
        // Handle incoming data from all servos
        // We'll set up a single data handler that routes to all servo controllers
        sharedSerialPort.on('data', (data) => {
            // Route data to all servo controllers - they'll filter by ID
            allServoControllers.forEach(servo => {
                if (servo && servo.handleIncomingData) {
                    servo.handleIncomingData(data);
                }
            });
        });
        
        sharedSerialPort.on('error', (error) => {
            console.error('Shared serial port error:', error.message);
        });
        
        // Attach write queue function to shared port for servo controllers to use
        sharedSerialPort._writeQueue = queueWrite;
        
    } catch (error) {
        console.error('Failed to initialize shared serial port:', error.message);
        process.exit(1);
    }
    
    // Create servo controllers, all sharing the same serial port
    for (let i = 0; i < JOINT_COUNT; i++) {
        // Pass the shared SerialPort instance instead of the path
        const servo = new RobotArm.ServoController(i + 1, sharedSerialPort, servoIds[i], SERIAL_BAUDRATE);
        
        try {
            // This will set up the data handler but won't try to open the port
            await servo.open();
            
            // Add servo to the list for data routing
            allServoControllers.push(servo);
            
            // Add a delay between servo initializations to avoid simultaneous writes
            if (i > 0) {
                await new Promise(resolve => setTimeout(resolve, 150));
            }
            
            // First, try to ping the servo to verify communication
            console.log(`Pinging servo ${i + 1} (ST3215 ID: ${servoIds[i]})...`);
            const pingResult = await servo.ping();
            if (!pingResult) {
                console.log(`⚠️  Servo ${i + 1} (ID: ${servoIds[i]}) did not respond to ping - skipping`);
                servos.push(null);
                continue;
            }
            console.log(`✓ Servo ${i + 1} (ID: ${servoIds[i]}) responded to ping`);
            
            // Small delay after ping
            await new Promise(resolve => setTimeout(resolve, 50));
            
            // Enable torque (start servo)
            await servo.startServo();
            
            servos.push(servo);
            console.log(`Servo ${i + 1} initialized (ST3215 ID: ${servoIds[i]})`);
        } catch (error) {
            console.error(`Failed to initialize servo ${i + 1} (ST3215 ID: ${servoIds[i]}):`, error.message);
            // Create a placeholder so the array stays aligned
            servos.push(null);
        }
    }
    
    console.log(`Initialized ${servos.filter(s => s !== null).length} of ${JOINT_COUNT} servos`);
}

/**
 * Get status from all servos
 * 
 * IMPORTANT: Read servos one-by-one on the shared serial bus.
 * This is simpler and more reliable than doing all reads in parallel.
 */
async function getAllServoStatus() {
    const statuses = [];
    const startAll = Date.now();

    for (let i = 0; i < servos.length; i++) {
        const servo = servos[i];

        if (servo === null) {
            // Servo not available - push default status
            statuses.push({
                joint: i + 1,
                available: false,
                isMoving: false,
                angleDegrees: 0,
                position: 0,
                stepPosition: 0,
                speed: 0,
                load: 0,
                voltage: 0,
                temperature: 0,
                torqueEnabled: false
            });
            continue;
        }

        const startServo = Date.now();
        try {
            // Fast path: one bulk read per servo (angle, speed, load, voltage, temp, moving, torque)
            const status = await servo.readQuickStatus();
            statuses.push({
                joint: i + 1,
                available: true,
                ...status,
                stepPosition: status.position
            });
        } catch (error) {
            console.error(`Error reading status from servo ${i + 1}:`, error.message);
            statuses.push({
                joint: i + 1,
                available: false,
                isMoving: false,
                angleDegrees: 0,
                position: 0,
                stepPosition: 0,
                speed: 0,
                load: 0,
                voltage: 0,
                temperature: 0,
                torqueEnabled: false,
                error: error.message
            });
        }

        const servoDuration = Date.now() - startServo;
        if (PERF_DEBUG && servoDuration > 50) {
            console.log(`PERF: readStatus for servo ${i + 1} took ${servoDuration} ms`);
        }
    }

    const totalDuration = Date.now() - startAll;
    if (PERF_DEBUG) {
        console.log(`PERF: getAllServoStatus for ${servos.length} servos took ${totalDuration} ms`);
    }

    // Already in joint order
    return statuses;
}

/**
 * Start the WebSocket server
 */
function startServer() {
    console.log(`Starting WebSocket server on port ${PORT}...`);
    
    // Create WebSocket server
    const wss = new WebSocket.Server({ port: PORT });
    
    console.log(`Server listening on port ${PORT}`);
    console.log('Waiting for clients to connect...');
    
    // Handle new client connections
    wss.on('connection', function connection(ws, req) {
        const clientIp = req.socket.remoteAddress;
        console.log(`Client connected from ${clientIp}`);
        
        // Send welcome message
        ws.send(JSON.stringify({
            type: 'connected',
            message: 'Connected to Robot Arm Server (ST3215)'
        }));
        
        // Handle incoming messages from client
        ws.on('message', async function incoming(message) {
            try {
                const data = JSON.parse(message);
                // Queue the command to ensure only one command is processed at a time
                await queueCommand(async () => {
                    await handleCommand(ws, data);
                });
            } catch (error) {
                console.error('Error handling message:', error);
                ws.send(JSON.stringify({
                    type: 'error',
                    message: error.message
                }));
            }
        });
        
        // Handle client disconnect
        ws.on('close', function() {
            console.log(`Client disconnected from ${clientIp}`);
        });
        
        // Handle errors
        ws.on('error', function(error) {
            console.error('WebSocket error:', error);
        });
    });
}

/**
 * Handle commands from the Electron app
 */
async function handleCommand(ws, data) {
    const command = data.command;
    
    // Handle different commands
    switch (command) {
        case 'getJointConfigs':
            // Return the number of servos discovered and their basic configuration
            const discoveredServos = servos.filter(s => s !== null).length;
            const jointConfigs = [];
            
            // Create config for each discovered servo
            for (let i = 0; i < servos.length; i++) {
                if (servos[i] !== null) {
                    jointConfigs.push({
                        jointNumber: i + 1,
                        servoId: servos[i].servoIdNumber,
                        available: true
                    });
                }
            }
            
            ws.send(JSON.stringify({
                type: 'jointConfigs',
                count: discoveredServos,
                total: servos.length,
                joints: jointConfigs
            }));
            break;
            
        case 'getStatus':
            // Send status of all servos
            const statuses = await getAllServoStatus();
            ws.send(JSON.stringify({
                type: 'status',
                joints: statuses
            }));
            break;
            
        case 'moveJoint':
            // Move a servo to a specific angle
            const jointNumber = data.joint - 1; // Convert to 0-based index
            const angle = data.angle;
            // Ensure speed is a valid number (default to 1500 if not provided or invalid)
            const moveSpeed = (typeof data.speed === 'number' && !isNaN(data.speed) && data.speed >= 0) ? data.speed : 1500;
            
            if (jointNumber < 0 || jointNumber >= servos.length) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                }));
                return;
            }
            
            const servo = servos[jointNumber];
            if (servo === null) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Servo ${data.joint} is not available`
                }));
                return;
            }
            
            try {
                await servo.moveToAngle(angle, moveSpeed);
                ws.send(JSON.stringify({
                    type: 'success',
                    message: `Servo ${data.joint} moving to ${angle}° at ${moveSpeed} step/s`
                }));
            } catch (error) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Failed to move servo ${data.joint}: ${error.message}`
                }));
            }
            break;
            
        case 'stopJoint':
            // Stop a specific servo
            const stopJointNumber = data.joint - 1;
            
            if (stopJointNumber < 0 || stopJointNumber >= servos.length) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                }));
                return;
            }
            
            const stopServo = servos[stopJointNumber];
            if (stopServo === null) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Servo ${data.joint} is not available`
                }));
                return;
            }
            
            try {
                await stopServo.stopServo();
                ws.send(JSON.stringify({
                    type: 'success',
                    message: `Servo ${data.joint} stopped`
                }));
            } catch (error) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Failed to stop servo ${data.joint}: ${error.message}`
                }));
            }
            break;
            
        case 'stopAllJoints':
            // Stop all servos
            try {
                for (let i = 0; i < servos.length; i++) {
                    if (servos[i] !== null) {
                        await servos[i].stopServo();
                    }
                }
                ws.send(JSON.stringify({
                    type: 'success',
                    message: 'All servos stopped'
                }));
            } catch (error) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Failed to stop all servos: ${error.message}`
                }));
            }
            break;
            
        case 'setServoAngle':
            // Set servo angle (for gripper or other servo-controlled joints)
            const servoJointNumber = data.joint - 1;
            const servoAngle = data.angle;
            
            if (servoJointNumber < 0 || servoJointNumber >= servos.length) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                }));
                return;
            }
            
            const servoJoint = servos[servoJointNumber];
            if (servoJoint === null) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Servo ${data.joint} is not available`
                }));
                return;
            }
            
            try {
                await servoJoint.moveToAngle(servoAngle);
                ws.send(JSON.stringify({
                    type: 'success',
                    message: `Servo ${data.joint} set to ${servoAngle}°`
                }));
            } catch (error) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Failed to set servo angle: ${error.message}`
                }));
            }
            break;
            
        case 'setSpeed':
            // Set servo speed
            const speedJointNumber = data.joint - 1;
            const speed = data.speed;
            
            if (speedJointNumber < 0 || speedJointNumber >= servos.length) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                }));
                return;
            }
            
            const speedServo = servos[speedJointNumber];
            if (speedServo === null) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Servo ${data.joint} is not available`
                }));
                return;
            }
            
            try {
                await speedServo.setSpeed(speed);
                ws.send(JSON.stringify({
                    type: 'success',
                    message: `Servo ${data.joint} speed set to ${speed} step/s`
                }));
            } catch (error) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Failed to set speed: ${error.message}`
                }));
            }
            break;
            
        case 'setSpeedAll':
            // Set servo speed for all joints
            const speedAll = data.speed;
            
            try {
                for (let i = 0; i < servos.length; i++) {
                    if (servos[i] !== null) {
                        await servos[i].setSpeed(speedAll);
                    }
                }
                ws.send(JSON.stringify({
                    type: 'success',
                    message: `All servos speed set to ${speedAll} step/s`
                }));
            } catch (error) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Failed to set servo speeds: ${error.message}`
                }));
            }
            break;
            
        case 'setTorqueAll':
            // Enable or disable torque for all joints
            const torqueEnabled = data.enabled !== false; // Default to true if not specified
            
            try {
                for (let i = 0; i < servos.length; i++) {
                    if (servos[i] !== null) {
                        if (torqueEnabled) {
                            await servos[i].startServo();
                        } else {
                            await servos[i].stopServo();
                        }
                    }
                }
                ws.send(JSON.stringify({
                    type: 'success',
                    message: `All servos torque ${torqueEnabled ? 'enabled' : 'disabled'}`
                }));
            } catch (error) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Failed to ${torqueEnabled ? 'enable' : 'disable'} torque: ${error.message}`
                }));
            }
            break;
            
        case 'setAcceleration':
            // Set servo acceleration
            const accJointNumber = data.joint - 1;
            const acc = data.acceleration;
            
            if (accJointNumber < 0 || accJointNumber >= servos.length) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                }));
                return;
            }
            
            const accServo = servos[accJointNumber];
            if (accServo === null) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Servo ${data.joint} is not available`
                }));
                return;
            }
            
            try {
                await accServo.setAcceleration(acc);
                ws.send(JSON.stringify({
                    type: 'success',
                    message: `Servo ${data.joint} acceleration set to ${acc}`
                }));
            } catch (error) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Failed to set acceleration: ${error.message}`
                }));
            }
            break;
            
        default:
            ws.send(JSON.stringify({
                type: 'error',
                message: `Unknown command: ${command}`
            }));
    }
}

/**
 * Cleanup function - called on exit
 */
async function cleanup() {
    console.log('Shutting down...');
    
    // Stop all servos
    for (let i = 0; i < servos.length; i++) {
        if (servos[i] !== null) {
            try {
                await servos[i].stopServo();
                await servos[i].close();
            } catch (error) {
                console.error(`Error closing servo ${i + 1}:`, error.message);
            }
        }
    }
    
    // Close shared serial port
    if (sharedSerialPort && sharedSerialPort.isOpen) {
        await new Promise((resolve) => {
            sharedSerialPort.close((error) => {
                if (error) {
                    console.error('Error closing shared serial port:', error.message);
                } else {
                    console.log('Shared serial port closed');
                }
                resolve();
            });
        });
    }
    
    process.exit(0);
}

// Handle process termination
process.on('SIGINT', cleanup);
process.on('SIGTERM', cleanup);

// Start the server
async function main() {
    try {
        // Initialize servos
        await initializeServos();
        
        // Start WebSocket server
        startServer();
    } catch (error) {
        console.error('Failed to start server:', error);
        process.exit(1);
    }
}

// Run the main function
main();

