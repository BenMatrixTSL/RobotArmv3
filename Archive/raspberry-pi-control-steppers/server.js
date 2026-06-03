/**
 * Robot Arm WebSocket Server
 * 
 * This server runs on the Raspberry Pi and connects the Electron desktop app
 * to the I2C controllers. It receives commands from the Electron app and
 * translates them to I2C commands for the PIC controllers.
 * 
 * Usage:
 *   node server.js
 * 
 * This server listens on port 8080 by default and communicates with:
 * - Electron desktop app (via WebSocket)
 * - PIC controllers (via I2C)
 */

const WebSocket = require('ws');
const RobotArm = require('./robotArmI2C');

// Configuration
const PORT = 8080;
const JOINT_COUNT = 2; // Number of robot arm joints

// Array to store joint controllers
const joints = [];

/**
 * Initialize all joint controllers
 */
async function initializeJoints() {
    console.log('Initializing joint controllers...');
    
    // Create controllers for each joint
    // Adjust I2C addresses as needed (default is 0x22)
    // Each joint must have a unique I2C address
    const addresses = [0x22, 0x23]; // Add more addresses as needed: 0x24, 0x25, etc.
    
    // Make sure we have enough addresses for all joints
    if (addresses.length < JOINT_COUNT) {
        console.error(`Error: Need ${JOINT_COUNT} I2C addresses but only ${addresses.length} provided`);
        console.error('Please add more addresses to the addresses array');
        process.exit(1);
    }
    
    for (let i = 0; i < JOINT_COUNT; i++) {
        const joint = new RobotArm.JointController(i + 1, '/dev/i2c-3', addresses[i]);
        
        try {
            await joint.open();
            joints.push(joint);
            console.log(`Joint ${i + 1} initialized at address 0x${addresses[i].toString(16)}`);
        } catch (error) {
            console.error(`Failed to initialize joint ${i + 1} at address 0x${addresses[i].toString(16)}:`, error.message);
            // Create a placeholder so the array stays aligned
            joints.push(null);
        }
    }
    
    console.log(`Initialized ${joints.filter(j => j !== null).length} of ${JOINT_COUNT} joints`);
}

/**
 * Get status from all joints
 */
async function getAllJointStatus() {
    const statuses = [];
    
    for (let i = 0; i < joints.length; i++) {
        const joint = joints[i];
        
        if (joint === null) {
            // Joint not available
            statuses.push({
                joint: i + 1,
                available: false,
                isMoving: false,
                stallDetected: false,
                stepPosition: 0,
                angleDegrees: 0
            });
        } else {
            try {
                const status = await joint.readStatus();
                statuses.push({
                    joint: i + 1,
                    available: true,
                    ...status
                });
            } catch (error) {
                console.error(`Error reading status from joint ${i + 1}:`, error.message);
                statuses.push({
                    joint: i + 1,
                    available: false,
                    error: error.message
                });
            }
        }
    }
    
    return statuses;
}

/**
 * Get configuration (kinematics parameters) from all joints
 */
async function getAllJointConfigurations() {
    const configs = [];
    
    for (let i = 0; i < joints.length; i++) {
        const joint = joints[i];
        
        if (joint === null) {
            // Joint not available
            configs.push({
                joint: i + 1,
                available: false,
                kinematics: null
            });
        } else {
            try {
                const settings = await joint.readSettings();
                configs.push({
                    joint: i + 1,
                    available: true,
                    kinematics: settings.kinematics,
                    sensorAddress: settings.sensorAddress,
                    sensorOffset: settings.sensorOffsetDegrees
                });
            } catch (error) {
                console.error(`Error reading configuration from joint ${i + 1}:`, error.message);
                configs.push({
                    joint: i + 1,
                    available: false,
                    error: error.message,
                    kinematics: null
                });
            }
        }
    }
    
    return configs;
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
            message: 'Connected to Robot Arm Server'
        }));
        
        // Handle incoming messages from client
        ws.on('message', async function incoming(message) {
            try {
                const data = JSON.parse(message);
                await handleCommand(ws, data);
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
        case 'getStatus':
            // Send status of all joints
            const statuses = await getAllJointStatus();
            ws.send(JSON.stringify({
                type: 'status',
                joints: statuses
            }));
            break;
            
        case 'getJointConfigs':
            // Send configuration (kinematics) of all joints
            const configs = await getAllJointConfigurations();
            ws.send(JSON.stringify({
                type: 'jointConfigs',
                joints: configs
            }));
            break;
            
        case 'moveJoint':
            // Move a joint to a specific angle
            const jointNumber = data.joint - 1; // Convert to 0-based index
            const angle = data.angle;
            
            if (jointNumber < 0 || jointNumber >= joints.length) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                }));
                return;
            }
            
            const joint = joints[jointNumber];
            if (joint === null) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Joint ${data.joint} is not available`
                }));
                return;
            }
            
            try {
                await joint.moveToAngle(angle);
                ws.send(JSON.stringify({
                    type: 'success',
                    message: `Joint ${data.joint} moving to ${angle}°`
                }));
            } catch (error) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Failed to move joint ${data.joint}: ${error.message}`
                }));
            }
            break;
            
        case 'stopJoint':
            // Stop a specific joint
            const stopJointNumber = data.joint - 1;
            
            if (stopJointNumber < 0 || stopJointNumber >= joints.length) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                }));
                return;
            }
            
            const stopJoint = joints[stopJointNumber];
            if (stopJoint === null) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Joint ${data.joint} is not available`
                }));
                return;
            }
            
            try {
                await stopJoint.stopMotion();
                ws.send(JSON.stringify({
                    type: 'success',
                    message: `Joint ${data.joint} stopped`
                }));
            } catch (error) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Failed to stop joint ${data.joint}: ${error.message}`
                }));
            }
            break;
            
        case 'stopAll':
            // Stop all joints
            console.log('Stopping all joints...');
            const stopPromises = [];
            
            for (let i = 0; i < joints.length; i++) {
                const j = joints[i];
                if (j !== null) {
                    stopPromises.push(j.stopMotion().catch(err => {
                        console.error(`Error stopping joint ${i + 1}:`, err);
                    }));
                }
            }
            
            await Promise.all(stopPromises);
            ws.send(JSON.stringify({
                type: 'success',
                message: 'All joints stopped'
            }));
            break;
            
        case 'setServo':
            // Set servo angle on a joint
            const servoJointNumber = data.joint - 1;
            const servoAngle = data.angle;
            
            if (servoJointNumber < 0 || servoJointNumber >= joints.length) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                }));
                return;
            }
            
            const servoJoint = joints[servoJointNumber];
            if (servoJoint === null) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Joint ${data.joint} is not available`
                }));
                return;
            }
            
            try {
                await servoJoint.setServoAngle(servoAngle);
                ws.send(JSON.stringify({
                    type: 'success',
                    message: `Joint ${data.joint} servo set to ${servoAngle}°`
                }));
            } catch (error) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Failed to set servo: ${error.message}`
                }));
            }
            break;
            
        case 'setKinematics':
            // Set kinematics parameter for a joint
            const kinJointNumber = data.joint - 1;
            const parameter = data.parameter; // 'd', 'a', 'theta', or 'alpha'
            const value = data.value;
            const unit = data.unit; // 'mm' or 'degrees'
            
            if (kinJointNumber < 0 || kinJointNumber >= joints.length) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                }));
                return;
            }
            
            const kinJoint = joints[kinJointNumber];
            if (kinJoint === null) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Joint ${data.joint} is not available`
                }));
                return;
            }
            
            try {
                await kinJoint.setKinematicsParameter(parameter, value, unit);
                ws.send(JSON.stringify({
                    type: 'success',
                    message: `Joint ${data.joint} kinematics ${parameter} set to ${value} ${unit}`
                }));
            } catch (error) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Failed to set kinematics: ${error.message}`
                }));
            }
            break;
            
        case 'setSensorOffset':
            // Set AS5600 sensor offset for a joint
            const offsetJointNumber = data.joint - 1;
            const offsetDegrees = data.offset;
            
            if (offsetJointNumber < 0 || offsetJointNumber >= joints.length) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                }));
                return;
            }
            
            const offsetJoint = joints[offsetJointNumber];
            if (offsetJoint === null) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Joint ${data.joint} is not available`
                }));
                return;
            }
            
            try {
                await offsetJoint.setSensorOffset(offsetDegrees);
                ws.send(JSON.stringify({
                    type: 'success',
                    message: `Joint ${data.joint} sensor offset set to ${offsetDegrees}°`
                }));
            } catch (error) {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: `Failed to set sensor offset: ${error.message}`
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
 * Cleanup function - closes all joints and exits
 */
async function cleanup() {
    console.log('\nShutting down...');
    
    // Close all joint connections
    for (let i = 0; i < joints.length; i++) {
        const joint = joints[i];
        if (joint !== null) {
            try {
                await joint.close();
            } catch (error) {
                console.error(`Error closing joint ${i + 1}:`, error);
            }
        }
    }
    
    process.exit(0);
}

// Handle Ctrl+C gracefully
process.on('SIGINT', cleanup);
process.on('SIGTERM', cleanup);

// Start the server
async function main() {
    try {
        // Initialize all joints
        await initializeJoints();
        
        // Start WebSocket server
        startServer();
        
        console.log('\nServer is running. Press Ctrl+C to stop.\n');
    } catch (error) {
        console.error('Failed to start server:', error);
        process.exit(1);
    }
}

// Run the server
main();

