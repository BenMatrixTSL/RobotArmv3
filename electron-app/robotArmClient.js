/**
 * Robot Arm Client
 * 
 * This module handles communication with the Raspberry Pi server.
 * It uses WebSocket to send commands and receive status updates.
 * 
 * Simple, beginner-friendly WebSocket client.
 */

class RobotArmClient {
    constructor() {
        this.ws = null;
        this.isConnected = false;
        this.piAddress = '192.168.1.160';
        this.piPort = 8080;
        this.onStatusUpdate = null; // Callback function for status updates
        this.onConfigUpdate = null; // Callback function for configuration updates
        this.reconnectInterval = null;
        this.jointConfigs = []; // Store joint configurations
        this.isConnecting = false; // Flag to prevent multiple simultaneous connection attempts
        this.hasEverConnected = false; // Track if we've ever successfully connected
    }

    /**
     * Connects to the Raspberry Pi server
     * @param {string} address - IP address of Raspberry Pi
     * @param {number} port - Port number (default: 8080)
     */
    async connect(address, port = 8080) {
        // Prevent multiple simultaneous connection attempts
        if (this.isConnecting) {
            console.log('Connection attempt already in progress, ignoring...');
            return Promise.reject(new Error('Connection attempt already in progress'));
        }
        
        // If already connected, don't reconnect
        if (this.isConnected && this.ws && this.ws.readyState === WebSocket.OPEN) {
            console.log('Already connected to Raspberry Pi');
            return Promise.resolve();
        }
        
        this.piAddress = address;
        this.piPort = port;
        this.isConnecting = true;

        return new Promise((resolve, reject) => {
            try {
                // Create WebSocket URL
                const wsUrl = `ws://${address}:${port}`;
                console.log('Connecting to:', wsUrl);

                // Create WebSocket connection
                this.ws = new WebSocket(wsUrl);

                // Handle connection open
                this.ws.onopen = () => {
                    console.log('Connected to Raspberry Pi');
                    this.isConnected = true;
                    this.hasEverConnected = true;
                    this.isConnecting = false;
                    this.updateConnectionStatus(true);
                    this.stopReconnect();
                    resolve();
                };

                // Handle incoming messages
                this.ws.onmessage = (event) => {
                    try {
                        const data = JSON.parse(event.data);
                        this.handleMessage(data);
                    } catch (error) {
                        console.error('Error parsing message:', error);
                    }
                };

                // Handle connection error
                this.ws.onerror = (error) => {
                    console.error('WebSocket error:', error);
                    this.isConnecting = false;
                    this.updateConnectionStatus(false);
                    reject(error);
                };

                // Handle connection close
                this.ws.onclose = () => {
                    console.log('Disconnected from Raspberry Pi');
                    this.isConnected = false;
                    this.isConnecting = false;
                    this.updateConnectionStatus(false);
                    // Only start reconnecting if we had successfully connected before
                    if (this.hasEverConnected) {
                    this.startReconnect();
                    }
                };

            } catch (error) {
                console.error('Failed to connect:', error);
                this.isConnecting = false;
                reject(error);
            }
        });
    }

    /**
     * Disconnects from the Raspberry Pi server
     */
    disconnect() {
        this.stopReconnect();
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
        this.isConnected = false;
        this.updateConnectionStatus(false);
    }

    /**
     * Sends a command to the Raspberry Pi
     * @param {string} command - Command name (e.g., 'moveJoint', 'stopJoint')
     * @param {Object} data - Command data/parameters
     */
    sendCommand(command, data = {}) {
        if (!this.isConnected || !this.ws) {
            console.error('Not connected to Raspberry Pi');
            return false;
        }

        try {
            const message = {
                command: command,
                ...data
            };
            this.ws.send(JSON.stringify(message));
            return true;
        } catch (error) {
            console.error('Error sending command:', error);
            return false;
        }
    }

    /**
     * Handles incoming messages from the Raspberry Pi
     * @param {Object} data - Parsed message data
     */
    handleMessage(data) {
        // Handle status updates
        if (data.type === 'status') {
            if (this.onStatusUpdate) {
                this.onStatusUpdate(data.joints);
            }
        }

        // Handle joint configuration updates
        if (data.type === 'jointConfigs') {
            // Store joint configurations
            // For ST3215 version, we get basic servo info (count, available servos)
            // For I2C version with kinematics, we get kinematics data
            if (data.joints && data.joints.length > 0 && data.joints[0].kinematics) {
                // I2C version with kinematics data
            this.jointConfigs = data.joints.map(jointConfig => {
                if (jointConfig.available && jointConfig.kinematics) {
                    return {
                        d: jointConfig.kinematics.d,
                        a: jointConfig.kinematics.a,
                        theta: jointConfig.kinematics.theta,
                        alpha: jointConfig.kinematics.alpha
                    };
                }
                return null;
            });
            } else {
                // ST3215 version - just store the count and available joints
                this.jointConfigs = data.joints || [];
                console.log(`Joint configurations loaded: ${data.count} of ${data.total} servos discovered`);
            }
            
            // Notify callback if set
            if (this.onConfigUpdate) {
                this.onConfigUpdate(this.jointConfigs);
            }
            
            console.log('Joint configurations loaded:', this.jointConfigs);
        }

        // Handle other message types here if needed
        if (data.type === 'error') {
            console.error('Server error:', data.message);
        }
    }

    /**
     * Updates the connection status indicator in the UI
     * @param {boolean} connected - Whether we're connected
     */
    updateConnectionStatus(connected) {
        const indicator = document.getElementById('statusIndicator');
        const statusText = document.getElementById('statusText');
        
        if (indicator && statusText) {
            if (connected) {
                indicator.classList.add('connected');
                statusText.textContent = 'Connected';
            } else {
                indicator.classList.remove('connected');
                statusText.textContent = 'Disconnected';
            }
        }
    }

    /**
     * Starts automatic reconnection attempts
     * Only reconnects if we had successfully connected before
     */
    startReconnect() {
        // Only reconnect if we had a previous successful connection
        if (this.isConnected || !this.hasEverConnected) {
            return;
        }
        
        // Don't start reconnecting if already connecting
        if (this.isConnecting) {
            return;
        }

        this.stopReconnect();
        
        console.log('Attempting to reconnect in 5 seconds...');
        this.reconnectInterval = setInterval(() => {
            // Don't try to reconnect if already connected or connecting
            if (this.isConnected || this.isConnecting) {
                this.stopReconnect();
                return;
            }
            
            console.log('Reconnecting...');
            this.connect(this.piAddress, this.piPort).catch(() => {
                console.log('Reconnection failed, will try again in 5 seconds...');
            });
        }, 5000);
    }

    /**
     * Stops automatic reconnection attempts
     */
    stopReconnect() {
        if (this.reconnectInterval) {
            clearInterval(this.reconnectInterval);
            this.reconnectInterval = null;
        }
    }

    // ===== Robot Arm Command Functions =====

    /**
     * Moves a joint to a specific angle
     * @param {number} jointNumber - Joint number (1, 2, 3, etc.)
     * @param {number} angle - Target angle in degrees
     * @param {number} speed - Optional speed in step/s (default: 1500)
     */
    moveJoint(jointNumber, angle, speed = 1500) {
        // Ensure speed is a valid number
        const speedValue = (typeof speed === 'number' && !isNaN(speed) && speed >= 0) ? speed : 1500;
        return this.sendCommand('moveJoint', {
            joint: jointNumber,
            angle: angle,
            speed: speedValue
        });
    }

    /**
     * Sets the movement speed for a specific joint
     * @param {number} jointNumber - Joint number (1, 2, 3, etc.)
     * @param {number} speed - Speed in step/s (0-3400)
     */
    setSpeed(jointNumber, speed) {
        return this.sendCommand('setSpeed', {
            joint: jointNumber,
            speed: speed
        });
    }

    /**
     * Sets the movement speed for all joints
     * @param {number} speed - Speed in step/s (0-3400)
     */
    setSpeedAll(speed) {
        return this.sendCommand('setSpeedAll', {
            speed: speed
        });
    }

    /**
     * Sets the acceleration for a specific joint
     * @param {number} jointNumber - Joint number (1, 2, 3, etc.)
     * @param {number} acceleration - Acceleration value (0-254, unit: 100 step/s²)
     */
    setAcceleration(jointNumber, acceleration) {
        // Ensure acceleration is a valid number and within range
        const accValue = (typeof acceleration === 'number' && !isNaN(acceleration) && acceleration >= 0) 
            ? Math.max(0, Math.min(254, Math.round(acceleration))) 
            : 50; // Default acceleration
        return this.sendCommand('setAcceleration', {
            joint: jointNumber,
            acceleration: accValue
        });
    }

    /**
     * Enables or disables torque for all joints
     * @param {boolean} enabled - True to enable torque, false to disable
     */
    setTorqueAll(enabled) {
        return this.sendCommand('setTorqueAll', {
            enabled: enabled
        });
    }

    /**
     * Stops motion on a specific joint
     * @param {number} jointNumber - Joint number (1, 2, 3, etc.)
     */
    stopJoint(jointNumber) {
        return this.sendCommand('stopJoint', {
            joint: jointNumber
        });
    }

    /**
     * Stops all joints immediately
     */
    stopAllJoints() {
        return this.sendCommand('stopAll');
    }

    /**
     * Sets servo angle on a joint
     * @param {number} jointNumber - Joint number
     * @param {number} angle - Servo angle (0-180)
     */
    setServoAngle(jointNumber, angle) {
        return this.sendCommand('setServo', {
            joint: jointNumber,
            angle: angle
        });
    }

    /**
     * Requests status update from all joints
     */
    requestStatus() {
        return this.sendCommand('getStatus');
    }

    /**
     * Gets the current status from all joints and returns it as a promise
     * This is useful for waiting for status in Blockly programs
     * @returns {Promise<Array>} Promise that resolves with array of joint status objects
     */
    async getStatus() {
        return new Promise((resolve, reject) => {
            if (!this.isConnected) {
                reject(new Error('Not connected to server'));
                return;
            }

            // Set up a one-time listener for the status response
            // We need to intercept the message handler temporarily to capture the response
            const originalHandleMessage = this.handleMessage.bind(this);
            let statusResolved = false;
            
            const timeout = setTimeout(() => {
                if (!statusResolved) {
                    statusResolved = true;
                    this.handleMessage = originalHandleMessage;
                    reject(new Error('Status request timeout'));
                }
            }, 3000); // 3 second timeout

            // Temporarily override handleMessage to capture the status response
            this.handleMessage = (data) => {
                // If this is a status response, resolve the promise
                if (data.type === 'status' && !statusResolved) {
                    statusResolved = true;
                    clearTimeout(timeout);
                    this.handleMessage = originalHandleMessage;
                    resolve(data.joints || []);
                    return;
                }
                
                // For all other messages, call the original handler
                originalHandleMessage(data);
            };

            // Request status
            this.requestStatus();
        });
    }

    /**
     * Requests joint configurations (kinematics parameters) from all joints
     */
    requestJointConfigs() {
        return this.sendCommand('getJointConfigs');
    }

    /**
     * Gets stored joint configurations
     * @returns {Array} Array of joint configurations
     */
    getJointConfigs() {
        return this.jointConfigs;
    }

    /**
     * Sets a kinematics parameter for a joint
     * @param {number} jointNumber - Joint number (1, 2, 3, etc.)
     * @param {string} parameter - 'd', 'a', 'theta', or 'alpha'
     * @param {number} value - Parameter value
     * @param {string} unit - 'mm' for d/a, 'degrees' for theta/alpha
     */
    setKinematics(jointNumber, parameter, value, unit) {
        return this.sendCommand('setKinematics', {
            joint: jointNumber,
            parameter: parameter,
            value: value,
            unit: unit
        });
    }

    /**
     * Sets the AS5600 sensor offset for a joint
     * @param {number} jointNumber - Joint number (1, 2, 3, etc.)
     * @param {number} offsetDegrees - Offset in degrees
     */
    setSensorOffset(jointNumber, offsetDegrees) {
        return this.sendCommand('setSensorOffset', {
            joint: jointNumber,
            offset: offsetDegrees
        });
    }
}

// Create a global instance
const robotArmClient = new RobotArmClient();

