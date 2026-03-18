/**
 * G-Code Processor
 * 
 * This module parses and processes G-code files.
 * It's simple and handles basic G-code commands (G0, G1, etc.)
 * 
 * Beginner-friendly G-code parser.
 */

class GCodeProcessor {
    constructor() {
        this.lines = [];
        this.commands = [];
        this.currentLineIndex = 0;
        this.isRunning = false;
        this.isPaused = false;
        this.onProgress = null; // Callback for progress updates
        this.onLog = null; // Callback for logging
        this.onLineChange = null; // Callback for current line changes
    }

    /**
     * Loads and parses a G-code file
     * @param {string} content - G-code file content
     */
    loadGCode(content) {
        // Reset state
        this.lines = [];
        this.commands = [];
        this.currentLineIndex = 0;

        // Split into lines and clean up
        const rawLines = content.split('\n');
        
        // Process each line
        for (let i = 0; i < rawLines.length; i++) {
            let line = rawLines[i].trim();
            
            // Skip empty lines
            if (line.length === 0) {
                continue;
            }

            // Remove comments (everything after ; or ())
            line = line.split(';')[0].trim();
            line = line.split('(')[0].trim();
            
            // Skip if line is now empty
            if (line.length === 0) {
                continue;
            }

            // Store the line
            this.lines.push(line);
            
            // Parse the command
            const command = this.parseLine(line);
            if (command) {
                this.commands.push(command);
            }
        }

        console.log(`Loaded ${this.lines.length} lines, ${this.commands.length} commands`);
        return {
            lines: this.lines.length,
            commands: this.commands.length
        };
    }

    /**
     * Parses a single G-code line into a command object
     * @param {string} line - G-code line to parse
     * @returns {Object|null} Parsed command object or null
     */
    parseLine(line) {
        // Match G-code commands (G0, G1, M codes, J codes, etc.)
        // Also match joint commands like J1=45 or standalone J1
        let match = line.match(/^([GM]\d+)/i);
        let commandCode = null;
        
        if (match) {
            commandCode = match[1].toUpperCase();
        } else {
            // Check for joint commands (J1, J2, etc.) or joint parameters
            match = line.match(/^(J\d+)/i);
            if (match) {
                commandCode = match[1].toUpperCase();
            } else {
                // Not a recognized command, return null
                return null;
            }
        }
        
        // Extract parameters (X, Y, Z, J1, J2, etc.)
        const params = {};
        
        // First, check for P parameter with string value (position name): P"Home" or P'Home'
        const pStringMatch = line.match(/P\s*["']([^"']+)["']/i);
        if (pStringMatch) {
            params.P = pStringMatch[1]; // Store as string
        }
        
        // Match standard parameters: X, Y, Z, I, J, K, F, R, S, P
        // Also match joint parameters: J1, J2, J3, etc. (with number)
        const paramPattern = /([XYZIJKFRSP])([-+]?\d*\.?\d+)/gi;
        let paramMatch;
        
        while ((paramMatch = paramPattern.exec(line)) !== null) {
            const letter = paramMatch[1].toUpperCase();
            const value = parseFloat(paramMatch[2]);
            
            // Skip P if we already found it as a string
            if (letter === 'P' && typeof params.P === 'string') {
                continue;
            }
            
            // For joint commands like J1=45, we need to handle J1, J2, etc. as separate parameters
            // Check if this is part of a joint parameter (J1, J2, etc.)
            const jointMatch = line.match(new RegExp(`(${letter}\\d+)\\s*=\\s*([-+]?\\d*\\.?\\d+)`, 'i'));
            if (jointMatch) {
                // This is a joint parameter like J1=45
                const jointParam = jointMatch[1].toUpperCase();
                const jointValue = parseFloat(jointMatch[2]);
                params[jointParam] = jointValue;
            } else {
                // Standard parameter
                params[letter] = value;
            }
        }
        
        // Also check for joint parameters in format J1=value J2=value (with equals sign)
        const jointParamPattern = /(J\d+)\s*=\s*([-+]?\d*\.?\d+)/gi;
        let jointParamMatch;
        while ((jointParamMatch = jointParamPattern.exec(line)) !== null) {
            const jointParam = jointParamMatch[1].toUpperCase();
            const jointValue = parseFloat(jointParamMatch[2]);
            params[jointParam] = jointValue;
        }

        return {
            code: commandCode,
            line: line,
            params: params
        };
    }

    /**
     * Gets all parsed commands
     * @returns {Array} Array of command objects
     */
    getCommands() {
        return this.commands;
    }

    /**
     * Gets the raw lines
     * @returns {Array} Array of G-code lines
     */
    getLines() {
        return this.lines;
    }

    /**
     * Starts executing the G-code
     * @param {Function} executeCommand - Function to execute each command
     */
    async start(executeCommand) {
        if (this.isRunning) {
            console.log('G-code is already running');
            return;
        }

        this.isRunning = true;
        this.isPaused = false;
        this.currentLineIndex = 0;

        this.log('Starting G-code execution...');

        // Execute each command
        for (let i = 0; i < this.commands.length; i++) {
            // Check if we should stop
            if (!this.isRunning) {
                this.log('Execution stopped');
                break;
            }

            // Check if paused
            while (this.isPaused && this.isRunning) {
                await new Promise(resolve => setTimeout(resolve, 100));
            }

            if (!this.isRunning) {
                break;
            }

            // Get the command first (needed for line change notification)
            const command = this.commands[i];

            // Update current line
            this.currentLineIndex = i;

            // Notify about line change
            if (this.onLineChange) {
                this.onLineChange(i, command.line);
            }

            // Update progress
            const progress = Math.round(((i + 1) / this.commands.length) * 100);
            if (this.onProgress) {
                this.onProgress(progress, i + 1, this.commands.length);
            }

            // Log the command
            this.log(`Executing: ${command.line}`);

            try {
                // Execute the command using the provided function
                await executeCommand(command);
            } catch (error) {
                this.log(`Error executing command: ${error.message}`);
                this.stop();
                break;
            }

            // Small delay between commands (can be adjusted)
            await new Promise(resolve => setTimeout(resolve, 10));
        }

        if (this.isRunning) {
            this.log('G-code execution complete!');
        }

        this.isRunning = false;
    }

    /**
     * Pauses execution
     */
    pause() {
        if (this.isRunning && !this.isPaused) {
            this.isPaused = true;
            this.log('Execution paused');
        }
    }

    /**
     * Resumes execution
     */
    resume() {
        if (this.isRunning && this.isPaused) {
            this.isPaused = false;
            this.log('Execution resumed');
        }
    }

    /**
     * Stops execution
     */
    stop() {
        this.isRunning = false;
        this.isPaused = false;
        this.log('Execution stopped');
    }

    /**
     * Gets current execution status
     * @returns {Object} Status object
     */
    getStatus() {
        return {
            isRunning: this.isRunning,
            isPaused: this.isPaused,
            currentLine: this.currentLineIndex + 1,
            totalLines: this.commands.length,
            progress: this.commands.length > 0 
                ? Math.round(((this.currentLineIndex + 1) / this.commands.length) * 100) 
                : 0
        };
    }

    /**
     * Logs a message
     * @param {string} message - Message to log
     */
    log(message) {
        const timestamp = new Date().toLocaleTimeString();
        const logMessage = `[${timestamp}] ${message}`;
        console.log(logMessage);
        
        if (this.onLog) {
            this.onLog(logMessage);
        }
    }
}

// Create a global instance
const gcodeProcessor = new GCodeProcessor();


