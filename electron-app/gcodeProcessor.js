/**
 * G-Code Processor
 *
 * This module parses and processes G-code files.
 * It's simple and handles basic G-code commands (G0, G1, etc.)
 *
 * Beginner-friendly G-code parser.
 *
 * Control flow (Fanuc "macro B" style) is handled here rather than in the
 * app's command executor, because it only changes *which* line runs next:
 *   N100                       line label (alone, or as a prefix: N100 G1 J1=45)
 *   GOTO 100                   jump to the line labelled N100
 *   #1 = 5   #1 = #1 + 1       variables #1..#999, with + - * / and [brackets]
 *   IF [#1 LT 10] GOTO 100     conditional jump: EQ NE GT GE LT LE (or == != > >= < <=)
 *   G1 J1=#1 F40   M0 P#2      #n and [expressions] are substituted wherever a number goes
 *   M30 / M2                   end the program
 */

class GCodeProcessor {
    constructor() {
        this.lines = [];
        this.commands = [];
        this.labels = {};       // N-number → command index
        this.vars = {};         // #n → value (reset on every run)
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
        this.labels = {};
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

            // Store the line. lines[] and commands[] stay index-aligned so the
            // editor can highlight the line being executed.
            this.lines.push(line);
            const command = this.parseLine(line) || { code: 'UNKNOWN', line: line, params: {} };
            if (command.label !== undefined) {
                if (this.labels[command.label] !== undefined) {
                    console.warn(`Duplicate label N${command.label} — first one wins`);
                } else {
                    this.labels[command.label] = this.commands.length;
                }
            }
            this.commands.push(command);
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
        // Optional N<number> label prefix. A bare "N100" line is a label-only no-op.
        let label;
        const labelMatch = line.match(/^N(\d+)\b\s*(.*)$/i);
        if (labelMatch) {
            label = parseInt(labelMatch[1], 10);
            line = labelMatch[2].trim();
            if (line.length === 0) {
                return { code: 'N', line: `N${label}`, params: {}, label: label };
            }
        }
        const withLabel = (cmd) => { if (label !== undefined) cmd.label = label; return cmd; };

        // Control flow: GOTO n
        let m = line.match(/^GOTO\s+(\d+)\s*$/i);
        if (m) {
            return withLabel({ code: 'GOTO', line: line, params: {}, target: parseInt(m[1], 10) });
        }

        // Control flow: IF [cond] GOTO n
        m = line.match(/^IF\s*\[(.+)\]\s*GOTO\s+(\d+)\s*$/i);
        if (m) {
            return withLabel({ code: 'IF', line: line, params: {}, condition: m[1].trim(), target: parseInt(m[2], 10) });
        }

        // Variable assignment: #n = expression
        m = line.match(/^#(\d+)\s*=\s*(.+)$/);
        if (m) {
            return withLabel({ code: 'SET', line: line, params: {}, varNumber: parseInt(m[1], 10), expression: m[2].trim() });
        }

        // Lines that reference variables or bracket expressions are re-parsed
        // at run time once the values are known (see resolveCommand).
        if (/#\d+|\[/.test(line)) {
            return withLabel({ code: 'DEFERRED', line: line, params: {} });
        }

        const cmd = this.parseCommandLine(line);
        return cmd ? withLabel(cmd) : null;
    }

    /**
     * Parses an ordinary G/M/J command line (no labels, variables or
     * expressions — those are stripped/substituted before this is called).
     * @param {string} line
     * @returns {Object|null}
     */
    parseCommandLine(line) {
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
        // The quoted name is removed before numeric parsing so letters inside it
        // (e.g. "Pos2A5") can't be mistaken for parameters.
        let paramSource = line;
        const pStringMatch = line.match(/P\s*["']([^"']+)["']/i);
        if (pStringMatch) {
            params.P = pStringMatch[1]; // Store as string
            paramSource = line.replace(pStringMatch[0], ' ');
        }

        // Match standard parameters: X, Y, Z, I, J, K, F, R, S, P, L, A, V
        // (L is the detected-block-scan position slot, e.g. M781 P0 L10;
        //  A is acceleration for M204; V is a #variable number for M780/M782/M783)
        // Also match joint parameters: J1, J2, J3, etc. (with number)
        const paramPattern = /([XYZIJKFRSPLAV])([-+]?\d*\.?\d+)/gi;
        let paramMatch;

        while ((paramMatch = paramPattern.exec(paramSource)) !== null) {
            const letter = paramMatch[1].toUpperCase();
            const value = parseFloat(paramMatch[2]);

            // Skip P if we already found it as a string
            if (letter === 'P' && typeof params.P === 'string') {
                continue;
            }

            // For joint commands like J1=45, we need to handle J1, J2, etc. as separate parameters
            // Check if this is part of a joint parameter (J1, J2, etc.)
            const jointMatch = paramSource.match(new RegExp(`(${letter}\\d+)\\s*=\\s*([-+]?\\d*\\.?\\d+)`, 'i'));
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
        while ((jointParamMatch = jointParamPattern.exec(paramSource)) !== null) {
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

    // ── Variables and expressions ─────────────────────────────────────────

    getVar(n) {
        const v = this.vars[n];
        return typeof v === 'number' ? v : 0;
    }

    setVar(n, value) {
        this.vars[n] = Number(value) || 0;
    }

    /**
     * Replaces #n references with their values, then evaluates every
     * [bracketed expression] in the text to a plain number.
     * @param {string} text
     * @returns {string}
     */
    substituteVariables(text) {
        let out = text.replace(/#(\d+)/g, (_, n) => this.formatNumber(this.getVar(parseInt(n, 10))));
        // Evaluate innermost brackets first so nesting works.
        let guard = 0;
        while (/\[[^\[\]]*\]/.test(out) && guard++ < 50) {
            out = out.replace(/\[([^\[\]]*)\]/g, (_, inner) => this.formatNumber(this.evaluateExpression(inner)));
        }
        return out;
    }

    formatNumber(v) {
        if (!isFinite(v)) return '0';
        // Keep the sign attached (G-code params are "X-10", never "X -10").
        return String(Math.round(v * 1e6) / 1e6);
    }

    /**
     * Evaluates an arithmetic expression (+ - * / with parentheses/brackets
     * and unary minus). #n references must already be substituted.
     * @param {string} expr
     * @returns {number}
     */
    evaluateExpression(expr) {
        const src = this.substituteVariables(String(expr)).replace(/\[/g, '(').replace(/\]/g, ')');
        let pos = 0;
        const peek = () => src[pos];
        const skipWs = () => { while (pos < src.length && /\s/.test(src[pos])) pos++; };
        const parsePrimary = () => {
            skipWs();
            const ch = peek();
            if (ch === '(') {
                pos++;
                const v = parseAddSub();
                skipWs();
                if (peek() === ')') pos++;
                return v;
            }
            if (ch === '-') { pos++; return -parsePrimary(); }
            if (ch === '+') { pos++; return parsePrimary(); }
            const m = src.slice(pos).match(/^\d*\.?\d+(?:[eE][-+]?\d+)?/);
            if (!m) throw new Error(`Bad expression "${expr}" at "${src.slice(pos, pos + 8)}"`);
            pos += m[0].length;
            return parseFloat(m[0]);
        };
        const parseMulDiv = () => {
            let v = parsePrimary();
            for (;;) {
                skipWs();
                const op = peek();
                if (op === '*') { pos++; v *= parsePrimary(); }
                else if (op === '/') { pos++; const d = parsePrimary(); v = d === 0 ? 0 : v / d; }
                else return v;
            }
        };
        const parseAddSub = () => {
            let v = parseMulDiv();
            for (;;) {
                skipWs();
                const op = peek();
                if (op === '+') { pos++; v += parseMulDiv(); }
                else if (op === '-') { pos++; v -= parseMulDiv(); }
                else return v;
            }
        };
        const result = parseAddSub();
        skipWs();
        if (pos < src.length) throw new Error(`Bad expression "${expr}" near "${src.slice(pos, pos + 8)}"`);
        return result;
    }

    /**
     * Evaluates an IF condition such as "#1 LT 10" or "#2 == #3".
     * @param {string} condition
     * @returns {boolean}
     */
    evaluateCondition(condition) {
        const m = condition.match(/^(.+?)\s*(EQ|NE|GT|GE|LT|LE|==|=|!=|<>|>=|<=|>|<)\s*(.+)$/i);
        if (!m) throw new Error(`Bad IF condition "${condition}" — expected e.g. [#1 LT 10]`);
        const a = this.evaluateExpression(m[1]);
        const b = this.evaluateExpression(m[3]);
        switch (m[2].toUpperCase()) {
            case 'EQ': case '==': case '=': return Math.abs(a - b) < 1e-9;
            case 'NE': case '!=': case '<>': return Math.abs(a - b) >= 1e-9;
            case 'GT': case '>':  return a > b;
            case 'GE': case '>=': return a >= b;
            case 'LT': case '<':  return a < b;
            case 'LE': case '<=': return a <= b;
        }
        return false;
    }

    /**
     * Returns the command to execute for a stored command, with any
     * variables/expressions substituted and parameters re-parsed.
     * @param {Object} command
     * @returns {Object}
     */
    resolveCommand(command) {
        if (command.code !== 'DEFERRED') return command;
        const resolvedLine = this.substituteVariables(command.line);
        const parsed = this.parseCommandLine(resolvedLine);
        if (!parsed) return { code: 'UNKNOWN', line: resolvedLine, params: {} };
        return parsed;
    }

    /**
     * Resolves a GOTO target label to a command index.
     * @param {number} target
     * @returns {number}
     */
    jumpTargetIndex(target) {
        const idx = this.labels[target];
        if (idx === undefined) throw new Error(`GOTO ${target}: no line labelled N${target}`);
        return idx;
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
        this.vars = {};

        this.log('Starting G-code execution...');

        // Execute commands; GOTO/IF change `i` so this can't be a simple for-loop.
        let i = 0;
        while (i < this.commands.length) {
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

            // Update progress. With loops the line index can go backwards, so
            // this is "position in the file", not "fraction of work done".
            const progress = Math.round(((i + 1) / this.commands.length) * 100);
            if (this.onProgress) {
                this.onProgress(progress, i + 1, this.commands.length);
            }

            let next = i + 1;
            try {
                switch (command.code) {
                    case 'N':
                        // Label only — nothing to do
                        break;
                    case 'GOTO':
                        next = this.jumpTargetIndex(command.target);
                        this.log(`GOTO ${command.target}`);
                        break;
                    case 'IF': {
                        const result = this.evaluateCondition(command.condition);
                        this.log(`IF [${this.substituteVariables(command.condition)}] → ${result ? `true, GOTO ${command.target}` : 'false'}`);
                        if (result) next = this.jumpTargetIndex(command.target);
                        break;
                    }
                    case 'SET': {
                        const value = this.evaluateExpression(command.expression);
                        this.setVar(command.varNumber, value);
                        this.log(`#${command.varNumber} = ${this.formatNumber(value)}`);
                        break;
                    }
                    case 'M30':
                    case 'M2':
                        this.log(`${command.code}: program end`);
                        next = this.commands.length;
                        break;
                    default: {
                        const resolved = this.resolveCommand(command);
                        this.log(`Executing: ${resolved.line}`);
                        // Execute the command using the provided function
                        await executeCommand(resolved);
                    }
                }
            } catch (error) {
                this.log(`Error executing command: ${error.message}`);
                this.stop();
                break;
            }

            i = next;

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


