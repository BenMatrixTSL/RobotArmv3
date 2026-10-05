/**
 * RAPID Processor
 *
 * Parses and runs the app's beginner-friendly subset of ABB RAPID. This file
 * owns the language side — variables, expressions and control flow — and
 * hands every arm command (MoveJ, MoveLXYZ, GripperOpen, …) to the executor
 * callback that app.js supplies, with a way to evaluate expressions inside
 * the command's arguments.
 *
 * Supported control flow (real RAPID syntax):
 *   VAR num count := 0;              declare a variable (initial value optional)
 *   count := count + 1;              assignment, with + - * / DIV MOD and ( )
 *   WHILE count < 3 DO … ENDWHILE    conditions: = <> < <= > >=  AND OR NOT  TRUE FALSE
 *   FOR i FROM 1 TO 5 [STEP 1] DO … ENDFOR
 *   IF a > b THEN … ELSEIF … THEN … ELSE … ENDIF
 *   again:  /  GOTO again;           labels and jumps
 *   EXIT;                            end the program
 *   TPWrite "text";                  print to the log (also TPWrite "n = " \Num:=count;)
 * Functions supplied by the executor, usable in any expression:
 *   GetBlockCount()  GetBlockX(i)  GetBlockY(i)
 */

class RapidProcessor {
    constructor() {
        this.statements = [];
        this.labels = {};
        this.vars = {};
        this.functions = {};    // name (lower-case) → async (args[]) => number
        this.isRunning = false;
        this.onLog = null;
        this.onLineChange = null;
    }

    // ── Loading ──────────────────────────────────────────────────────────

    /**
     * Parses RAPID source into statements and matches up loop/if structure.
     * Throws on structural errors (unterminated WHILE, ELSE without IF, …).
     * @param {string} source
     * @returns {{statements: number}}
     */
    load(source) {
        this.statements = [];
        this.labels = {};
        const rawLines = String(source || '').split(/\r?\n/);

        for (let i = 0; i < rawLines.length; i++) {
            let text = rawLines[i].trim();
            if (text.length === 0 || text.startsWith('!')) continue;
            // Strip a trailing "! comment" (ignoring "!" inside strings) and the ";" terminator
            text = this.stripComment(text).replace(/;\s*$/, '').trim();
            if (text.length === 0) continue;
            const stmt = this.parseStatement(text);
            stmt.lineNumber = i + 1;
            stmt.text = text;
            if (stmt.type === 'LABEL') {
                if (this.labels[stmt.name] !== undefined) {
                    throw new Error(`Line ${i + 1}: label "${stmt.name}" is defined twice`);
                }
                this.labels[stmt.name] = this.statements.length;
            }
            this.statements.push(stmt);
        }

        this.linkStructure();
        return { statements: this.statements.length };
    }

    stripComment(text) {
        let inString = false;
        for (let i = 0; i < text.length; i++) {
            const ch = text[i];
            if (ch === '"') inString = !inString;
            else if (ch === '!' && !inString) return text.slice(0, i).trim();
        }
        return text;
    }

    parseStatement(text) {
        let m;
        if ((m = text.match(/^WHILE\s+(.+?)\s+DO$/i)))   return { type: 'WHILE', condition: m[1] };
        if (/^ENDWHILE$/i.test(text))                     return { type: 'ENDWHILE' };
        if ((m = text.match(/^FOR\s+([A-Za-z_]\w*)\s+FROM\s+(.+?)\s+TO\s+(.+?)(?:\s+STEP\s+(.+?))?\s+DO$/i))) {
            return { type: 'FOR', name: m[1], from: m[2], to: m[3], step: m[4] || '1' };
        }
        if (/^ENDFOR$/i.test(text))                       return { type: 'ENDFOR' };
        if ((m = text.match(/^IF\s+(.+?)\s+THEN$/i)))     return { type: 'IF', condition: m[1] };
        if ((m = text.match(/^ELSEIF\s+(.+?)\s+THEN$/i))) return { type: 'ELSEIF', condition: m[1] };
        if (/^ELSE$/i.test(text))                         return { type: 'ELSE' };
        if (/^ENDIF$/i.test(text))                        return { type: 'ENDIF' };
        if ((m = text.match(/^([A-Za-z_]\w*):$/)))        return { type: 'LABEL', name: m[1] };
        if ((m = text.match(/^GOTO\s+([A-Za-z_]\w*)$/i))) return { type: 'GOTO', name: m[1] };
        if (/^(EXIT|Stop)$/i.test(text))                  return { type: 'EXIT' };
        if ((m = text.match(/^VAR\s+(num|bool)\s+([A-Za-z_]\w*)\s*(?::=\s*(.+))?$/i))) {
            return { type: 'VAR', name: m[2], expression: m[3] || null };
        }
        if ((m = text.match(/^([A-Za-z_]\w*)\s*:=\s*(.+)$/))) return { type: 'ASSIGN', name: m[1], expression: m[2] };
        if ((m = text.match(/^TPWrite\s+"([^"]*)"\s*(?:\\Num\s*:=\s*(.+))?$/i))) {
            return { type: 'TPWRITE', message: m[1], expression: m[2] || null };
        }
        if ((m = text.match(/^TPWrite\s+(.+)$/i)))        return { type: 'TPWRITE', message: '', expression: m[1] };
        const kw = text.match(/^([A-Za-z_]\w*)/);
        return { type: 'COMMAND', keyword: kw ? kw[1] : text, line: text };
    }

    /**
     * Matches WHILE/ENDWHILE, FOR/ENDFOR and IF/ELSEIF/ELSE/ENDIF so the run
     * loop can jump straight to the right statement.
     */
    linkStructure() {
        const stack = [];
        const st = this.statements;
        const where = (s) => `Line ${s.lineNumber} ("${s.text}")`;
        for (let i = 0; i < st.length; i++) {
            const s = st[i];
            switch (s.type) {
                case 'WHILE':
                case 'FOR':
                case 'IF':
                    s.chain = [];
                    stack.push(i);
                    break;
                case 'ELSEIF':
                case 'ELSE': {
                    const open = stack.length ? st[stack[stack.length - 1]] : null;
                    if (!open || open.type !== 'IF') throw new Error(`${where(s)}: ${s.type} without a matching IF`);
                    if (open.chain.length && st[open.chain[open.chain.length - 1]].type === 'ELSE') {
                        throw new Error(`${where(s)}: ${s.type} after ELSE`);
                    }
                    open.chain.push(i);
                    break;
                }
                case 'ENDWHILE':
                case 'ENDFOR':
                case 'ENDIF': {
                    const expected = { ENDWHILE: 'WHILE', ENDFOR: 'FOR', ENDIF: 'IF' }[s.type];
                    const openIdx = stack.length ? stack[stack.length - 1] : -1;
                    if (openIdx < 0 || st[openIdx].type !== expected) {
                        throw new Error(`${where(s)}: ${s.type} without a matching ${expected}`);
                    }
                    stack.pop();
                    st[openIdx].endIndex = i;
                    s.startIndex = openIdx;
                    // ELSEIF/ELSE need the ENDIF to skip to once their branch is done
                    if (s.type === 'ENDIF') st[openIdx].chain.forEach(c => { st[c].endIndex = i; });
                    break;
                }
                case 'GOTO':
                    if (this.labels[s.name] === undefined) throw new Error(`${where(s)}: no label "${s.name}:" found`);
                    break;
            }
        }
        if (stack.length) {
            const s = st[stack[stack.length - 1]];
            throw new Error(`${where(s)}: ${s.type} is never closed (missing END${s.type})`);
        }
    }

    // ── Variables ────────────────────────────────────────────────────────

    getVar(name) {
        const v = this.vars[name.toLowerCase()];
        if (v === undefined) throw new Error(`Unknown variable "${name}" — declare it first with VAR num ${name} := 0;`);
        return v;
    }

    setVar(name, value) {
        this.vars[name.toLowerCase()] = Number(value) || 0;
    }

    hasVar(name) {
        return this.vars[name.toLowerCase()] !== undefined;
    }

    // ── Expressions ──────────────────────────────────────────────────────

    /**
     * Evaluates a RAPID expression. Booleans are 1/0. Async because
     * functions such as GetBlockCount() talk to the camera.
     * @param {string} expr
     * @returns {Promise<number>}
     */
    async evaluate(expr) {
        const tokens = this.tokenize(String(expr));
        let pos = 0;
        const peek = () => tokens[pos];
        const take = () => tokens[pos++];
        const isOp = (t, ...ops) => t && t.type === 'op' && ops.includes(t.value);
        const isWord = (t, w) => t && t.type === 'id' && t.value.toUpperCase() === w;
        const self = this;

        async function parseOr() {
            let v = await parseAnd();
            while (isWord(peek(), 'OR')) { take(); const r = await parseAnd(); v = (v || r) ? 1 : 0; }
            return v;
        }
        async function parseAnd() {
            let v = await parseNot();
            while (isWord(peek(), 'AND')) { take(); const r = await parseNot(); v = (v && r) ? 1 : 0; }
            return v;
        }
        async function parseNot() {
            if (isWord(peek(), 'NOT')) { take(); return (await parseNot()) ? 0 : 1; }
            return parseComparison();
        }
        async function parseComparison() {
            const a = await parseAdditive();
            const t = peek();
            if (isOp(t, '=', '<>', '<', '<=', '>', '>=')) {
                take();
                const b = await parseAdditive();
                switch (t.value) {
                    case '=':  return Math.abs(a - b) < 1e-9 ? 1 : 0;
                    case '<>': return Math.abs(a - b) < 1e-9 ? 0 : 1;
                    case '<':  return a < b ? 1 : 0;
                    case '<=': return a <= b ? 1 : 0;
                    case '>':  return a > b ? 1 : 0;
                    case '>=': return a >= b ? 1 : 0;
                }
            }
            return a;
        }
        async function parseAdditive() {
            let v = await parseMultiplicative();
            for (;;) {
                const t = peek();
                if (isOp(t, '+')) { take(); v += await parseMultiplicative(); }
                else if (isOp(t, '-')) { take(); v -= await parseMultiplicative(); }
                else return v;
            }
        }
        async function parseMultiplicative() {
            let v = await parseUnary();
            for (;;) {
                const t = peek();
                if (isOp(t, '*')) { take(); v *= await parseUnary(); }
                else if (isOp(t, '/')) { take(); const d = await parseUnary(); v = d === 0 ? 0 : v / d; }
                else if (isWord(t, 'DIV')) { take(); const d = await parseUnary(); v = d === 0 ? 0 : Math.trunc(v / d); }
                else if (isWord(t, 'MOD')) { take(); const d = await parseUnary(); v = d === 0 ? 0 : v % d; }
                else return v;
            }
        }
        async function parseUnary() {
            const t = peek();
            if (isOp(t, '-')) { take(); return -(await parseUnary()); }
            if (isOp(t, '+')) { take(); return parseUnary(); }
            return parsePrimary();
        }
        async function parsePrimary() {
            const t = take();
            if (!t) throw new Error(`Unexpected end of expression "${expr}"`);
            if (t.type === 'num') return t.value;
            if (isOp(t, '(')) {
                const v = await parseOr();
                if (!isOp(take(), ')')) throw new Error(`Missing ")" in "${expr}"`);
                return v;
            }
            if (t.type === 'id') {
                const upper = t.value.toUpperCase();
                if (upper === 'TRUE') return 1;
                if (upper === 'FALSE') return 0;
                if (isOp(peek(), '(')) {
                    take();
                    const args = [];
                    if (!isOp(peek(), ')')) {
                        args.push(await parseOr());
                        while (isOp(peek(), ',')) { take(); args.push(await parseOr()); }
                    }
                    if (!isOp(take(), ')')) throw new Error(`Missing ")" after ${t.value}( in "${expr}"`);
                    const fn = self.functions[t.value.toLowerCase()];
                    if (!fn) throw new Error(`Unknown function "${t.value}()"`);
                    const result = await fn(args);
                    return Number(result) || 0;
                }
                return self.getVar(t.value);
            }
            throw new Error(`Unexpected "${t.value}" in "${expr}"`);
        }

        const result = await parseOr();
        if (pos < tokens.length) throw new Error(`Unexpected "${tokens[pos].value}" in "${expr}"`);
        return result;
    }

    tokenize(src) {
        const tokens = [];
        const re = /\s*(?:(\d*\.?\d+(?:[eE][-+]?\d+)?)|([A-Za-z_]\w*)|(<=|>=|<>|[-+*/()=<>,]))/y;
        let pos = 0;
        while (pos < src.length) {
            re.lastIndex = pos;
            const m = re.exec(src);
            if (!m || m[0].length === 0) {
                if (/^\s*$/.test(src.slice(pos))) break;
                throw new Error(`Bad character "${src[pos]}" in "${src}"`);
            }
            if (m[1] !== undefined) tokens.push({ type: 'num', value: parseFloat(m[1]) });
            else if (m[2] !== undefined) tokens.push({ type: 'id', value: m[2] });
            else tokens.push({ type: 'op', value: m[3] });
            pos = re.lastIndex;
        }
        return tokens;
    }

    /**
     * Evaluates a comma-separated list of expressions, e.g. the inside of
     * MoveAbsJ [[i * 10, 0, 20]].
     * @param {string} list
     * @returns {Promise<number[]>}
     */
    async evaluateList(list) {
        const parts = [];
        let depth = 0, current = '';
        for (const ch of String(list)) {
            if (ch === '(') depth++;
            if (ch === ')') depth--;
            if (ch === ',' && depth === 0) { parts.push(current); current = ''; }
            else current += ch;
        }
        parts.push(current);
        const values = [];
        for (const p of parts) values.push(p.trim() === '' ? 0 : await this.evaluate(p));
        return values;
    }

    // ── Running ──────────────────────────────────────────────────────────

    log(message) {
        console.log('[RAPID] ' + message);
        if (this.onLog) this.onLog(message);
    }

    /**
     * Runs the loaded program.
     * @param {function(Object): Promise} executeCommand - called for each arm
     *        command statement ({ keyword, line, lineNumber }); use
     *        processor.evaluate()/evaluateList() to resolve its arguments.
     */
    async start(executeCommand) {
        if (this.isRunning) { this.log('A RAPID program is already running'); return; }
        this.isRunning = true;
        this.vars = {};
        const st = this.statements;
        const forState = {};
        let pc = 0;

        this.log('Starting RAPID program...');
        try {
            while (pc < st.length) {
                if (!this.isRunning) { this.log('Program stopped'); break; }
                const s = st[pc];
                if (this.onLineChange) this.onLineChange(s.lineNumber, s.text);
                let next = pc + 1;

                switch (s.type) {
                    case 'LABEL':
                    case 'ENDIF':
                        break;
                    case 'GOTO':
                        next = this.labels[s.name];
                        break;
                    case 'EXIT':
                        this.log('EXIT — program ended');
                        next = st.length;
                        break;
                    case 'VAR':
                        this.setVar(s.name, s.expression === null ? 0 : await this.evaluate(s.expression));
                        break;
                    case 'ASSIGN':
                        if (!this.hasVar(s.name)) throw new Error(`Line ${s.lineNumber}: "${s.name}" is not declared — add VAR num ${s.name} := 0; first`);
                        this.setVar(s.name, await this.evaluate(s.expression));
                        break;
                    case 'TPWRITE': {
                        const value = s.expression === null ? '' : this.formatNumber(await this.evaluate(s.expression));
                        this.log(`${s.message}${value}`);
                        break;
                    }
                    case 'WHILE':
                        if (!(await this.evaluate(s.condition))) next = s.endIndex + 1;
                        break;
                    case 'ENDWHILE':
                        next = s.startIndex;
                        break;
                    case 'FOR': {
                        const from = await this.evaluate(s.from);
                        const to = await this.evaluate(s.to);
                        const step = await this.evaluate(s.step);
                        if (step === 0) throw new Error(`Line ${s.lineNumber}: FOR step cannot be 0`);
                        this.setVar(s.name, from);
                        forState[pc] = { to, step };
                        if (step > 0 ? from > to : from < to) next = s.endIndex + 1;
                        break;
                    }
                    case 'ENDFOR': {
                        const f = st[s.startIndex];
                        const state = forState[s.startIndex];
                        if (!state) break; // jumped into the loop body — just fall through
                        const v = this.getVar(f.name) + state.step;
                        this.setVar(f.name, v);
                        if (state.step > 0 ? v <= state.to : v >= state.to) next = s.startIndex + 1;
                        break;
                    }
                    case 'IF': {
                        if (await this.evaluate(s.condition)) break;
                        next = await this.nextBranch(s);
                        break;
                    }
                    case 'ELSEIF':
                    case 'ELSE':
                        // Reached by falling off the end of the previous branch
                        next = s.endIndex + 1;
                        break;
                    case 'COMMAND':
                        await executeCommand({ keyword: s.keyword, line: s.line, lineNumber: s.lineNumber });
                        break;
                }

                pc = next;
                await new Promise(resolve => setTimeout(resolve, 10));
            }
            if (this.isRunning) this.log('RAPID program finished.');
        } catch (error) {
            this.log(`Error: ${error.message}`);
        }
        this.isRunning = false;
    }

    /**
     * After a false IF/ELSEIF: finds the first ELSEIF whose condition holds,
     * the ELSE, or the statement after ENDIF.
     */
    async nextBranch(ifStmt) {
        for (const idx of ifStmt.chain) {
            const c = this.statements[idx];
            if (c.type === 'ELSE') return idx + 1;
            if (await this.evaluate(c.condition)) return idx + 1;
        }
        return ifStmt.endIndex + 1;
    }

    stop() {
        this.isRunning = false;
    }

    formatNumber(v) {
        return String(Math.round(v * 1e6) / 1e6);
    }
}

const rapidProcessor = new RapidProcessor();
