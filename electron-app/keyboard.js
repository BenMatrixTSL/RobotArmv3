/**
 * On-Screen Keyboard for kiosk mode
 *
 * Chromium in kiosk mode has no system on-screen keyboard, so this provides
 * a touch-friendly numeric keypad and full QWERTY keyboard that attaches to
 * any input/textarea automatically via a single delegated focus listener.
 * Only active when body has the "kiosk-mode" class (see app.js).
 */
(function () {
    const QWERTY_ROWS = [
        ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0'],
        ['q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p'],
        ['a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', '.'],
        ['shift', 'z', 'x', 'c', 'v', 'b', 'n', 'm', '-', 'backspace'],
        ['symbols', 'space', 'enter']
    ];

    const SYMBOL_ROWS = [
        ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0'],
        ['!', '@', '#', '$', '%', '^', '&', '*', '(', ')'],
        ['_', '+', '=', ';', ':', ',', '.', '/', '?', '\''],
        ['[', ']', '{', '}', '<', '>', '|', '~', '`', 'backspace'],
        ['abc', 'space', 'enter']
    ];

    const NUMPAD_ROWS = [
        ['7', '8', '9'],
        ['4', '5', '6'],
        ['1', '2', '3'],
        ['-', '0', '.'],
        ['backspace', 'enter']
    ];

    let panel = null;
    let activeField = null;
    let shiftOn = false;
    let symbolsOn = false;

    function isKioskMode() {
        return document.body.classList.contains('kiosk-mode');
    }

    function fieldWantsNumeric(el) {
        if (el.dataset.kbd === 'numeric') return true;
        if (el.dataset.kbd === 'full') return false;
        return el.tagName === 'INPUT' && el.type === 'number';
    }

    function fieldWantsKeyboard(el) {
        if (!el || el.disabled || el.readOnly) return false;
        if (el.tagName === 'TEXTAREA') return true;
        if (el.tagName !== 'INPUT') return false;
        return ['text', 'number', 'search', 'tel', 'url', 'email'].includes(el.type);
    }

    function buildPanel() {
        panel = document.createElement('div');
        panel.id = 'onScreenKeyboard';
        panel.className = 'osk-panel';
        // Prevent the keyboard from stealing focus away from the active field.
        panel.addEventListener('mousedown', (e) => e.preventDefault());
        document.body.appendChild(panel);
    }

    function keyLabel(key) {
        switch (key) {
            case 'backspace': return '&larr;';
            case 'enter': return 'Enter';
            case 'space': return ' ';
            case 'shift': return 'Shift';
            case 'symbols': return '#+=';
            case 'abc': return 'ABC';
            default: return shiftOn && key.length === 1 && /[a-z]/.test(key) ? key.toUpperCase() : key;
        }
    }

    function keyClass(key) {
        const wide = ['space', 'enter', 'backspace', 'shift', 'symbols', 'abc'];
        return wide.includes(key) ? 'osk-key osk-key-wide' : 'osk-key';
    }

    function render(rows) {
        panel.innerHTML = '';
        rows.forEach((row) => {
            const rowEl = document.createElement('div');
            rowEl.className = 'osk-row';
            row.forEach((key) => {
                const btn = document.createElement('button');
                btn.type = 'button';
                btn.className = keyClass(key);
                if (key === 'shift' && shiftOn) btn.classList.add('osk-key-active');
                btn.innerHTML = keyLabel(key);
                btn.addEventListener('click', () => handleKey(key));
                rowEl.appendChild(btn);
            });
            panel.appendChild(rowEl);
        });
    }

    function renderForActiveField() {
        if (!activeField) return;
        if (fieldWantsNumeric(activeField)) {
            render(NUMPAD_ROWS);
        } else {
            render(symbolsOn ? SYMBOL_ROWS : QWERTY_ROWS);
        }
    }

    function insertText(text) {
        if (!activeField) return;
        const start = activeField.selectionStart ?? activeField.value.length;
        const end = activeField.selectionEnd ?? activeField.value.length;
        const value = activeField.value;
        activeField.value = value.slice(0, start) + text + value.slice(end);
        const caret = start + text.length;
        activeField.setSelectionRange(caret, caret);
        activeField.dispatchEvent(new Event('input', { bubbles: true }));
    }

    function backspace() {
        if (!activeField) return;
        const start = activeField.selectionStart ?? activeField.value.length;
        const end = activeField.selectionEnd ?? activeField.value.length;
        const value = activeField.value;
        if (start === end) {
            if (start === 0) return;
            activeField.value = value.slice(0, start - 1) + value.slice(end);
            activeField.setSelectionRange(start - 1, start - 1);
        } else {
            activeField.value = value.slice(0, start) + value.slice(end);
            activeField.setSelectionRange(start, start);
        }
        activeField.dispatchEvent(new Event('input', { bubbles: true }));
    }

    function handleKey(key) {
        if (!activeField) return;
        switch (key) {
            case 'backspace':
                backspace();
                break;
            case 'enter':
                activeField.dispatchEvent(new Event('change', { bubbles: true }));
                if (activeField.tagName === 'TEXTAREA') {
                    insertText('\n');
                } else {
                    hide();
                }
                break;
            case 'space':
                insertText(' ');
                break;
            case 'shift':
                shiftOn = !shiftOn;
                renderForActiveField();
                break;
            case 'symbols':
                symbolsOn = true;
                renderForActiveField();
                break;
            case 'abc':
                symbolsOn = false;
                renderForActiveField();
                break;
            default:
                insertText(keyLabel(key));
                if (shiftOn) {
                    shiftOn = false;
                    renderForActiveField();
                }
        }
    }

    function show(field) {
        activeField = field;
        shiftOn = false;
        symbolsOn = false;
        if (!panel) buildPanel();
        renderForActiveField();
        panel.classList.add('osk-visible');
        document.body.classList.add('osk-open');
        // Keep the focused field visible above the keyboard.
        window.requestAnimationFrame(() => {
            field.scrollIntoView({ block: 'center', behavior: 'smooth' });
        });
    }

    function hide() {
        activeField = null;
        if (panel) panel.classList.remove('osk-visible');
        document.body.classList.remove('osk-open');
    }

    function onFocusIn(e) {
        if (!isKioskMode()) return;
        const el = e.target;
        if (!fieldWantsKeyboard(el)) return;
        show(el);
    }

    function onFocusOut(e) {
        if (!isKioskMode()) return;
        // If focus is moving to the keyboard itself, ignore (mousedown already
        // prevented default so this normally won't fire, but guard anyway).
        window.setTimeout(() => {
            const active = document.activeElement;
            if (panel && panel.contains(active)) return;
            if (active === activeField) return;
            hide();
        }, 0);
    }

    document.addEventListener('focusin', onFocusIn);
    document.addEventListener('focusout', onFocusOut);
})();
