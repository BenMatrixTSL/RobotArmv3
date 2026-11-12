/**
 * Drag-to-scroll for the Pi touchscreen kiosk (?kiosk=1 only).
 *
 * On the Pi, a finger on the screen often reaches Chromium as a mouse (X11, and
 * some labwc versions), so dragging selected text instead of scrolling. Here a
 * press-and-drag scrolls the nearest scrollable area instead, the way a phone
 * does. Text selection is switched off in kiosk mode (styles.css for the app,
 * and an injected style for same-origin iframes such as the Curriculum tab).
 *
 * - Movement under DRAG_START_PX is still a tap, so buttons work as normal.
 * - A drag may start on a button; the click that follows the drag is cancelled,
 *   so lifting your finger after scrolling never presses anything.
 * - Things that need dragging themselves (3D view, Blockly, gamepad sticks,
 *   sliders, text fields, the on-screen keyboard) never start a scroll.
 * - Real touch events (when the screen sends them) are left to Chromium's own
 *   scrolling; only mouse-type input is handled here.
 */
(function () {
    'use strict';

    if (window.location.search.indexOf('kiosk=1') < 0) {
        return;
    }

    const DRAG_START_PX = 10;

    // Presses here keep their own behaviour and never scroll the page.
    // Add data-no-drag-scroll to any new element that needs to be dragged.
    const NO_DRAG_SCROLL = [
        'input', 'textarea', 'select', 'option', '[contenteditable]',
        'canvas',
        '#robotArm3DContainer',
        '#blockly-workspace', '.blocklyWidgetDiv', '.blocklyDropDownDiv',
        '.gp-controller', '.gp-stick', '.gp-trigger', '.gp-bumper',
        '#onScreenKeyboard',
        '[data-no-drag-scroll]'
    ].join(', ');

    // For iframe pages, which don't load styles.css
    const IFRAME_CSS =
        '*{user-select:none;-webkit-user-select:none;-webkit-tap-highlight-color:transparent}' +
        'input,textarea,[contenteditable],[contenteditable] *{user-select:text;-webkit-user-select:text}' +
        'img,a{-webkit-user-drag:none}' +
        'html.kiosk-drag-scrolling *{pointer-events:none}';

    /** Adds drag-to-scroll to one window (the app, or a same-origin iframe). */
    function install(win) {
        const doc = win.document;
        if (!doc || doc.__kioskDragScroll) return;
        doc.__kioskDragScroll = true;

        let drag = null;            // { el, startX, startY, startTop, moved }
        let cancelNextClick = false;
        let cancelTimer = null;

        function scrollableAncestor(node) {
            let el = node && node.nodeType === 1 ? node : node && node.parentElement;
            while (el && el !== doc.body && el !== doc.documentElement) {
                const overflowY = win.getComputedStyle(el).overflowY;
                if ((overflowY === 'auto' || overflowY === 'scroll') && el.scrollHeight > el.clientHeight + 1) {
                    return el;
                }
                el = el.parentElement;
            }
            const page = doc.scrollingElement;
            return page && page.scrollHeight > page.clientHeight + 1 ? page : null;
        }

        function endDrag() {
            if (!drag) return;
            if (drag.moved) {
                doc.documentElement.classList.remove('kiosk-drag-scrolling');
                if (doc.body) doc.body.classList.remove('kiosk-drag-scrolling');
                // The click (if any) fires straight after mouseup; drop it. If the
                // finger ended on a different element there is no click, so the
                // flag times out rather than eating the next real tap.
                cancelNextClick = true;
                clearTimeout(cancelTimer);
                cancelTimer = setTimeout(function () { cancelNextClick = false; }, 300);
            }
            drag = null;
        }

        doc.addEventListener('mousedown', function (e) {
            if (e.button !== 0) return;
            if (e.target.closest && e.target.closest(NO_DRAG_SCROLL)) return;
            const el = scrollableAncestor(e.target);
            if (!el) return;
            drag = { el: el, startX: e.clientX, startY: e.clientY, startTop: el.scrollTop, moved: false };
        }, true);

        win.addEventListener('mousemove', function (e) {
            if (!drag) return;
            if (!(e.buttons & 1)) { endDrag(); return; }   // button released outside the window
            const dy = drag.startY - e.clientY;
            if (!drag.moved) {
                const dx = drag.startX - e.clientX;
                if (Math.abs(dy) < DRAG_START_PX || Math.abs(dy) < Math.abs(dx)) return;
                drag.moved = true;
                doc.documentElement.classList.add('kiosk-drag-scrolling');
                if (doc.body) doc.body.classList.add('kiosk-drag-scrolling');
                const sel = win.getSelection && win.getSelection();
                if (sel && sel.removeAllRanges) sel.removeAllRanges();
            }
            drag.el.scrollTop = drag.startTop + dy;
            e.preventDefault();
        }, true);

        win.addEventListener('mouseup', endDrag, true);
        win.addEventListener('blur', endDrag);

        win.addEventListener('click', function (e) {
            if (!cancelNextClick) return;
            cancelNextClick = false;
            clearTimeout(cancelTimer);
            e.preventDefault();
            e.stopPropagation();
            e.stopImmediatePropagation();
        }, true);

        // Dragging an image or link would otherwise pick it up instead of scrolling
        doc.addEventListener('dragstart', function (e) {
            if (e.target.closest && e.target.closest(NO_DRAG_SCROLL)) return;
            e.preventDefault();
        }, true);
    }

    /** Same-origin iframes (the Curriculum tab) get the same behaviour on every page they load. */
    function watchIframe(frame) {
        if (frame.__kioskDragScroll) return;
        frame.__kioskDragScroll = true;
        const setup = function () {
            let w;
            try { w = frame.contentWindow; if (!w || !w.document || !w.document.head) return; }
            catch (_) { return; }   // other origin: leave it alone
            if (!w.document.getElementById('kiosk-drag-scroll-style')) {
                const style = w.document.createElement('style');
                style.id = 'kiosk-drag-scroll-style';
                style.textContent = IFRAME_CSS;
                w.document.head.appendChild(style);
            }
            install(w);
            w.document.querySelectorAll('iframe').forEach(watchIframe);
        };
        frame.addEventListener('load', setup);
        setup();
    }

    install(window);

    function watchAllIframes() {
        document.querySelectorAll('iframe').forEach(watchIframe);
    }
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', watchAllIframes);
    } else {
        watchAllIframes();
    }
})();
