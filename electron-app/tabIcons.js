/**
 * Tab icons for the Robot Arm Control Application.
 *
 * One entry per sidebar tab: a 24x24 stroke icon and the colour group the
 * tab belongs to. The same definitions are used in three places:
 *   - app.js inlines the SVG into each .tab-button at startup
 *     (decorateTabButtons), so on narrow screens the icon can stand alone
 *   - generate-tab-icons.js writes standalone SVG files to
 *     Curriculum/images/tabs/ for the curriculum and other documents
 *   - Curriculum/worksheet.js shows the icon beside "App tab: ..." chips
 *
 * Groups are colour-coded so related tabs read as a set:
 *   control    - moving the arm by hand
 *   program    - writing programs for the arm
 *   model      - understanding the arm's geometry
 *   workspace  - data about the arm's surroundings
 *   system     - connection, calibration and settings
 *   learn      - the curriculum
 *
 * Paths are drawn for stroke="currentColor", stroke-width 2, round caps.
 * Works as a browser global (TAB_ICONS / TAB_GROUPS) and as a Node module.
 */
(function (root, factory) {
    const api = factory();
    root.TAB_GROUPS = api.TAB_GROUPS;
    root.TAB_ICONS = api.TAB_ICONS;
    root.tabIconSvg = api.tabIconSvg;
    try {
        if (typeof module === 'object' && module.exports) {
            module.exports = api;
        }
    } catch (e) {
        // Browser without CommonJS - globals above are enough
    }
}(typeof window !== 'undefined' ? window : globalThis, function () {

    const TAB_GROUPS = {
        control:   { name: 'Arm control',        colour: '#2980b9' },
        program:   { name: 'Programming',        colour: '#27ae60' },
        model:     { name: 'Model & kinematics', colour: '#8e44ad' },
        workspace: { name: 'Workspace',          colour: '#e67e22' },
        system:    { name: 'System',             colour: '#5d6d7e' },
        learn:     { name: 'Learning',           colour: '#16a085' }
    };

    // Keys match the data-tab attribute on each .tab-button.
    const TAB_ICONS = {
        connection: {
            label: 'Connection', group: 'system',
            paths: '<path d="M8 7V3M16 7V3"/><path d="M5 7h14v3a7 7 0 0 1-14 0z"/><path d="M12 17v4"/>'
        },
        pendant: {
            label: 'Joint Control', group: 'control',
            paths: '<path d="M3 21h11"/><path d="M8.5 21v-6l3.5-7 6 3"/><circle cx="8.5" cy="15" r="1.7"/><circle cx="12" cy="8" r="1.7"/><circle cx="18" cy="11" r="1.7"/><path d="M19.5 10l2-2.5M19.6 12l2.4 1"/>'
        },
        'pendant-ui': {
            label: 'Pendant Control', group: 'control',
            paths: '<rect x="5" y="2" width="14" height="20" rx="3"/><path d="M12 7v6M9 10h6"/><circle cx="9" cy="17.5" r="1.1"/><circle cx="15" cy="17.5" r="1.1"/>'
        },
        gcode: {
            label: 'G-Code Control', group: 'program',
            paths: '<path d="M8 6l-5 6 5 6M16 6l5 6-5 6"/><path d="M14 5l-4 14"/>'
        },
        rapid: {
            label: 'RAPID', group: 'program',
            paths: '<rect x="3" y="4" width="18" height="16" rx="2"/><path d="M7 9l3 3-3 3M12 15h5"/>'
        },
        blockly: {
            label: 'Visual Programming', group: 'program',
            paths: '<rect x="3" y="3.5" width="12" height="5" rx="1"/><rect x="7" y="9.5" width="12" height="5" rx="1"/><rect x="3" y="15.5" width="12" height="5" rx="1"/>'
        },
        visualization: {
            label: '3D Visualisation', group: 'model',
            paths: '<path d="M12 2l9 5v10l-9 5-9-5V7z"/><path d="M12 12l9-5M12 12v10M12 12L3 7"/>'
        },
        kinematics: {
            label: 'Kinematics', group: 'model',
            paths: '<path d="M5 20V4M5 20h16M5 20l9-9"/><path d="M3 7l2-3 2 3M18 17l3 3-3 3M11.5 10.5l2.5.5-.5 2.5"/>'
        },
        positions: {
            label: 'Stored Positions', group: 'workspace',
            paths: '<path d="M12 22s7-6.5 7-12a7 7 0 1 0-14 0c0 5.5 7 12 7 12z"/><circle cx="12" cy="10" r="2.5"/>'
        },
        calibration: {
            label: 'Calibration', group: 'system',
            paths: '<circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/>'
        },
        deadzones: {
            label: 'Dead Zones', group: 'workspace',
            paths: '<rect x="4" y="4" width="16" height="16" rx="2" stroke-dasharray="3 2.5"/><path d="M7 7l10 10M17 7L7 17"/>'
        },
        camera: {
            label: 'Camera', group: 'workspace',
            paths: '<path d="M4 8h3l2-3h6l2 3h3a1 1 0 0 1 1 1v10a1 1 0 0 1-1 1H4a1 1 0 0 1-1-1V9a1 1 0 0 1 1-1z"/><circle cx="12" cy="13" r="3.5"/>'
        },
        settings: {
            label: 'Settings', group: 'system',
            paths: '<circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3M4.9 4.9l2.1 2.1M17 17l2.1 2.1M4.9 19.1L7 17M17 7l2.1-2.1"/>'
        },
        curriculum: {
            label: 'Curriculum', group: 'learn',
            paths: '<path d="M3 5h6a3 3 0 0 1 3 3v13a3 3 0 0 0-3-3H3z"/><path d="M21 5h-6a3 3 0 0 0-3 3v13a3 3 0 0 1 3-3h6z"/>'
        }
    };

    /**
     * Builds the SVG markup for a tab icon.
     * @param {string} tabId - Key of TAB_ICONS
     * @param {Object} [options] - size (px, default 24), colour (default currentColor),
     *                             standalone (adds the xmlns for a .svg file)
     * @returns {string} SVG markup, or '' for an unknown tab
     */
    function tabIconSvg(tabId, options) {
        const icon = TAB_ICONS[tabId];
        if (!icon) return '';
        const opts = options || {};
        const size = opts.size || 24;
        const colour = opts.colour || 'currentColor';
        const xmlns = opts.standalone ? ' xmlns="http://www.w3.org/2000/svg"' : '';
        const label = icon.label.replace(/&/g, '&amp;').replace(/</g, '&lt;');
        return '<svg' + xmlns + ' viewBox="0 0 24 24" width="' + size + '" height="' + size + '"' +
            ' fill="none" stroke="' + colour + '" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"' +
            ' role="img" aria-label="' + label + '">' +
            (opts.standalone ? '<title>' + label + '</title>' : '') +
            icon.paths + '</svg>';
    }

    return { TAB_GROUPS: TAB_GROUPS, TAB_ICONS: TAB_ICONS, tabIconSvg: tabIconSvg };
}));
