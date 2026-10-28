#!/usr/bin/env node
/**
 * Writes a standalone SVG file for every tab icon in tabIcons.js to
 * Curriculum/images/tabs/, coloured by its group, plus a legend sheet
 * (tab-icons-legend.svg) showing all icons with their labels and groups.
 *
 * Run after changing tabIcons.js:
 *   node generate-tab-icons.js
 */
const fs = require('fs');
const path = require('path');
const { TAB_GROUPS, TAB_ICONS, tabIconSvg } = require('./tabIcons.js');

const outDir = path.join(__dirname, 'Curriculum', 'images', 'tabs');
fs.mkdirSync(outDir, { recursive: true });

const esc = function (t) {
    return String(t).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
};

const ids = Object.keys(TAB_ICONS);
ids.forEach(function (id) {
    const icon = TAB_ICONS[id];
    const colour = TAB_GROUPS[icon.group].colour;
    const svg = tabIconSvg(id, { size: 48, colour: colour, standalone: true });
    fs.writeFileSync(path.join(outDir, id + '.svg'), svg + '\n');
});

// Legend: one row per tab, grouped, for the curriculum's reference pages
const rowH = 40;
const groupOrder = Object.keys(TAB_GROUPS);
let y = 20;
let body = '';
groupOrder.forEach(function (groupId) {
    const group = TAB_GROUPS[groupId];
    const members = ids.filter(function (id) { return TAB_ICONS[id].group === groupId; });
    if (!members.length) return;
    body += '<text x="16" y="' + (y + 14) + '" font-size="13" font-weight="700" fill="' + group.colour + '">' +
        esc(group.name.toUpperCase()) + '</text>';
    y += 24;
    members.forEach(function (id) {
        const icon = TAB_ICONS[id];
        body += '<g transform="translate(16 ' + y + ')">' +
            tabIconSvg(id, { size: 28, colour: group.colour }).replace(/ role="img" aria-label="[^"]*"/, '') +
            '<text x="40" y="19" font-size="14" fill="#2c3e50">' + esc(icon.label) + '</text></g>';
        y += rowH;
    });
    y += 8;
});
const legend = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 320 ' + (y + 8) + '" width="320" height="' + (y + 8) + '"' +
    ' font-family="Segoe UI, Arial, sans-serif">' +
    '<title>Robot Arm Control Application - tab icons</title>' +
    '<rect width="320" height="' + (y + 8) + '" fill="#fff"/>' + body + '</svg>\n';
fs.writeFileSync(path.join(outDir, 'tab-icons-legend.svg'), legend);

console.log('Wrote ' + ids.length + ' icons and legend to ' + path.relative(process.cwd(), outDir));
