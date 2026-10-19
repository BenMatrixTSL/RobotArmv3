/**
 * Rebuilds worksheets-bundle.js from the worksheet card in every module-NN.html.
 *
 * The bundle is what "Print All Worksheets" on the curriculum index uses, so run
 * this whenever a worksheet is added or changed:
 *
 *   node build-worksheets-bundle.js
 *
 * It then checks that each bundled worksheet has the same number of answer
 * fields as its page - answers are saved by field position, so they must match.
 */
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const dir = __dirname;

// Return the HTML of the first element with class "worksheet-card",
// following nested <div>s to find its matching closing tag
function extractCard(html) {
  const open = /<div\b[^>]*class="[^"]*\bworksheet-card\b[^"]*"[^>]*>/.exec(html);
  if (!open) return null;
  const tagRe = /<\/?div\b[^>]*>/g;
  tagRe.lastIndex = open.index;
  let depth = 0;
  let m;
  while ((m = tagRe.exec(html))) {
    depth += m[0].startsWith('</') ? -1 : 1;
    if (depth === 0) return html.slice(open.index, tagRe.lastIndex);
  }
  throw new Error('worksheet-card has no matching closing </div>');
}

function readPage(file) {
  return fs.readFileSync(path.join(dir, file), 'utf8').replace(/\r\n/g, '\n');
}

const modules = fs.readdirSync(dir).filter(f => /^module-\d\d\.html$/.test(f)).sort();
const entries = [];
for (const file of modules) {
  const card = extractCard(readPage(file));
  if (!card) {
    console.log(file, '- no worksheet, skipped');
    continue;
  }
  const compact = card.split('\n').filter(line => line.trim() !== '').join('\n');
  const literal = "'" + compact.replace(/\\/g, '\\\\').replace(/'/g, "\\'").replace(/\n/g, '\\n') + "'";
  entries.push("  '" + file + "': " + literal);
}

const output =
  '/**\n' +
  ' * Static HTML for every worksheet card, extracted from the module-NN.html pages.\n' +
  ' * Used by index.html "Print All Worksheets" so printing works even before a\n' +
  ' * module has been visited.\n' +
  ' *\n' +
  ' * GENERATED FILE - do not edit by hand. Run: node build-worksheets-bundle.js\n' +
  ' */\n' +
  'var WORKSHEET_BUNDLE = {\n' + entries.join(',\n\n') + '\n};\n';

fs.writeFileSync(path.join(dir, 'worksheets-bundle.js'), output);

// Check the bundle parses and every worksheet's field count matches its page
const context = {};
vm.runInNewContext(output + ';this.bundle = WORKSHEET_BUNDLE;', context);
const fieldRe = /<(input [^>]*type="(text|number|checkbox)"|textarea)[\s>]/g;
let problems = 0;
for (const file of Object.keys(context.bundle)) {
  const bundled = (context.bundle[file].match(fieldRe) || []).length;
  const onPage = (extractCard(readPage(file)).match(fieldRe) || []).length;
  const ok = bundled === onPage;
  if (!ok) problems++;
  console.log(file.padEnd(16), String(bundled).padStart(3), 'fields', ok ? 'ok' : 'MISMATCH (page has ' + onPage + ')');
}
console.log(problems ? problems + ' problem(s) found' : 'worksheets-bundle.js rebuilt - ' + entries.length + ' worksheets');
process.exitCode = problems ? 1 : 0;
