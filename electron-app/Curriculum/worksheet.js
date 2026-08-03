/**
 * Worksheet helpers for the Robot Arm curriculum.
 * - Saves answers in the browser (localStorage) so they survive page changes
 * - Saves an HTML snapshot of each card so the index page can print all
 *   worksheets without making any network requests (works for file:// too)
 * - Adds a "Print worksheet" button to each worksheet card
 *
 * Keep this file simple and readable — no advanced patterns.
 */
(function () {
  // Find every worksheet on this page
  var cards = document.querySelectorAll('.worksheet-card');
  if (cards.length === 0) {
    return;
  }

  // Use the HTML file name so each module has its own saved answers
  // Example: module-03.html  →  curriculum-ws-module-03.html
  var parts = window.location.pathname.split('/');
  var pageName = parts[parts.length - 1];
  if (!pageName) {
    pageName = 'unknown-page';
  }
  var storageKey = 'curriculum-ws-' + pageName;
  var htmlKey    = storageKey + '-html';

  // Load any previously saved answers
  var saved = {};
  try {
    var raw = localStorage.getItem(storageKey);
    if (raw) {
      saved = JSON.parse(raw);
    }
  } catch (e) {
    saved = {};
  }

  // Set up each worksheet card
  cards.forEach(function (card, cardIndex) {
    var fields = getFields(card);

    fields.forEach(function (field, fieldIndex) {
      var key = 'c' + cardIndex + '-f' + fieldIndex;

      // Put the saved answer back into the field
      if (saved[key] !== undefined && saved[key] !== null) {
        if (field.type === 'checkbox') {
          field.checked = saved[key] === true || saved[key] === 'true';
        } else {
          field.value = saved[key];
        }
      }

      // Save whenever the student types or ticks a box
      field.addEventListener('input', saveAll);
      field.addEventListener('change', saveAll);
    });

    // Add the Print button (and a short note) once
    if (!card.querySelector('.ws-print-btn')) {
      var actions = document.createElement('div');
      actions.className = 'ws-actions no-print';

      var printBtn = document.createElement('button');
      printBtn.type = 'button';
      printBtn.className = 'ws-print-btn';
      printBtn.textContent = 'Print worksheet';
      printBtn.addEventListener('click', function () {
        printOneWorksheet(card);
      });

      var saveNote = document.createElement('span');
      saveNote.className = 'ws-save-note';
      saveNote.textContent = 'Answers are saved automatically on this computer.';

      actions.appendChild(printBtn);
      actions.appendChild(saveNote);
      card.appendChild(actions);
    }
  });

  // Save an HTML snapshot now that fields are populated — the index page reads
  // these snapshots for "Print All Worksheets" without needing any network requests.
  updateHtmlSnapshot();

  // Collect every fillable field inside a worksheet
  function getFields(card) {
    return card.querySelectorAll(
      'input[type="text"], input[type="number"], input[type="checkbox"], textarea'
    );
  }

  // Save all answers from all worksheets on this page
  function saveAll() {
    var data = {};

    cards.forEach(function (card, cardIndex) {
      var fields = getFields(card);
      fields.forEach(function (field, fieldIndex) {
        var key = 'c' + cardIndex + '-f' + fieldIndex;
        if (field.type === 'checkbox') {
          data[key] = field.checked;
        } else {
          data[key] = field.value;
        }
      });
    });

    try {
      localStorage.setItem(storageKey, JSON.stringify(data));
    } catch (e) {
      // If storage is full or blocked, do nothing
    }

    updateHtmlSnapshot();
  }

  // Save a serialised copy of each card (with current values baked into
  // attributes) so the index page can reconstruct them without loading this page.
  function updateHtmlSnapshot() {
    var snapshots = {};

    for (var ci = 0; ci < cards.length; ci++) {
      var card = cards[ci];
      var clone = card.cloneNode(true);

      // Remove the print-button bar from the snapshot
      var actions = clone.querySelector('.ws-actions');
      if (actions) actions.parentNode.removeChild(actions);

      // Sync live .value properties → HTML attributes so they survive serialisation
      var liveFields  = getFields(card);
      var cloneFields = getFields(clone);
      for (var fi = 0; fi < liveFields.length; fi++) {
        var lf = liveFields[fi];
        var cf = cloneFields[fi];
        if (lf.type === 'checkbox') {
          if (lf.checked) cf.setAttribute('checked', 'checked');
          else cf.removeAttribute('checked');
        } else if (lf.tagName === 'TEXTAREA') {
          cf.textContent = lf.value;
        } else {
          cf.setAttribute('value', lf.value);
        }
      }

      snapshots[ci] = clone.outerHTML;
    }

    try {
      localStorage.setItem(htmlKey, JSON.stringify(snapshots));
    } catch (e) {}
  }

  // Print only this one worksheet card
  function printOneWorksheet(card) {
    // Sync live field values to HTML attributes so they survive cloning
    var fields = getFields(card);
    for (var fi = 0; fi < fields.length; fi++) {
      var f = fields[fi];
      if (f.type === 'checkbox') {
        if (f.checked) f.setAttribute('checked', 'checked');
        else f.removeAttribute('checked');
      } else if (f.tagName === 'TEXTAREA') {
        f.textContent = f.value;
      } else {
        f.setAttribute('value', f.value);
      }
    }

    var clone = card.cloneNode(true);
    var actions = clone.querySelector('.ws-actions');
    if (actions) actions.parentNode.removeChild(actions);

    var win = window.open('', '_blank');
    if (!win) {
      // Fallback if popups are blocked
      document.body.classList.add('print-worksheet-only');
      card.classList.add('is-printing');
      window.print();
      document.body.classList.remove('print-worksheet-only');
      card.classList.remove('is-printing');
      return;
    }

    win.document.write(
      '<!DOCTYPE html><html><head><meta charset="utf-8"><title>Worksheet</title>' +
      '<style>' + PRINT_STYLES + '</style></head><body>' +
      clone.outerHTML +
      '</body></html>'
    );
    win.document.close();
    win.focus();
    setTimeout(function () { win.print(); }, 400);
  }

  var PRINT_STYLES = [
    '* { box-sizing: border-box; }',
    'body { font-family: system-ui, -apple-system, Segoe UI, sans-serif; font-size: 13px; line-height: 1.5; color: #1a1d26; padding: 0.5rem 1.5rem; }',
    '.worksheet-card { padding: 1rem 0; }',
    '.worksheet-card h3 { font-size: 1.05rem; margin: 0 0 0.75rem; border-bottom: 2px solid #1e5a8a; padding-bottom: 0.4rem; color: #1e5a8a; }',
    '.ws-field { margin-bottom: 0.6rem; }',
    '.ws-field label { display: block; font-weight: 600; font-size: 0.85rem; margin-bottom: 0.2rem; }',
    '.ws-field input[type="text"], .ws-field input[type="number"] { width: 100%; border: 1px solid #ccc; border-radius: 4px; padding: 0.35rem 0.5rem; font-size: 0.85rem; font-family: inherit; background: #fafafa; }',
    '.ws-field textarea { width: 100%; border: 1px solid #ccc; border-radius: 4px; padding: 0.4rem 0.5rem; font-size: 0.85rem; font-family: inherit; min-height: 60px; background: #fafafa; }',
    '.ws-table { width: 100%; border-collapse: collapse; font-size: 0.82rem; margin: 0.4rem 0 0.75rem; }',
    '.ws-table th { background: #ede9f7; padding: 0.35rem 0.5rem; text-align: left; border: 1px solid #c4b5fd; font-size: 0.78rem; }',
    '.ws-table td { border: 1px solid #c4b5fd; padding: 0; }',
    '.ws-table td input[type="text"] { width: 100%; border: none; background: transparent; padding: 0.3rem 0.4rem; font-size: 0.82rem; font-family: inherit; }',
  ].join('\n');
})();
