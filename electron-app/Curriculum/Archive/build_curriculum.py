"""
Regenerate HTML curriculum pages from Firmware/BTEC Specs sources.
Run from anywhere: python Curriculum/build_curriculum.py
"""
from __future__ import annotations

import html as html_module
import re
from pathlib import Path

import markdown

ROOT = Path(__file__).resolve().parent.parent
MATERIALS = ROOT / "Firmware" / "BTEC Specs" / "Module_Materials"
PLAN_PATH = ROOT / "Firmware" / "BTEC Specs" / "BTEC_Robot_Arm_Curriculum_Plan.md"
OUT_DIR = Path(__file__).resolve().parent

MD_EXTENSIONS = ["tables", "fenced_code", "nl2br", "sane_lists"]
MD = markdown.Markdown(extensions=MD_EXTENSIONS)


def md_to_html(text: str) -> str:
    MD.reset()
    return MD.convert(text)


def extract_plan_section(plan_text: str, module_num: int) -> str:
    start_re = re.compile(rf"(?m)^## \*\*Module {module_num}:[^\n]*\*\*\s*\n")
    m = start_re.search(plan_text)
    if not m:
        return f"*(Curriculum plan excerpt for module {module_num} was not found.)*\n"
    start_idx = m.start()
    tail = plan_text[m.end() :]
    end_re = re.compile(r"(?m)^## \*\*(?:Module \d+:[^\n]*\*\*|Curriculum Summary\*\*)")
    m2 = end_re.search(tail)
    if m2:
        section = plan_text[start_idx : m.end() + m2.start()].rstrip()
    else:
        section = plan_text[start_idx:].rstrip()
    return section + "\n"


def module_title_from_theory(theory_path: Path) -> str:
    if not theory_path.is_file():
        return f"Module {theory_path.stem.split('_')[1]}"
    first = theory_path.read_text(encoding="utf-8").splitlines()[0]
    m = re.match(r"^#\s*Module\s+\d+:\s*(.+)\s*$", first)
    return m.group(1).strip() if m else first.lstrip("# ").strip()


def nav_html(module_num: int, title: str) -> str:
    safe_title = html_module.escape(title)
    prev_link = ""
    if module_num > 1:
        prev_link = f'<a class="nav-link" href="module-{module_num - 1:02d}.html">← Previous</a>'
    next_link = ""
    if module_num < 16:
        next_link = f'<a class="nav-link" href="module-{module_num + 1:02d}.html">Next →</a>'
    return f"""<nav class="top-nav">
  <a class="nav-link" href="index.html">All modules</a>
  {prev_link}
  <span class="nav-title">Module {module_num}: {safe_title}</span>
  {next_link}
</nav>"""


def page_shell(title: str, body_inner: str, module_num: int | None = None) -> str:
    extra = ""
    if module_num is not None:
        extra = f'<meta name="curriculum-module" content="{module_num}">'
    safe_title = html_module.escape(title)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  {extra}
  <title>{safe_title}</title>
  <style>
    :root {{
      --bg: #f6f7f9;
      --card: #fff;
      --text: #1a1d26;
      --muted: #5c6370;
      --accent: #1e5a8a;
      --border: #d8dde6;
      --code-bg: #eef1f6;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: Georgia, "Times New Roman", serif;
      font-size: 17px;
      line-height: 1.55;
      color: var(--text);
      background: var(--bg);
    }}
    .wrap {{
      max-width: 52rem;
      margin: 0 auto;
      padding: 1.25rem 1.25rem 3rem;
    }}
    .top-nav {{
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      gap: 0.75rem 1rem;
      padding: 0.75rem 0;
      margin-bottom: 1.25rem;
      border-bottom: 1px solid var(--border);
      font-family: system-ui, -apple-system, Segoe UI, sans-serif;
      font-size: 0.9rem;
    }}
    .nav-title {{
      flex: 1 1 auto;
      font-weight: 600;
      color: var(--muted);
      min-width: 12rem;
    }}
    a.nav-link {{
      color: var(--accent);
      text-decoration: none;
    }}
    a.nav-link:hover {{ text-decoration: underline; }}
    h1.page-title {{
      font-family: system-ui, -apple-system, Segoe UI, sans-serif;
      font-size: 1.65rem;
      font-weight: 700;
      margin: 0 0 0.5rem;
      line-height: 1.25;
    }}
    p.lead {{
      font-family: system-ui, -apple-system, Segoe UI, sans-serif;
      color: var(--muted);
      margin: 0 0 1.5rem;
      font-size: 0.95rem;
    }}
    section.card {{
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 1.25rem 1.5rem 1.5rem;
      margin-bottom: 1.5rem;
      box-shadow: 0 1px 2px rgba(0,0,0,0.04);
    }}
    section.card h2 {{
      font-family: system-ui, -apple-system, Segoe UI, sans-serif;
      font-size: 1.15rem;
      margin: 0 0 1rem;
      padding-bottom: 0.5rem;
      border-bottom: 2px solid var(--accent);
      color: var(--accent);
    }}
    .card-inner {{
      font-family: Georgia, "Times New Roman", serif;
    }}
    .card-inner h1, .card-inner h2, .card-inner h3, .card-inner h4 {{
      font-family: system-ui, -apple-system, Segoe UI, sans-serif;
    }}
    .card-inner h1 {{ font-size: 1.35rem; }}
    .card-inner h2 {{ font-size: 1.15rem; border-bottom: none; color: var(--text); }}
    .card-inner h3 {{ font-size: 1.05rem; }}
    .card-inner h4 {{ font-size: 1rem; }}
    .card-inner code {{
      font-family: Consolas, "Courier New", monospace;
      font-size: 0.9em;
      background: var(--code-bg);
      padding: 0.1em 0.35em;
      border-radius: 4px;
    }}
    .card-inner pre {{
      background: var(--code-bg);
      padding: 1rem;
      overflow-x: auto;
      border-radius: 6px;
      border: 1px solid var(--border);
    }}
    .card-inner pre code {{
      background: none;
      padding: 0;
    }}
    .card-inner table {{
      border-collapse: collapse;
      width: 100%;
      font-size: 0.95rem;
      margin: 1rem 0;
    }}
    .card-inner th, .card-inner td {{
      border: 1px solid var(--border);
      padding: 0.45rem 0.6rem;
      text-align: left;
      vertical-align: top;
    }}
    .card-inner th {{
      background: #eef3f8;
      font-family: system-ui, -apple-system, Segoe UI, sans-serif;
      font-weight: 600;
    }}
    ul.index-list {{
      list-style: none;
      padding: 0;
      margin: 0;
      font-family: system-ui, -apple-system, Segoe UI, sans-serif;
    }}
    ul.index-list li {{
      margin: 0.35rem 0;
      padding: 0.5rem 0.65rem;
      border-radius: 6px;
      border: 1px solid var(--border);
      background: var(--card);
    }}
    ul.index-list a {{
      color: var(--accent);
      font-weight: 600;
      text-decoration: none;
    }}
    ul.index-list a:hover {{ text-decoration: underline; }}
    .index-meta {{
      display: block;
      font-size: 0.85rem;
      color: var(--muted);
      font-weight: normal;
      margin-top: 0.2rem;
    }}
  </style>
</head>
<body>
  <div class="wrap">
    {body_inner}
  </div>
</body>
</html>
"""


def build_index(entries: list[tuple[int, str]]) -> None:
    lis = []
    for num, title in entries:
        safe = html_module.escape(title)
        lis.append(
            f'<li><a href="module-{num:02d}.html">Module {num}: {safe}'
            f'<span class="index-meta">Open module → theory, worksheet, MCQ, rubric</span></a></li>'
        )
    body = f"""<nav class="top-nav">
  <span class="nav-title">Robot Arm BTEC Curriculum</span>
</nav>
<h1 class="page-title">Curriculum (HTML)</h1>
<p class="lead">Step-by-step modules aligned with <code>Firmware/BTEC Specs</code>. Each page pulls together the curriculum plan excerpt, theory notes, student worksheet, MCQs, and rubric.</p>
<section class="card">
  <h2>Modules</h2>
  <ul class="index-list">
    {chr(10).join(lis)}
  </ul>
</section>
<section class="card">
  <h2>Source files</h2>
  <div class="card-inner">
    <p>Markdown originals live in <code>Firmware/BTEC Specs/Module_Materials</code> and <code>BTEC_Robot_Arm_Curriculum_Plan.md</code>. Rebuild pages by running:</p>
    <pre><code>python Curriculum/build_curriculum.py</code></pre>
  </div>
</section>"""
    html = page_shell("Robot Arm Curriculum — Index", body, None)
    (OUT_DIR / "index.html").write_text(html, encoding="utf-8")


def main() -> None:
    plan_text = PLAN_PATH.read_text(encoding="utf-8")
    entries: list[tuple[int, str]] = []

    for n in range(1, 17):
        theory = MATERIALS / f"Module_{n}_Theory.md"
        worksheet = MATERIALS / f"Module_{n}_Worksheet.md"
        mcq = MATERIALS / f"Module_{n}_MCQ.md"
        rubric = MATERIALS / f"Module_{n}_Rubric.md"

        title = module_title_from_theory(theory)
        entries.append((n, title))

        plan_md = extract_plan_section(plan_text, n)
        parts: list[str] = []
        parts.append(nav_html(n, title))
        parts.append(f'<h1 class="page-title">Module {n}: {html_module.escape(title)}</h1>')
        parts.append(
            '<p class="lead">BTEC robot arm curriculum — one page per module. Work through sections in order.</p>'
        )

        sections = [
            ("Curriculum plan (from BTEC_Robot_Arm_Curriculum_Plan.md)", plan_md),
            ("Theory & teaching notes", theory.read_text(encoding="utf-8") if theory.is_file() else "*Missing theory file.*\n"),
            ("Student worksheet", worksheet.read_text(encoding="utf-8") if worksheet.is_file() else "*Missing worksheet.*\n"),
            ("Multiple choice questions", mcq.read_text(encoding="utf-8") if mcq.is_file() else "*Missing MCQ file.*\n"),
            ("Assessment rubric", rubric.read_text(encoding="utf-8") if rubric.is_file() else "*Missing rubric.*\n"),
        ]

        for heading, md in sections:
            inner = md_to_html(md)
            parts.append(
                f'<section class="card"><h2>{heading}</h2><div class="card-inner">{inner}</div></section>'
            )

        html = page_shell(f"Module {n}: {title}", "\n".join(parts), n)
        out_path = OUT_DIR / f"module-{n:02d}.html"
        out_path.write_text(html, encoding="utf-8")
        print("Wrote", out_path.relative_to(ROOT))

    build_index(entries)
    print("Wrote", (OUT_DIR / "index.html").relative_to(ROOT))


if __name__ == "__main__":
    main()
