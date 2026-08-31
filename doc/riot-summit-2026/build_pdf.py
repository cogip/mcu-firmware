#!/usr/bin/env python3
"""Render the Marp deck to PDF with Mermaid diagrams pre-rendered to inline SVG.

Marp's headless PDF export prints before a CDN-loaded Mermaid script finishes,
leaving diagrams blank or mis-sized. This script side-steps that entirely:

  1. extract every <div class="mermaid">...</div> block from the source deck,
  2. render each to a standalone SVG with mermaid-cli (mmdc), fonts settled,
  3. emit a build copy where each block is replaced by its inline SVG,
     and the runtime <script> is dropped,
  4. hand that static deck to marp --pdf (no JS needed at print time).

Usage: python3 build_pdf.py [source.md] [out.pdf]
"""
import base64
import json
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "cogip-riot-firmware.md")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "cogip-riot-firmware.pdf")

MERMAID_RE = re.compile(r'<div class="mermaid">\s*(.*?)\s*</div>', re.DOTALL)
SCRIPT_RE = re.compile(r'<script type="module">.*?</script>', re.DOTALL)

PUPPETEER_CFG = os.path.join(HERE, "puppeteer-config.json")

# Chrome/Chromium used by both mermaid-cli and marp. Overridable via $CHROME_PATH;
# otherwise taken from puppeteer-config.json, then a sane default.
with open(PUPPETEER_CFG) as _f:
    PUPPETEER_BASE = json.load(_f)
CHROME = os.environ.get("CHROME_PATH") or PUPPETEER_BASE.get(
    "executablePath", "/usr/bin/google-chrome"
)


def puppeteer_cfg(work):
    """Write a puppeteer config pinned to $CHROME for mermaid-cli.

    mermaid-cli only learns the browser path from this file (it ignores
    $CHROME_PATH), so $CHROME_PATH would otherwise apply to marp alone and the
    diagram step would keep using the checked-in default.
    """
    path = os.path.join(work, "puppeteer.json")
    with open(path, "w") as f:
        json.dump(dict(PUPPETEER_BASE, executablePath=CHROME), f)
    return path


def mmdc(mmd_text, work, idx, cfg):
    """Render one Mermaid source to an SVG string via mermaid-cli."""
    src = os.path.join(work, f"d{idx}.mmd")
    dst = os.path.join(work, f"d{idx}.svg")
    with open(src, "w") as f:
        f.write(mmd_text)
    cmd = [
        "npx", "--yes", "@mermaid-js/mermaid-cli",
        "-i", src, "-o", dst,
        "-b", "transparent",
        "-p", cfg,
        "-c", os.path.join(HERE, "mermaid-config.json"),
        "--quiet",
    ]
    env = dict(os.environ, PUPPETEER_SKIP_DOWNLOAD="1")
    subprocess.run(cmd, check=True, env=env)
    with open(dst, "rb") as f:
        raw = f.read()
    # Embed as a data-URI <img>. Rendered as an image, the SVG lays out in its
    # own context: the slide's font/CSS cannot reach its foreignObject labels,
    # so nothing re-flows or clips at print time.
    b64 = base64.b64encode(raw).decode("ascii")
    return f'<img class="diagram" src="data:image/svg+xml;base64,{b64}" />'


def main():
    with open(SRC) as f:
        doc = f.read()

    blocks = MERMAID_RE.findall(doc)
    print(f"[build] {len(blocks)} Mermaid diagrams to render")

    with tempfile.TemporaryDirectory() as work:
        cfg = puppeteer_cfg(work)
        svgs = [mmdc(b, work, i, cfg) for i, b in enumerate(blocks)]
        it = iter(svgs)
        built = MERMAID_RE.sub(
            lambda _m: f'<div class="mermaid">{next(it)}</div>', doc
        )
        built = SCRIPT_RE.sub("", built)

        # Write the build copy next to the source (not in the temp dir) so that
        # relative asset paths in the deck (e.g. the logo png) still resolve.
        build_md = os.path.join(HERE, ".deck.build.md")
        with open(build_md, "w") as f:
            f.write(built)

        print("[build] marp --pdf ...")
        env = dict(
            os.environ,
            PUPPETEER_EXECUTABLE_PATH=CHROME,
            CHROME_PATH=CHROME,
        )
        html_out = os.path.splitext(OUT)[0] + ".html"
        try:
            # PDF (offline fallback) and a self-contained HTML with the same
            # pre-rendered diagrams (reliable for presenting; press "p" for the
            # presenter view with the speaker notes).
            for out in (OUT, html_out):
                subprocess.run(
                    ["npx", "--yes", "@marp-team/marp-cli",
                     "--html", "--allow-local-files",
                     build_md, "-o", out],
                    check=True, env=env,
                )
        finally:
            os.remove(build_md)
    print(f"[build] wrote {OUT}")
    print(f"[build] wrote {html_out}  (present from this; press 'p' for notes)")


if __name__ == "__main__":
    main()
