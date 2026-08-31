# RIOT Summit 2026, Cogip firmware deck

Marp slide deck: `cogip-riot-firmware.md`.

COGIP brand theme (black + red). The title/watermark logo is `cogip-logo.png`
(local file), and diagrams are Mermaid rendered client-side from a CDN in preview
mode. Because the deck uses raw HTML and a local image, Marp needs both `--html`
and `--allow-local-files`.

## Presenting (recommended)

`./build-pdf.sh` also emits `cogip-riot-firmware.html` with the **diagrams
pre-rendered** (baked SVG, no CDN). Present from that file:

```bash
./build-pdf.sh
xdg-open cogip-riot-firmware.html    # press "p" for the presenter view + notes
```

Fullscreen: `f`. Next / previous: arrows. Presenter view (speaker notes, timer,
next slide): `p`. Works offline.

Do **not** present from `marp --preview` of the raw `.md`: there Mermaid renders
live from a CDN and mis-sizes the boxes (text clipped). The pre-rendered HTML
above avoids that.

## Quick draft preview (diagrams may clip)

```bash
npx @marp-team/marp-cli --html --allow-local-files --watch cogip-riot-firmware.md
```

Fine for editing text/layout; not for the final presentation.

## Export to PDF (recommended)

```bash
./build-pdf.sh                  # -> cogip-riot-firmware.pdf (runs from anywhere)
# equivalently: python3 build_pdf.py [<source.md> <out.pdf>]
# other browser: CHROME_PATH=/path/to/chromium ./build-pdf.sh
```

Why a script: Marp's headless PDF prints **before** the CDN Mermaid script runs,
so diagrams come out blank; and Mermaid's HTML labels re-flow under the slide
font and clip. `build_pdf.py` avoids both by pre-rendering every diagram to SVG
with `mermaid-cli` (fonts settled), embeds each as an isolated data-URI `<img>`
(immune to the slide CSS), strips the runtime script, then runs `marp --pdf` on
the now-static deck.

Requirements: `python3`, `npx` (pulls `@mermaid-js/mermaid-cli` and `marp-cli`
on first run), and a Chrome/Chromium at `/usr/bin/google-chrome` (edit
`puppeteer-config.json` if elsewhere). Mermaid render options live in
`mermaid-config.json`.

### Alternatives (no script)

1. **VS Code**: install *Marp for VS Code*, open the deck, "Export slide deck"
   → PDF. The preview evaluates the Mermaid script.
2. **Print from the HTML**: open `deck.html`, wait for diagrams, browser Print →
   Save as PDF (landscape).

## Notes

- Theme: `uncover`. Change in the front-matter `theme:` field.
- Language: English.
- No external assets: everything is inline in the single `.md`.
