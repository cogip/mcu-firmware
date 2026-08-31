#!/usr/bin/env bash
# Build the RIOT Summit deck to PDF with Mermaid diagrams pre-rendered.
# Runs from anywhere. Override the browser with CHROME_PATH=... ./build-pdf.sh
set -euo pipefail
cd "$(dirname "$0")"
exec python3 build_pdf.py "$@"
