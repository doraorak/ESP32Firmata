#!/usr/bin/env bash
# Build + upload the C++ ESP32Firmata sketch in one go — the same thing the
# Arduino IDE "Upload" button does, but from the terminal.
#
#   ./flash.sh                      # auto-detects the USB port
#   ./flash.sh /dev/cu.wchusbserial110   # or name the port yourself
set -e
cd "$(dirname "$0")"

PORT="${1:-$(ls /dev/cu.* 2>/dev/null | grep -iE 'usbserial|wchusbserial|SLAB' | head -1)}"
if [ -z "$PORT" ]; then
  echo "❌ No ESP32 serial port found. Plug the board into USB and re-run,"
  echo "   or pass it explicitly:   ./flash.sh /dev/cu.wchusbserial110"
  exit 1
fi

echo "→ Building + uploading to $PORT …"
arduino-cli compile --upload -p "$PORT" --fqbn esp32:esp32:esp32:PartitionScheme=huge_app .
echo "✅ Uploaded. To watch the boot log and find the board's IP:   ./monitor.sh"
