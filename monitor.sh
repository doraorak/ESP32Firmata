#!/usr/bin/env bash
# Watch the serial log (boot messages, Wi-Fi IP, Bonjour). Press Ctrl-C to quit.
#
#   ./monitor.sh                    # auto-detects the USB port
#   ./monitor.sh /dev/cu.wchusbserial110
set -e
cd "$(dirname "$0")"

PORT="${1:-$(ls /dev/cu.* 2>/dev/null | grep -iE 'usbserial|wchusbserial|SLAB' | head -1)}"
[ -z "$PORT" ] && { echo "❌ No ESP32 serial port found."; exit 1; }

echo "→ Monitoring $PORT @115200.  Press Ctrl-C to quit."
arduino-cli monitor -p "$PORT" -c baudrate=115200
