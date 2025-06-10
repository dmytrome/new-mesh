#!/bin/bash

# Default port - can be overridden as first argument
PORT=${1:-/dev/cu.usbmodem101}

echo "=== Flashing Gateway Firmware ==="
echo "Target port: $PORT"
echo "Sourcing ESP-IDF environment..."

# Source ESP-IDF
source ~/esp/esp-idf/export.sh

echo "Flashing and monitoring gateway firmware..."
idf.py -p $PORT flash monitor

echo "Gateway flash and monitor completed." 