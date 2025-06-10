#!/bin/bash

# Default port - can be overridden as first argument
PORT=${1:-/dev/cu.usbmodem1101}

echo "=== Flashing Sensor Firmware ==="
echo "Target port: $PORT"
echo "Sourcing ESP-IDF environment..."

# Source ESP-IDF
source ~/esp/esp-idf/export.sh

echo "Flashing and monitoring sensor firmware..."
idf.py -p $PORT flash monitor

echo "Sensor flash and monitor completed." 