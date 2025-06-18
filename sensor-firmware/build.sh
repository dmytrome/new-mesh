#!/bin/bash

echo "=== Building Sensor Firmware ==="
echo "Sourcing ESP-IDF environment..."

# Source ESP-IDF
source ~/esp/esp-idf/export.sh

rm -rf build

echo "Building ssensor firmware..."
idf.py build

if [ $? -eq 0 ]; then
    echo "✅ Sensor firmware build successful!"
    echo "Binary size information:"
    idf.py size
else
    echo "❌ Sensor firmware build failed!"
    exit 1
fi 