#!/bin/bash

echo "=== Building Gateway Firmware ==="
echo "Sourcing ESP-IDF environment..."

# Source ESP-IDF
source ~/esp/esp-idf/export.sh

echo "Building gateway firmware..."
idf.py build

if [ $? -eq 0 ]; then
    echo "✅ Gateway firmware build successful!"
    echo "Binary size information:"
    idf.py size
else
    echo "❌ Gateway firmware build failed!"
    exit 1
fi 