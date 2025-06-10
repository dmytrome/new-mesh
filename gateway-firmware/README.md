# Gateway Firmware

ESP32-S3 Mesh Network Gateway

## Requirements

- **Hardware**: ESP32-S3 with 2MB+ flash
- **Environment**: ESP-IDF v5.4+

## Build & Deploy

### Quick Start
```bash
# Build
./build.sh

# Flash & Monitor (default port)
./flash.sh

# Flash to custom port
./flash.sh /dev/cu.usbmodem102
```

### Manual Commands
```bash
# Source ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Build
idf.py build

# Flash and monitor
idf.py -p /dev/cu.usbmodem101 flash monitor

# Build size info
idf.py size
```

## Development

### Clean Build
```bash
idf.py fullclean
./build.sh
```

### Monitor Only
```bash
idf.py -p /dev/cu.usbmodem101 monitor
```
