# Location Configuration Guide

This guide explains how to configure the BME280 pressure sensor for different countries and altitudes.

## Quick Configuration

To change your location settings, edit `main/include/location_config.h`:

### 1. Set Your Altitude

```c
// Set your location's altitude above sea level in meters
#define LOCATION_ALTITUDE_METERS      100.0f
```

### 2. Select Your Country/Region

Uncomment ONE of the following configurations:

```c
// UK Standard Atmosphere (default)
#define LOCATION_UK
// #define LOCATION_USA
// #define LOCATION_TROPICAL
// #define LOCATION_CUSTOM
```

## Common Locations

| City/Region | Altitude (m) | Configuration |
|-------------|--------------|---------------|
| London, UK | ~35 | `LOCATION_UK` + `35.0f` |
| Manchester, UK | ~38 | `LOCATION_UK` + `38.0f` |
| Edinburgh, UK | ~47 | `LOCATION_UK` + `47.0f` |
| New York, USA | ~10 | `LOCATION_USA` + `10.0f` |
| Los Angeles, USA | ~93 | `LOCATION_USA` + `93.0f` |
| Denver, USA | ~1609 | `LOCATION_USA` + `1609.0f` |
| Singapore | ~15 | `LOCATION_TROPICAL` + `15.0f` |
| Tokyo, Japan | ~40 | `LOCATION_USA` + `40.0f` |
| Sydney, Australia | ~6 | `LOCATION_USA` + `6.0f` |
| Toronto, Canada | ~76 | `LOCATION_USA` + `76.0f` |

## Example Configurations

### For London, UK
```c
#define LOCATION_ALTITUDE_METERS      35.0f
#define LOCATION_UK
// #define LOCATION_USA
// #define LOCATION_TROPICAL
// #define LOCATION_CUSTOM
```

### For Denver, USA (High Altitude)
```c
#define LOCATION_ALTITUDE_METERS      1609.0f
// #define LOCATION_UK
#define LOCATION_USA
// #define LOCATION_TROPICAL
// #define LOCATION_CUSTOM
```

### For Singapore (Tropical)
```c
#define LOCATION_ALTITUDE_METERS      15.0f
// #define LOCATION_UK
// #define LOCATION_USA
#define LOCATION_TROPICAL
// #define LOCATION_CUSTOM
```

## Custom Configuration

If you need custom atmospheric parameters, use `LOCATION_CUSTOM` and modify the values:

```c
#define LOCATION_ALTITUDE_METERS      500.0f
// #define LOCATION_UK
// #define LOCATION_USA
// #define LOCATION_TROPICAL
#define LOCATION_CUSTOM

// Then modify these values as needed:
#define ATMOSPHERE_SEA_LEVEL_TEMP       20.0f    // °C at sea level
#define ATMOSPHERE_TEMP_LAPSE_RATE      0.0065f  // °C/m temperature lapse rate
#define ATMOSPHERE_GRAVITY              9.80665f // m/s² standard gravity
#define ATMOSPHERE_GAS_CONSTANT         287.05f  // J/(kg·K) specific gas constant for dry air
```

## What This Does

The configuration affects how the BME280 pressure sensor compensates for altitude:

1. **Raw Pressure**: The sensor reads the actual atmospheric pressure at your altitude
2. **Sea Level Compensation**: The firmware converts this to sea level pressure using:
   - Your altitude above sea level
   - Country-specific atmospheric parameters (temperature, lapse rate, etc.)
3. **Result**: You get pressure readings that are comparable to weather forecasts and other weather stations

## Verification

After configuration, the sensor logs will show:
```
Temperature: 20.50°C, Humidity: 65.20%, Pressure: 1013.25 hPa (sea level, UK, altitude: 35m)
```

This indicates the pressure has been compensated for your location's altitude and atmospheric conditions. 