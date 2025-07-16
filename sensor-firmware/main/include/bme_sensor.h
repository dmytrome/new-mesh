#ifndef BME_SENSOR_H
#define BME_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/*******************************************************
 *                BME280 Configuration
 *******************************************************/
#define BME_I2C_PORT             0
#define BME_I2C_SDA_PIN          8
#define BME_I2C_SCL_PIN          9
#define BME_I2C_FREQ_HZ          100000

/*******************************************************
 *                BME280 Configuration
 *******************************************************/

// BME280 I2C address
#define BME280_I2C_ADDR          0x76    // or 0x77

// BME280 Register addresses
#define BME280_REG_CHIP_ID       0xD0
#define BME280_REG_RESET         0xE0
#define BME280_REG_CTRL_HUM      0xF2
#define BME280_REG_CTRL_MEAS     0xF4
#define BME280_REG_CONFIG        0xF5
#define BME280_REG_PRESS_MSB     0xF7
#define BME280_REG_TEMP_MSB      0xFA
#define BME280_REG_HUM_MSB       0xFD

// BME280 Chip ID
#define BME280_CHIP_ID           0x60

/*******************************************************
 *                BME280 Sensor Data Structure
 *******************************************************/
typedef struct {
    float temperature;               // Temperature in Celsius
    float humidity;                  // Relative humidity in %
    float pressure;                  // Pressure in hPa (sea level compensated)
    bool data_valid;                 // True if reading is valid
    uint32_t timestamp;              // When reading was taken
} bme_data_t;

typedef enum {
    BME_TYPE_UNKNOWN = 0,
    BME_TYPE_BME280
} bme_sensor_type_t;

/*******************************************************
 *                Public Functions
 *******************************************************/

/**
 * @brief Initialize BME280 sensor
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bme_sensor_init(void);

/**
 * @brief Read temperature, humidity, and pressure from BME280 sensor
 * @param temperature Pointer to store temperature (°C)
 * @param humidity Pointer to store humidity (%)
 * @param pressure Pointer to store pressure (hPa, sea level compensated)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bme_sensor_read_temp_hum_pressure(float* temperature, float* humidity, float* pressure);

/**
 * @brief Get sensor type (BME280)
 * @return Detected sensor type
 */
bme_sensor_type_t bme_sensor_get_type(void);

/**
 * @brief Check if BME280 sensor is available and working
 * @return true if sensor is available, false otherwise
 */
bool bme_sensor_is_available(void);

/**
 * @brief Deinitialize BME280 sensor and free resources
 * @return ESP_OK on success
 */
esp_err_t bme_sensor_deinit(void);

#endif // BME_SENSOR_H 