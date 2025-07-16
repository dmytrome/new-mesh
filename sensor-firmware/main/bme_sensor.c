#include "bme_sensor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

// Standard atmospheric parameters for pressure compensation
#define ATMOSPHERE_SEA_LEVEL_TEMP       15.0f    // °C at sea level
#define ATMOSPHERE_TEMP_LAPSE_RATE      0.0065f  // °C/m temperature lapse rate
#define ATMOSPHERE_GRAVITY              9.80665f // m/s² standard gravity
#define ATMOSPHERE_GAS_CONSTANT         287.05f  // J/(kg·K) specific gas constant for dry air

static const char* TAG = "BME_SENSOR";

/*******************************************************
 *                Private Variables
 *******************************************************/
static bool bme_initialized = false;
static bme_sensor_type_t sensor_type = BME_TYPE_UNKNOWN;
static uint8_t bme_i2c_addr = BME280_I2C_ADDR;
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t bme_dev_handle = NULL;

// BME280 calibration parameters
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} bme280_calib_data_t;

static bme280_calib_data_t calib_data;
static int32_t t_fine; // Global variable for temperature compensation

/*******************************************************
 *                Private Functions
 *******************************************************/

static esp_err_t bme_i2c_read_reg(uint8_t reg_addr, uint8_t* data, size_t len) {
    if (bme_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return i2c_master_transmit_receive(bme_dev_handle, &reg_addr, 1, data, len, 1000);
}

static esp_err_t bme_i2c_write_reg(uint8_t reg_addr, uint8_t data) {
    if (bme_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t write_data[2] = {reg_addr, data};
    return i2c_master_transmit(bme_dev_handle, write_data, 2, 1000);
}

static esp_err_t bme_detect_sensor_type(void) {
    uint8_t chip_id;
    esp_err_t ret;
    
    ret = bme_i2c_read_reg(BME280_REG_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
        
    switch (chip_id) {
        case BME280_CHIP_ID:
            sensor_type = BME_TYPE_BME280;
            break;
        default:
            ESP_LOGE(TAG, "Unknown chip ID: 0x%02X (expected 0x%02X for BME280)", 
                     chip_id, BME280_CHIP_ID);
            sensor_type = BME_TYPE_UNKNOWN;
            return ESP_FAIL;
    }
    
    return ESP_OK;
}

static esp_err_t bme280_read_calibration_data(void) {
    uint8_t calib_data_raw[32];
    esp_err_t ret;
    
    // Read temperature and pressure calibration data (0x88-0x9F)
    ret = bme_i2c_read_reg(0x88, calib_data_raw, 24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data part 1");
        return ret;
    }
    
    // Read humidity calibration data H1 (0xA1)
    ret = bme_i2c_read_reg(0xA1, &calib_data_raw[24], 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data part 2");
        return ret;
    }
    
    // Read humidity calibration data H2-H6 (0xE1-0xE7)
    ret = bme_i2c_read_reg(0xE1, &calib_data_raw[25], 7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data part 3");
        return ret;
    }
    
    // Parse calibration data (little-endian format)
    calib_data.dig_T1 = (calib_data_raw[1] << 8) | calib_data_raw[0];
    calib_data.dig_T2 = (int16_t)((calib_data_raw[3] << 8) | calib_data_raw[2]);
    calib_data.dig_T3 = (int16_t)((calib_data_raw[5] << 8) | calib_data_raw[4]);
    
    calib_data.dig_P1 = (calib_data_raw[7] << 8) | calib_data_raw[6];
    calib_data.dig_P2 = (int16_t)((calib_data_raw[9] << 8) | calib_data_raw[8]);
    calib_data.dig_P3 = (int16_t)((calib_data_raw[11] << 8) | calib_data_raw[10]);
    calib_data.dig_P4 = (int16_t)((calib_data_raw[13] << 8) | calib_data_raw[12]);
    calib_data.dig_P5 = (int16_t)((calib_data_raw[15] << 8) | calib_data_raw[14]);
    calib_data.dig_P6 = (int16_t)((calib_data_raw[17] << 8) | calib_data_raw[16]);
    calib_data.dig_P7 = (int16_t)((calib_data_raw[19] << 8) | calib_data_raw[18]);
    calib_data.dig_P8 = (int16_t)((calib_data_raw[21] << 8) | calib_data_raw[20]);
    calib_data.dig_P9 = (int16_t)((calib_data_raw[23] << 8) | calib_data_raw[22]);
    
    // Humidity calibration data
    calib_data.dig_H1 = calib_data_raw[24];
    calib_data.dig_H2 = (int16_t)((calib_data_raw[26] << 8) | calib_data_raw[25]);
    calib_data.dig_H3 = calib_data_raw[27];
    calib_data.dig_H4 = (int16_t)((calib_data_raw[28] << 4) | (calib_data_raw[29] & 0x0F));
    calib_data.dig_H5 = (int16_t)((calib_data_raw[30] << 4) | (calib_data_raw[29] >> 4));
    calib_data.dig_H6 = (int8_t)calib_data_raw[31];
    
    return ESP_OK;
}

static esp_err_t bme_configure_sensor(void) {
    esp_err_t ret;
    
    if (sensor_type == BME_TYPE_BME280) {
        // Configure humidity oversampling (before CTRL_MEAS!)
        ret = bme_i2c_write_reg(BME280_REG_CTRL_HUM, 0x01); // Humidity oversampling x1
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure humidity control");
            return ret;
        }
        
        // Configure temperature and pressure oversampling + mode
        ret = bme_i2c_write_reg(BME280_REG_CTRL_MEAS, 0x27); // Temp x1, Press x1, Normal mode
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure measurement control");
            return ret;
        }
        
        // Configure config register
        ret = bme_i2c_write_reg(BME280_REG_CONFIG, 0xA0); // Standby 1000ms, filter off
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure config register");
            return ret;
        }
        
        // Read calibration data
        ret = bme280_read_calibration_data();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read calibration data");
            return ret;
        }
    }
    
    return ESP_OK;
}

// BME280 temperature compensation
static int32_t bme280_compensate_temperature(int32_t adc_T) {
    int32_t var1, var2, T;
    
    var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    
    return T;
}

// BME280 pressure compensation
static uint32_t bme280_compensate_pressure(int32_t adc_P) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
    
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    return (uint32_t)p;
}

// BME280 humidity compensation
static uint32_t bme280_compensate_humidity(int32_t adc_H) {
    int32_t v_x1_u32r;
    
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib_data.dig_H4) << 20) - (((int32_t)calib_data.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib_data.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calib_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)calib_data.dig_H2) + 8192) >> 14));
    
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib_data.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    
    return (uint32_t)(v_x1_u32r >> 12);
}

// Configurable altitude compensation for pressure readings
// Converts pressure to sea level pressure using configurable atmospheric parameters
static float bme280_altitude_compensation(float pressure_hpa, float temperature_c, float altitude_m) {
    // Use configurable atmospheric parameters from location_config.h
    const float temp_lapse_rate = ATMOSPHERE_TEMP_LAPSE_RATE;
    const float gravity = ATMOSPHERE_GRAVITY;
    const float gas_constant = ATMOSPHERE_GAS_CONSTANT;

    // Use International Standard Atmosphere (ISA) barometric formula
    // P_sea = P_alt * (1 - L*h/T)^(-g/(R*L))
    // where: L = lapse rate, h = altitude, T = temperature in Kelvin
    // This is more accurate than the exponential approximation
    
    float temp_kelvin = temperature_c + 273.15f;
    float exponent = gravity / (gas_constant * temp_lapse_rate);
    float ratio = 1.0f - (temp_lapse_rate * altitude_m) / temp_kelvin;
    float sea_level_pressure = pressure_hpa * powf(ratio, -exponent);
    
    return sea_level_pressure;
}

/*******************************************************
 *                Public Functions
 *******************************************************/

esp_err_t bme_sensor_init(void) {
    esp_err_t ret;
    
    if (bme_initialized) {
        return ESP_OK;
    }
    
    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = BME_I2C_PORT,
        .sda_io_num = BME_I2C_SDA_PIN,
        .scl_io_num = BME_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };
    
    ret = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for sensor to be ready
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Try to detect sensor at different addresses
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_I2C_ADDR,
        .scl_speed_hz = BME_I2C_FREQ_HZ,
    };
    
    // Try primary address first (0x76)
    bme_i2c_addr = BME280_I2C_ADDR;
    dev_config.device_address = bme_i2c_addr;
    
    ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &bme_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BME device to I2C bus: %s", esp_err_to_name(ret));
        i2c_del_master_bus(i2c_bus_handle);
        return ret;
    }
    
    // Detect sensor type
    ret = bme_detect_sensor_type();
    if (ret != ESP_OK) {
        // Try alternate address (0x77)
        i2c_master_bus_rm_device(bme_dev_handle);
        bme_i2c_addr = 0x77; // Alternate BME280 address
        dev_config.device_address = bme_i2c_addr;
        ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &bme_dev_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add BME device to I2C bus (alternate address): %s", esp_err_to_name(ret));
            i2c_del_master_bus(i2c_bus_handle);
            return ret;
        }
        
        ret = bme_detect_sensor_type();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to detect BME280 sensor at both addresses (0x%02X and 0x77)", 
                     BME280_I2C_ADDR);
            ESP_LOGE(TAG, "Check wiring: SDA=%d, SCL=%d, VCC=3.3V, GND", 
                     BME_I2C_SDA_PIN, BME_I2C_SCL_PIN);
            i2c_master_bus_rm_device(bme_dev_handle);
            i2c_del_master_bus(i2c_bus_handle);
            return ret;
        }
    }
    
    // Configure sensor
    ret = bme_configure_sensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BME sensor: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(bme_dev_handle);
        i2c_del_master_bus(i2c_bus_handle);
        return ret;
    }
    
    bme_initialized = true;
    ESP_LOGI(TAG, "BME280 sensor initialized successfully at address 0x%02X", bme_i2c_addr);
    
    return ESP_OK;
}

esp_err_t bme_sensor_read_temp_hum_pressure(float* temperature, float* humidity, float* pressure) {
    if (!bme_initialized) {
        ESP_LOGE(TAG, "BME sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!temperature || !humidity || !pressure) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t data[8];
    esp_err_t ret = bme_i2c_read_reg(BME280_REG_PRESS_MSB, data, 8);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return ret;
    }
    
    // Extract raw ADC values
    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    int32_t adc_H = (data[6] << 8) | data[7];
    
    // Compensate temperature (needed for pressure and humidity)
    int32_t temp_compensated = bme280_compensate_temperature(adc_T);
    *temperature = temp_compensated / 100.0f;
    
    // Compensate pressure
    uint32_t press_compensated = bme280_compensate_pressure(adc_P);
    float raw_pressure = press_compensated / 256.0f; // Pa
    float raw_pressure_hpa = raw_pressure / 100.0f; // Convert to hPa
    
    // Apply configurable altitude compensation
    *pressure = bme280_altitude_compensation(raw_pressure_hpa, *temperature, CONFIG_LOCATION_ALTITUDE_METERS);
    
    // Compensate humidity
    uint32_t hum_compensated = bme280_compensate_humidity(adc_H);
    *humidity = hum_compensated / 1024.0f;
    
    
    return ESP_OK;
}

bme_sensor_type_t bme_sensor_get_type(void) {
    return sensor_type;
}

bool bme_sensor_is_available(void) {
    return bme_initialized && (sensor_type != BME_TYPE_UNKNOWN);
}

esp_err_t bme_sensor_deinit(void) {
    if (bme_initialized) {
        if (bme_dev_handle) {
            i2c_master_bus_rm_device(bme_dev_handle);
            bme_dev_handle = NULL;
        }
        if (i2c_bus_handle) {
            i2c_del_master_bus(i2c_bus_handle);
            i2c_bus_handle = NULL;
        }
        bme_initialized = false;
        sensor_type = BME_TYPE_UNKNOWN;
        ESP_LOGI(TAG, "BME sensor deinitialized");
    }
    return ESP_OK;
} 