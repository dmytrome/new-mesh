#include "sim800l.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

/*******************************************************
 *                Constants and Variables
 *******************************************************/
static const char *SIM800L_TAG = "sim800l";

// Configuration
static sim800l_config_t s_config = {0};
static bool s_initialized = false;
static sim800l_status_t s_status = SIM800L_STATUS_DISCONNECTED;

// UART buffer
#define SIM800L_BUF_SIZE 1024
static char s_uart_buffer[SIM800L_BUF_SIZE];

// AT command timeouts (ms)
#define AT_TIMEOUT_DEFAULT 5000
#define AT_TIMEOUT_CONNECT 30000

/*******************************************************
 *                Private Function Declarations
 *******************************************************/
static esp_err_t sim800l_send_at_command(const char* command, const char* expected_response, 
                                         uint32_t timeout_ms);
static esp_err_t sim800l_uart_setup(void);
static esp_err_t sim800l_hardware_reset(void);
static esp_err_t sim800l_check_basic_functionality(void);
static void sim800l_task(void* param);

/*******************************************************
 *                Public Interface Functions
 *******************************************************/

esp_err_t sim800l_init(const sim800l_config_t* config)
{
    if (!config) {
        ESP_LOGE(SIM800L_TAG, "Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }

    if (s_initialized) {
        ESP_LOGW(SIM800L_TAG, "Already initialized");
        return ESP_OK;
    }

    // Copy configuration
    memcpy(&s_config, config, sizeof(sim800l_config_t));
    
    ESP_LOGI(SIM800L_TAG, "Initializing SIM800L on UART%d (TX:%d, RX:%d)", 
             config->uart_port, config->tx_pin, config->rx_pin);

    // Setup UART
    ESP_ERROR_CHECK(sim800l_uart_setup());
    
    // Initialize status
    s_status = SIM800L_STATUS_INITIALIZING;
    
    // Create task for handling SIM800L operations
    xTaskCreate(sim800l_task, "sim800l_task", 4096, NULL, 5, NULL);
    
    s_initialized = true;
    ESP_LOGI(SIM800L_TAG, "SIM800L initialized successfully");
    
    return ESP_OK;
}

esp_err_t sim800l_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(SIM800L_TAG, "Deinitializing SIM800L");
    
    // Disconnect GPRS if connected
    sim800l_disconnect_gprs();
    
    // Deinitialize UART
    uart_driver_delete(s_config.uart_port);
    
    s_initialized = false;
    s_status = SIM800L_STATUS_DISCONNECTED;
    
    return ESP_OK;
}

sim800l_status_t sim800l_get_status(void)
{
    return s_status;
}

esp_err_t sim800l_connect_gprs(void)
{
    if (!s_initialized) {
        ESP_LOGE(SIM800L_TAG, "Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_status == SIM800L_STATUS_GPRS_CONNECTED) {
        ESP_LOGW(SIM800L_TAG, "GPRS already connected");
        return ESP_OK;
    }

    ESP_LOGI(SIM800L_TAG, "Connecting to GPRS...");

    // Configure APN
    char apn_cmd[128];
    snprintf(apn_cmd, sizeof(apn_cmd), "AT+CSTT=\"%s\",\"%s\",\"%s\"", 
             s_config.apn, 
             s_config.apn_username ? s_config.apn_username : "",
             s_config.apn_password ? s_config.apn_password : "");
    
    ESP_ERROR_CHECK(sim800l_send_at_command(apn_cmd, "OK", AT_TIMEOUT_DEFAULT));
    
    // Bring up GPRS connection
    ESP_ERROR_CHECK(sim800l_send_at_command("AT+CIICR", "OK", AT_TIMEOUT_CONNECT));
    
    // Get IP address
    ESP_ERROR_CHECK(sim800l_send_at_command("AT+CIFSR", ".", AT_TIMEOUT_DEFAULT));
    
    s_status = SIM800L_STATUS_GPRS_CONNECTED;
    ESP_LOGI(SIM800L_TAG, "GPRS connected successfully");
    
    return ESP_OK;
}

esp_err_t sim800l_disconnect_gprs(void)
{
    if (!s_initialized || s_status != SIM800L_STATUS_GPRS_CONNECTED) {
        return ESP_OK;
    }

    ESP_LOGI(SIM800L_TAG, "Disconnecting GPRS...");
    
    sim800l_send_at_command("AT+CIPSHUT", "SHUT OK", AT_TIMEOUT_DEFAULT);
    s_status = SIM800L_STATUS_NETWORK_REGISTERED;
    
    ESP_LOGI(SIM800L_TAG, "GPRS disconnected");
    return ESP_OK;
}

esp_err_t sim800l_http_post(const char* url, const char* data, size_t data_len, 
                           char* response, size_t response_len)
{
    if (!s_initialized || s_status != SIM800L_STATUS_GPRS_CONNECTED) {
        ESP_LOGE(SIM800L_TAG, "GPRS not connected");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(SIM800L_TAG, "Sending HTTP POST to %s", url);

    // Initialize HTTP service
    ESP_ERROR_CHECK(sim800l_send_at_command("AT+HTTPINIT", "OK", AT_TIMEOUT_DEFAULT));
    
    // Set HTTP parameters
    char http_param[256];
    snprintf(http_param, sizeof(http_param), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    ESP_ERROR_CHECK(sim800l_send_at_command(http_param, "OK", AT_TIMEOUT_DEFAULT));
    
    ESP_ERROR_CHECK(sim800l_send_at_command("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 
                                           "OK", AT_TIMEOUT_DEFAULT));
    
    // Send HTTP POST data
    char data_cmd[64];
    snprintf(data_cmd, sizeof(data_cmd), "AT+HTTPDATA=%zu,10000", data_len);
    ESP_ERROR_CHECK(sim800l_send_at_command(data_cmd, "DOWNLOAD", AT_TIMEOUT_DEFAULT));
    
    // Send actual data
    uart_write_bytes(s_config.uart_port, data, data_len);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Execute HTTP POST
    ESP_ERROR_CHECK(sim800l_send_at_command("AT+HTTPACTION=1", "OK", AT_TIMEOUT_DEFAULT));
    
    // Wait for response and read if needed
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    if (response && response_len > 0) {
        sim800l_send_at_command("AT+HTTPREAD", "OK", AT_TIMEOUT_DEFAULT);
        // Copy response data (simplified)
        strncpy(response, s_uart_buffer, response_len - 1);
        response[response_len - 1] = '\0';
    }
    
    // Terminate HTTP service
    sim800l_send_at_command("AT+HTTPTERM", "OK", AT_TIMEOUT_DEFAULT);
    
    ESP_LOGI(SIM800L_TAG, "HTTP POST completed");
    return ESP_OK;
}

esp_err_t sim800l_send_sms(const char* phone_number, const char* message)
{
    if (!s_initialized || s_status < SIM800L_STATUS_NETWORK_REGISTERED) {
        ESP_LOGE(SIM800L_TAG, "Network not available");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(SIM800L_TAG, "Sending SMS to %s", phone_number);
    
    // Set SMS text mode
    ESP_ERROR_CHECK(sim800l_send_at_command("AT+CMGF=1", "OK", AT_TIMEOUT_DEFAULT));
    
    // Set SMS destination
    char sms_cmd[64];
    snprintf(sms_cmd, sizeof(sms_cmd), "AT+CMGS=\"%s\"", phone_number);
    ESP_ERROR_CHECK(sim800l_send_at_command(sms_cmd, ">", AT_TIMEOUT_DEFAULT));
    
    // Send message content
    uart_write_bytes(s_config.uart_port, message, strlen(message));
    uart_write_bytes(s_config.uart_port, "\x1A", 1); // Ctrl+Z to send
    
    vTaskDelay(pdMS_TO_TICKS(10000)); // Wait for SMS to be sent
    
    ESP_LOGI(SIM800L_TAG, "SMS sent");
    return ESP_OK;
}

bool sim800l_is_sim_ready(void)
{
    if (!s_initialized) {
        return false;
    }

    esp_err_t err = sim800l_send_at_command("AT+CPIN?", "+CPIN: READY", AT_TIMEOUT_DEFAULT);
    return (err == ESP_OK);
}

int sim800l_get_signal_strength(void)
{
    if (!s_initialized) {
        return 99; // Unknown
    }

    if (sim800l_send_at_command("AT+CSQ", "+CSQ:", AT_TIMEOUT_DEFAULT) == ESP_OK) {
        // Parse signal strength from response (simplified)
        char* csq_pos = strstr(s_uart_buffer, "+CSQ:");
        if (csq_pos) {
            int rssi;
            if (sscanf(csq_pos, "+CSQ: %d", &rssi) == 1) {
                return rssi;
            }
        }
    }
    
    return 99; // Unknown
}

/*******************************************************
 *                Private Functions
 *******************************************************/

static esp_err_t sim800l_uart_setup(void)
{
    uart_config_t uart_config = {
        .baud_rate = s_config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(s_config.uart_port, SIM800L_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(s_config.uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(s_config.uart_port, s_config.tx_pin, s_config.rx_pin, 
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}

static esp_err_t sim800l_send_at_command(const char* command, const char* expected_response, 
                                        uint32_t timeout_ms)
{
    if (!command || !expected_response) {
        return ESP_ERR_INVALID_ARG;
    }

    // Clear buffer
    memset(s_uart_buffer, 0, SIM800L_BUF_SIZE);
    uart_flush(s_config.uart_port);

    // Send command
    uart_write_bytes(s_config.uart_port, command, strlen(command));
    uart_write_bytes(s_config.uart_port, "\r\n", 2);

    // Wait for response
    uint32_t start_time = xTaskGetTickCount();
    int len = 0;
    
    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(timeout_ms)) {
        int bytes_available = uart_read_bytes(s_config.uart_port, 
                                            (uint8_t*)s_uart_buffer + len, 
                                            SIM800L_BUF_SIZE - len - 1, 
                                            pdMS_TO_TICKS(100));
        if (bytes_available > 0) {
            len += bytes_available;
            s_uart_buffer[len] = '\0';
            
            // Check if expected response is found
            if (strstr(s_uart_buffer, expected_response)) {
                ESP_LOGD(SIM800L_TAG, "AT Command: %s -> SUCCESS", command);
                return ESP_OK;
            }
            
            // Check for error responses
            if (strstr(s_uart_buffer, "ERROR")) {
                ESP_LOGW(SIM800L_TAG, "AT Command: %s -> ERROR", command);
                return ESP_FAIL;
            }
        }
    }

    ESP_LOGW(SIM800L_TAG, "AT Command: %s -> TIMEOUT", command);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t sim800l_check_basic_functionality(void)
{
    ESP_LOGI(SIM800L_TAG, "Checking basic functionality...");

    // Basic AT command
    ESP_ERROR_CHECK(sim800l_send_at_command("AT", "OK", AT_TIMEOUT_DEFAULT));
    
    // Check SIM status
    if (!sim800l_is_sim_ready()) {
        ESP_LOGE(SIM800L_TAG, "SIM card not ready");
        return ESP_FAIL;
    }
    
    // Check network registration
    ESP_ERROR_CHECK(sim800l_send_at_command("AT+CREG?", "OK", AT_TIMEOUT_DEFAULT));
    
    ESP_LOGI(SIM800L_TAG, "Basic functionality check completed");
    return ESP_OK;
}

static void sim800l_task(void* param)
{
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for module to boot
    
    ESP_LOGI(SIM800L_TAG, "Starting SIM800L initialization sequence");
    
    if (sim800l_check_basic_functionality() == ESP_OK) {
        s_status = SIM800L_STATUS_NETWORK_REGISTERED;
        ESP_LOGI(SIM800L_TAG, "SIM800L ready for operations");
    } else {
        s_status = SIM800L_STATUS_ERROR;
        ESP_LOGE(SIM800L_TAG, "SIM800L initialization failed");
    }
    
    // Task can be deleted or continue with periodic status checks
    vTaskDelete(NULL);
} 