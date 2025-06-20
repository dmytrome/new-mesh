#ifndef SIM800L_H
#define SIM800L_H

#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"

/**
 * @brief SIM800L module configuration structure
 */
typedef struct {
    uart_port_t uart_port;      ///< UART port for SIM800L communication
    int tx_pin;                 ///< TX pin number
    int rx_pin;                 ///< RX pin number
    int baud_rate;              ///< Communication baud rate
    const char* apn;            ///< APN for GPRS connection
    const char* apn_username;   ///< APN username (can be NULL)
    const char* apn_password;   ///< APN password (can be NULL)
} sim800l_config_t;

/**
 * @brief SIM800L connection status
 */
typedef enum {
    SIM800L_STATUS_DISCONNECTED = 0,
    SIM800L_STATUS_INITIALIZING,
    SIM800L_STATUS_NETWORK_REGISTERED,
    SIM800L_STATUS_GPRS_CONNECTED,
    SIM800L_STATUS_ERROR
} sim800l_status_t;

/**
 * @brief Initialize SIM800L module
 * 
 * @param config Configuration parameters for SIM800L
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t sim800l_init(const sim800l_config_t* config);

/**
 * @brief Deinitialize SIM800L module
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t sim800l_deinit(void);

/**
 * @brief Get current SIM800L status
 * 
 * @return sim800l_status_t Current status
 */
sim800l_status_t sim800l_get_status(void);

/**
 * @brief Connect to GPRS network
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t sim800l_connect_gprs(void);

/**
 * @brief Disconnect from GPRS network
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t sim800l_disconnect_gprs(void);

/**
 * @brief Send HTTP POST request
 * 
 * @param url Target URL
 * @param data Data to send
 * @param data_len Length of data
 * @param response Buffer to store response (can be NULL)
 * @param response_len Length of response buffer
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t sim800l_http_post(const char* url, const char* data, size_t data_len, 
                           char* response, size_t response_len);

/**
 * @brief Send SMS message
 * 
 * @param phone_number Target phone number
 * @param message SMS message content
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t sim800l_send_sms(const char* phone_number, const char* message);

/**
 * @brief Check if SIM card is ready
 * 
 * @return true if SIM card is ready, false otherwise
 */
bool sim800l_is_sim_ready(void);

/**
 * @brief Get signal strength (CSQ)
 * 
 * @return int Signal strength value (0-31, 99 for unknown)
 */
int sim800l_get_signal_strength(void);

#endif // SIM800L_H 