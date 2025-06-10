#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include "esp_mesh.h"
#include "esp_err.h"
#include <stdint.h>
#include "message_protocol.h"

/*******************************************************
 *                Constants
 *******************************************************/
// Communication Constants (extracted from mesh_main.c)
#define RX_SIZE          (1500)
#define TX_SIZE          (1460)

/*******************************************************
 *                Interface Functions
 *******************************************************/
esp_err_t message_handler_init_sensor(void);
void message_handler_start_mac_tx_task(void);
void message_handler_start_p2p_tasks(void);
esp_err_t message_handler_send_mac_data(void);
esp_err_t message_handler_send_sensor_data(void);
void message_handler_stop(void);

/*******************************************************
 *                Task Functions (used by mesh_network module)
 *******************************************************/
void esp_mesh_sensor_mac_tx_main(void *arg);
void esp_mesh_p2p_tx_main(void *arg);
void esp_mesh_p2p_rx_main(void *arg);
esp_err_t esp_mesh_comm_p2p_start(void);

/*******************************************************
 *                Sensor Data Collection
 *******************************************************/
esp_err_t collect_sensor_readings(sensor_data_t* data);

#endif // MESSAGE_HANDLER_H 