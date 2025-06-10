#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include "esp_mesh.h"
#include "esp_err.h"
#include <stdint.h>

/*******************************************************
 *                Constants
 *******************************************************/
// Communication Constants (extracted from mesh_main.c)
#define RX_SIZE          (1500)
#define TX_SIZE          (1460)

// Message Type Constants
#define MSG_TYPE_SENSOR_MAC 0x01

/*******************************************************
 *                Message Structures
 *******************************************************/
// Message structure (extracted from mesh_main.c)
typedef struct {
    uint8_t msg_type;           // Message type identifier
    uint8_t mac_addr[6];        // MAC address
    uint8_t layer;              // Mesh layer
    uint32_t timestamp;         // Timestamp
} sensor_mac_msg_t;

/*******************************************************
 *                Interface Functions
 *******************************************************/
esp_err_t message_handler_init_sensor(void);
void message_handler_start_mac_tx_task(void);
void message_handler_start_p2p_tasks(void);
esp_err_t message_handler_send_mac_data(void);
void message_handler_stop(void);

/*******************************************************
 *                Task Functions (used by mesh_network module)
 *******************************************************/
void esp_mesh_sensor_mac_tx_main(void *arg);
void esp_mesh_p2p_tx_main(void *arg);
void esp_mesh_p2p_rx_main(void *arg);
esp_err_t esp_mesh_comm_p2p_start(void);

#endif // MESSAGE_HANDLER_H 