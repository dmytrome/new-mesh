#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include "esp_mesh.h"

// Function declarations
esp_err_t esp_mesh_comm_p2p_start(void);
void esp_mesh_p2p_tx_main(void *arg);
void esp_mesh_p2p_rx_main(void *arg);
void send_time_sync_message(const mesh_addr_t* dest_addr);

// Time coordination functions
void initialize_time_coordination(void);

#endif // MESSAGE_HANDLER_H 