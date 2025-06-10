#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include "esp_mesh.h"
#include "mesh_network.h"

// Function declarations
esp_err_t esp_mesh_comm_p2p_start(void);
esp_err_t message_handler_start_p2p(void);  // New convenient wrapper function
void esp_mesh_p2p_tx_main(void *arg);
void esp_mesh_p2p_rx_main(void *arg);

#endif // MESSAGE_HANDLER_H 