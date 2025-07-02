#ifndef MESH_NETWORK_H
#define MESH_NETWORK_H

#include "esp_mesh.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_err.h"

/*******************************************************
 *                Constants
 *******************************************************/
// Mesh constants that will be extracted from main
// Note: MESH_ID is now static and initialized at runtime with unique MAC address

// Communication Constants
#define RX_SIZE          (1500)
#define TX_SIZE          (1460)

/*******************************************************
 *                Interface Functions
 *******************************************************/
esp_err_t mesh_network_init_sensor(void);
esp_err_t mesh_network_start_p2p_communication(void);
bool mesh_network_is_connected_to_parent(void);
int mesh_network_get_current_layer(void);
esp_err_t mesh_network_init_full_system(bool is_fast_init);

/*******************************************************
 *                Event Handlers (used by other modules)
 *******************************************************/
void mesh_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

/*******************************************************
 *                State Access Functions
 *******************************************************/
bool mesh_network_is_mesh_connected(void);
mesh_addr_t mesh_network_get_parent_addr(void);
esp_netif_t* mesh_network_get_netif_sta(void);

#endif // MESH_NETWORK_H 