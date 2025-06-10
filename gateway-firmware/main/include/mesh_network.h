#ifndef MESH_NETWORK_H
#define MESH_NETWORK_H

#include "esp_mesh.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"

// Mesh constants
extern const uint8_t MESH_ID[6];
#define RX_SIZE          (1500)
#define TX_SIZE          (1460)

// Function declarations
esp_err_t init_full_mesh_system(void);
bool is_mesh_connected_status(void);
int get_mesh_layer(void);
esp_netif_t* get_netif_sta(void);
void mesh_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

#endif // MESH_NETWORK_H 