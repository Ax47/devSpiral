#ifndef _SLAVE_H
#define _SLAVE_H
#include "data.h"

typedef struct SLAVES_conf SLAVES_conf;
struct SLAVES_conf {
    char* id;
	UNS8 node;      // NodeId
	UNS32 vendor;   // Vendor ID
	UNS32 product;  // Product Code
	UNS32 revision; // Product Code
    UNS32 serial;   // Serial number
    int state;      // Etat global
    int state_error;// Erreur lors des passages d'état
    UNS16 powerstate; // Etat de marche
};

UNS8 slave_get_node_with_index(int i);
UNS8 slave_get_node_with_id(char* id);
int slave_get_state_with_index(int i);
int slave_get_state_with_id(char* slave_id);
void slave_set_state_with_index(int i, int dat);
void slave_set_state_with_id(char* slave_id, int dat);
int slave_get_state_error_with_index(int i);
int slave_get_state_error_with_id(char* slave_id);
void slave_set_state_error_with_index (int i, int dat);
void slave_set_state_error_with_id (char* slave_id, int dat);
UNS16 slave_get_powerstate_with_index(int i);
UNS16 slave_get_powerstate_with_id(char* slave_id);
void slave_set_powerstate_with_index (int i, int dat);
void slave_set_powerstate_with_id (char* slave_id, int dat);
int slave_get_index_with_node(UNS8 nodeID);
int slave_get_index_with_id(char* slave_id);
char* slave_get_id_with_index(int i);
char* slave_get_id_with_node(UNS8 node);
int slave_id_exist(char* slave_id);

char* slave_get_state_title(int state);
char* slave_get_state_img (int state);
char* slave_get_state_error_title(int state);
int slave_gen_config_file(char* slave_id,...);
int slave_check_config_file(char* slave_id);

int slave_config();
int slave_load_config();
#endif // _SLAVE_H
