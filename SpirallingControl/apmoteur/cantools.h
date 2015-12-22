#ifndef _CANTOOLS_H
#define _CANTOOLS_H
#include "canfestival.h"
#include <gtk/gtk.h>

typedef struct SDOR SDOR;
struct SDOR {
	UNS16 index;
	UNS8 subindex;
	UNS8 type; // as defined in objdictdef.h
};

int cantools_read_sdo(UNS8 nodeid,SDOR sdo, void* data);
int cantools_write_sdo(UNS8 nodeid,SDOR sdo, void* data);
int cantools_write_local(UNS16 Mindex, UNS8 Msubindex, void* data, UNS32 datsize);
int cantools_PDO_trans(UNS8 nodeID, UNS16 index, UNS8 trans,UNS16 inhibit,UNS8 comp_entry, UNS16 event_timer);
int cantools_PDO_recv(UNS8 nodeID, UNS16 index, UNS8 trans, UNS16 inhibit,UNS8 comp_entry, UNS16 event_timer);
int cantools_PDO_map_config(UNS8 nodeID, UNS16 PDOMapIndex,...);
int cantools_sync(int start);

int cantools_init_laser(void);
void cantools_checkPlotState_laser(void);
void cantools_exit_laser(void);
int cantools_reinit_laser(void);

void cantools_ApplyVitTrans(INTEGER32 * vit);
void cantools_ApplyVitRot(INTEGER32 * vit);

gpointer cantools_init_loop(gpointer inutile);

#endif
