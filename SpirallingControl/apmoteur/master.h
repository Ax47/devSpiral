#ifndef _MASTER_H
#define _MASTER_H
#include "SpirallingMaster.h"


// Parametres

void SpirallingMaster_heartbeatError(CO_Data* d, UNS8);

UNS8 SpirallingMaster_canSend(Message *);

void SpirallingMaster_initialisation(CO_Data* d);
void SpirallingMaster_preOperational(CO_Data* d);
void SpirallingMaster_operational(CO_Data* d);
void SpirallingMaster_stopped(CO_Data* d);

void SpirallingMaster_post_sync(CO_Data* d);
void SpirallingMaster_post_TPDO(CO_Data* d);
void SpirallingMaster_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg);
void SpirallingMaster_post_SlaveBootup(CO_Data* d, UNS8 nodeid);
void SpirallingMaster_post_SlaveStateChange(CO_Data* d, UNS8 nodeId, e_nodeState newNodeState);

void master_init();
static int master_start_sync(int start);



#endif
