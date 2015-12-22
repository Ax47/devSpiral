/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
#include "master.h"
#include "slave.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include "SpirallingControl.h"
#include "keyword.h"
#include "cantools.h"
#include "lsstools.h"
#include "gui.h"
#include "od_callback.h"
#include "errgen.h"

extern s_BOARD MasterBoard;
char* baud_rate="1M";
extern SLAVES_conf slaves[SLAVE_NUMBER];

/*****************************************************************************/

/**
* Initialisation du maitre
* Appelé dans le programme principal
**/
void master_init() {
    *SpirallingMaster_Data.bDeviceNodeId = 0x01;
    setState(&SpirallingMaster_Data, Initialisation);
}
/**
* START AND STOP SYNC
* 0 : erreur
* 1 : ok
**/
static int master_start_sync(int start) {
    UNS32 dat = 0x00000080;
    if (start)
        dat = 0x40000080;
    if(!cantools_write_local(0x1005,0x00,&dat,sizeof(UNS32))) {
        return 0;
    } else {
        return 1;
    }
}
/**
* Master configuration
**/
static int master_config() {
    UNS32 dat;
    UNS16 Inhibit_Time;
    UNS8 trans_type;
    int i = 0;
    /** SYNC PERIOD **/
    if(!master_start_sync(0)) { // Arret de la synchro
        errgen_set(ERR_MASTER_START_SYNC);
        return 0;
    }
    dat = CYCLE_PERIOD; // Affectation de la periode de synchro
    if(!cantools_write_local(0x1006,0x00,&dat,sizeof(UNS32))) {
        errgen_set(ERR_MASTER_SET_PERIOD);
        return 0;
    }
    /** HEARTBEAT CONSUMER **/

    for (i=0; i < SLAVE_NUMBER; i++) {
        dat = HB_CONS_BASE + (slave_get_node_with_index(i)-1)*HB_CONS_BASE_OFFSET; // 500 + (n-1)*50
        if(!cantools_write_local(0x1016,slave_get_node_with_index(i)-1,&dat,sizeof(UNS32))) {
            errgen_set(ERR_MASTER_SET_HB_CONS);
            return 0;
        }
    }

    /** PDOR CONFIGURATION **/
    UNS8 j;

    for (i=0; i < SLAVE_NUMBER; i++) {
        j = (slave_get_node_with_index(i) - 0x02)*3;
        dat = 0x40000180 + slave_get_node_with_index(i);
        if (!cantools_write_local(0x1400+j,0x01,&dat,sizeof(UNS32))) {
            errgen_set(ERR_MASTER_CONFIG_PDOR1);
            return 0;
        }
        j++;
        dat = 0x40000280 + slave_get_node_with_index(i);
        if (!cantools_write_local(0x1400+j,0x01,&dat,sizeof(UNS32))) {
            errgen_set(ERR_MASTER_CONFIG_PDOR2);
            return 0;
        }
        j++;
        dat = 0x40000380 + slave_get_node_with_index(i);
        if (!cantools_write_local(0x1400+j,0x01,&dat,sizeof(UNS32))) {
            errgen_set(ERR_MASTER_CONFIG_PDOR3);
            return 0;
        }
        j++;
    }
    /** PDOT CONFIGURATION **/
    for (i=0; i < SLAVE_NUMBER; i++) {
        j = (slave_get_node_with_index(i) - 0x02)*1;
        dat = 0x40000200 + slave_get_node_with_index(i);
        trans_type = 0xFF;
        if (!cantools_write_local(0x1800+j,0x01,&dat,sizeof(UNS32))) {
            errgen_set(ERR_MASTER_CONFIG_PDOT1);
            return 0;
        }
        if (!cantools_write_local(0x1800+j,0x02,&trans_type,sizeof(UNS8))) {
            errgen_set(ERR_MASTER_CONFIG_PDOT1);
            return 0;
        }
        j++;
    }
    /** CALLBACKS **/
    RegisterSetODentryCallBack(&SpirallingMaster_Data, 0x2000, 0, &OnStatusWordVUpdate);
    RegisterSetODentryCallBack(&SpirallingMaster_Data, 0x2004, 0, &OnStatusWordVauxUpdate);
    RegisterSetODentryCallBack(&SpirallingMaster_Data, 0x200C, 0, &OnStatusWordRUpdate);

    return 1;
}


void SpirallingMaster_heartbeatError(CO_Data* d, UNS8 heartbeatID) {
	printf("SpirallingMaster_heartbeatError %d\n", heartbeatID);
}


void SpirallingMaster_initialisation(CO_Data* d) {
	printf("SpirallingMaster_initialisation\n");
    if(!master_config()) {     //configuration locale
        errgen_set(ERR_MASTER_CONFIG);
    }
}

void SpirallingMaster_preOperational(CO_Data* d) {
	printf("SpirallingMaster_preOperational\n");
    int i;
    if (!master_start_sync(1)) { // Démarrage de la synchro
        errgen_set(ERR_MASTER_START_SYNC);
    }
    for (i=0; i<SLAVE_NUMBER; i++) {
        slave_set_state_with_index(i,1);
    }
}

void SpirallingMaster_operational(CO_Data* d) {
	printf("SpirallingMaster_operational\n");
}

void SpirallingMaster_stopped(CO_Data* d) {
	printf("SpirallingMaster_stopped\n");
}

static unsigned long MasterSyncCount = 0;


void SpirallingMaster_post_sync(CO_Data* d) {

    MasterSyncCount++;

}

void SpirallingMaster_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg) {
	printf("Master received EMCY message. Node: %2.2x  ErrorCode: %4.4x  ErrorRegister: %2.2x\n", nodeID, errCode, errReg);
}

void SpirallingMaster_post_SlaveStateChange(CO_Data* d, UNS8 nodeId, e_nodeState newNodeState) {
    switch(newNodeState) {
        case Initialisation:
            printf("Node %u state is now  : Initialisation\n", nodeId);
        break;
        case Disconnected:
            printf("\n\nNode %u state is now  : Disconnected\n\n", nodeId);
        break;
        case Connecting:
            printf("Node %u state is now  : Connecting\n", nodeId);
        break;
        case Preparing:
            printf("Node %u state is now  : Preparing\n", nodeId);
        break;
        case Stopped:
            printf("Node %u state is now  : Stopped\n", nodeId);
        break;
        case Operational:
            printf("Node %u state is now  : Operational\n", nodeId);
            slave_set_state_with_index(slave_get_index_with_node(nodeId),STATE_SON);
        break;
        case Unknown_state:
            printf("Node %u state is now  : Unknown_state\n", nodeId);
        break;
        default:
            printf("Error : node %u unexpected state : %d\n", nodeId, newNodeState);
        break;
    }
}

void SpirallingMaster_post_TPDO(CO_Data* d) {
/*
	printf("\nSpirallingMaster_post_TPDO MasterSyncCount = %lu \n", MasterSyncCount);

    printf("MotRotVars : StatusWord = %x, Erreur = %x, ConsigneVitesse = %d, CaptureVitesse = %d, CapturePosition = %d, Acceleration = %u, Deceleration = %u, Tension = %d, Temperature = %d\n",
        StatusWord_MotRot, ErrorCode_MotRot, ConsigneVitesse_MotRot, CaptureVitesse_MotRot, CapturePosition_MotRot, ConsigneAcceleration_MotRot, ConsigneDeceleration_MotRot, Voltage_MotRot, InternalTemp_MotRot);

    printf("MotTransVars : StatusWord = %x, Erreur = %x, ConsigneVitesse = %d, CaptureVitesse = %d, Acceleration = %u, Deceleration = %u,Tension = %d, Temperature = %d\n",
        StatusWord_V, ErrorCode_V, Vel2Send_V, Velocity_V, Acceleration_V, Deceleration_V, Voltage_V, InternalTemp_V);

    printf("MotTransAuxVars : StatusWord = %x, Erreur = %x, ConsigneVitesse = %d, CaptureVitesse = %d, Tension = %d, Temperature = %d\n",
        StatusWord_Vaux, ErrorCode_Vaux, Vel2Send_Vaux, Velocity_Vaux, Voltage_MotRotAux, InternalTemp_Vaux);
*/
}

void SpirallingMaster_post_SlaveBootup(CO_Data* d, UNS8 nodeid) {
	printf("SpirallingMaster_post_SlaveBootup %x\n", nodeid);
    slave_set_state_with_index(slave_get_index_with_node(nodeid),STATE_CONFIG);
}
