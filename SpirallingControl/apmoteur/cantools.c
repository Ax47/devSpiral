#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "cantools.h"
#include "strtools.h"
#include "canfestival.h"
#include "gui.h"
#include "keyword.h"
#include "master.h"
#include "slave.h"
#include "SpirallingControl.h"
#include "motor.h"
#include "od_callback.h"
#include "errgen.h"
#include "gtksig.h"

#include "laser_asserv.h"


static int SDO_step_error = 0;

extern SLAVES_conf slaves[SLAVE_NUMBER];
extern pthread_mutex_t lock_slave;
extern int run_init;

/**
* Lecture d'une SDO
**/
int cantools_read_sdo(UNS8 nodeid,SDOR sdo, void* data) {
	UNS32 abortCode;
    UNS32 size;

	UNS8 res;
    UNS8 datatype = 0;
    if (sdo.type == 0x01) {
        size = 1;
    } else if (sdo.type == 0x02) {
        size = 1;
    } else if (sdo.type == 0x03) {
        size = 2;
    } else if (sdo.type == 0x04) {
        size = 4;
    } else if (sdo.type == 0x05) {
        size = 1;
    } else if (sdo.type == 0x06) {
		size = 2;
    } else if (sdo.type == 0x07) {
		size = 4;
    } else if (sdo.type == 0x09) {
        size = 4;
    }
	readNetworkDict(&SpirallingMaster_Data, nodeid, sdo.index, sdo.subindex,datatype, 0);
	res = getReadResultNetworkDict(&SpirallingMaster_Data, nodeid, data, &size, &abortCode);

	if (res == SDO_UPLOAD_IN_PROGRESS || res == SDO_DOWNLOAD_IN_PROGRESS ) {
        time_t deb = time (NULL);
        time_t fin;
 		while (res == SDO_UPLOAD_IN_PROGRESS || res == SDO_DOWNLOAD_IN_PROGRESS) {
            usleep(100);
            fin = time (NULL);
			res = getReadResultNetworkDict(&SpirallingMaster_Data, nodeid, data, &size, &abortCode);
			if (difftime(fin,deb) > SDO_TIMEOUT) {
                res = SDO_ABORTED_INTERNAL;
                gui_push_state(strtools_concat(SDO_READING_TIMEOUT, " (",strtools_gnum2str(&sdo.index,0x06)," | ",strtools_gnum2str(&sdo.subindex,0x05),")",NULL));
            }
		}
    }
    closeSDOtransfer(&SpirallingMaster_Data, nodeid, SDO_CLIENT);
    if (res == SDO_FINISHED) {
        gui_push_state(strtools_concat(SDO_READING_SUCCESS, " (",strtools_gnum2str(&sdo.index,0x06)," | ",strtools_gnum2str(&sdo.subindex,0x05),")",NULL));
        SDO_step_error=0;
        return 1;
	} else {
        SDO_step_error++;
        if (SDO_step_error < MAX_SDO_ERROR) {
            cantools_read_sdo(nodeid,sdo,data);
        } else {
            SDO_step_error=0;
            gui_push_state(strtools_concat(SDO_READING_ERROR, " (",strtools_gnum2str(&sdo.index,0x06)," | ",strtools_gnum2str(&sdo.subindex,0x05),")",NULL));
            return 0;
        }
	}
}

/**
* Ecriture d'une SDO
**/
int cantools_write_sdo(UNS8 nodeid,SDOR sdo, void* data) {
	UNS32 abortCode;
	UNS32 size;
	UNS8 res;
	UNS8 datatype = 0;

	if (sdo.type == 0x04 || sdo.type == 0x07) {
		size = 4;
 	} else if (sdo.type == 0x03 || sdo.type == 0x06 ) {
		size = 2;
 	} else if (sdo.type == 0x01 || sdo.type == 0x02 || sdo.type == 0x05 ) {
		size = 1;
    } else if (sdo.type == 0X09) {
		size = 8;
        datatype = visible_string;
	} else {
        printf("Type error"); exit(EXIT_FAILURE);
	}

	writeNetworkDict(&SpirallingMaster_Data, nodeid, sdo.index, sdo.subindex, size, datatype, data, 0);
	res = getWriteResultNetworkDict(&SpirallingMaster_Data, nodeid, &abortCode);
	if (res == SDO_DOWNLOAD_IN_PROGRESS || res == SDO_UPLOAD_IN_PROGRESS) {
        time_t deb = time (NULL);
        time_t fin;
		while (res == SDO_DOWNLOAD_IN_PROGRESS || res == SDO_UPLOAD_IN_PROGRESS) {
            usleep(100);
            fin = time (NULL);
 			res = getWriteResultNetworkDict(&SpirallingMaster_Data, nodeid, &abortCode);
			if (difftime(fin,deb) > SDO_TIMEOUT) {
                res = SDO_ABORTED_INTERNAL;
                gui_push_state(SDO_WRITING_TIMEOUT);
            }
		}
    }
	closeSDOtransfer(&SpirallingMaster_Data, nodeid, SDO_CLIENT);
    if (res == SDO_FINISHED) {
        gui_push_state(strtools_concat(SDO_WRITING_SUCCESS, " (",strtools_gnum2str(&sdo.index,0x06)," | ",strtools_gnum2str(&sdo.subindex,0x05),")",NULL));
        SDO_step_error=0;
        return 1;
	} else {
        SDO_step_error++;
        if (SDO_step_error < MAX_SDO_ERROR) {
            cantools_write_sdo(nodeid,sdo,data);
        } else {
            SDO_step_error=0;
            gui_push_state(strtools_concat(SDO_WRITING_ERROR, " (",strtools_gnum2str(&sdo.index,0x06)," | ",strtools_gnum2str(&sdo.subindex,0x05),")",NULL));
        return 0;
        }
    }
}

/**
* Ecriture dans le dictionnaire local
**/
int cantools_write_local(UNS16 Mindex, UNS8 Msubindex, void* data, UNS32 datsize) {
	UNS32 res;
	res = writeLocalDict(&SpirallingMaster_Data, Mindex, Msubindex, data, &datsize,1);
    if (res == OD_SUCCESSFUL) {
        gui_push_state(strtools_concat(LOCAL_WRITING_SUCCESS, " -> ",strtools_gnum2str(&Mindex,0x06)," (", " | ",
        strtools_gnum2str(&Msubindex,0x05),")",NULL));
        return 1;
    } else {
        gui_push_state(strtools_concat(LOCAL_WRITING_ERROR, " -> ",strtools_gnum2str(&Mindex,0x06)," (", " | ",
        strtools_gnum2str(&Msubindex,0x05),")",NULL));
        return 0;
    }
}

/**
* Configuration du type de transmission des PDOs
**/
int cantools_PDO_trans(UNS8 nodeID, UNS16 index, UNS8 trans, UNS16 inhibit, UNS8 comp_entry, UNS16 event_timer) {
    UNS16 cobID;
    if (index == 0x1800) cobID = 0x0180;
    if (index == 0x1801) cobID = 0x0280;
    if (index == 0x1802) cobID = 0x0380;
    if (index == 0x1803) cobID = 0x0480;
    UNS32 EnabledCobid = 0x40000000 + cobID + nodeID;
    UNS32 DisabledCobid = 0x80000000 + cobID + nodeID;
    SDOR SDO_cobid = {index,0x01,0x07};
    SDOR SDO_trans = {index,0x02,0x05};
    SDOR SDO_inhib = {index,0x03,0x06};
    SDOR SDO_comp_entry = {index, 0x04, uint8};
    SDOR SDO_event_timer = {index, 0x05, uint16};

    // Désactivation pour modification
    if(!cantools_write_sdo(nodeID,SDO_cobid,&DisabledCobid)) {
        printf("Erreur : desactivation\n");
        return 0;
    }
    // Modification transmission
    if(!cantools_write_sdo(nodeID,SDO_trans,&trans)) {
        printf("Erreur : trans\n");
        return 0;
    }
    // Modification de l'inhibit
    if(!cantools_write_sdo(nodeID,SDO_inhib,&inhibit)) {
        printf("Erreur : inhib\n");
        return 0;
    }
    /*
    // Modification de compatibility entry
    if(!cantools_write_sdo(nodeID,SDO_comp_entry,&comp_entry)) {
        printf("Erreur : comp entry\n");
        return 0;
    }
    */
    // Modification de event timer
    if(!cantools_write_sdo(nodeID,SDO_event_timer,&event_timer)) {
        printf("Erreur : event timer\n");
        return 0;
    }
    // Activation transmission
    if (!cantools_write_sdo(nodeID,SDO_cobid,&EnabledCobid)) {
        printf("Erreur : activation %#.8x\n", EnabledCobid);
        return 0;
    }
    return 1;
}

/**
* Configuration du type de reception des PDOs
**/
int cantools_PDO_recv(UNS8 nodeID, UNS16 index, UNS8 trans, UNS16 inhibit, UNS8 comp_entry, UNS16 event_timer) {
    UNS16 cobID;
    if (index == 0x1400) cobID = 0x0180;
    if (index == 0x1401) cobID = 0x0280;
    if (index == 0x1402) cobID = 0x0380;
    if (index == 0x1403) cobID = 0x0480;
    UNS32 EnabledCobid = 0x40000000 + cobID + nodeID;
    UNS32 DisabledCobid = 0x80000000 + cobID + nodeID;
    SDOR SDO_trans = {index,0x02,0x05};
    SDOR SDO_cobid = {index,0x01,0x07};
    SDOR SDO_inhib = {index,0x03,0x06};
    SDOR SDO_comp_entry = {index, 0x04, uint8};
    SDOR SDO_event_timer = {index, 0x05, uint16};

    // Désactivation pour modification
    if(!cantools_write_sdo(nodeID,SDO_cobid,&DisabledCobid)) {
        printf("Erreur : desactivation\n");
        return 0;
    }
    // Modification transmission
    if(!cantools_write_sdo(nodeID,SDO_trans,&trans)) {
        printf("Erreur : trans\n");
        return 0;
    }
    // Modification de l'inhibit
    if(!cantools_write_sdo(nodeID,SDO_inhib,&inhibit)) {
        printf("Erreur : inhib\n");
        return 0;
    }
    /*
    // Modification de compatibility entry
    if(!cantools_write_sdo(nodeID,SDO_comp_entry,&comp_entry)) {
        printf("Erreur : comp entry\n");
        return 0;
    }
    */
    // Modification de event timer
    if(!cantools_write_sdo(nodeID,SDO_event_timer,&event_timer)) {
        printf("Erreur : event timer\n");
        return 0;
    }

    // Activation transmission
    if (!cantools_write_sdo(nodeID,SDO_cobid,&EnabledCobid)) {
        printf("Erreur : activation %#.8x\n", EnabledCobid);
        return 0;
    }
    return 1;
}
/**
* Configuration des PDO map d'un esclave
* UNS32 PDOMapData : index + subindex + size : 60400020
* UNS16 PDOParamData : 0x0180 / 0x0280 ...
**/
int cantools_PDO_map_config(UNS8 nodeID, UNS16 PDOMapIndex,...) {
    UNS16 PDOParamData;
    if (PDOMapIndex == 0x1A00) PDOParamData = 0x0180;
    if (PDOMapIndex == 0x1A01) PDOParamData = 0x0280;
    if (PDOMapIndex == 0x1A02) PDOParamData = 0x0380;
    if (PDOMapIndex == 0x1A03) PDOParamData = 0x0480;

    if (PDOMapIndex == 0x1600) PDOParamData = 0x0200;
    if (PDOMapIndex == 0x1601) PDOParamData = 0x0300;
    if (PDOMapIndex == 0x1602) PDOParamData = 0x0400;
    if (PDOMapIndex == 0x1603) PDOParamData = 0x0500;

    va_list ap;
    va_start(ap,PDOMapIndex);
    UNS32 arg = va_arg(ap,UNS32);


    SDOR PDOMapAddress = {PDOMapIndex,0x00,0x05};

    // Désactivation de la transmission pour modification
    SDOR PDOParamAddress = {PDOMapIndex-0x0200,0x01,0x07};
    UNS32 PDOParamVal = 0x80000000 + PDOParamData + nodeID;
    if(!cantools_write_sdo(nodeID,PDOParamAddress,&PDOParamVal)) {
        printf("Erreur : désactivation\n");
        return 0;
    }
    // Mise à 0 du subindex pour activer le mode edition
    UNS8 i =0x00;
    if(!cantools_write_sdo(nodeID,PDOMapAddress,&i)) {
        printf("Erreur : mise a 0 subindex\n");
        return 0;
    }
    // Ecriture de tout les index en paramètres

    while (arg != 0) {
        i++;
        SDOR PDOMapAddress2 = {PDOMapIndex,i,0x07};
        if(!cantools_write_sdo(nodeID,PDOMapAddress2,&arg)) {
            printf("ERREUR ecriture index : %#.8x\n",arg);
            return 0;
        }
        arg = va_arg(ap,UNS32);
    }
    va_end(ap);

    // Sortie du mode édition, Ecriture nombre d'index
    if(!cantools_write_sdo(nodeID,PDOMapAddress,&i)) {
        printf("Erreur : Ecriture nombre subindex\n");
        return 0;
    }

    // Activation PDO trans
    PDOParamVal = 0x40000000 + PDOParamData + nodeID;
    if(!cantools_write_sdo(nodeID,PDOParamAddress,&PDOParamVal)) {
        printf("Erreur : Acivation\n");
        return 0;
    }
    return 1;
}


UNS32 pdonum_for_thread_trsl;
gpointer cantools_waitStopTrsl_Thread (gpointer pdonum)
{
    int apply0 = 1;
    while (abs(Velocity_V)>3){
        usleep(100);
        if(abs(Vel2Send_V)>1){
            apply0 = 0;
            break;
        }
    }
    if(apply0){
        Vel2Send_V = 0;
        printf("pdonum = %d\n\n\n\n\n", *(UNS32*)pdonum);

        EnterMutex();
        PDOEventTimerAlarm(&SpirallingMaster_Data, *(UNS32*)pdonum);
        LeaveMutex();
    }

    g_thread_exit(NULL);
}

void cantools_ApplyVitTrans(INTEGER32 * vit)
{
    int i;
    UNS32 pdonum;
    INTEGER32 vtm, vtc;
    //printf("7\n");
    EnterMutex();
    if(Vel2Send_V != *vit)
        Vel2Send_V = *vit;
    LeaveMutex();

    for (i=0;i<SLAVE_NUMBER; i++){
        if(!strcmp(slave_get_id_with_index(i),"vitesse")){
            //printf("8\n");
            EnterMutex();
            vtm = Velocity_V;
            vtc = Vel2Send_V;
            LeaveMutex();
            pdonum = (slave_get_node_with_index(i)-0x02)*1;
            //gerer les problèmes de non-application de l'acceleration au demarrage et a l'arrêt
            if(vtm == 0 || vtc == 0){
                if((vtm == 0) && (vtc != 0)){
                    Vel2Send_V = vtc/abs(vtc);
                    EnterMutex();
                    PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
                    Vel2Send_V = vtc;
                    LeaveMutex();
                    usleep(1000);
                } else if ((vtc == 0) && (vtm != 0)) {
                    pdonum_for_thread_trsl = pdonum;
                    Vel2Send_V = vtm/abs(vtm);
                    EnterMutex();
                    PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
                    LeaveMutex();

                    if(g_thread_try_new(NULL, cantools_waitStopTrsl_Thread, &pdonum_for_thread_trsl, NULL) == NULL){
                        //bloquer le moteur direct
                        Vel2Send_V = vtc;
                        EnterMutex();
                        PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
                        LeaveMutex();
                    }

                    return;
                } else return;//la vitesse de 0 est déjà appliquée.

            }
            EnterMutex();
            PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
            LeaveMutex();
        }
        //ajouter mt couple translation

    }
}

UNS32 pdonum_for_thread_rot;
gpointer cantools_waitStopRot_Thread (gpointer pdonum)
{
    int apply0 = 1;
    while (abs(CaptureVitesse_MotRot)>3){
        usleep(100);
        if(abs(ConsigneVitesse_MotRot) > 1){
            apply0 = 0;
            break;
        }
    }
    if(apply0){
        ConsigneVitesse_MotRot = 0;
        printf("pdonum = %d\n\n\n\n\n", *(UNS32*)pdonum);

        EnterMutex();
        PDOEventTimerAlarm(&SpirallingMaster_Data, *(UNS32*)pdonum);
        LeaveMutex();
    }

    g_thread_exit(NULL);
}

void cantools_ApplyVitRot(INTEGER32 * vit)
{
    int i;
    INTEGER32 vtm, vtc;
    UNS32 pdonum;
    printf("cantools_ApplyVitRot\n");
    ConsigneVitesse_MotRot = *vit;

    for (i=0;i<SLAVE_NUMBER; i++){
        if(!strcmp(slave_get_id_with_index(i),"rotation")){

            EnterMutex();
            vtm = CaptureVitesse_MotRot;
            vtc = ConsigneVitesse_MotRot;
            LeaveMutex();
            pdonum = (slave_get_node_with_index(i)-0x02)*1;
            //gerer les problèmes de non-application de l'acceleration au demarrage et a l'arrêt
            if(vtm == 0 || vtc == 0){
                if((vtm == 0) && (vtc != 0)){
                    ConsigneVitesse_MotRot = vtc/abs(vtc);
                    EnterMutex();
                    PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
                    ConsigneVitesse_MotRot = vtc;
                    LeaveMutex();
                    usleep(1000);
                } else if ((vtc == 0) && (vtm != 0)) {
                    pdonum_for_thread_rot = pdonum;
                    ConsigneVitesse_MotRot = vtm/abs(vtm);
                    EnterMutex();
                    PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
                    LeaveMutex();

                    if(g_thread_try_new(NULL, cantools_waitStopRot_Thread, &pdonum_for_thread_rot, NULL) == NULL){
                        //bloquer le moteur direct
                        ConsigneVitesse_MotRot = vtc;
                        EnterMutex();
                        PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
                        LeaveMutex();
                    }
                    return;
                } else return;//la vitesse de 0 est déjà appliquée.

            }
            EnterMutex();
            PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
            LeaveMutex();

        }
        //ajouter moteurs couples rotation
    }

}

static int gtksig_in_reinit_laser = 0;
int cantools_init_laser(void)
{
    unsigned int err_l;
    unsigned int laser_state;
    struct laser_data d;

    if(gtksig_in_reinit_laser) errgen_laserState = 0;

    /**configuring serial port acces permissions**/
    if(Laser_serial_config()){
        errgen_set(ERR_LASER_SERIAL_CONFIG);
        return 1;
    }

    /**INIT 2 LASER**/
    if(err_l = Laser_Init2Laser(&ml, &sl)){
        if(err_l == ERR_LASER_INIT_FATAL){
            printf("1\n");
            errgen_set(ERR_LASER_INIT_FATAL);
            errgen_laserState = ERR_LASER_INIT_FATAL;
            return 1;
        }
        if(err_l == LASER_MASTER_INIT_ERROR){
            printf("2\n");
            errgen_set(LASER_ERROR(LASER_MASTER_INIT_ERROR));
            errgen_laserState = MASTER_NOT_STARTED;
            if(Laser_Start1Laser(&sl)){
                printf("3\n");
                errgen_set(ERR_LASER_INIT_FATAL);
                errgen_laserState = ERR_LASER_INIT_FATAL;
                return 1;
            }
            else{
                gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
                gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
                gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
            }
        }
        else if (err_l == LASER_MASTER_INIT_ERROR2){
            printf("4\n");
            errgen_set(LASER_ERROR(LASER_MASTER_INIT_ERROR2));
            errgen_laserState = MASTER_NOT_STARTED;
            if(Laser_Start1Laser(&sl)){
                printf("5\n");
                errgen_laserState = ERR_LASER_INIT_FATAL;
                errgen_set(ERR_LASER_INIT_FATAL);
                return 1;
            }
            else{
                gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
                gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
                gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
            }
        }
        else if (err_l == LASER_SLAVE_INIT_ERROR){
            printf("6\n");
            errgen_set(LASER_ERROR(LASER_SLAVE_INIT_ERROR));
            errgen_laserState = SLAVE_NOT_STARTED;
            if(Laser_Start1Laser(&ml)){
                printf("7\n");
                errgen_set(ERR_LASER_INIT_FATAL);
                errgen_laserState = ERR_LASER_INIT_FATAL;
                return 1;
            }
            else{
                gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
                gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
                gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
            }
        }
        else if (err_l == LASER_SLAVE_INIT_ERROR2){
            printf("8\n");
            errgen_set(LASER_ERROR(LASER_SLAVE_INIT_ERROR2));
            errgen_laserState = SLAVE_NOT_STARTED;
            if(Laser_Start1Laser(&ml)){
                printf("9\n");
                errgen_set(ERR_LASER_INIT_FATAL);
                errgen_laserState = ERR_LASER_INIT_FATAL;
                return 1;
            }
            else{
                gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
                gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
                gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
            }
        }
    }
    else{
        /**START 2 LASER**/
        if(err_l = Laser_Start2Laser(&ml, &sl)){
            if(err_l == ERR_LASER_INIT_FATAL){
                if(err_l & LASER_GETPOSOFFSET_ERROR){
                    errgen_set(LASER_ERROR(LASER_GETPOSOFFSET_ERROR));
                }
                printf("10\n");
                errgen_laserState = ERR_LASER_INIT_FATAL;
                errgen_set(ERR_LASER_INIT_FATAL);
                return 1;
            }
            if(err_l == LASER_MASTER_START_ERROR){
                printf("11\n");
                errgen_set(LASER_ERROR(LASER_MASTER_START_ERROR));
                errgen_laserState = MASTER_NOT_STARTED;
                gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
                gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
                gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
            }
            else if(err_l == LASER_SLAVE_START_ERROR){
                printf("12\n");
                errgen_set(LASER_ERROR(LASER_SLAVE_START_ERROR));
                errgen_laserState = SLAVE_NOT_STARTED;
                gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
                gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
                gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
            }
        }
        if(!err_l){
            printf("13\n");
            gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
            gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
            gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
            gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
            errgen_laserState = 0;
        }
    }
    //printf ("err_l = %x", err_l);

    return 0;

}

void cantools_checkPlotState_laser(void)
{
    unsigned int err_l;
    unsigned int laser_state;
    unsigned long mes1, mes2;
    struct laser_data d;
    int i;

    if ((errgen_laserState == ERR_LASER_FATAL) || ((errgen_laserState & MASTER_NOT_STARTED) && (errgen_laserState & SLAVE_NOT_STARTED))){
        gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
        gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
        return;
    }


    if(laser_state = Laser_GetData(&ml, &sl, &d)){
        if((laser_state & ERR_LASER_FATAL) == ERR_LASER_FATAL){
            errgen_set(LASER_ERROR(ERR_LASER_FATAL));
            gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
            gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
            errgen_laserState |= ERR_LASER_FATAL;
            //arrêt des moteurs
            for (i=0; i<SLAVE_NUMBER; i++) {
                motor_start(slave_get_id_with_index(i),0);
            }
            return;
            //todo arrêter les moteurs
        }
        if(laser_state & MASTER_FAILED_TIMEOUT){
            gui_label_set("labLaser1StatInfo", MASTER_FAILED_TIMEOUT_LABEL);
            gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
            errgen_laserState |= MASTER_FAILED_TIMEOUT;
            }
        else if(laser_state & SLAVE_FAILED_TIMEOUT){
            gui_label_set("labLaser2StatInfo", MASTER_FAILED_TIMEOUT_LABEL);
            gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
            errgen_laserState |= SLAVE_FAILED_TIMEOUT;
        }
        else if(laser_state & MASTER_FAILED_DATCONSISTENCY){
            gui_label_set("labLaser1StatInfo", MASTER_FAILED_DATCONSISTENCY_LABEL);
            gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
            errgen_laserState |= MASTER_FAILED_DATCONSISTENCY;
        }
        else if(laser_state & SLAVE_FAILED_DATCONSISTENCY){
            gui_label_set("labLaser2StatInfo", SLAVE_FAILED_DATCONSISTENCY_LABEL);
            gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
            errgen_laserState |= SLAVE_FAILED_DATCONSISTENCY;
        }
        if(laser_state & MASTER_NOT_STARTED){
            if(!(errgen_laserState & MASTER_NOT_STARTED)){
                errgen_set(LASER_ERROR(MASTER_NOT_STARTED));
                gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
                gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
                errgen_laserState |= MASTER_NOT_STARTED;
            }
        }
        else if (laser_state & SLAVE_NOT_STARTED){
            if(!(errgen_laserState & SLAVE_NOT_STARTED)){
                errgen_set(LASER_ERROR(SLAVE_NOT_STARTED));
                gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
                gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
                errgen_laserState |= SLAVE_NOT_STARTED;
            }
        }
    }
    else{
        gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
        gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
        gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
        gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
        errgen_laserState = 0;
    }

    //printf("laser_state checkplotstate= %x, laser_errgen_laserState = %x\n", laser_state, errgen_laserState);

//affichage mesures non vérifiées
    Laser_GetUnverifiedData(&sl, &mes1);
    gui_label_set("labMesureLaser2", strtools_gnum2str((UNS32*)&mes1, 0x50));

    Laser_GetUnverifiedData(&ml, &mes2);
    gui_label_set("labMesureLaser1", strtools_gnum2str((UNS32*)&mes2, 0x50));

    /**Decommenter pour affichage uniquement si la valeur est correcte**/
/*
    if(!((laser_state & SLAVE_FAILED_TIMEOUT) || (laser_state & SLAVE_FAILED_DATCONSISTENCY) || (laser_state & SLAVE_NOT_STARTED))){
        Laser_GetUnverifiedData(&sl, &mes1);
        gui_label_set("labMesureLaser2", strtools_gnum2str((UNS32*)&mes1, 0x50));
    }
    else gui_label_set("labMesureLaser2", "NA");

    if(!((laser_state & MASTER_FAILED_TIMEOUT) || (laser_state & MASTER_FAILED_DATCONSISTENCY) || (laser_state & MASTER_NOT_STARTED))){
        Laser_GetUnverifiedData(&ml, &mes2);
        gui_label_set("labMesureLaser1", strtools_gnum2str((UNS32*)&mes2, 0x50));
    }
    else gui_label_set("labMesureLaser1", "NA");
*/
    //Affichage mesure vérifiée
    gui_label_set("labMesureLaserGlobal", strtools_gnum2str((UNS32*)&(d.mes), 0x50));

}

void cantools_exit_laser(void)
{
    printf("cantools exit laser: errgen_laserState = %x\n", errgen_laserState);

    if(!(errgen_laserState & MASTER_NOT_STARTED)){
        printf("In exit ml\n");
        if (Laser_Exit1Laser( &ml )<0)
            errgen_set(LASER_ERROR(LASER_MASTER_EXIT_ERROR));
    }
    if(!(errgen_laserState & SLAVE_NOT_STARTED)){
        printf("In exit sl\n");
        if (Laser_Exit1Laser( &sl )<0)
            errgen_set(LASER_ERROR(LASER_SLAVE_EXIT_ERROR));
    }
    if(errgen_laserState == ERR_LASER_INIT_FATAL)
        return;

    gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
    gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
    gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
    gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
    errgen_laserState = MASTER_NOT_STARTED | SLAVE_NOT_STARTED;
}

int cantools_reinit_laser(void){
    int res;
    gtksig_in_reinit_laser = 1;
    if(errgen_laserState){
        cantools_exit_laser();
        if(cantools_init_laser())
            return 1;

        if(Laser_GetPositionOffset(&ml, &sl)<0)
            return 1;
    }
    gtksig_in_reinit_laser = 0;
    return 0;
}
/**
* INITIALISATION LOOP
**/
gpointer cantools_init_loop(gpointer inutile) {
    int i,k=0;
    INTEGER32 j = 0;

    //Init Laser
    if(cantools_init_laser()); //return;

    while (run_init) {

        //check laser state
        if(!gtksig_in_reinit_laser) cantools_checkPlotState_laser();

        //Affichage erreur position cables estimée
        //gui_label_set("labEstimErrPosCabl", "NA");

        /**MOTORS**/
        j++;
        k = 0;
        for (i=0; i<SLAVE_NUMBER;i++) {
            if (slave_get_state_with_index(i) <= STATE_LSS_CONFIG) {
                k=1;
            }
        }
        if (k==1) {
            if (getState(&SpirallingMaster_Data) == Operational)
                setState(&SpirallingMaster_Data, Pre_operational);
        } else {
            if (getState(&SpirallingMaster_Data) == Pre_operational)
                setState(&SpirallingMaster_Data, Operational);
        }
        for (i=0; i<SLAVE_NUMBER;i++) {
            // Configuration LSS
            if (slave_get_state_with_index(i) == STATE_LSS_CONFIG) {
                printf("\n\nSLAVE STATE : %s \n\n",slave_get_state_title(slave_get_state_with_index(i)));
                masterSendNMTstateChange (&SpirallingMaster_Data, slave_get_node_with_index(i), NMT_Stop_Node);
                if (!lsstools_loop(slave_get_node_with_index(i),0)) {
                    slave_set_state_error_with_index(i,ERROR_STATE_LSS);
                    errgen_set(ERR_LSS_CONFIG);
                } else {
                    slave_set_state_with_index(i,STATE_PREOP);
                }
            }
            // Boot up
            if (slave_get_state_with_index(i) == STATE_PREOP) {
                masterSendNMTstateChange (&SpirallingMaster_Data, slave_get_node_with_index(i), NMT_Reset_Node);
            }
            // Configuration PreOp

            if (slave_get_state_with_index(i) == STATE_CONFIG) {
                printf("\n\nSLAVE STATE : %s \n\n",slave_get_state_title(slave_get_state_with_index(i)));
                if(slave_config(slave_get_id_with_index(i))) { //preop a controler
                    slave_set_state_with_index(i,STATE_OP);
                } else {
                    slave_set_state_error_with_index (i,ERROR_STATE_PREOP);
                    errgen_set(ERR_SLAVE_CONFIG);
                }
            }

            // Passage en mode operational
            if (slave_get_state_with_index(i) == STATE_OP) {
                masterSendNMTstateChange (&SpirallingMaster_Data, slave_get_node_with_index(i), NMT_Start_Node);
            }
            // Mise des moteurs en mode opérationnel
            if (slave_get_state_with_index(i) == STATE_SON) {
                printf("\n\nSLAVE STATE : %s \n\n",slave_get_state_title(slave_get_state_with_index(i)));
                if(motor_switch_on(slave_get_id_with_index(i))) {
                    slave_set_state_with_index(i,STATE_READY);
                } else {
                    slave_set_state_error_with_index (i,ERROR_STATE_SON);
                    errgen_set(ERR_SLAVE_CONFIG_MOTOR_SON);
                }
            }
        }
        keyword_maj();
        usleep(500000);
    }
    g_thread_exit(NULL);
}

