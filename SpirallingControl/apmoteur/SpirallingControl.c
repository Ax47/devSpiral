#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#include <gtk/gtk.h>
#include <glib.h>
#include "canfestival.h"
#include "keyword.h"

#include "master.h"
#include "slave.h"
#include "SpirallingControl.h"
#include "cantools.h"
#include "gtksig.h"
#include "gui.h"
#include "lsstools.h"
#include "od_callback.h"
#include "strtools.h"
#include "errgen.h"

#include "laser_asserv.h"


//lasers
laser ml, sl, lsim;
//definition de l'hélice
struct Helix_User_Data HelixUserData;


// Plateforme MASTER
s_BOARD MasterBoard = {"0", "1M"};

// Définition des esclaves
volatile SLAVES_conf slaves[SLAVE_NUMBER] = {
        //{"vitesse",0x02,0x0800005A,0x00018E70,0x00000710,282150321,0,0,0x0000},
       // {"vitesse_aux",0x03,0x0800005A,0x00018E70,0x00000710,282150324/*0x138F04FD*/,0,0,0x0000}//,
        //{"rotation",0x04,0x0800005A,0x00018E70,0x00000710,328140027,0,0,0x0000}//,
        //{"rotation",0x04,0x0800005A,0x00018E70,0x00000710,0x10d145b4,0,0,0x0000}//,
};
pthread_mutex_t lock_slave = PTHREAD_MUTEX_INITIALIZER; // Mutex de slaves

int run_init = 1; // Boucle d'initialisation
// Incrémentation vitese
INTEGER32 vel_inc_V = 1000;

// A définir mettre a jour
void catch_signal(int sig) {
  signal(SIGTERM, catch_signal);
  signal(SIGINT, catch_signal);
  printf("Got Signal %d\n",sig);
}

/**
* Fermeture de l'application
**/
void Exit() {
    unsigned int err_l;
    int i = 0;
    run_init = 0;

    //exit asserv
    if(run_asserv){
        printf("EXIT ASSERV\n");
        if(laser_asserv_stop()){
            errgen_set(ERR_LASER_ASSERV_STOP);
        }
    }

    //fermeture laser
    cantools_exit_laser();

    //fermeture moteurs
    for (i=0; i<SLAVE_NUMBER; i++) {
        motor_start(slave_get_id_with_index(i),0);
        motor_switch_off(slave_get_id_with_index(i));
    }
    masterSendNMTstateChange (&SpirallingMaster_Data, 0, NMT_Stop_Node);
    setState(&SpirallingMaster_Data, Stopped);
    gtk_main_quit();
    canClose(&SpirallingMaster_Data);
    TimerCleanup();
}

int main(int argc,char **argv) {
    // Initialisation de l'interface graphique
    gchar* thgui = "gui";
    GError * guierr;
    XInitThreads();
    GThread * guithread = g_thread_try_new (thgui, gui_init,NULL, &guierr);
    if (guierr == NULL)
        errgen_set(ERR_GUI_LOOP_RUN);
    sleep(1);
// Configuration du socket
    pid_t pid = fork();

    if (pid < 0) {
        printf("A fork error has occurred.\n");
        exit(-1);
    } else {
        if (pid == 0) {
            execlp("./test.sh","test.sh",NULL);
            errgen_set(ERR_DRIVER_UP); // Driver error
        } else {
            wait(0);
        }
    }

// Control des fichiers de configuration
    if(!slave_check_config_file("vitesse")) {
        if(!slave_gen_config_file("vitesse","Acceleration 1000000","Deceleration 1000000","QS_Deceleration 1000000","Velocity_Inc 1000",NULL)) {
            errgen_set(ERR_FILE_CONFIG_GEN);
        }
    }
    if(!slave_check_config_file("couple")) {
        if(!slave_gen_config_file("couple","Torque 50","Torque_Slope 1000",NULL)) {
            errgen_set(ERR_FILE_CONFIG_GEN);
        }
    }
    if(!slave_check_config_file("rotation")) {
        if(!slave_gen_config_file("rotation","Acceleration 1000000","Deceleration 1000000","QS_Deceleration 1000000",NULL)) {
            errgen_set(ERR_FILE_CONFIG_GEN);
        }
    }

 //DEFINITION DE LA CONSIGNE HELICE UTILISATEUR

    //a faire : interface
  unsigned long d[1];
  long int p[1];
  d[0] = 30000;
  p[0] = 4000;

  HelixUserData.dmax = d[0];
  HelixUserData.NbrOfEntries = 1;
  HelixUserData.d = d;
  HelixUserData.p = p;

// Handler pour arret manuel
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

// Chemin vers la librairie CANFESTIVAL
    char* LibraryPath="../drivers/can_socket/libcanfestival_can_socket.so";

// Chargement de la libraire
	if (LoadCanDriver(LibraryPath) == NULL)
        errgen_set(ERR_DRIVER_LOAD);

	TimerInit();


    // Callback du maitre
    SpirallingMaster_Data.heartbeatError = SpirallingMaster_heartbeatError;
    SpirallingMaster_Data.initialisation = SpirallingMaster_initialisation;
    SpirallingMaster_Data.preOperational = SpirallingMaster_preOperational;
    SpirallingMaster_Data.operational = SpirallingMaster_operational;
    SpirallingMaster_Data.stopped = SpirallingMaster_stopped;
    SpirallingMaster_Data.post_sync = SpirallingMaster_post_sync;
    SpirallingMaster_Data.post_TPDO = SpirallingMaster_post_TPDO;
    SpirallingMaster_Data.post_emcy = SpirallingMaster_post_emcy;
    SpirallingMaster_Data.post_SlaveBootup = SpirallingMaster_post_SlaveBootup;
    SpirallingMaster_Data.post_SlaveStateChange = SpirallingMaster_post_SlaveStateChange;

    if(!canOpen(&MasterBoard,&SpirallingMaster_Data)){
        errgen_set(ERR_DRIVER_OPEN);
	}

    // Initialisation du maitre
    master_init();

    // Initialisation de l'interface graphique
    gchar* thinit = "init_loop";
    GError * initerr;
    if(!((errgen_state & ERR_DRIVER_OPEN) || (errgen_state & ERR_DRIVER_LOAD) || (errgen_state & ERR_DRIVER_UP) || (errgen_state & ERR_FILE_CONFIG_GEN))){
        GThread * initthread = g_thread_try_new (thinit, cantools_init_loop,NULL, &initerr);
        if (initerr == NULL)
            errgen_set(ERR_INIT_LOOP_RUN);

        g_thread_join (initthread);
    }

    g_thread_join (guithread);

	return 0;
}


