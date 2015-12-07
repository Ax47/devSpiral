#include "slave.h"
#include "SpirallingControl.h"
#include "cantools.h"
#include "keyword.h"
#include "strtools.h"
#include <stdlib.h>
#include "errgen.h"
#include "gui.h"
#include "motor.h"
#include <glib.h>
extern pthread_mutex_t lock_slave;
extern SLAVES_conf slaves[SLAVE_NUMBER];
extern INTEGER32 vel_inc_V;

char* motor_v_fn = "motor_v_config.txt";
char* motor_c_fn = "motor_c_config.txt";
char* motor_r_fn = "motor_r_config.txt";


/**
* Série de configuration suite à la mise d'un noeud en état préop
* Renvoie un nombre d'érreur, si 0 pas d'erreur
**/
int slave_config(char* slave_id) {
    UNS8 nodeID = slave_get_node_with_id(slave_id);
    /** ACTIVATION HEARTBEAT PRODUCER **/
    UNS16 heartbeat_prod = HB_PROD;
    SDOR hbprod = {0x1017,0x00,0x06};
    int error = 0, i=0;
    if(!cantools_write_sdo(nodeID,hbprod,&heartbeat_prod)) {
        errgen_set(ERR_SLAVE_CONFIG_HB);
        return 0;
    }
    /** CONFIGURATION PDOT **/
    // StatusWord, Error Code
    if(!cantools_PDO_map_config(nodeID,0x1A00,0x60410010,0x603F0010,0)) {
        errgen_set(ERR_SLAVE_CONFIG_PDOT1);
        return 0;
    }
    // Voltage, Temp, Polarity
    if(!cantools_PDO_map_config(nodeID,0x1A01,0x20150110,0x20180108,0)) {
        errgen_set(ERR_SLAVE_CONFIG_PDOT2);
        return 0;
    }
    // Velocity
    if(!cantools_PDO_map_config(nodeID,0x1A02,0x606C0020,0)) {
        errgen_set(ERR_SLAVE_CONFIG_PDOT3);
        return 0;
   }
    /** CONFIGURATION PDOR **/
    // Vitesse
    if(!cantools_PDO_map_config(nodeID,0x1600,0x60FF0020,0)) {
        errgen_set(ERR_SLAVE_CONFIG_PDOR1);
        return 0;
    }
    /** ACTIVATION DES PDOs **/
    if (!cantools_PDO_trans(nodeID,0x1800,0xFF,0x05)) {
        errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO);
        return 0;
    }
    if (!cantools_PDO_trans(nodeID,0x1801,0xFF,0x05)) {
        errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO);
        return 0;
    }
    if (!cantools_PDO_trans(nodeID,0x1802,0xFF,0x00)) {
        errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO);
        return 0;
    }
    /** FULL CURRENT **/
    SDOR curAdd = {0x2204,0x00,0x05};
    UNS8 cur = 0x64;
    if (!cantools_write_sdo(nodeID,curAdd,&cur)) {
        errgen_set(ERR_SLAVE_CONFIG_CURRENT);
        return 0;
    }
    /** FILE CONFIG **/
    FILE* motor_fn = NULL;
    SDOR profAdd = {0x6060,0x00,0x02};
    UNS8 prof=0;
    if (slave_id == "vitesse" || slave_id == "vitesse_aux") {
        motor_fn = fopen(motor_v_fn,"r");
        prof = 3;
    } else if (slave_id == "couple") {
        motor_fn = fopen(motor_c_fn,"r");
        prof = 4;
    } else if (slave_id == "rotation") {
        motor_fn = fopen(motor_r_fn,"r");
        prof = 3;
    }
    if (!cantools_write_sdo(nodeID,profAdd,&prof)) {
        errgen_set(ERR_SLAVE_CONFIG_PROFILE);
        return 0;
    }
    if (motor_fn != NULL) {
        char title[20];
        int data;
        while(fscanf(motor_fn, "%s %d",&title,&data) == 2) {
            if (strcmp(title,"Max_Velocity") == 0) {
                SDOR mvelAdd = {0x6081,0x00,0x07};
                UNS32 mvel = (UNS32)data;
                if (!cantools_write_sdo(nodeID,mvelAdd,&mvel)) {
                    errgen_set(ERR_SLAVE_CONFIG_MAX_VELOCITY);
                    return 0;
                }
            }
            if (strcmp(title,"Acceleration") == 0) {
                SDOR accAdd = {0x6083,0x00,0x07};
                UNS32 acc = (UNS32)data;
                if (!cantools_write_sdo(nodeID,accAdd,&acc)) {
                    errgen_set(ERR_SLAVE_CONFIG_ACCELERATION);
                    return 0;
                }
            }
            if (strcmp(title,"Deceleration") ==0) {
                SDOR dccAdd = {0x6084,0x00,0x07};
                UNS32 dcc = (UNS32)data;
                if (!cantools_write_sdo(nodeID,dccAdd,&dcc)) {
                    errgen_set(ERR_SLAVE_CONFIG_DECELERATION);
                    return 0;
                }
            }
            if (strcmp(title,"QS_Deceleration") ==0) {
                SDOR qsdccAdd = {0x6085,0x00,0x07};
                UNS32 qsdcc = (UNS32)data;
                if (!cantools_write_sdo(nodeID,qsdccAdd,&qsdcc)) {
                    errgen_set(ERR_SLAVE_CONFIG_DECELERATION_QS);
                    return 0;
                }
            }
            if (strcmp(title,"Torque") ==0) {
                SDOR torqueAdd = {0x6071,0x00,0x03};
                INTEGER16 torque = (INTEGER16)data;
                if (!cantools_write_sdo(nodeID,torqueAdd,&torque)) {
                    errgen_set(ERR_SLAVE_CONFIG_TORQUE);
                    return 0;
                }
            }
            if (strcmp(title,"Torque_Slope") ==0) {
                SDOR tslopeAdd = {0x6087,0x00,0x07};
                UNS32 tslope = (UNS32)data;
                if (!cantools_write_sdo(nodeID,tslopeAdd,&tslope)) {
                    errgen_set(ERR_SLAVE_CONFIG_TORQUE_SLOPE);
                    return 0;
                }
            }
            if (strcmp(title,"Torque_Velocity") ==0) {
                SDOR tmvelAdd = {0x2704,0x00,0x05};
                UNS8 tmvel = (UNS8)data;
                if (!cantools_write_sdo(nodeID,tmvelAdd,&tmvel)) {
                    errgen_set(ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY);
                    return 0;
                }
            }
            if (strcmp(title,"Velocity_Inc") ==0) {
                vel_inc_V = (INTEGER32)data;
            }
        }
        fclose(motor_fn);
    } else {
        return 0;
    }
    return 1;
}
/**
* Affiche les paramètres
**/
int slave_load_config() {
    INTEGER32 vali32 = 0;
    SDOR acc_ad = {0x6083,0x00,0x07};
    SDOR dcc_ad = {0x6084,0x00,0x07};
    SDOR qsdcc_ad = {0x6085,0x00,0x07};
    SDOR torque_ad = {0x6071,0x00,0x03};
    SDOR torqueSlope_ad = {0x6087,0x00,0x07};
    int i = 0;
    UNS32 datu32;
    UNS8 nodeID;
    for (i=0;i<SLAVE_NUMBER;i++) {
        nodeID = slave_get_node_with_index(i);
        if (slave_get_id_with_index(i) == "vitesse") {
            /** Moteur Vitesse **/
            // Acceleration
            if (!cantools_read_sdo(nodeID,acc_ad,&datu32)) {
                errgen_set(ERR_LOAD_ACCELERATION);
                return 0;
            }
            vali32 = datu32;
            gui_entry_set("entParamMVelAcc",strtools_gnum2str(&vali32,0x04));
            // Deceleration
            if (!cantools_read_sdo(nodeID,dcc_ad,&datu32)) {
                errgen_set(ERR_LOAD_DECELERATION);
                return 0;
            }
            vali32 = datu32;
            gui_entry_set("entParamMVelDcc",strtools_gnum2str(&vali32,0x04));
            // QS Deceleration
            if (!cantools_read_sdo(nodeID,qsdcc_ad,&datu32)) {
                errgen_set(ERR_LOAD_DECELERATION_QS);
                return 0;
            }
            vali32 = datu32;
            gui_entry_set("entParamMVelDccqs",strtools_gnum2str(&vali32,0x04));
            // Vel inc
            gui_entry_set("entParamMVelVelinc",strtools_gnum2str(&vel_inc_V,0x04));
        }
        if (slave_get_id_with_index(i) == "couple") {
            /** Moteur Couple **/
            INTEGER16 dati16;
            // Torque
            if (!cantools_read_sdo(nodeID,torque_ad,&dati16)) {
                errgen_set(ERR_LOAD_TORQUE);
                return 0;
            }
            gui_entry_set("entParamMCoupleCouple",strtools_gnum2str(&dati16,0x03));
            // Torque Slope
            if (!cantools_read_sdo(nodeID,torqueSlope_ad,&datu32)) {
                errgen_set(ERR_LOAD_TORQUE_SLOPE);
                return 0;
            }
            vali32 = (INTEGER32)datu32;
            gui_entry_set("entParamMCoupleSlope",strtools_gnum2str(&vali32,0x04));
        }
    }
    return 1;
}

int slave_save_config(int gen_config) {
    INTEGER32 vali32 = 0;
    INTEGER16 vali16 = 0;
    UNS32 datu32;
    SDOR acc_ad = {0x6083,0x00,0x07};
    SDOR dcc_ad = {0x6084,0x00,0x07};
    SDOR qsdcc_ad = {0x6085,0x00,0x07};
    SDOR torque_ad = {0x6071,0x00,0x03};
    SDOR torqueSlope_ad = {0x6087,0x00,0x07};
    UNS8 nodeID;
    int i;
    for (i=0;i<SLAVE_NUMBER;i++) {
        nodeID = slave_get_node_with_index(i);
        if (slave_get_id_with_index(i) == "vitesse" || "vitesse_aux") {
        /** Moteur Vitesse **/
            // Acceleration
            vali32 = (INTEGER32)gui_str2num(gui_entry_get("entParamMVelAcc"));
            if (!motor_check_acceleration(vali32)) {
                errgen_set(ERR_ACCELERATION_SET);
                return 0;
            }
            datu32 = vali32;
            char* strvelacc = strtools_concat("Acceleration ",gui_entry_get("entParamMVelAcc"),NULL);
            if (!cantools_write_sdo(nodeID,acc_ad,&datu32)) {
                errgen_set(ERR_ACCELERATION_SAVE);
                return 0;
            }
            // Deceleration
            vali32 = (INTEGER32)gui_str2num(gui_entry_get("entParamMVelDcc"));
            if (!motor_check_acceleration(vali32)) {
                errgen_set(ERR_ACCELERATION_SET);
                return 0;
            }
            datu32 = vali32;
            char* strveldcc = strtools_concat("Deceleration ",gui_entry_get("entParamMVelDcc"),NULL);
            if (!cantools_write_sdo(nodeID,dcc_ad,&datu32)) {
                errgen_set(ERR_DECELERATION_SAVE);
                return 0;
            }
            // QS Deceleration
            vali32 = (INTEGER32)gui_str2num(gui_entry_get("entParamMVelDccqs"));
            if (!motor_check_acceleration(vali32)) {
                errgen_set(ERR_ACCELERATION_SET);
                return 0;
            }
            datu32 = vali32;
            char* strveldccqs = strtools_concat("QS_Deceleration ",gui_entry_get("entParamMVelDccqs"),NULL);
            if (!cantools_write_sdo(nodeID,qsdcc_ad,&datu32)) {
                errgen_set(ERR_DECELERATION_QS_SAVE);
                return 0;
            }
            // Vel inc
            vel_inc_V = (INTEGER32)gui_str2num(gui_entry_get("entParamMVelVelinc"));
            if (!motor_check_velocity(vel_inc_V)) {
                errgen_set(ERR_VELOCITY_SET);
                return 0;
            }
            char* strvelvelinc = strtools_concat("Velocity_Inc ",gui_entry_get("entParamMVelVelinc"),NULL);
            // Sauvegarde des fichiers
            if (gen_config) {
                if (!slave_gen_config_file("vitesse",strvelacc,strveldcc,strveldccqs,strvelvelinc,NULL)){
                    errgen_set(ERR_SAVE_FILE_GEN);
                    return 0;
                }
            }
        }
        if (slave_get_id_with_index(i) == "couple") {
        /** Moteur Vitesse **/
            // Torque
            vali16 = (INTEGER16)gui_str2num(gui_entry_get("entParamMCoupleCouple"));
            if (!motor_check_torque(vali16)) {
                errgen_set(ERR_TORQUE_SET);
                return 0;
            }
            char* strcouple = strtools_concat("Torque ",gui_entry_get("entParamMCoupleCouple"),NULL);
            if (!cantools_write_sdo(nodeID,torque_ad,&vali16)) {
                errgen_set(ERR_TORQUE_SAVE);
                return 0;
            }
            // Torque Slope
            vali32 = (INTEGER32)gui_str2num(gui_entry_get("entParamMCoupleSlope"));
            if (!motor_check_torque_slope(vali32)) {
                errgen_set(ERR_TORQUE_SLOPE_SET);
                return 0;
            }

            datu32 = vali32;
            char* strcoupleslope = strtools_concat("Torque_Slope ",gui_entry_get("entParamMCoupleSlope"),NULL);
            if (!cantools_write_sdo(nodeID,torqueSlope_ad,&datu32)) {
                errgen_set(ERR_TORQUE_SLOPE_SAVE);
                return 0;
            }

            if (gen_config) {
                if (!slave_gen_config_file("couple",strcouple,strcoupleslope,NULL)){
                    errgen_set(ERR_SAVE_FILE_GEN);
                    return 0;
                }
            }
        }
    }
    return 1;
}

/** VERIFICATION DES FICHIERS DE CONFIGURATIONS **/
int slave_check_config_file(char* slave_id) {
    FILE* f = NULL;
    if (slave_id == "vitesse") {
        f = fopen(motor_v_fn,"r");
        if (f != NULL) {
            char title[20];
            int data;
            int i = 0;
            while(fscanf(f, "%s %d",&title,&data) == 2) {
                if (strcmp(title,"Acceleration") == 0) i++;
                if (strcmp(title,"Deceleration") == 0) i++;
                if (strcmp(title,"QS_Deceleration") == 0) i++;
                if (strcmp(title,"Velocity_Inc") == 0) i++;
            }
            fclose(f);
            if (i == 4) return 1;
            else return 0;
        } else return 0;
    } else if (slave_id == "couple") {
        f = fopen(motor_c_fn,"r");
        if (f != NULL) {
            char title[20];
            int data;
            int i = 0;
            while(fscanf(f, "%s %d",&title,&data) == 2) {
                if (strcmp(title,"Torque") == 0) i++;
                if (strcmp(title,"Torque_Slope") == 0) i++;
            }
            fclose(f);
            if (i == 2) return 1;
            else return 0;
        } else return 0;
    } else if (slave_id == "rotation") {
        f = fopen(motor_r_fn,"r");
        if (f != NULL) {
            char title[20];
            int data;
            int i = 0;
            while(fscanf(f, "%s %d",&title,&data) == 2) {
                if (strcmp(title,"Acceleration") == 0) i++;
                if (strcmp(title,"Deceleration") == 0) i++;
                if (strcmp(title,"QS_Deceleration") == 0) i++;
            }
            fclose(f);
            if (i == 3) return 1;
            else return 0;
        } else return 0;
    }
    return 0;
}

/** Generation des fichiers de configuration **/
int slave_gen_config_file(char* slave_id, ...) {
    FILE * f = NULL;
    va_list ap;
    va_start(ap,slave_id);
    char *arg = va_arg(ap,char*);
    if (slave_id == "vitesse") f = fopen(motor_v_fn,  "w+");
    else if (slave_id == "couple") f = fopen(motor_c_fn,  "w+");
    else if (slave_id == "rotation") f = fopen(motor_r_fn,  "w+");
    else return 0;
    if (f != NULL) {
        char* res = "";
        while ( arg != NULL) {
            res = strtools_concat(res,arg,"\n",NULL);
            arg = va_arg(ap,char*);
        }
        res = strtools_concat(res,"/** Fichier de configuration du moteur ",slave_id," **/",NULL);
        va_end(ap);
        fputs(res,f);
        fclose(f);
        return 1;
    } else {
        printf("Le fichier ne s'ouvre pas");
        return 0;
    }
}


/**
* Retourne node en fonction de i
**/
UNS8 slave_get_node_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        UNS8 dat = slaves[i].node;
        pthread_mutex_unlock (&lock_slave);
        return dat;
    } else {
        printf("Index trop grand : %d \n",i); exit(EXIT_FAILURE);
    }
}
/**
* Retourne node en fonction de id
**/
UNS8 slave_get_node_with_id(char* id) {
    int i,res = -1;
    pthread_mutex_lock (&lock_slave);
    for (i=0; i<SLAVE_NUMBER; i++) {
        if (slaves[i].id == id) res=i;
    }
    pthread_mutex_unlock (&lock_slave);
    if (res == -1) {
        printf("ERREUR ID %s non trouvé",id);
        exit(EXIT_FAILURE);
    } else
        return slaves[res].node;
}
/**
* Retourne state en fonction index
**/
int slave_get_state_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        int dat = slaves[i].state;
        pthread_mutex_unlock (&lock_slave);
        return dat;
    } else {
        printf("Index trop grand : %d \n",i); exit(EXIT_FAILURE);
    }
}
/**
* Retourne state en fonction id
**/
int slave_get_state_with_id(char* slave_id) {
    int i = slave_get_index_with_id(slave_id);
    pthread_mutex_lock (&lock_slave);
    int dat = slaves[i].state;
    pthread_mutex_unlock (&lock_slave);
    return dat;
}
/**
* Affectation du state en fonction index
**/
void slave_set_state_with_index(int i, int dat) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        slaves[i].state = dat;
        pthread_mutex_unlock (&lock_slave);
    } else {
        printf("Index trop grand : %d \n",i); exit(EXIT_FAILURE);
    }
}
/**
* Affectation du state en fonction id
**/
void slave_set_state_with_id(char* slave_id, int dat) {
    int i = slave_get_index_with_id(slave_id);
    pthread_mutex_lock (&lock_slave);
    slaves[i].state = dat;
    pthread_mutex_unlock (&lock_slave);
}
/**
* Retourne stateerror en fonction index
**/
int slave_get_state_error_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        int dat = slaves[i].state_error;
        pthread_mutex_unlock (&lock_slave);
        return dat;
    } else {
        printf("Index trop grand : %d",i);
        exit(EXIT_FAILURE);
    }
}
/**
* Retourne stateerror en fonction id
**/
int slave_get_state_error_with_id(char* slave_id) {
    int i = slave_get_index_with_id(slave_id);
    pthread_mutex_lock (&lock_slave);
    int dat = slaves[i].state_error;
    pthread_mutex_unlock (&lock_slave);
    return dat;
}
/**
* Affectation du state_error en fonction de l'index
**/
void slave_set_state_error_with_index (int i, int dat) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        slaves[i].state_error = dat;
        pthread_mutex_unlock (&lock_slave);
    } else {
        printf("Index trop grand : %d",i);
        exit(EXIT_FAILURE);
    }
}
/**
* Affectation du state_error en fonction de l'index
**/
void slave_set_state_error_with_id (char* slave_id, int dat) {
    int i = slave_get_index_with_id(slave_id);
    pthread_mutex_lock (&lock_slave);
    slaves[i].state_error = dat;
    pthread_mutex_unlock (&lock_slave);
}
/**
* Retourne powerstate en fonction de l'index
**/
UNS16 slave_get_powerstate_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        UNS16 dat = slaves[i].powerstate;
        pthread_mutex_unlock (&lock_slave);
        return dat;
    } else {
        printf("Index trop grand : %d",i);
        exit(EXIT_FAILURE);
    }
}
/**
* Retourne powerstate en fonction de l'id
**/
UNS16 slave_get_powerstate_with_id(char* slave_id) {
    int i = slave_get_index_with_id(slave_id);
    pthread_mutex_lock (&lock_slave);
    UNS16 dat = slaves[i].powerstate;
    pthread_mutex_unlock (&lock_slave);
    return dat;
}
/**
* Affectation du powerstate en fonction de l'index
**/
void slave_set_powerstate_with_index (int i, int dat) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        slaves[i].powerstate = dat;
        pthread_mutex_unlock (&lock_slave);
    } else {
        printf("Index trop grand : %d",i);
        exit(EXIT_FAILURE);
    }
}
/**
* Affectation du powerstate en fonction de l'id
**/
void slave_set_powerstate_with_id (char* slave_id, int dat) {
    int i = slave_get_index_with_id(slave_id);
    pthread_mutex_lock (&lock_slave);
    slaves[i].powerstate = dat;
    pthread_mutex_unlock (&lock_slave);
}
/**
* Retourne index en fonction du nodeID
**/
int slave_get_index_with_node(UNS8 nodeID) {
    int i,res = -1;
    pthread_mutex_lock (&lock_slave);
    for (i=0; i<SLAVE_NUMBER; i++) {
        if (slaves[i].node == nodeID) res=i;
    }
    pthread_mutex_unlock (&lock_slave);
    if (res != -1)
        return res;
    else {
        printf("nodeID non trouvé %#.2x",nodeID);
        exit(EXIT_FAILURE);
    }
}
/**
* Retourne index en fonction de ID
**/
int slave_get_index_with_id(char* slave_id) {
    int i,res = -1;
    pthread_mutex_lock (&lock_slave);
    for (i=0; i<SLAVE_NUMBER; i++) {
        if (slaves[i].id == slave_id) res=i;
    }
    pthread_mutex_unlock (&lock_slave);
    if (res != -1)
        return res;
    else {
        printf("Slave id introuvable : %s",slave_id);
        exit(EXIT_FAILURE);
    }
}
/**
* retourne id en fonction de l'index
**/
char* slave_get_id_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        char* dat;
        pthread_mutex_lock (&lock_slave);
        dat = slaves[i].id;
        pthread_mutex_unlock (&lock_slave);
        return dat;
    } else {
        printf("Index trop grand : %d",i);
        exit(EXIT_FAILURE);
    }
}
/**
* retourne id en fonction de l'index
**/
char* slave_get_id_with_node(UNS8 node) {
    int i = slave_get_index_with_node(node);
    char* dat;
    pthread_mutex_lock (&lock_slave);
    dat = slaves[i].id;
    pthread_mutex_unlock (&lock_slave);
    return dat;
}
/**
* Retourne la correspondance en texte de l'état
**/
char* slave_get_state_title(int state) {
    if(state == STATE_LSS_CONFIG) return STATE_LSS_CONFIG_TXT;
    else if(state == STATE_PREOP) return STATE_PREOP_TXT;
    else if(state == STATE_CONFIG) return STATE_CONFIG_TXT;
    else if(state == STATE_OP) return STATE_OP_TXT;
    else if(state == STATE_SON) return STATE_SON_TXT;
    else if(state == STATE_READY) return STATE_READY_TXT;
    else return DEFAULT;
}
/**
* Retourne le nom de l'icone correspondant à l'état
**/
char* slave_get_state_img (int state) {
    if(state == STATE_LSS_CONFIG) return "gtk-properties";
    else if(state == STATE_PREOP) return "media-seek-forward";
    else if(state == STATE_CONFIG) return "gtk-execute";
    else if(state == STATE_OP) return "go-next";
    else if(state == STATE_SON) return "gtk-connect";
    else if(state == STATE_READY) return "gtk-yes";
    else return "gtk-no";
}
/**
* Retourne la correspondance en texte de l'état erreur
**/
char* slave_get_state_error_title(int state) {
    if(state == ERROR_STATE_NULL) return ERROR_STATE_NULL_TXT;
    else if(state == ERROR_STATE_LSS) return ERROR_STATE_LSS_TXT;
    else if(state == ERROR_STATE_PREOP) return ERROR_STATE_PREOP_TXT;
    else if(state == ERROR_STATE_OP) return ERROR_STATE_OP_TXT;
    else if(state == ERROR_STATE_SON) return ERROR_STATE_SON_TXT;
    else return DEFAULT;
}

int slave_id_exist(char* slave_id) {
    int i = 0;
    int res = 0;
    pthread_mutex_lock (&lock_slave);
    for (i=0;i<SLAVE_NUMBER;i++) {
        if (slaves[i].id == slave_id) res = 1;
    }
    pthread_mutex_unlock (&lock_slave);
    if (res) return 1;
    else return 0;
}
