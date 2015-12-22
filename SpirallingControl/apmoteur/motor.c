#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include "motor.h"
#include "strtools.h"
#include "gui.h"
#include "keyword.h"
#include "SpirallingControl.h"
#include "master.h"
#include "canfestival.h"
#include "cantools.h"
#include "errgen.h"

#include "laser_asserv.h"
#include "laser_simulation.h"

int motor_switch_step = 0;
const int motor_switch_step_max = 10;


UNS8 motor_get_state(UNS16 state) {
    char *bstate = strtools_hex2bin(&state,0x06);
    if (bstate != NULL) {
        if (bstate[0] == '0' && bstate[1] == '0' && bstate[2] == '0' && bstate[3] == '0' &&
            bstate[6] == '0' ) {
            return NR2SON;

        } else if (bstate[0] == '0' && bstate[1] == '0' && bstate[2] == '0' && bstate[3] == '0'  &&
            bstate[6] == '1' ) {
            return SOND;

        } else if (bstate[0] == '1' && bstate[1] == '0' && bstate[2] == '0' && bstate[3] == '0'  &&
            bstate[5] == '1' && bstate[6] == '0' ) {
            return R2SON;

        } else if (bstate[0] == '1' && bstate[1] == '1' && bstate[2] == '0' && bstate[3] == '0'  &&
            bstate[5] == '1' && bstate[6] == '0' ) {
            return SON;

        } else if (bstate[0] == '1' && bstate[1] == '1' && bstate[2] == '1' && bstate[3] == '0'  &&
            bstate[5] == '1' && bstate[6] == '0' ) {
            return OENABLED;

        } else if (bstate[0] == '1' && bstate[1] == '1' && bstate[2] == '1' && bstate[3] == '0'  &&
            bstate[5] == '0' && bstate[6] == '0' ) {
            return QUICK_STOP_A;

        } else if (bstate[0] == '1' && bstate[1] == '1' && bstate[2] == '1' && bstate[3] == '1'  &&
            bstate[6] == '0' ) {
            return FAULT_REACTION_A;

        } else if (bstate[0] == '0' && bstate[1] == '0' && bstate[2] == '0' && bstate[3] == '1'  &&
            bstate[6] == '0' ) {
            return FAULT;
        } else {
            return NR2SON;
        }
    } else {
        return NR2SON;
    }
}

/**
* AFFICHAGE DU TARGET
* Retourne 2 à l'arret
* retourne 1 mouvement en cours
* retourne 0 erreur
**/
int motor_target(UNS16 state) {
    char *bstate = strtools_hex2bin(&state,0x06);
    if (bstate != NULL) {
        if (motor_get_state(state) == OENABLED) {
            if (bstate[10] == 0)
                return 1;
            else
                return 2;
        } else return 2;
    } else
        return 0;
}



char* motor_get_state_title(UNS8 key) {
    if (key == 0x00) {
        return NOT_DEFINED_txt;
    } else if (key == 0x01) {
        return NR2SON_txt;
    } else if (key == 0x02) {
        return SOND_txt;
    } else if (key == 0x03) {
        return R2SON_txt;
    } else if (key == 0x04) {
        return SON_txt;
    } else if (key == 0x05) {
        return OENABLED_txt;
    } else if (key == 0x06) {
        return QUICK_STOP_A_txt;
    } else if (key == 0x07) {
        return FAULT_REACTION_A_txt;
    } else if (key == 0x08) {
        return FAULT_txt;
    }
}

char* motor_get_error_title(UNS16 data) {
    if (data == MOTOR_ERROR_CAN_OVERRUN ) return MOTOR_ERROR_CAN_OVERRUN_TXT;
    if (data == MOTOR_ERROR_CAN_PASSIVE ) return MOTOR_ERROR_CAN_PASSIVE_TXT;
    if (data == MOTOR_ERROR_HEARTBEAT ) return MOTOR_ERROR_HEARTBEAT_TXT;
    if (data == MOTOR_ERROR_BUSOFF_RECOVER ) return MOTOR_ERROR_BUSOFF_RECOVER_TXT;
    if (data == MOTOR_ERROR_BUSOFF_PENDING ) return MOTOR_ERROR_BUSOFF_PENDING_TXT;
    if (data == MOTOR_ERROR_BUSOFF_RECOVER ) return MOTOR_ERROR_BUSOFF_RECOVER_TXT;
    if (data == MOTOR_ERROR_PDO ) return MOTOR_ERROR_PDO_TXT;
    if (data == MOTOR_ERROR_TEMP ) return MOTOR_ERROR_TEMP_TXT;
    if (data == MOTOR_ERROR_IDLE ) return MOTOR_ERROR_IDLE_TXT;
}

int motor_switch_on(char* slave_id) {
    SDOR c_word={0x6040,0x00,0x06};
    UNS16 motorStatus = 0x000;
    motorStatus = slave_get_powerstate_with_id(slave_id);
    UNS8 strState = motor_get_state(motorStatus);
    if (strState != NR2SON) {
        if (motor_switch_step >= motor_switch_step_max) {
            motor_switch_step = 0;
            return 0;
        } else {
            if (strState == SOND) {
                //printf("state 6\n");
                UNS16 word = 0x0006;
                if(!cantools_write_sdo(slave_get_node_with_id(slave_id),c_word,&word)) {
                    motor_switch_step++;
                }
                motor_switch_on(slave_id);
            }
            if (strState == R2SON) {
                //printf("state 7\n");
                UNS16 word = 0x0007;
                if(!cantools_write_sdo(slave_get_node_with_id(slave_id),c_word,&word)) {
                    motor_switch_step++;
                }
                motor_switch_on(slave_id);
            }
            if (strState == SON) {
                //printf("state OK\n");
                motor_switch_step = 0;
                return 1;
            }
        }
    } else {
        //printf("state ERR\n");
        motor_switch_step = 0;
        return 0;
    }
}

/**
* Arret d'un moteur
* Mode SOND
**/
int motor_switch_off(char* slave_id) {
    if (motor_switch_step >= motor_switch_step_max){
        motor_switch_step = 0;
        return 0;
    }
    UNS8 strState = motor_get_state(slave_get_powerstate_with_id(slave_id));
    if (strState != NR2SON) {
        if (strState != SOND) {
            SDOR c_word={0x6040,0x00,0x06};
            UNS16 word = 0x0002;
            if (cantools_write_sdo(slave_get_node_with_id(slave_id),c_word,&word)) {
                motor_switch_step = 0;
                return 1;
            } else {
                motor_switch_step++;
                motor_switch_off(slave_id);
            }
        } else {
            motor_switch_step = 0;
            return 1;
        }
    } else {
        motor_switch_step = 0;
        return 0;
    }
}
/**
* Changement de profile
* 1 : position
* 3 : vitesse
* 4 : torque
* 5 : homming
**/
int motor_change_profile(char* slave_id, int Vmode) {
    SDOR mode = {0x6060,0x00,0x05};
    if(cantools_write_sdo(slave_get_node_with_id(slave_id),mode,&Vmode)) {
        return 1;
    } else {
        return 0;
    }
}
/**
* Passage du mode SON à OP
* start : 1
* stop : 0
**/
int motor_start(char* slave_id,int start) {
    SDOR c_word={0x6040,0x00,0x06};
    UNS16 word;
    if (start) word = 0x000F;
    else word = 0x0007;
    if (cantools_write_sdo(slave_get_node_with_id(slave_id),c_word,&word))
        return 1;
    else
        return 0;
}
/**
* Changement de polarité
* 1 : avant
* 0 : arrière
*/
int motor_forward(char* slave_id,int polar) {
    INTEGER8 dat;
    int error = 0;
    SDOR polar_adress = {0x2030,0x00,0x02};
    if (polar)
        dat = 1;
    else
        dat = -1;
    if (!motor_start(slave_id,0))
        return 0;
    if (!cantools_write_sdo(slave_get_node_with_id(slave_id),polar_adress,&dat))
        return 0;
    if (!motor_start(slave_id,1))
        return 0;
    return 1;
}

int motor_set_torque_speed(UNS8 dat) {
    SDOR torqueSpeed_ad = {0x2704,0x00,0x05};
    if(!cantools_write_sdo(slave_get_node_with_id("couple"),torqueSpeed_ad,&dat))
        return 0;
    return 1;
}
UNS8 motor_get_torque_speed() {
    SDOR torqueSpeed_ad = {0x2704,0x00,0x05};
    UNS8 dat;
    if(!cantools_read_sdo(slave_get_node_with_id("couple"),torqueSpeed_ad,&dat))
        return 0x00;
    return dat;
}

int motor_check_acceleration(INTEGER32 acc) {
    if (acc > 0 && acc < 2000000 ) return 1;
    else return 0;
}
int motor_check_velocity(INTEGER32 acc) {
    if (acc >= 0 && acc < 1000000 ) {return 1;
    printf("acc : %d",acc);}
    else {
    printf("acc : %d",acc);
    return 0;
    }
}

int motor_check_torque(INTEGER16 torque) {
    if (torque > 0 && torque < 1000 ) return 1;
    else return 0;
}

int motor_check_torque_slope(INTEGER32 torqueSlope) {
    if (torqueSlope > 0 && torqueSlope < 10000 ) return 1;
    else return 0;
}

void motor_set_MotRot_Accel(void)
{
    if(!slave_id_exist("rotation"))
        return;

    UNS32 accel_T, decel_T;
    UNS32 accel_R, decel_R;
    SDOR Accel = {0x6083, 0x00, 0x07};
    SDOR Decel = {0x6084, 0x00, 0x07};
    if(slave_id_exist("vitesse"))
    {
        //lecture des accelerations
        if (!cantools_read_sdo(slave_get_node_with_id("vitesse"),Accel,&accel_T)) {
            errgen_set(ERR_ROT_GET_ACCEL);
            return;
        }
        if (!cantools_read_sdo(slave_get_node_with_id("vitesse"),Decel,&decel_T)) {
            errgen_set(ERR_ROT_GET_DECEL);
            return;
        }
        Acceleration_V = accel_T;
        Deceleration_V = decel_T;
    }
    else {
        accel_T = 64000;
        decel_T = 64000;
        Acceleration_V = accel_T;
        Deceleration_V = decel_T;
    }
    //calcul des acceleration rotation correspondante
    if(!laser_simu){
        if(laser_asserv_CalcRotAccel(&ml, &sl, &accel_T, &accel_R)){
            errgen_set(ERR_ROT_CALC_ACCEL);
            return;
        }
        if(laser_asserv_CalcRotAccel(&ml, &sl, &decel_T, &decel_R)){
            errgen_set(ERR_ROT_CALC_ACCEL);
            return;
        }
    } else {
        if(laser_asserv_CalcRotAccel(&lsim, NULL, &accel_T, &accel_R)){
            errgen_set(ERR_ROT_CALC_ACCEL);
            return;
        }
        if(laser_asserv_CalcRotAccel(&lsim, NULL, &decel_T, &decel_R)){
            errgen_set(ERR_ROT_CALC_ACCEL);
            return;
        }

    }
    //ecriture des accelerations
    if (!cantools_write_sdo(slave_get_node_with_id("rotation"), Accel, &accel_R)){
        errgen_set(ERR_ROT_WRITE_ACCEL);
        return;
    }
    if (!cantools_write_sdo(slave_get_node_with_id("rotation"), Decel, &decel_R)){
        errgen_set(ERR_ROT_WRITE_ACCEL);
        return;
    }
    ConsigneAcceleration_MotRot = accel_R;
    ConsigneDeceleration_MotRot = decel_R;
    //printf("A1 = %u, A2 = %u, D1 = %u, D2 = %u\n", accel_T, decel_T, accel_R, decel_R);

}
