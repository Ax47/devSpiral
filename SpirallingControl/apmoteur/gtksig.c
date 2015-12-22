#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <gtk/gtk.h>
#include "canfestival.h"

#include "gtksig.h"
#include "keyword.h"
#include "master.h"
#include "gui.h"
#include "cantools.h"
#include "SpirallingControl.h"
#include "strtools.h"
#include "motor.h"
#include "errgen.h"
#include "slave.h"

#include "laser_asserv.h"
#include "laser_simulation.h"

GtkBuilder *builder;

extern INTEGER32 vel_inc_V;
extern int run_init;

void gtksig_init () {
    // SIGNALS MAIN
    g_signal_connect (gtk_builder_get_object (builder, "butStop"), "clicked", G_CALLBACK (on_butStop_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "mainWindow"), "delete-event", G_CALLBACK (on_butQuit_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butParams"), "clicked", G_CALLBACK (on_butParams_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "radForward"), "toggled", G_CALLBACK (on_radForward_toggled),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "radBackward"), "toggled", G_CALLBACK (on_radBackward_toggled),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butVelUp"), "clicked", G_CALLBACK (on_butVelUp_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butVelDown"), "clicked", G_CALLBACK (on_butVelDown_clicked),NULL);

    g_signal_connect (gtk_builder_get_object (builder, "butVelStart"), "notify", G_CALLBACK (on_butVelStart_active_notify),NULL);
    // SIGNALS DIAL
    g_signal_connect (gtk_builder_get_object (builder, "butInitDialClose"), "clicked", G_CALLBACK (on_butInitDialClose_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "windowDialInit"), "response", G_CALLBACK (on_butInitDialClose_clicked),NULL);

    //SIGNALS PARAMS
    g_signal_connect (gtk_builder_get_object (builder, "butParamReturn"), "clicked", G_CALLBACK (on_butParamReturn_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butParamSave"), "clicked", G_CALLBACK (on_butParamSave_clicked),NULL);

    // SIGNALS DIAL YES NO
    g_signal_connect (gtk_builder_get_object (builder, "butDialYes"), "clicked", G_CALLBACK (on_butDialYes_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butDialNo"), "clicked", G_CALLBACK (on_butDialNo_clicked),NULL);


    //SINGNAL LASER
    g_signal_connect (gtk_builder_get_object (builder, "butReinitLaser"), "clicked", G_CALLBACK (on_butReinitLaser_clicked), NULL);

    // SIGNALS ROTATION
    g_signal_connect (gtk_builder_get_object (builder, "but_Start_R"), "notify", G_CALLBACK (on_but_StartRot_notify), NULL);

    //SIGNAL SIMU LASER
    g_signal_connect (gtk_builder_get_object (builder, "but_LaserSimuOpt"), "clicked", G_CALLBACK (on_LaserSimuOpt_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "but_LaserSimu_CloseWindow"), "clicked", G_CALLBACK (on_LaserSimuClose_clicked), NULL);

    g_signal_connect (gtk_builder_get_object (builder, "but_LaserSimuStart"), "clicked", G_CALLBACK (on_LASERSTARTSIMU_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "but_LaserSimuStop"), "clicked", G_CALLBACK (on_LASERSTOPSIMU_clicked), NULL);

    g_signal_connect (gtk_builder_get_object (builder, "but_sim_inc_pos"), "clicked", G_CALLBACK (on_sim_inc_pos_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "but_sim_dec_pos"), "clicked", G_CALLBACK (on_sim_dec_pos_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "but_sim_inc_vel"), "clicked", G_CALLBACK (on_sim_inc_vel_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "but_sim_dec_vel"), "clicked", G_CALLBACK (on_sim_dec_vel_clicked), NULL);



}

// Bouton arret
void on_butStop_clicked (GtkWidget* pEntry) {
    int i = 0;
    for (i=0; i<SLAVE_NUMBER; i++) {
        if(!motor_start(slave_get_id_with_index(i),0))
            errgen_set(ERR_MOTOR_PAUSE);
    }
}

// Fermeture de la boite de dialogue
void on_butInitDialClose_clicked (GtkWidget* pEntry) {
    gtk_widget_hide(GTK_WIDGET(gtk_builder_get_object (builder, "windowDialInit")));
}

// Fermeture de la window
void on_butQuit_clicked (GtkWidget* pEntry) {
    Exit();
}


void on_butParams_clicked(GtkWidget* pEntry) {
    gtk_switch_set_active(gui_get_switch("butVelStart"),FALSE);
    if(slave_load_config())
        gui_widget2show("windowParams",NULL);
    else
        errgen_set(ERR_LOAD_SLAVE_CONFIG);
}

void on_butVelStart_active_notify(GtkWidget* pEntry) {
    if (slave_id_exist("vitesse")) {
        if (gtk_switch_get_active(gui_get_switch("butVelStart")) == TRUE && motor_get_state(slave_get_powerstate_with_id("vitesse")) == SON) {
            if (!motor_start("vitesse",1)) errgen_set(ERR_MOTOR_RUN);
            Vel2Send_V = 0;
        }
        if (gtk_switch_get_active(gui_get_switch("butVelStart")) == FALSE && motor_get_state(slave_get_powerstate_with_id("vitesse")) == OENABLED) {
            if (!motor_start("vitesse",0)) errgen_set(ERR_MOTOR_PAUSE);
            Vel2Send_V = 0;
        }
    }
    if (slave_id_exist("vitesse_aux")) {
        if (gtk_switch_get_active(gui_get_switch("butVelStart")) == TRUE && motor_get_state(slave_get_powerstate_with_id("vitesse_aux")) == SON) {
            if (!motor_start("vitesse_aux",1)) errgen_set(ERR_MOTOR_RUN);
            Vel2Send_Vaux = 0;
        }
        if (gtk_switch_get_active(gui_get_switch("butVelStart")) == FALSE && motor_get_state(slave_get_powerstate_with_id("vitesse_aux")) == OENABLED) {
            if (!motor_start("vitesse_aux",0)) errgen_set(ERR_MOTOR_PAUSE);
            Vel2Send_Vaux = 0;
        }
    }

}
void on_LaserSimuOpt_clicked(GtkWidget* pEntry)
{
    gui_widget2show("window_laser_sim",NULL);
}
void on_LaserSimuClose_clicked(GtkWidget* pEntry)
{
    gui_widget2hide("window_laser_sim", NULL);
}
void on_but_StartRot_notify(GtkWidget* pEntry)
{
    struct laser_data d;
    UNS8 pdonum;

    if (slave_id_exist("rotation")){
        if(gtk_switch_get_active(gui_get_switch("but_Start_R")) == TRUE && motor_get_state(slave_get_powerstate_with_id("rotation")) == SON){
            ConsigneVitesse_MotRot = 0;
            if(!run_asserv){
                /**Start Motor**/
                if (!motor_start("rotation", 1)){
                errgen_set(ERR_MOTOR_RUN);
                return;
                }
                ConsigneVitesse_MotRot = 0;
                /**Get Laser Start Position**/
                if(laser_asserv_GetStartPosition(&ml, &sl)) return;

                printf("START DISTANCE = %lu", Start_Distance);
                /**Get_Ref_Position MotRot**/
                SDOR RefPos_R = {0x6064, 0x00, int32};
                if (!cantools_read_sdo(slave_get_node_with_id("rotation"),RefPos_R,&MotRot_Ref_Position)) {
                    errgen_set(ERR_SLAVE_CONFIG_ROT_REFPOS);
                    return;
                }
                /**Set MotRot Acceleration**/
                motor_set_MotRot_Accel();
                /**A faire: Set MotRot Quick Stop Accel**/

                /**LANCE ASSERVISSEMENT**/
                if(laser_asserv_lance()){
                    errgen_set(ERR_LASER_ASSERV_START);
                }
            }
        }
        if(gtk_switch_get_active(gui_get_switch("but_Start_R")) == FALSE && motor_get_state(slave_get_powerstate_with_id("rotation")) == OENABLED){
            if (!motor_start("rotation", 0)) errgen_set(ERR_MOTOR_PAUSE);
            ConsigneVitesse_MotRot = 0;
            if(run_asserv){
                printf("EXIT ASSERV\n");
                /**STOPPE ASSERV**/
                if(laser_asserv_stop()){
                    errgen_set(ERR_LASER_ASSERV_STOP);
                }
            }
        }//endif switch 2
    }//end if id exist
}

void on_butReinitLaser_clicked(GtkWidget* pEntry)
{
    if(cantools_reinit_laser())
        errgen_set(ERR_LASER_REINIT);
}

void on_LASERSTARTSIMU_clicked(GtkWidget* pEntry)
{
    UNS32 accel_T, decel_T;
    UNS32 accel_R, decel_R;
    if(!laser_simu){
        //si exist asserv l'arreter
        if(run_asserv){
            if(laser_asserv_stop()){
                errgen_set(ERR_LASER_ASSERV_STOP);
                return;
            }
        }

        /**Get_Ref_Position MotRot**/
        SDOR RefPos_R = {0x6064, 0x00, int32};
        if (!cantools_read_sdo(slave_get_node_with_id("rotation"),RefPos_R,&MotRot_Ref_Position)) {
            errgen_set(ERR_SLAVE_CONFIG_ROT_REFPOS);
            return;
        }

        printf("MOTROT REF POS  = %d\n\n", MotRot_Ref_Position);

        /**Fixation artificielle Laser Start Position**/

        Start_Distance = 300000;

        /**Lancement du thread simu laser**/

        laser_simu = 1;
        if(pthread_create(&(lsim.recv_thread), NULL, laser_simulation_SimuThread_func, NULL)){
            printf("ERROR IN LASERTHREAD simu CREATE\n");
            errgen_set(ERR_LASER_SIMU_START);
            return;
        }

        /**Set MotRot Acceleration**/
        motor_set_MotRot_Accel();
        /**A faire: Set MotRot Quick Stop Accel**/


        /**Lancement du thread asserv**/
        if(laser_asserv_lance_simu()){
            errgen_set(ERR_LASER_SIMU_START);
        }
    }
}

void on_LASERSTOPSIMU_clicked(GtkWidget* pEntry)
{
    if(laser_simu && run_asserv){
        //stopper l'asservissement
        if(laser_asserv_stop()){
            errgen_set(ERR_LASER_ASSERV_STOP);
        }
        //mettre la vitesse des moteurs a 0
        ConsigneVitesse_MotRot = 0;
        cantools_ApplyVitRot(&ConsigneVitesse_MotRot);
        //stopper le thread simu laser
        laser_simu = 0;
        if(pthread_join(lsim.recv_thread, NULL)){
            printf("ERROR in LASERTHREAD simu STOP\n");
            errgen_set(ERR_LASER_SIMU_STOP);
        }
        //relancer l'asserv réel si les laser sont ok
        struct laser_data d;
        if((Laser_GetData(&ml, &sl, &d) & ERR_LASER_FATAL) != ERR_LASER_FATAL){
            if(laser_asserv_lance()){
                errgen_set(ERR_LASER_ASSERV_START);
            }
        }
    }
}
void on_sim_inc_pos_clicked(GtkWidget* pEntry)
{
    laser_simulation_inc_const_err((int)gui_str2num(gui_entry_get("entry_sim_inc_pos")));
}
void on_sim_dec_pos_clicked(GtkWidget* pEntry)
{
    laser_simulation_dec_const_err((int)gui_str2num(gui_entry_get("entry_sim_inc_pos")));
}
void on_sim_inc_vel_clicked(GtkWidget* pEntry)
{
    laser_simulation_inc_vel_err((int)gui_str2num(gui_entry_get("entry_sim_inc_vel")));
}
void on_sim_dec_vel_clicked(GtkWidget* pEntry)
{
    laser_simulation_dec_vel_err((int)gui_str2num(gui_entry_get("entry_sim_inc_vel")));
}


void on_radBackward_toggled(GtkWidget* pEntry) {
    if(gui_but_is_checked("radBackward"))
        if(!motor_forward("vitesse",0))
            errgen_set(ERR_MOTOR_FORWARD);
}
void on_radForward_toggled(GtkWidget* pEntry) {
    if(gui_but_is_checked("radForward"))
        if(!motor_forward("vitesse",1))
            errgen_set(ERR_MOTOR_FORWARD);
}

void on_butVelUp_clicked(GtkWidget* pEntry) {
    Vel2Send_V += vel_inc_V;
    //Vel2Send_V += 51200;
    Vel2Send_Vaux = Vel2Send_V;
    printf("1\n");
    //rotation pas active
    if(!run_asserv) {
        printf("2\n");
        cantools_ApplyVitTrans(&Vel2Send_V);
        return;
    }
    //rotation active
    if(!laser_simu){
        printf("3\n");
        laser_asserv_User_Velocity_Change(&ml, &sl, Vel2Send_V);
    } else if (laser_simu && (slave_id_exist("vitesse") || slave_id_exist("vitesse_aux"))){
        printf("4\n");
        laser_asserv_User_Velocity_Change(&lsim, NULL, Vel2Send_V);
    } else {
        printf("5\n");
        laser_simulation_on_ACCELERER_clicked();
    }

}
void on_butVelDown_clicked(GtkWidget* pEntry) {
    if (Vel2Send_V - vel_inc_V > 0){
        Vel2Send_V -= vel_inc_V;
        //Vel2Send_V -= 51200;
    }
    else
        Vel2Send_V = 0;
    Vel2Send_Vaux = Vel2Send_V;

   //rotation pas active
    if(!run_asserv) {
        printf("2\n");
        cantools_ApplyVitTrans(&Vel2Send_V);
        return;
    }
    //rotation active
    if(!laser_simu){
        printf("3\n");
        laser_asserv_User_Velocity_Change(&ml, &sl, Vel2Send_V);
    } else if (laser_simu && (slave_id_exist("vitesse") || slave_id_exist("vitesse_aux"))){
        printf("4\n");
        laser_asserv_User_Velocity_Change(&lsim, NULL, Vel2Send_V);
    } else {
        printf("5\n");
        laser_simulation_on_RALENTIR_clicked();
    }

}

/** BOITE DIAL PARAMS **/
void on_butParamReturn_clicked(GtkWidget* pEntry) {
    gui_widget2hide("windowParams",NULL);
}
void on_butParamSave_clicked(GtkWidget* pEntry) {
    gui_widget2show("windowDialYesNo",NULL);
}
/** BOITE DIAL YES NO **/
void on_butDialYes_clicked(GtkWidget* pEntry) {
    gui_widget2hide("windowDialYesNo","windowParams",NULL);
    if (!slave_save_config(1))
       errgen_set(ERR_SLAVE_SAVE_CONFIG);
}

void on_butDialNo_clicked(GtkWidget* pEntry) {
    gui_widget2hide("windowDialYesNo","windowParams",NULL);
    if (!slave_save_config(0))
       errgen_set(ERR_SLAVE_SAVE_CONFIG);
}
