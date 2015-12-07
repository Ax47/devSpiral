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
    Vel2Send_Vaux = Vel2Send_V;
}
void on_butVelDown_clicked(GtkWidget* pEntry) {
    if (Vel2Send_V - vel_inc_V > 0)
        Vel2Send_V -= vel_inc_V;
    else
        Vel2Send_V = 0;
    Vel2Send_Vaux = Vel2Send_V;
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
