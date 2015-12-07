#include "keyword.h"
#include <gtk/gtk.h>
#include "gui.h"
#include "master.h"
#include "strtools.h"
#include "slave.h"
#include "motor.h"
#include "SpirallingControl.h"


void keyword_init () {

/** TITRE DES WINDOWS **/
    gtk_window_set_title(gui_get_window("mainWindow"), APPLI_TITLE);

/** HEADER BUTTONS **/
    gui_button_set("butInit",INITIALIZATION,"gtk-apply");
    gui_button_set("butStop",ARRET,"gtk-close");
    gui_button_set("butParams",CONFIGURE,"gtk-preferences");

/** BOX STATUTS **/
    // Titre des colonnes
    gui_label_set("labStateTitle",STATUT_TITLE);
    gui_label_set("labState",STATUT);
    gui_label_set("labMVelStatTitle", MOTOR_VEL);
    gui_label_set("labMCoupleStatTitle", MOTOR_COUPLE);
    gui_label_set("labMRotStatTitle", MOTOR_ROT);
    gui_label_set("labLaserGStatTitle", LASERG);
    gui_label_set("labLaserDStatTitle", LASERD);
    gui_label_set("labStateError", STATE_ERROR );
    gui_label_set("labPower", POWER_STATE);
    gui_label_set("labPowerError", POWER_STATE_ERROR);
    gui_label_set("labVolt", VOLTAGE);
    gui_label_set("labTemp", TEMPERATURE);

    // Valeur par défaut
    gui_label_set_objs(DEFAULT,"labMVelState","labMCoupleState","labMRotState","labLaserGState","labLaserDState",
                               "labMVelStateError","labMCoupleStateError","labMRotStateError",
                               "labMVelPower","labMCouplePower","labMRotPower",
                               "labMVelPowerError","labMCouplePowerError","labMRotPowerError",
                               "labMVelVolt","labMCoupleVolt","labMRotVolt",
                               "labMvelTemp","labMvelTemp","labMvelTemp"
                               ,NULL);

    gui_image_set("imgMVelRun","media-playback-pause",0);
    gui_image_set("imgMCoupleRun","media-playback-pause",0);
    gui_image_set("imgMRotRun","media-playback-pause",0);
//media-playback-start
/** BOX TRANSLATION **/
    // Labels boutons
    gui_label_set("labBoxTransTitle",BOX_TRANS_TITLE);
    gui_label_set("labTransDirection",DIRECTION);
    gui_label_set("labSetVel",VELOCITY_SET);
    gui_label_set("labVel",VELOCITY);
    gui_label_set("labVelStop",HALT);
    gui_label_set("labMVel",DEFAULT);

    gui_button_set("radForward",FORWARD,NULL);
    gui_button_set("butVelUp",NULL,"gtk-add");
    gui_button_set("butVelDown",NULL,"gtk-remove");

    gtk_switch_set_active(gui_get_switch("butVelStart"),FALSE);

/** BOX STATUT **/
    gui_label_set("labBoxStatTitle",ETAT);
    gui_label_set("stateBarTitle",ETAT);


    gtk_label_set_ellipsize (gui_get_label("stateBarContent"),PANGO_ELLIPSIZE_START);

/** BOX PARAMS **/
    gui_label_set("labParamTitle",PARAMETERS);
    gui_label_set("labParamMVelTitle",MOTOR_VEL);
    gui_label_set("labParamMCoupleTitle",MOTOR_COUPLE);
    gui_label_set("labParamMVelAcc",ACCELERATION);
    gui_label_set("labParamMVelDcc",DECELERATION);
    gui_label_set("labParamMVelDccqs", DECELERATION_QS);
    gui_label_set("labParamMVelVelinc", VELOCITY_INC);
    gui_label_set("labParamMCoupleCouple",COUPLE);
    gui_label_set("labParamMCoupleSlope",COUPLE_SLOPE);

    gui_button_set("butParamReturn",RETURN,"gtk-close");
    gui_button_set("butParamSave",SAVE,"gtk-save");

/** BOX YES NO **/
    gui_label_set("labDialYesNoContent",SAVE_CONFIG);
    gui_image_set("imgDialYesNo","gtk-info",3);
    gui_button_set("butDialNo",NO, "gtk-no");
    gui_button_set("butDialYes",YES, "gtk-yes");
    gui_widget2hide("windowDialYesNo",NULL);
}
static void keyword_maj_vitesse() {
    int j,i = 0;
    gui_label_set("labMVelPowerError",motor_get_error_title(ErrorCode_V));
    INTEGER16 data = Voltage_V/10;
    gui_label_set("labMVelVolt",strtools_gnum2str(&data,0x03));
    gui_label_set("labMVelTemp",strtools_gnum2str(&InternalTemp_V,0x02));
    gui_label_set("labMVel",strtools_gnum2str(&Velocity_V,0x04));
}
static void keyword_maj_vitesse_aux() {
    int j,i = 0;
    gui_label_set("labMVelauxPowerError",motor_get_error_title(ErrorCode_Vaux));
    INTEGER16 data = Voltage_Vaux/10;
    gui_label_set("labMVelauxVolt",strtools_gnum2str(&data,0x03));
    gui_label_set("labMVelauxTemp",strtools_gnum2str(&InternalTemp_Vaux,0x02));
    gui_label_set("labMVelaux",strtools_gnum2str(&Velocity_Vaux,0x04));
}
static void keyword_maj_rotation(){
}
static void keyword_maj_couple(){
//    gui_label_set("labMCouplePowerError",motor_get_error_title(ErrorCode_C));
//    INTEGER16 data = Voltage_C/10;
//    gui_label_set("labMCoupleVolt",strtools_gnum2str(&data,0x03));
//    gui_label_set("labMCoupleTemp",strtools_gnum2str(&InternalTemp_C,0x02));
//    gui_label_set("labMCouple",strtools_gnum2str(&Velocity_C,0x04));
}
void keyword_maj() {
    int i =0;
    for (i=0; i<SLAVE_NUMBER; i++) {
        char* slave_id = slave_get_id_with_index(i);
        char* slave_gui_ref;
        if (slave_id == "vitesse") {
            slave_gui_ref = "Vel";
            keyword_maj_vitesse();
        } else if (slave_id == "vitesse_aux") {
            slave_gui_ref = "Velaux";
            keyword_maj_vitesse_aux();
        } else if (slave_id == "couple") {
            slave_gui_ref = "Couple";
            keyword_maj_couple();
        } else if (slave_id == "rotation") {
            slave_gui_ref = "Rot";
            keyword_maj_rotation();
        }
        gui_label_set(strtools_concat("labM",slave_gui_ref,"State",NULL),slave_get_state_title(slave_get_state_with_id(slave_id)));
        gui_label_set(strtools_concat("labM",slave_gui_ref,"StateError",NULL),slave_get_state_error_title(slave_get_state_error_with_id(slave_id)));
        gui_label_set(strtools_concat("labM",slave_gui_ref,"Power",NULL),motor_get_state_title(motor_get_state(slave_get_powerstate_with_id(slave_id))));
        gui_image_set(strtools_concat("imgM",slave_gui_ref,"State",NULL),slave_get_state_img(slave_get_state_with_id(slave_id)),2);
        int i = motor_target(slave_get_powerstate_with_id(slave_id));
        if (i == 2) gui_image_set(strtools_concat("imgM",slave_gui_ref,"Run",NULL),"gtk-media-pause",0);
        else if (i == 1) gui_image_set(strtools_concat("imgM",slave_gui_ref,"Run",NULL),"media-playback-start",0);
        else gui_image_set(strtools_concat("imgM",slave_gui_ref,"Run",NULL),"gtk-delete",0);
    }
    // Vérification du switch translation
    if (gtk_switch_get_active(gui_get_switch("butVelStart")) == TRUE) {
        if (slave_id_exist("vitesse") && slave_id_exist("vitesse_aux")) {
            if (motor_get_state(slave_get_powerstate_with_id("vitesse")) == SON || motor_get_state(slave_get_powerstate_with_id("vitesse_aux")) == SON)
                gtk_switch_set_active(gui_get_switch("butVelStart"),FALSE);
        } else if(slave_id_exist("vitesse")) {
            if (motor_get_state(slave_get_powerstate_with_id("vitesse")) == SON)
                gtk_switch_set_active(gui_get_switch("butVelStart"),FALSE);
        } else if(slave_id_exist("vitesse_aux")) {
            if (motor_get_state(slave_get_powerstate_with_id("vitesse_aux")) == SON)
                gtk_switch_set_active(gui_get_switch("butVelStart"),FALSE);
        }
    }
    if (gtk_switch_get_active(gui_get_switch("butVelStart")) == FALSE) {
        if (slave_id_exist("vitesse") && slave_id_exist("vitesse_aux")) {
            if (motor_get_state(slave_get_powerstate_with_id("vitesse")) == OENABLED || motor_get_state(slave_get_powerstate_with_id("vitesse_aux")) == OENABLED)
                gtk_switch_set_active(gui_get_switch("butVelStart"),TRUE);
        } else if(slave_id_exist("vitesse")) {
            if (motor_get_state(slave_get_powerstate_with_id("vitesse")) == OENABLED)
                gtk_switch_set_active(gui_get_switch("butVelStart"),TRUE);
        } else if(slave_id_exist("vitesse_aux")) {
            if (motor_get_state(slave_get_powerstate_with_id("vitesse_aux")) == OENABLED)
                gtk_switch_set_active(gui_get_switch("butVelStart"),TRUE);
        }
    }

}


