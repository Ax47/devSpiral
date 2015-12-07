#include "od_callback.h"
#include "canfestival.h"
#include "gui.h"
#include "cantools.h"
#include "slave.h"
#include "SpirallingMaster.h"
#include "strtools.h"

UNS32 OnStatusWordVUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex) {
	slave_set_powerstate_with_id("vitesse",StatusWord_V);
}
UNS32 OnStatusWordVauxUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex) {
	slave_set_powerstate_with_id("vitesse_aux",StatusWord_Vaux);
}
UNS32 OnVelocityVUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex) {
    gui_label_set("labVel",strtools_gnum2str(&Velocity_V,0x04));
}

