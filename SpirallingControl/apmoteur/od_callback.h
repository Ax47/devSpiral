#ifndef _OD_CALLBACK_H
#define _OD_CALLBACK_H
#include "canfestival.h"
UNS32 OnStatusWordVUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 OnStatusWordVauxUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 OnStatusWordRUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 OnVelocityVUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);

#endif
