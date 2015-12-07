#ifndef _LSSTOOLS_H
#define _LSSTOOLS_H
#include "data.h"

int lsstools_loop(UNS8 nodeID, int error);
void lsstools_config(UNS8 nodeID, UNS8 command,void* dat2send1, void* dat2send2,int checkactive);
#endif
