#ifndef _SPIRALLING_CONTROL_H
#define _SPIRALLING_CONTROL_H
#include "canfestival.h"

#include "laser.h"

// DONNEE PROGRAMME
#define SLAVE_NUMBER 2 // Nombre d'esclave
#define CYCLE_PERIOD 0x0000C350 // Sync tou les 50000us = 50ms
#define HB_CONS_BASE 0x000001F4 // Verification du heartbeat tout les 500ms
#define HB_CONS_BASE_OFFSET 0x00000032 // Décalage de vérification 50ms
#define HB_PROD 0x0064 // Hearbeat esclave envoyé tout les 100ms


void catch_signal(int sig);
void Exit();

extern laser ml, sl;

#endif

