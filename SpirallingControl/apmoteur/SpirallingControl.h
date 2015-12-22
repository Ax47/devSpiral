#ifndef _SPIRALLING_CONTROL_H
#define _SPIRALLING_CONTROL_H
#include "canfestival.h"

// DONNEE PROGRAMME
#define SLAVE_NUMBER 0 // Nombre d'esclave
#define CYCLE_PERIOD 1000000//0x0000C350 // Sync tou les 50000us = 50ms
#define HB_CONS_BASE 2000//0x000001F4 // Verification du heartbeat tout les 500ms
#define HB_CONS_BASE_OFFSET 0x00000032 // Décalage de vérification 50ms
#define HB_PROD 1000//0x0128 // Hearbeat esclave envoyé tout les 200ms


void catch_signal(int sig);
void Exit();



#endif

