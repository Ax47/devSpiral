
/* File generated by gen_cfile.py. Should not be modified. */

#ifndef SPIRALLINGMASTER_H
#define SPIRALLINGMASTER_H

#include "data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 SpirallingMaster_valueRangeTest (UNS8 typeValue, void * value);
const indextable * SpirallingMaster_scanIndexOD (UNS16 wIndex, UNS32 * errorCode, ODCallback_t **callbacks);

/* Master node data struct */
extern CO_Data SpirallingMaster_Data;
extern UNS16 StatusWord_V;		/* Mapped at index 0x2000, subindex 0x00*/
extern INTEGER8 InternalTemp_V;		/* Mapped at index 0x2001, subindex 0x00*/
extern UNS16 Voltage_V;		/* Mapped at index 0x2002, subindex 0x00*/
extern UNS16 ErrorCode_V;		/* Mapped at index 0x2003, subindex 0x00*/
extern UNS16 StatusWord_Vaux;		/* Mapped at index 0x2004, subindex 0x00*/
extern INTEGER32 Velocity_V;		/* Mapped at index 0x2005, subindex 0x00*/
extern INTEGER32 Vel2Send_V;		/* Mapped at index 0x2006, subindex 0x00*/
extern INTEGER8 InternalTemp_Vaux;		/* Mapped at index 0x2007, subindex 0x00*/
extern UNS16 Voltage_Vaux;		/* Mapped at index 0x2008, subindex 0x00*/
extern UNS16 ErrorCode_Vaux;		/* Mapped at index 0x2009, subindex 0x00*/
extern INTEGER32 Velocity_Vaux;		/* Mapped at index 0x200A, subindex 0x00*/
extern INTEGER32 Vel2Send_Vaux;		/* Mapped at index 0x200B, subindex 0x00*/

#endif // SPIRALLINGMASTER_H