#include "laser.h"
#include "TestMaster.h"
#include "canfestival.h"
//#include "laser_global.h"
//#include "struct_laser.h"

//params
#define DELTA_VITESSE_CORRECTION 51200 //steps/s
#define INCERTITUDE_POSITION 3 //en mm (incertitude mesure laser)
#define MOTROT_POS_INCERTITUDE 19200 // 1mm erreur laser pour p=4m/tour equivaut à 2.5x10^-4 tour machine soit 1/8 tour moteur(r=500) soit 6400 step
#define ERR_VITESSE 30
#define MAX_MOTROT_VELOCITY 500000

//status mvt_check
#define MOVEMENT_OK 0x0100
#define TYPE_OF_MVT_FAILURE 0x0200
#define FIND_CONSIGNE_FAILURE 0x0400
#define ASSERV_FAILURE 0x0800
#define MAX_ROT_VEL_REACHED 0x1000
#define USER_POS_TOL_ERROR 0x2000

//type of mvt
#define VITESSE_CONSTANTE 1
#define PHASE_ACCEL 2
#define PHASE_DECEL 3
#define QUICKSTOP 4
#define STOPPED 5

//bit of status word motrot
#define TARGET_REACHED 0x0400
//controlword
#define HALT_BIT_CTRL_WD 0x0100

//facteurs
#define STEPS_PER_REV 51200
#define REDUCTION_ROTATION 500
#define REDUCTION_TRANS 75
#define REL_TRSL_ROT 1000  //roue de diametre tel que pi*D=1000mm (1000mm/min ## (1000=REL_TRSL_ROT)mm/tour *1 tour/min)
#define PIPE_DIAM 3260 //en dmm

//defs
//movement phases

struct Laser_x_thread
{
  unsigned long time;
  INTEGER32 cv;//consigne vitesse steps/s
  INTEGER32 dv; //dernière vitesse steps/s
}; 

extern struct Helix_User_Data HelixUserData;
extern unsigned int PositionOffset;// en 10eme de mm

extern unsigned long Pipe_Length;
extern unsigned long Start_Distance;
extern INTEGER32 MotRot_Ref_Position;
extern unsigned long User_Pos_Tolerance;//en dixieme de mm

unsigned int laser_asserv_Verify_Movement(laser * ml, laser * sl, unsigned long * next_Sync_call_Nr, unsigned long * SyncTimeInterval/*us*/);
unsigned int laser_asserv_Follow_Consigne(laser * ml, laser * sl);

unsigned int laser_asserv_User_Velocity_Change(laser * ml, laser * sl, INTEGER32 v_cons_mt);

//donner une valeur aux variables globales (à définir par l'utilisateur)
unsigned int laser_asserv_GetPipeLength(laser * ml, laser * sl);
unsigned int laser_asserv_GetStartPosition(laser * ml, laser * sl);

int laser_asserv_FindAccel_Relation_Constant(unsigned long * d, double * K);//K tq arot = K*atrans a en step/s^2
