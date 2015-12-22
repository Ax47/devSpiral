#include "laser.h"
#include "canfestival.h"


#include "laser_asserv_cst.h"

//struct asservissement
struct Laser_x_thread
{
  unsigned long time;
  INTEGER32 cv;//consigne vitesse steps/s
  INTEGER32 dv; //dernière vitesse steps/s
};

//Struct thread asserv
struct laser_asserv_thread
{
    pthread_t thread_Nr;
    pthread_mutex_t mutex;
};

//indicateur de fonctionnement du module
extern int run_asserv;//1=running, 0=not running

/**a fournir par l'utilisateur**/

extern laser ml, sl, lsim;

//donnée hélice utilisateur
extern struct Helix_User_Data HelixUserData;

//données ext à fournir. Sinon utilisation des valeurs par défault.
extern unsigned long Pipe_Length; //mm
extern INTEGER32 MotRot_Ref_Position; //step
extern unsigned long Start_Distance;

//décommenter pour spécifier une tolerance autre que par default
//extern unsigned long User_Pos_Tolerance;//en dixieme de mm
/**a definir avant lancement: acceleration de consigne rotation**/
unsigned int laser_asserv_CalcRotAccel(laser * ml, laser * sl, UNS32 * a_cons_mt, UNS32 * a_cons_mrot);

/**lancement et arrêt du module**/
int laser_asserv_lance(void);
int laser_asserv_lance_simu(void);
int laser_asserv_stop(void);


/*

unsigned int laser_asserv_Verify_Movement(laser * ml, laser * sl, unsigned long * next_Sync_call_Nr, unsigned long * SyncTimeInterval);
unsigned int laser_asserv_Follow_Consigne(laser * ml, laser * sl);

unsigned int laser_asserv_User_Velocity_Change(laser * ml, laser * sl, INTEGER32 v_cons_mt);

//donner une valeur aux variables globales (à définir par l'utilisateur)
unsigned int laser_asserv_GetPipeLength(laser * ml, laser * sl);
unsigned int laser_asserv_GetStartPosition(laser * ml, laser * sl);

int laser_asserv_FindAccel_Relation_Constant(unsigned long * d, double * K);//K tq arot = K*atrans a en step/s^2
*/
