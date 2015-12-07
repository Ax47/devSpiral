#include "stdlib.h"
#include "stdio.h"
#include "math.h"//racine
#include "unistd.h"
#include "time.h"
#include "pthread.h"
#include "laser_asserv.h"


#define STAND_ALONE_TEST 0
#if STAND_ALONE_TEST
struct Helix_User_Data HelixUserData;//simule la variable globale utilisateur
INTEGER32 MotRot_Ref_Position = 0;
#endif
unsigned long Start_Distance;
unsigned long Pipe_Length;
unsigned int User_Tolerance = 200;//en dixieme de mm.

unsigned int laser_asserv_GetPipeLength(laser * ml, laser * sl)
{
  unsigned int status_laser = 0;
  struct laser_data d;
  if((status_laser = Laser_GetData(ml, sl, &d))==FATAL_ERROR)
    return FATAL_ERROR;

  Pipe_Length = d.mes;
  return 0;
}

unsigned int laser_asserv_GetStartPosition(laser * ml, laser * sl)
{
  unsigned int status_laser = 0;
  struct laser_data d;
  if((status_laser = Laser_GetData(ml, sl, &d))==FATAL_ERROR)
    return FATAL_ERROR;

  Start_Distance = d.mes;
  return 0;
}

int Recup_NumInterval(unsigned long * d, long int * res)//d en ddixieme de mm
{
  long int i;
  long int NumInterval = 0;
  long int l = 0;
 //verifier que la distance n'est pas supérieures au max possible
  if(HelixUserData.dmax < (double)(*d)/10)
    {
      printf("Distance measured is higher than max user distance\n");
      return -1;
    }

  //trouver l'intervalle de distance dans lequel on se trouve
  for (i=0;i<HelixUserData.NbrOfEntries; i++)
    {
      l += HelixUserData.d[i];
      //printf("i = %ld\n", i);
      //printf("l = %lu\n", l);
      //printf("d/10 = %lf\n", (double)(*d)/10);
      if( (double)(*d)/10 <= l )
	{
	  NumInterval = i;
	  break;
	}
	
    }   

  *res = NumInterval;
  //printf("NumInterval = %ld, l = %lu\n", NumInterval, l);
  return 0;
}

int FindConsigne_PosRot(unsigned long * d/*distance mesurée dmm*/, long int * Consigne)//res en steps
{
  long int i;
  long int NumInterval = 0;
  long int l = 0;
  long int PosRot = MotRot_Ref_Position;   
  
  if(Recup_NumInterval(d, &NumInterval))
    return -1;

  l = 0;
  //s'il n'y a qu'un interval
  if(NumInterval == 0 && HelixUserData.p[0]>0)
     {
       PosRot += (long int)((double)((*d))*REDUCTION_ROTATION/((double)HelixUserData.p[NumInterval]*10/STEPS_PER_REV));   
       *Consigne = PosRot;
       return 0;
     }
  else if (NumInterval == 0 && HelixUserData.p[0] == 0)
    {
      *Consigne = PosRot;
      return 0;
    }
  //si plus d'un interval trouver la pos départ consigne de notre intervalle
  for (i=0; i<NumInterval; i++)
    {
      if(HelixUserData.p[i] != 0)
	{
	  l += HelixUserData.d[i];	    
	  PosRot += (long int)((double)(HelixUserData.d[i])*REDUCTION_ROTATION/((double)HelixUserData.p[i]/STEPS_PER_REV));
	  //printf("PosRot = %lu\n", PosRot);
	}
      else if (HelixUserData.p[i] == 0)
	l += HelixUserData.d[i];
      else
	{
	  printf("Incorrect User Entries Helix data\n");
	  return -1;
	}	
    }
  
  if(HelixUserData.p[NumInterval]!=0)
    PosRot += (long int)((double)(((double)(*d)/10-(double)l)*REDUCTION_ROTATION)/((double)HelixUserData.p[NumInterval]/STEPS_PER_REV));

       
  //printf("(*d)/10-l =  %lu, l = %lu", ((*d)/10-l), l);
  *Consigne = PosRot;

  if(labs(PosRot)>0x7FFFFFFF) {
    printf("FindConsigne PosRot ERROR: Calculated Position is too large for an int32\n");    
    return 1;
  }  

  return 0;
}

int FindConsigne_VitRot(unsigned long * d/*distance mesuree dmm*/, INTEGER32 * v_mtrans, INTEGER32 * res)
{

  long int NumInterval;
  long int p;

  if(Recup_NumInterval(d, &NumInterval))
    return -1;
  //printf("d = %lu, Numinterval = %ld\n", *d, NumInterval);
  p = HelixUserData.p[NumInterval]; //en mm/tour
  //printf("p = %ld\n", p);
  if(p == 0)    
    *res = 0;
  else
    {
      *res = (INTEGER32)((*v_mtrans) * (double)REL_TRSL_ROT/p * (double)REDUCTION_ROTATION/REDUCTION_TRANS);     
    }
  
  //printf("res = %d, p = %ld, v_mtrans = %f\n", *res, p, (float)REL_TRSL_ROT/p * (float)REDUCTION_ROTATION/REDUCTION_TRANS);
  return 0;
}

//K tel que arot = K*atrans
int laser_asserv_FindAccel_Relation_Constant(unsigned long * d, double * K)
{
  long int p = 0;
  long int NumInterval = 0;

  if(Recup_NumInterval(d, &NumInterval))
    return -1;

  p = HelixUserData.p[NumInterval];

  if(p == 0)
    *K = 0.0;
  else
    {
      *K = (double)REL_TRSL_ROT/p * (double)REDUCTION_ROTATION/REDUCTION_TRANS;
    }
  return 0;
}

//v_cons_mrot en dixieme de mm/s
unsigned int FindTypeOfMvt(UNS16 ctrl_mot, UNS16 status_mot, INTEGER32 v_cons_m, INTEGER32 v_capt_m)
{
  if((status_mot & TARGET_REACHED) && (ctrl_mot & HALT_BIT_CTRL_WD) ) 
    return STOPPED;      
  else if (!(status_mot & TARGET_REACHED) && (ctrl_mot & HALT_BIT_CTRL_WD))
    return QUICKSTOP;
  else
    {
      //printf("FINDTYPEOFMVT v_capt = %d, v_cons_m = %d, labs(v1-v2) = %d", v_capt_m, v_cons_m, labs(v_capt_m-v_cons_m));
      if((labs(v_capt_m)>labs(v_cons_m)+ERR_VITESSE) & ((long long int)v_capt_m*v_cons_m >= 0))	
	return PHASE_DECEL;	
      else if((labs(v_capt_m)<labs(v_cons_m)-ERR_VITESSE) & ((long long int)v_capt_m*v_cons_m >= 0))	
	return PHASE_ACCEL;	
      else if (labs(v_capt_m - v_cons_m) <= ERR_VITESSE)	
	return VITESSE_CONSTANTE;	
      else
	{	
	  printf("ERROR FindTypeOfMvt\n");	
	  return TYPE_OF_MVT_FAILURE;
	}
    }   
}

void ApplyVitesseRot(INTEGER32 * v)
{
  //printf("In applyVitesseRot\n");
  if(ConsigneVitesse_MotRot!=(*v))
    ConsigneVitesse_MotRot = (*v);
  
  //a faire: rajouter un mot couple
  PDOEventTimerAlarm(&TestMaster_Data,0x01);
}
void ApplyVitesseTrsl(INTEGER32 * v)
{
  if(ConsigneVitesse_T!=(*v))
    ConsigneVitesse_T = (*v);

  //a faire: rajouter un mot couple
  //PDOEventTimerAlarm(&TestMaster_Data,0x01);
}

void * Thread_Appli_Vitesse(void * vc)
{
  struct Laser_x_thread v = *(struct Laser_x_thread*)vc;

  printf("VITESSE APPLIQUEE -> %d\n",v.cv );
  ApplyVitesseRot(&(v.cv));  
  
  usleep(v.time);

  printf("VITESSE APPLIQUEE -> %d\n",v.dv );
  ApplyVitesseRot(&(v.dv));    
  
  pthread_exit(NULL);
}
//cp: consigne position en step
//p_capt_mrot: position motrot capturee en step
//v_old_mrot: vitesse motrot en step/s
//vc: sortie: struct d'asservissement (voir .h)
int Find_Asserv_Params( long int * cp, INTEGER32 * p_capt_mrot, INTEGER32 * v_old_mrot, struct Laser_x_thread * vc)
{
  double a  = (double)ConsigneAcceleration_MotRot;//tjr >0
  double DeltaS = (double)labs(*cp - *p_capt_mrot);
  double t1 = (double)DELTA_VITESSE_CORRECTION/a;  
  double res = 0;
  
  vc->dv = *v_old_mrot;

  if((res = (DeltaS-a*t1*t1)/DELTA_VITESSE_CORRECTION*1000000)<=0)    
    vc->time = (unsigned long)(2*sqrt(DeltaS/a));    
  else    
    vc->time = (unsigned long)((DeltaS-a*t1*t1)/DELTA_VITESSE_CORRECTION*1000000+2*t1);//en micro s            

  printf("DeltaS = %lf, a = %lf, t1 = %lf, res = %lf\n", DeltaS, a, t1, res);

  if(*cp - *p_capt_mrot > MOTROT_POS_INCERTITUDE)
    vc->cv = *v_old_mrot+DELTA_VITESSE_CORRECTION;
  else if (*cp - *p_capt_mrot < -MOTROT_POS_INCERTITUDE)
    vc->cv = *v_old_mrot-DELTA_VITESSE_CORRECTION;
  else
    {
      printf("Captured velocity is equal to find Asserv_Param output velocity\n");
      return -1;
    }

  
  return 0;
}

int Find_Asserv_Params2( long int * cp, INTEGER32 * p_capt_mrot, INTEGER32 * v_old_mrot, struct Laser_x_thread * vc)
{
  return 0;
}

unsigned int CheckAndCorrect_VelocityRelation(struct laser_data * d, INTEGER32 * v_cons_mt, INTEGER32 * v_cons_mrot)
{
  INTEGER32 v_cons_calc_mrot;
  unsigned int res = 0;

  if(FindConsigne_VitRot(&(d->mes), v_cons_mt, &v_cons_calc_mrot))
    return FIND_CONSIGNE_FAILURE;

  if(labs(v_cons_calc_mrot) > MAX_MOTROT_VELOCITY)
    {
      printf("ERROR: Maximum velocity rotation reached\n");
      v_cons_calc_mrot = (v_cons_calc_mrot/labs(v_cons_calc_mrot))*MAX_MOTROT_VELOCITY;
      res |= MAX_ROT_VEL_REACHED;

    }

  printf("v_cons_mrot %d, v_cons_calc_mrot %d\n", *v_cons_mrot, v_cons_calc_mrot);

  if((*v_cons_mrot) != v_cons_calc_mrot)
    {    
      ApplyVitesseRot(&v_cons_calc_mrot);
      *v_cons_mrot = v_cons_calc_mrot;
    }
    

  return res;
}

//applique la consigne en même temps. Change la vitesse du(des) moteur rot en fonction de la consigne vitesse translation.
unsigned int laser_asserv_User_Velocity_Change(laser * ml, laser * sl, INTEGER32 v_cons_mt)
{
  struct laser_data d;
  INTEGER32 v_cons_mrot;
  unsigned int res = 0;
  if((res = Laser_GetData(ml, sl, &d))==FATAL_ERROR)
    return res;

  if((res |= FindConsigne_VitRot(&(d.mes), &v_cons_mt, &v_cons_mrot)) & FIND_CONSIGNE_FAILURE)
    return res;
  
  ApplyVitesseRot(&v_cons_mrot);
  ApplyVitesseTrsl(&v_cons_mt);
  //printf("\nUSER VELOCITY CHANGED TO %d\n\n", v_cons_mt);
  return 0;
}

//d: distance laser mesuree
//d: consigne de pos en step calcule
//pcaptrot: position moteur rot en en step
//user_tol: tolerance d'écart en dixieme de mm.
int laser_asserv_estim_error_dmm(INTEGER32 posErr, unsigned int * res)
{
  long double c=0;

  c = M_PI*PIPE_DIAM*abs(posErr)/STEPS_PER_REV/REDUCTION_ROTATION;//evlt multiplier par cos(alpha)
  *res = (unsigned int)c;
    
  return 0;
}



//calcule et donne bêtement les consigne utilisateur au moteur
unsigned int laser_asserv_Follow_Consigne(laser * ml, laser * sl)
{
  struct laser_data d;
  INTEGER32 v_cons_mt, v_cons_mrot;
  unsigned int res = 0;

  v_cons_mt = ConsigneVitesse_T;
  v_cons_mrot = ConsigneVitesse_MotRot;

  if((res |= Laser_GetData(ml, sl, &d))==FATAL_ERROR)
    return FATAL_ERROR;

  //printf("RESULTAT MESURE LASER : d = %lu, t = %lu, v = %ld\n", d.mes, d.t, d.vitesse);

  if((res |= CheckAndCorrect_VelocityRelation(&d, &v_cons_mt, &v_cons_mrot)) & FIND_CONSIGNE_FAILURE)
    {
      printf("Find Consigne Failure\n");
      return res;
    }

  return res;
}


unsigned int laser_asserv_Verify_Movement(laser * ml, laser * sl, unsigned long * next_Sync_call_Nr, unsigned long * SyncTimeInterval/*us*/)
//variables globale: data moteur rotation 
//"CaptureVitesse_MotRot","CapturePosition_MotRot", "ConsigneVitesse_MotRot", "Status_word_MotRot",  "Control_word_MotRot
//"CaptureVitesse_T, CapturePosition_T, ConsigneVitesse_T, ConsignePosition_T
//data moteur translation: "Status_word_MotTrans"
{
  struct laser_data d;
  INTEGER32 v_capt_mrot, p_capt_mrot, v_cons_mrot;
  INTEGER32 v_cons_mt;
  INTEGER32 temp;
  UNS16 stat_motrot, ctrl_motrot;
  unsigned int TypeOfMvt = 0;
  long int cp;//consigne position motrot en step
  unsigned int res = 0;
  unsigned int err = 0;

  printf("Laser_Verify_Movement=>");

  //si aucun evenement ou une erreur on rappelle la fonction au prochain sync.
  *next_Sync_call_Nr += 1;

  v_capt_mrot = CaptureVitesse_MotRot;
  p_capt_mrot = CapturePosition_MotRot;
  v_cons_mrot = ConsigneVitesse_MotRot;
  stat_motrot = StatusWord_MotRot;
  ctrl_motrot = ControlWord_MotRot;
  v_cons_mt = ConsigneVitesse_T;

  if((res |= Laser_GetData(ml, sl, &d))==FATAL_ERROR)
    return FATAL_ERROR;

  printf("RESULTAT MESURE LASER : d = %lu, t = %lu, v = %ld\n", d.mes, d.t, d.vitesse);

  //TODOFaire les vérif : moins de la pos initiale ou plus que pos finale->déclencher arrêt (Utilise d, Start_Distance, Pipe_Length).

  temp = v_cons_mrot;
  if((res |= CheckAndCorrect_VelocityRelation(&d, &v_cons_mt, &v_cons_mrot)) & FIND_CONSIGNE_FAILURE)
    return res;
  //s'il y a eu correction sortir et attendre le prochain sync
  if(temp!=v_cons_mrot)
    {
      printf("CheckAndCorrect_VelRel v_cons_mrot = %d\n", v_cons_mrot);
      return res;
    }

  if(FindConsigne_PosRot(&(d.mes), &cp))
    return (res | FIND_CONSIGNE_FAILURE);

  printf("CONSIGNE POSROT = %ld, POSITION MESUREE = %d\n", cp, p_capt_mrot);
  printf("cp - p_capt_mrot = %ld\n", cp - p_capt_mrot);


  if((TypeOfMvt = FindTypeOfMvt(ctrl_motrot, stat_motrot, v_cons_mrot, v_capt_mrot)) & TYPE_OF_MVT_FAILURE)
    return (res | TYPE_OF_MVT_FAILURE);  

  printf("TYPE OF MOVEMENT = %u\n", TypeOfMvt);

  if(labs(cp-p_capt_mrot)<MOTROT_POS_INCERTITUDE)
    {
      printf("POSITION OK\n");
      return (res | MOVEMENT_OK);
    }
  else
    {
      switch(TypeOfMvt)
	{
	case VITESSE_CONSTANTE: 
	  {
	    pthread_t thread_v_const;
	    struct Laser_x_thread vc;
	    printf("VITESSE CONSTANTE + erreur pos\n");	    
	    
	    Laser_estim_error_dmm((INTEGER32)(cp-p_capt_mrot), &err);
	     
	    if(err < User_Tolerance)
	      {
		if(Find_Asserv_Params(&cp, &p_capt_mrot, &v_cons_mrot/*entrees*/, &vc/*sortie*/))	      
		  {
		    printf("ERROR in Find Asserv_Params\n");
		    return (res | ASSERV_FAILURE);
		  }
		else
		  {
		    printf("ASSERV PARAMS time = %lu, cv = %d, dv = %d\n", vc.time, vc.cv, vc.dv);
		    *next_Sync_call_Nr += vc.time/(*SyncTimeInterval);
		    if(pthread_create(&thread_v_const, NULL, Thread_Appli_Vitesse, &vc))
		      {
			printf("ERROR in correction thread create. Laser_verify_mvt\n");
			return (res | ASSERV_FAILURE);
		      }
		  }
	      }
	    else
	      {
		printf("USER POSITION TOLERANCE ERROR. ERROR = %u\n", err);
		res |= USER_POS_TOL_ERROR;
		return res;
	      }
		  
	    break;
	  }     
	case PHASE_ACCEL:
	case PHASE_DECEL:
	case QUICKSTOP:
	case STOPPED:
	  printf("PHASE ACCEL/DECEl/STOPPED OR QUICKSTOP\n");
	default:
	  //rien
	  break;
	}
    }

  return res;
}


#if STAND_ALONE_TEST
int main(void)
{
  struct laser_data ld;
  unsigned long d[3];
  long int p[3];
  long int res = 0;
  d[0] = 1000;
  d[1] = 30000;
  d[2] = 1000;
  p[0] = 0;
  p[1] = 4000;
  p[2] = 0;//mm
  
  HelixUserData.dmax = d[0]+d[1]+d[2];
  HelixUserData.NbrOfEntries = 3;
  HelixUserData.d = d;
  HelixUserData.p = p;
  
  printf("h.d = %lu %lu %lu, h.p = %ld %ld %ld\n", HelixUserData.d[0], HelixUserData.d[1], HelixUserData.d[2], 
	 HelixUserData.p[0], HelixUserData.p[1], HelixUserData.p[2]);
  
  ld.mes = 310540;//mm/10!
  ld.vitesse = 0;
  ld.t = 120000;
  
  if(FindConsigne_PosRot(&(ld.mes), &res))
    printf("ERROR in FindConsigne PosRot\n");
  else
    printf("Consigne Pos Rot = %ld\n", res);
  
  //User_Velocity_Change
  laser l;
  INTEGER32 res2;
  INTEGER32 v_trans = 67200;
  int count=0;
  
  if(FindConsigne_VitRot(&ld.mes, &v_trans, &res2)<0)
    printf("ERROR in FindConsigne_VitRot\n");
  else
    printf("Vit trans %d -> Vit Rot = %d\n", v_trans, res2);

  res2 = 0;
  CheckAndCorrect_VelocityRelation(&ld ,&v_trans, &res2);  
  printf("v_trans = %d, vrotcons calc %d\n", v_trans, res2);
  /*
    Laser_Init_Simu(&l);
    Laser_Start_Simu(&l);
    sleep(5);
  while (count<200)
    {
      User_Velocity_Change(&l, NULL, 67200);     
      pthread_mutex_lock(&(l.mutex));
      printf("\nRESULTAT MESURE LASER : d = %lu, t = %lu, v = %ld\n", l.laser_dat[0].mes, l.laser_dat[0].t, l.laser_dat[0].vitesse);
      pthread_mutex_unlock(&(l.mutex));
      printf("Consigne trsl = %d=>", ConsigneVitesse_T);
      printf("Consigne rot = %d\n", ConsigneVitesse_MotRot);
      count++;
      usleep(150000);
    }
  */
  
  return 0;
}
#endif
