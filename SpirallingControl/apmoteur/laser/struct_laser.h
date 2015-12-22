#include <pthread.h> //les threads
#define ORDRE_APPROX 10

struct laser_data
{
  unsigned long mes;//d n dixieme de mm
  long int vitesse;
  long int vitesse_1s;
  unsigned long t;//tabs en micro s
};

struct laser_struct
{
  pthread_t recv_thread;
  pthread_mutex_t mutex;
  int isSimu;
  int fd;
  char portName[255];
  int portNumber;
  int running;
  int ready_for_analyse;
  struct laser_data laser_dat[ORDRE_APPROX];
};

typedef struct laser_struct laser;

struct Helix_User_Data
{
  unsigned long * d;//distance en mm
  long int * p;//pas hélice associé en mm/tour signe indique sens
  unsigned long dmax;
  unsigned long NbrOfEntries;
};
