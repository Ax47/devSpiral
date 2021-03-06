#include <stdio.h> // standard input / output functions
#include <stdlib.h>//strtoul
#include <string.h> // string function definitions

//#include<sys/select.h> //pour select
//#include<sys/types.h> //pour selectù

#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <pthread.h> //les threads
#include <unistd.h> // UNIX standard function definitions

#include "laser.h"
#include "Laser_Close_Cfile_gen.h"

#define STAND_ALONE_TEST 0
#define LASER_SIMULATION 0 //uniquement dans le cas STAND_ALONE_TEST

static long int PositionOffset = 0;
//enregistrement de la dernière mesure laser
static struct laser_data LastLaserData;//dernière donnée vérifiée

static int MasterControl = 1;//flag indiquant quel laser détient le contrôle 0 ou 1
//MasterControl: dans Laser_GetData et VerifyMeasureConsistency


unsigned long GetDate_us(void)
{
        static unsigned long InitDate=0;
        struct timespec t1;
        clock_gettime(CLOCK_MONOTONIC,&t1);
        if(InitDate == 0)
                InitDate=(t1.tv_sec * 1000000 + t1.tv_nsec / 1000);
        return (t1.tv_sec * 1000000 + t1.tv_nsec / 1000 - InitDate);
}

static int isopen[10] = {0,0,0,0,0,0,0,0,0,0};

int openPort(laser * l)
{
    int fd = -1;
    char * portName = "/dev/ttyUSB";
    int imax=10;
    int i = 0;
    char ic[imax/10+1];
    printf("Open Port:");
    while (fd == -1 && i<imax){
        if(!isopen[i]){
            sprintf(ic, "%d", i);
            //printf("ic = %s\n", ic);
            strcpy(l->portName, portName);
            //printf("portName1 = %s\n", l->portName);
            strcat(l->portName, ic);
            if((fd = open(l->portName, O_RDWR))!=-1){
                l->fd = fd;
                l->portNumber = i;
                isopen[i] = 1;
                return 0;
            }
            else{
                int err = errno;
                printf("Open portName = %s failed=>", l->portName);
                printf("ErrCode = %d\n", err);
            }
        }

        i++;
    }

    return -1;

}

//mise à 0 donnees laser
void init_laser_data(laser * l)
{
  int i;
  pthread_mutex_lock(&(l->mutex));
  l->ready_for_analyse = 0;
  l->running = 0;
  for (i=0; i<sizeof(l->laser_dat)/sizeof(struct laser_data); i++)
    {
      l->laser_dat[i].t = 0;
      l->laser_dat[i].mes = 0;
    }
  pthread_mutex_unlock(&(l->mutex));
  LastLaserData.t = 0;
  LastLaserData.mes = 0;
  LastLaserData.vitesse = 0;
  LastLaserData.vitesse_1s = 0;
}

void Laser_Init_Simu(laser * l)
{
  pthread_mutex_t m = PTHREAD_MUTEX_INITIALIZER;
  l->mutex = m;
  l->fd = 0;
  l->isSimu = 1;
  init_laser_data(l);
}

int Laser_serial_config(void)
{
    pid_t pid = fork();
    if (pid < 0) {
        printf("A fork error in Laser_serial_config has occurred.\n");
        exit(-1);
    } else {
        if (pid == 0) {
            #if STAND_ALONE_TEST
            execlp("./serial_conf.sh", "serial_conf.sh", NULL);
            #else
            execlp("./laser/serial_conf.sh","laser/serial_conf.sh",NULL);
            #endif // STAND_ALONE_TEST
            printf("laser serial config script error\n");
            return -1;
        } else {
            wait(0);
        }
    }
    return 0;
}

int Laser_Init(laser * l)
{
  pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;
  l->mutex = mut;
  l->isSimu = 0;
  init_laser_data(l);

  /** open serial ports**/
  if(openPort(l)<0)
    {
      printf("openPort ERROR\n");
      return -2;
    }
  else
    {
    /**Configuring serial port transmission type**/
      struct termios newtio;

      //printf("port %s opened. fd = %d->", l->portName, l->fd);
      //printf("Setting port %d params->", l->fd);

      if(!isatty(l->fd))
	{
	  printf("port %d is not a tty\n", l->fd);
	  return -1;
	}
      /*
      if(tcgetattr(l->fd, &oldtio) < 0)
	{
	  printf("Failed to get current config of port %d\n", l->fd);
	  return -1;
	}
      */
      bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

      //setting c_cflags
      // set baud rates
      tcflush(l->fd, TCIOFLUSH);
      if(cfsetispeed(&newtio, B19200) < 0 || cfsetospeed(&newtio, B19200) < 0)
	{
	  printf("Impossible to set baud rates of port %d\n", l->fd);
	  return -1;
	}

      //even parity
      newtio.c_cflag |= PARENB;//parity check
      newtio.c_cflag &= ~PARODD;
      newtio.c_cflag &=  ~CSTOPB;//1 stop bit

      newtio.c_cflag &= ~CSIZE;//forcing all size bits to 0
      newtio.c_cflag |= CS7;//set 7 bits

      newtio.c_cflag |= (CREAD | CLOCAL);

      newtio.c_cflag &= ~CRTSCTS;//turn off hw flow control
      //l_flags

      newtio.c_lflag = 0;// set input mode non-canonical, no echo

      //i_flags
      newtio.c_iflag |= IGNCR;//ignore CR on input
      newtio.c_iflag |= IGNPAR; // ignore bytes with parity error
      newtio.c_iflag &= ~(IXON | IXOFF | IXANY);//turn off sw flow control

      //o_flags
      newtio.c_oflag |= ONLCR;//output translate NL to CR NL termination

      //other flags
      newtio.c_cc[VMIN] = 13; //blocking read until 13 char arrives
      newtio.c_cc[VTIME] = 1; //Inter-char timer 1s read timeout

      //apply settings

      tcflush(l->fd, TCIOFLUSH);
      if(tcsetattr(l->fd, TCSANOW, &newtio)<0)
	{
	  printf("Unsable to apply given port settings\n");
	  return -1;
	}
      //printf("succeed in setting port %d params\n", l->fd);
      return 0;
    }
}


int Laser_Close(laser * l)
{
  //printf("Closing Laser %d port=>", l->fd);
  if(l->fd == -1)
  {
    return 0;
  }
  if(close(l->fd)<0)
    {
      printf("close portNumber %d error\n", l->portNumber);
      return -1;
    }
  else{
    isopen[l->portNumber] = 0;
  }
  //printf("Laser_Close isopen[%d] = %d\n", l->portNumber, isopen[l->portNumber]);
  return 0;
}


int write_port(laser * l, const char * buf)
{
  int res;
  pthread_mutex_lock(&(l->mutex));
  //printf("write port %d=>", l->fd);

  if( (res = write(l->fd, buf, sizeof(buf))) < 0)
    {
      pthread_mutex_unlock(&(l->mutex));
      printf("write failed\n");
      return -1;
    }
  else if (res != sizeof(buf))
    {
      pthread_mutex_unlock(&(l->mutex));
      printf("Write not completed\n");
      return -1;
    }
  else
    {
      tcdrain(l->fd);//attendre la fin de l'envoie.
      pthread_mutex_unlock(&(l->mutex));
      //printf("Data written to port %d: %s\n", l->fd, buf);
    }

  return res;
}


void Save_Data_Laser(laser * l, unsigned long mes, unsigned long t)
{
  int i;
  long int dx;
  double dt;
  //faire de la place
  pthread_mutex_lock(&(l->mutex));
   for (i=sizeof(l->laser_dat)/sizeof(struct laser_data)-1; i>0; i--)
    {
      l->laser_dat[i].mes = l->laser_dat[i-1].mes;
      l->laser_dat[i].t = l->laser_dat[i-1].t;
      l->laser_dat[i].vitesse = l->laser_dat[i-1].vitesse;
      l->laser_dat[i].vitesse_1s = l->laser_dat[i-1].vitesse_1s;
    }

  l->laser_dat[0].mes = mes;
  l->laser_dat[0].t = t;
  //vitesse instantannée err 30dmm/s
  dx = mes-(l->laser_dat[ORDRE_APPROX-1].mes);
  dt = CONVERT_MUS_S(t-(l->laser_dat[ORDRE_APPROX-1].t));
  l->laser_dat[0].vitesse_1s = (long int)(dx/dt);
  //vitesse 1s err 3dmm/s
  dx = mes-(l->laser_dat[1].mes);
  dt = CONVERT_MUS_S(t-(l->laser_dat[1].t));
  l->laser_dat[0].vitesse = (long int)(dx/dt);

  pthread_mutex_unlock(&(l->mutex));
  //printf("laser %d, mes = %lu, t = %lu, v = %ld, v1s = %ld\n", l->fd, mes, t, l->laser_dat[0].vitesse, l->laser_dat[0].vitesse_1s);
  return;
}

int read_port(laser * l, int recvBytes, char * buf)
{
  unsigned long time;
  unsigned long mesure;
  int res, fd;
  //select
  fd_set rfds;
  struct timeval tv;
  int retval = 0;
  //unsigned long t1, t2;
  pthread_mutex_lock(&(l->mutex));
  fd = l->fd;
  pthread_mutex_unlock(&(l->mutex));

  //printf("Read Port %d\n", l->fd);
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);
  //Timeout 500ms
  tv.tv_sec = 0;
  tv.tv_usec = 500000;

  while(retval == 0)
    {
      retval = select(fd+1, &rfds, NULL, NULL, &tv);
      if(retval == -1)
	{
	  printf("Select Error in read port\n");
	  return -1;
	}
      else if (retval)
	{
	  pthread_mutex_lock(&(l->mutex));
	  res = read(l->fd, buf, recvBytes);  time = GetDate_us();
	  pthread_mutex_unlock(&(l->mutex));
	  break;
	}
      else//timeout
	{
	  printf("Read_port timeout\n");
	  return -1;
	}
    }
  if(res<0)
    {
      printf("ERROR reading laser fd = %d", l->fd);
      return -1;
    }
  else
    {
      if(res==13){
        buf[res] = 0;
        //printf("chaine = %s\n", buf);
        if((mesure = (unsigned long)strtoul(&buf[4], NULL, 10))<0xFFFFFFFFFFFFFFFF){
            //printf("from laser %d got res = %d, mes = %lu time = %lu\n", res, l->fd, mesure, time);
            Save_Data_Laser(l, mesure, time);
        }
      }
      else if (res == 0) {
            printf("Erreur port usb deconnecté. chaine = %s, res = %d\n", buf, res);
            Laser_Close(l);
            laser_verify_usb_connection(l);
            return -1;
      }
      else{
            buf[res] = 0;
            printf("Erreur laser %d: Synchro perdue\n", l->fd);
            printf("Mesure: buf = %s, res = %d", buf, res);
            sleep(1);
            tcflush(l->fd, TCIOFLUSH);
            return -1;
      }
    }
  return 0;

}

void * Boucle_Reception_Laser(void * l)
{
  char buf[14];
  long long int count = 0;

  //printf("Reading Data\n");

  pthread_mutex_lock(&(((laser*)l)->mutex));
  ((laser*)l)->running = 1;
  //pthread_mutex_unlock(&(((laser*)l)->mutex));

  while(((laser*)l)->running)
    {
      pthread_mutex_unlock(&(((laser*)l)->mutex));
      if(read_port((laser *)l, 14, buf)>=0)
	count++;

    //init
      if(count == ORDRE_APPROX)
	{
	  ((laser*)l)->ready_for_analyse = 1;
	  //printf("Ready for analyse %d\n", ((laser*)l)->fd);
	  pthread_mutex_lock(&(((laser*)l)->mutex));
	}
      else
	pthread_mutex_lock(&(((laser*)l)->mutex));
    }

  pthread_mutex_unlock(&(((laser*)l)->mutex));
  pthread_exit(NULL);

}

int Laser_wait_until_ready(laser * l)
{
  unsigned long time = GetDate_us();
  //printf("Laser_wait_until_ready\n");
  pthread_mutex_lock(&(((laser*)l)->mutex));
  while(!(l->ready_for_analyse) && (GetDate_us()-time < 5000000))
    {
      //printf("\nBoucle while\n\n");
      pthread_mutex_unlock(&(((laser*)l)->mutex));
      usleep(1000000);
      pthread_mutex_lock(&(((laser*)l)->mutex));
    }
  //printf("après, ready? = %d\n", l->ready_for_analyse);
  pthread_mutex_unlock(&(((laser*)l)->mutex));

  if(l->ready_for_analyse)
    return 0;
  else
    return -1;

}
int Laser_Recv_Start(laser * l)
{
  char * data = "s0h\n";
  int res;
  if((res = write_port(l, data))>3)
    {
      if(pthread_create(&(l->recv_thread), NULL, Boucle_Reception_Laser, l))
        {
        printf("ERROR in creation boucle laser (laser %d)\n", l->fd);
        return -1;
        }
      //printf("Receive thread started\n");
    }
  else
    {
      printf("Laser_Recv_Start: Failed to write starting message\n");
      return res;
    }

  return 0;
}

void laser_genCfile_Close(int portNumber)
{
  FILE * f;
  #if STAND_ALONE_TEST
  char * fname = "toto.c";
  #else
  char * fname = "laser/toto.c";
  #endif // STAND_ALONE_TEST
  printf("writing close file for portNumer %d", portNumber);
  f = fopen(fname, "w");
  fprintf(f, "%s%d", LASER_CLOSE_CFILE, portNumber);
  fprintf(f, "%s", LASER_CLOSE_CFILE_P2);
  fclose(f);
}

int Laser_Recv_Stop(laser * l)
{
  char * data = "s0c\n";
  int res;
  //printf("Laser_Recv_Stop laser %d=>", l->fd);

  pthread_mutex_lock(&(l->mutex));
  l->running = 0;
  pthread_mutex_unlock(&(l->mutex));

  //attendre la fin du thread recv
  if(pthread_join(l->recv_thread, NULL))
    {
      printf("pthread_join error");
      return -1;
    }
  printf("recv Stop: recv joined\n");

  //dire au laser d'arrêter de mesurer

  /**V1: ubuntu 12.04**/

/*
    if((res = write_port(l, data))<4)
        {
          printf("ERROR in Stop_Laser_Recv->write_port\n");
          return res;
        }
    else
        {
          printf("write sent to port %s, fd = %d\n", l->portName, l->fd);
          //Laser_Close(l);
        }
*/

 /**V2: ubuntu 12.04 amélioré**/
/*
  int succes = 0;
  char buf[5];
  struct termios oldtio, newtio;
  int count = 0;
  //modif les param du port pour lire 4 données
  pthread_mutex_lock(&(l->mutex));

  if(tcgetattr(l->fd, &oldtio) < 0)
	{
	  pthread_mutex_unlock(&(l->mutex));
	  printf("Recv_Stop: Failed to get current config of port %d\n", l->fd);
	  return -1;
	}
	newtio = oldtio;
    newtio.c_cc[VMIN] = 4;
    tcflush( l->fd, TCIFLUSH );
      if(tcsetattr(l->fd, TCSANOW, &newtio)<0)
	{
	  pthread_mutex_unlock(&(l->mutex));
	  printf("Recv_Stop: Unable to apply given port settings\n");
	  return -1;
	}

  pthread_mutex_unlock(&(l->mutex));

  while (!succes && count<10){

    if((res = write_port(l, data))<4)
        {
            printf("ERROR in Stop_Laser_Recv->write_port\n");
            return res;
        }
    pthread_mutex_lock(&(l->mutex));
    tcdrain (l->fd);
    pthread_mutex_unlock(&(l->mutex));

    if(res = read_port(l, 5, buf) == 0){
        printf("buf = %s\n", buf);
        if(buf == "g0?\n")
            succes = 1;
    }
    count++;
  }
*/
/**V3: version bidouille. Affaire: trouver la raison de la merde avec write**/

    //fermeture du port serie
    if(Laser_Close(l)<0){
        printf("Laser RecvStop Laser %d Close error\n", l->portNumber);
        return -1;
    }

    pid_t pid = fork();
    if (pid < 0) {
        printf("A fork error has occurred.\n");
        exit(-1);
    } else {
        if (pid == 0) {
            laser_genCfile_Close(l->portNumber);
            #if STAND_ALONE_TEST
            execlp("./laser_stop_sa.sh","laser_stop_sa.sh",NULL);
            #else
            execlp("./laser/laser_stop.sh","laser/laser_stop.sh",NULL);
            #endif // STAND_ALONE_TEST
            printf("laser measuring stop script error\n");
            return -1;
        } else {
            wait(0);
        }
    }

  //printf("Laser_Recv_Stop port %d DONE\n", l->fd);
  return 0;
}


unsigned int VerifyMeasureConsistency_OneLaser(struct laser_data * Data)
{
  long int v = LastLaserData.vitesse;
  unsigned long dt = ((*Data).t-LastLaserData.t)/1000000+1;
  unsigned long diff;
  unsigned long tac;

  if(MasterControl)
    {
      diff = labs((*Data).mes-LastLaserData.mes);
    }
  else
    {
      diff = labs((*Data).mes-LastLaserData.mes+PositionOffset);
    }

  tac = MAX_ACCEL*dt*dt/2+labs(v*dt);
  //printf("diff = %lu, tac = %lu, v = %ld, dt = %lu, valcomp %ld \n", diff, tac, v, dt, (MESCONSISTENCY_VAL(v)+tac) );

  if((diff > MESCONSISTENCY_VAL(v)+tac))
    {
      printf("Returning Fatal Error\n");
      return ERR_LASER_FATAL;
    }
  else
    {
      return LASER_STATUS_OK;
    }

}

unsigned int VerifyMeasureConsistency(struct laser_data * MasterData,
				      struct laser_data * SlaveData)
{
  unsigned long v = LastLaserData.vitesse;
  unsigned long dt2 = ((*MasterData).t-LastLaserData.t)/1000000+1;
  unsigned long dt3 = ((*SlaveData).t-LastLaserData.t)/1000000+1;
  unsigned long diff1 = labs((*MasterData).mes-(*SlaveData).mes-PositionOffset);
  unsigned long diff2;
  unsigned long diff3;

  unsigned int laser_status = 0;
  unsigned long tac2, tac3;
  //offset
  long int MasterOffset;
  long int SlaveOffset;

  MasterOffset = 0;
  SlaveOffset = -PositionOffset;

  diff2 = labs((*MasterData).mes-LastLaserData.mes-MasterOffset);
  diff3 =  labs((*SlaveData).mes-LastLaserData.mes-SlaveOffset);

  //termes accel
  tac3 = MAX_ACCEL*labs(dt3)*labs(dt3)/2+labs(v*dt3);
  tac2 = MAX_ACCEL*labs(dt2)*labs(dt2)/2+labs(v*dt2);
  //printf("\nConsistency val: LastPos = %lu, LastT = %lu, v = %ld dt2 = %lu dt3 = %lu diff1 = %lu diff2 = %lu diff3 = %lu tac2 = %lu tac3 = %lu\n",
  //	 LastLaserData.mes, LastLaserData.t, v, dt2, dt3, diff1, diff2, diff3, tac2, tac3);
  //printf("Mesconsistency_val = %ld, MasterDat.mes = %lu, SLaveDat.mes = %lu, PositionOffset %ld\n\n", MESCONSISTENCY_VAL(v), (*MasterData).mes, (*SlaveData).mes, PositionOffset);
  if(diff1 < POSITIONCONSISTENCY_VAL)
    {
      //printf("Case diff1< machin\n");
      return LASER_STATUS_OK;
    }
  else if(diff1 > POSITIONCONSISTENCY_VAL)
    {
      //printf("Case diff1> machin\n");
      if((diff2 < MESCONSISTENCY_VAL(v)+tac2) && (diff3 < MESCONSISTENCY_VAL(v)+tac3))
	{
	  //printf("Case diff2< && diff3< machin\n");
	  return LASER_STATUS_OK;
	}
      else if((diff2 > MESCONSISTENCY_VAL(v)+tac2) && (diff3 > MESCONSISTENCY_VAL(v)+tac3))
	{
	  //printf("Case diff2> && diff3>\n");
	  return ERR_LASER_FATAL;
	}
      else if((diff2 > MESCONSISTENCY_VAL(v)+tac2) && (diff3 < MESCONSISTENCY_VAL(v)+tac3))
	{
	  //printf("Case diff2> && diff3< machin\n");
	  laser_status |= MASTER_FAILED_DATCONSISTENCY;
	}
      else if((diff2 < MESCONSISTENCY_VAL(v)+tac2) && (diff3 > MESCONSISTENCY_VAL(v)+tac3))
	{
	  //printf("Case diff2< && diff3> machin\n");
	  laser_status |= SLAVE_FAILED_DATCONSISTENCY;
	}
      else
	{
	  printf("Strange case\n");
	  return ERR_LASER_FATAL;
	}
    }
  else
    printf("ERREUR VerifyConsistencyVal\n");

  return laser_status;

}
unsigned int VerifyTimeConsistency_OneLaser(unsigned long LaserTime, unsigned long reftime)
{
  //printf("LaserTime-reftime = %lu, TIMECONSISTENCY_VAL = %u\n", labs(LaserTime-reftime), TIMECONSISTENCY_VAL);
  if(labs(LaserTime-reftime)<TIMECONSISTENCY_VAL){
    return LASER_STATUS_OK;
    }
  else{
    return ERR_LASER_FATAL;
    }
}
unsigned int VerifyTimeConsistency(unsigned long MasterTime, unsigned long SlaveTime, unsigned long time)
{
    //printf("mastertime-slavetime = %lu, mastertime-time = %lu, slavetime-time = %lu", labs(MasterTime-SlaveTime), labs(MasterTime-time), labs(SlaveTime-time));
  if(!(labs(MasterTime-time)<TIMECONSISTENCY_VAL || labs(SlaveTime-time)<TIMECONSISTENCY_VAL))
    {
      //printf("case t1> t2>\n");
      return ERR_LASER_FATAL;
    }
  else if(labs(MasterTime-time)>TIMECONSISTENCY_VAL && labs(SlaveTime-time)<TIMECONSISTENCY_VAL)
    {
      //printf("case t1> t2<\n");
      return MASTER_FAILED_TIMEOUT;//Master Failed
    }
  else if(labs(MasterTime-time)<TIMECONSISTENCY_VAL && labs(SlaveTime-time)>TIMECONSISTENCY_VAL)
    {
      //printf("case t1> t2<\n");
      return SLAVE_FAILED_TIMEOUT;//Slave Failed
    }
  else
    {
      //printf("ok\n");
      return LASER_STATUS_OK; //Slave and Master OK
    }
}

int laser_verify_usb_connection(laser * l)
{
    if (!(isopen[l->portNumber])){//pas connecté
        pthread_mutex_lock(&(l->mutex));
        l->ready_for_analyse = 0;
        l->running = 0;
        pthread_mutex_unlock(&(l->mutex));
        return 1;
    }

    return 0;//connecté
}

int Laser_GetPositionOffset(laser * master, laser * slave)
{
  struct laser_data mdat, sdat;
  //printf("Laser_GetPositionOffset=>");

  if(master->ready_for_analyse && slave->ready_for_analyse)
    {
      //printf("cas 1\n");
      pthread_mutex_lock(&(master->mutex));
      mdat = master->laser_dat[0];
      pthread_mutex_unlock(&(master->mutex));


      pthread_mutex_lock(&(slave->mutex));
      sdat = slave->laser_dat[0];
      pthread_mutex_unlock(&(slave->mutex));


      if(labs(mdat.t-sdat.t)<TIMECONSISTENCY_VAL)
	{
	  PositionOffset = mdat.mes - sdat.mes;
	  LastLaserData = mdat;
	  //printf("PositionOffset = %ld\n", PositionOffset);
	  return 0;
	}

      return -1;
    }
  else if(master->ready_for_analyse && !slave->ready_for_analyse)
    {
      //printf("cas2\n");
      pthread_mutex_lock(&(master->mutex));
      mdat = master->laser_dat[0];
      pthread_mutex_unlock(&(master->mutex));
      LastLaserData = mdat;
      PositionOffset = 0;
    }
  else if(!master->ready_for_analyse && slave->ready_for_analyse)
    {
      //printf("cas3\n");
      pthread_mutex_lock(&(slave->mutex));
      sdat = slave->laser_dat[0];
      pthread_mutex_unlock(&(slave->mutex));
      LastLaserData = sdat;
      PositionOffset = 0;
    }
  else
    {
      printf("cas 4\n");
      return -1;// laser not ready
    }

  //printf("PositionOffset = %ld\n", PositionOffset);
  return 0;
}

unsigned int Laser_GetVitesse(laser * master, laser * slave, struct laser_data * d)
{
  unsigned long t0,t1;
  unsigned int laser_status;
  struct laser_data d0, d1;
  int count = 0;
  int func_stat = 0;
  if(master->ready_for_analyse!=1 && slave->ready_for_analyse!=1)
    return ERR_LASER_FATAL;

  t1 = 0;t0 = 0;
  while((!func_stat) && (count < 10))
    {
      t0 = GetDate_us();
      if(((laser_status = Laser_GetData(master, slave, &d0))!=ERR_LASER_FATAL) && (GetDate_us()-t0<TIME_ERROR_VAL_VITESSE))
	{
	  usleep(TIME_VITESSE_CALC);
	  t1 = GetDate_us();
	  if(((laser_status = Laser_GetData(master, slave, &d1))!=ERR_LASER_FATAL) && (GetDate_us()-t1<TIME_ERROR_VAL_VITESSE))
	    {
	      d->vitesse = (long int)(((long int)d1.mes-(long int)d0.mes)/CONVERT_MUS_S(t1-t0));
	      //printf("d1.mes = %lu, d0.mes = %lu, d1-d0 = %ld, t1 = %lu, t0 = %lu, v = %ld, t1-t0 en s= %lf\n",
	      // d1.mes, d0.mes, (long int)(d1.mes-d0.mes), t1, t0, d->vitesse, CONVERT_MUS_S(t1-t0));
	      d->mes = d1.mes;
	      d->t = t1;
	      func_stat = 1;
	    }
	  else
	    usleep(200000);
	}
      else
	usleep(200000);
      if(laser_status == ERR_LASER_FATAL)
	break;

      count++;
    }



  if(labs(t1-t0-TIME_VITESSE_CALC)>TIME_ERROR_VAL_VITESSE+100/*init value*/)//todo mettre un truc mieux
    {
      printf("ERROR in Laser_GetVitesse\n");
      d = NULL;
    }

  return laser_status;
}

unsigned int Laser_GetUnverifiedData(laser * l, unsigned long * mesl)
{
  unsigned int res = 0;

  pthread_mutex_lock(&(l->mutex));
  if(l->ready_for_analyse==1)
    {
      //t0 = GetDate_us();//printf("Time0 = %lu=>", GetDate_us());
      *mesl = l->laser_dat[0].mes;
    }
  else
    res = 1;

  pthread_mutex_unlock(&(l->mutex));

  return res;
}

unsigned int Laser_GetData(laser * master, laser * slave, struct laser_data * d)
{

  struct laser_data MasterData, SlaveData;
  unsigned long time;
  unsigned int laser_status = LASER_STATUS_OK;

  unsigned long t0, t2;

  if(master->isSimu){
    pthread_mutex_lock(&(master->mutex));
    d->mes = master->laser_dat[0].mes;
    d->t = master->laser_dat[0].t;
    d->vitesse = master->laser_dat[0].vitesse;
    pthread_mutex_unlock(&(master->mutex));
    return 0;
  }

  if(master->ready_for_analyse==1)
    {
      //t0 = GetDate_us();//printf("Time0 = %lu=>", GetDate_us());
      pthread_mutex_lock(&(master->mutex));

      if(laser_verify_usb_connection(master)){
        laser_status |= MASTER_NOT_STARTED;//ready_for_analyse set to 0
      }

      MasterData = master->laser_dat[0];
      pthread_mutex_unlock(&(master->mutex));
    }
  time = GetDate_us();
  //printf("Time1 = %lu=>", GetDate_us());

  if (slave->ready_for_analyse==1)
    {

      pthread_mutex_lock(&(slave->mutex));
      SlaveData = slave->laser_dat[0];

      if(laser_verify_usb_connection(slave)){
        laser_status |= SLAVE_NOT_STARTED;//ready_for_analyse set to 0
      }

      pthread_mutex_unlock(&(slave->mutex));
    }

  if(master->ready_for_analyse!=1 && slave->ready_for_analyse!=1){
    //printf("boucle0\n");
    return ERR_LASER_FATAL;
  }
  //t2 = GetDate_us();
  //printf("Time2 = %lu %lu\n", t2-time, time-t0);

  //printf("Master dt = %lu, Slave dt = %lu\n, MasterSlave dt = %lu\n", labs(MasterData.t-time), labs(SlaveData.t-time), labs(MasterData.t-SlaveData.t));
  //printf("Master time = %lu, Slave time = %lu, time = %lu\n", MasterData.t, SlaveData.t, time);
  if(master->ready_for_analyse==1 && slave->ready_for_analyse==1)
    {
      //printf("boucle1\n");
      switch(laser_status |= VerifyTimeConsistency(MasterData.t, SlaveData.t, time))
	{
	case LASER_STATUS_OK:
	  laser_status |= VerifyMeasureConsistency(&MasterData, &SlaveData) ;
	  break;
	case SLAVE_FAILED_TIMEOUT:
	  laser_status |= VerifyMeasureConsistency_OneLaser(&MasterData);
	  break;
	case MASTER_FAILED_TIMEOUT:
	  MasterControl = 0;
	  laser_status |= VerifyMeasureConsistency_OneLaser(&SlaveData);
	  break;
	case ERR_LASER_FATAL:
	  {
	    printf("TIMECONSISTENCY ERROR\n");
	    return ERR_LASER_FATAL;
	  }
	default:
	  printf("ERREUR in Laser_GetData(). Laser_status unknown\n");
	}
      //printf("Time consistency = %u\n",VerifyTimeConsistency(MasterData.t, SlaveData.t, time));
    }
  else if (master->ready_for_analyse==1 && slave->ready_for_analyse!=1)
    {
      //printf("boucle2\n");
      laser_status |= SLAVE_NOT_STARTED;
      switch(laser_status |= VerifyTimeConsistency_OneLaser(MasterData.t, time))
	{
	case ERR_LASER_FATAL:
	  return ERR_LASER_FATAL;
	default:
        //printf ("Timeconsistency ok\n");
	  laser_status |= VerifyMeasureConsistency_OneLaser(&MasterData);
	  break;
	}
    }
  else if (master->ready_for_analyse!=1 && slave->ready_for_analyse==1)
    {
      //printf("boucle3\n");
      laser_status |= MASTER_NOT_STARTED;
      MasterControl = 0;
      //printf("laser_status 1 = %x\n", laser_status);

      switch(laser_status |= VerifyTimeConsistency_OneLaser(SlaveData.t, time))
	{
	case ERR_LASER_FATAL:
	  return ERR_LASER_FATAL;
	default:
	  laser_status |= VerifyMeasureConsistency_OneLaser(&SlaveData);
	  //printf("laser_status 2 = %x\n", laser_status);
	  break;
	}
    }
  else
    {
      printf("Laser_GetData internal error var l->ready_for_analyse\n");
      return ERR_LASER_FATAL;
    }
//printf("laser_status 3= %x\n", laser_status);
  switch(laser_status)
    {
    case ERR_LASER_FATAL:
      printf("Case ERR_LASER_FATAL\n");
      MasterControl = 1;
      d = NULL;
      break;
    case MASTER_FAILED_DATCONSISTENCY:
      LastLaserData = SlaveData;
      LastLaserData.mes += PositionOffset;
      MasterControl = 0;
      *d = SlaveData;
      (*d).mes += PositionOffset;
      break;
    case MASTER_FAILED_TIMEOUT:
      LastLaserData = SlaveData;
      LastLaserData.mes += PositionOffset;
      MasterControl = 0;
      *d = SlaveData;
      (*d).mes += PositionOffset;
      break;
    case MASTER_NOT_STARTED:
      LastLaserData = SlaveData;
      LastLaserData.mes += PositionOffset;
      MasterControl = 0;
      *d = SlaveData;
      (*d).mes += PositionOffset;
      break;
    default:
      MasterControl = 1;
      LastLaserData = MasterData;
      *d = MasterData;
      break;
    }


  if(master->ready_for_analyse!=1 && slave->ready_for_analyse==1)
    laser_status |= MASTER_NOT_STARTED;
  else if(master->ready_for_analyse==1 && slave->ready_for_analyse!=1)
    laser_status |= SLAVE_NOT_STARTED;


  //printf("laser_status = %x\n",laser_status);
  return laser_status;
}

unsigned int Laser_Init2Laser(laser * ml, laser * sl)
{
    unsigned int returnval = 0;
    int i;

    printf("date = %lu\n", GetDate_us());
    for (i=0;i<10;i++)
        printf("isopen[%d] = %d", i, isopen[i]);

    printf("\n");

    //initialise mesures laser
    int res;
    if((res = Laser_Init(ml))<0){
        printf("Init master laser failed\n");
        if(res == -1){
            if(Laser_Close(ml)<0){
                returnval |= LASER_MASTER_INIT_ERROR2;
                printf("Failed to close master laser port\n");
            }
        }
        printf("Master Laser closed\n");
        returnval |= LASER_MASTER_INIT_ERROR;
        }
    for (i=0;i<10;i++)
        printf("isopen[%d] = %d", i, isopen[i]);
    printf("\n");
    if((res = Laser_Init(sl))<0){
        printf("Init slave laser failed\n");
        if(res == -1){
            if(Laser_Close(sl)<0){
                printf("Failed to close slave laser port\n");
                returnval |= LASER_SLAVE_INIT_ERROR2;
            }
        }

        printf("Slave Laser closed\n");
        returnval |= LASER_SLAVE_INIT_ERROR;
    }
    if((returnval & LASER_MASTER_INIT_ERROR) && (returnval & LASER_SLAVE_INIT_ERROR))
        return ERR_LASER_INIT_FATAL;

  return returnval;
}

unsigned int Laser_Start1Laser(laser * l)
{
    int returnval = 0;

    if(Laser_Recv_Start(l)<0)
    {
      printf("Laser %d failed to start\n", l->fd);
      return 1;
    }
  //else
    //printf("Recv start master laser\n");

  if(Laser_wait_until_ready(l)<0)
    {
      printf("Laser failed to start\n");
      return 1;
    }

    pthread_mutex_lock(&(l->mutex));
    LastLaserData = l->laser_dat[0];
    pthread_mutex_unlock(&(l->mutex));

  return 0;
}

unsigned int Laser_Start2Laser(laser * ml, laser * sl)
{
  unsigned int returnval = 0;
  //lance mesures laser
  if(Laser_Recv_Start(ml)<0)
    {
      printf("Master Laser failed to start\n");
      returnval |= LASER_MASTER_START_ERROR;
    }
  else{
      if(Laser_wait_until_ready(ml)<0)
        {
          printf("Master Laser failed to start\n");
          returnval |= LASER_MASTER_START_ERROR;
        }
    }
  //else
    //printf("Master Laser ready\n");

  if(Laser_Recv_Start(sl)<0 )
    {
      printf("Slave Laser failed to start\n");
      returnval |= LASER_SLAVE_START_ERROR;
    }
  else{
      if(Laser_wait_until_ready(sl)<0)
        {
          printf("Slave Laser failed to start\n");
          returnval |= LASER_SLAVE_START_ERROR;
        }
    }
  //else
    //printf("Slave Laser Ready\n");
  if(!(((returnval & 0x0F) & LASER_MASTER_START_ERROR) && ((returnval & 0x0F) & LASER_SLAVE_START_ERROR))){
      if(Laser_GetPositionOffset(ml, sl)<0)
        {
          printf("Laser_GetPositionOffset error\n");
          returnval |= LASER_GETPOSOFFSET_ERROR;
        }
    }
  //else
  //printf("GetPositionOffset Ok\n");
  //printf("returnval = %x\n", returnval);

  if((((returnval & 0x0F) & LASER_MASTER_START_ERROR) && ((returnval & 0x0F) & LASER_SLAVE_START_ERROR)) || ((returnval & 0x0F) & LASER_GETPOSOFFSET_ERROR))
    return ERR_LASER_INIT_FATAL;


  return returnval;

}


int Laser_Exit1Laser(laser * l)
{
    if(Laser_Recv_Stop(l)<0)
    {
      printf("Failed to stop continuus measuring laser %d\n", l->fd);
      return -1;
    }
    else isopen[l->portNumber] = 0;

    return 0;
}

void printout_status(unsigned int status)
{
  unsigned int s;
  if((status & MASTER_NOT_STARTED) && (status & SLAVE_NOT_STARTED))
    {
      printf("ERR_LASER_FATAL by master and slave not ready\n");
      return;
    }
  else if((status & MASTER_NOT_STARTED)==0 && (status & SLAVE_NOT_STARTED))
    printf("SLAVE_NOT_STARTED=>");
  else if((status & MASTER_NOT_STARTED) && (status & SLAVE_NOT_STARTED)==0)
    printf("MASTER_NOT_STARTED=>");

  s = status;
  s &= ~MASTER_NOT_STARTED;
  s &= ~SLAVE_NOT_STARTED;
  switch(s)
    {
    case LASER_STATUS_OK:
      printf("LASER_STATUS_OK\n");
      break;
    case ERR_LASER_FATAL:
      printf("LASER: ERR_LASER_FATAL\n");
      break;
    case SLAVE_FAILED_TIMEOUT:
      printf("SLAVE_FAILED_TIMEOUT\n");
      break;
    case MASTER_FAILED_TIMEOUT:
      printf("MASTER_FAILED_TIMEOUT\n");
      break;
    case MASTER_FAILED_DATCONSISTENCY:
      printf("MASTER FAILED DATCONSISTENCY\n");
      break;
    case SLAVE_FAILED_DATCONSISTENCY:
      printf("SLAVE FAILED DATCONSISTENCY\n");
      break;
    case MASTER_NOT_STARTED:
      printf("MASTER_NOT_STARTED\n");
      break;
    case SLAVE_NOT_STARTED:
      printf("SLAVE_NOT_STARTED\n");
      break;
    default:
      printf("STATUS NOT KNOWN\n");
    }
}
void printout_data(laser * l)
{
  int i;
  printf("Print out laser data:\n");
  pthread_mutex_lock(&(l->mutex));
  for (i = 0; i<sizeof(l->laser_dat)/sizeof(struct laser_data); i++)
    printf("mes = %lu, vitesse = %ld at time = %lu\n",l->laser_dat[i].mes, l->laser_dat[i].vitesse, l->laser_dat[i].t);
  pthread_mutex_unlock(&(l->mutex));
  return;
}


#if STAND_ALONE_TEST==1
int main(void)
{
  laser l0, l1;
  unsigned int status_laser;
  int count = 0;
  struct laser_data recv_and_checked_data;
  printf("date = %lu\n", GetDate_us());
#if LASER_SIMULATION==0
  //initialise mesures laser
  if((Laser_Init(&l0))<0)
    {
      printf("Init laser l0 failed\n");
      return -1;
    }
  else
    printf("Laser l0 inited\n");
  if((Laser_Init(&l1))<0)
    {
      printf("Init laser l1 failed\n");
      return -1;
    }
  else
    printf("Laser l1 initted\n");

  //lance mesures laser
  if(Laser_Recv_Start(&l0)<0)
    {
      printf("Laser l0 failed to start\n");
      return -1;
    }
  else
    {
      printf("Receive Laser Data l0 started\n");
      if(Laser_wait_until_ready(&l0)<0)
	printf("Laser l0 wait_until_ready TIMEOUT\n");
      else
	printf("Ready to analyse data\n");
    }
  if(Laser_Recv_Start(&l1)<0)
    {
      printf("Laser l1 failed to start\n");
      return -1;
    }
  else
    {
      printf("Receive Laser Data started\n");
      if(Laser_wait_until_ready(&l1)<0)
	printf("Laser l0 wait_until_ready TIMEOUT\n");
      else
	printf("Ready to analyse data\n");
    }
  //lance reception mesures
  if(Laser_GetPositionOffset(&l0, &l1)<0)
    {
      printf("Laser_GetPositionOffset error\n");
      goto fail;
    }

  //printf("PositionOffset = %ld", PositionOffset);
  while(count<5)
    {
      unsigned long t1, t2;
      t1 = GetDate_us();
      if(((status_laser = Laser_GetData(&l0, &l1, &recv_and_checked_data)) & ERR_LASER_FATAL) != ERR_LASER_FATAL)
	{
	  t2 = GetDate_us();
	  printf("\n");
	  printf("RESULTAT MESURE LASER : d = %lu, t = %lu, v = %ld\n", recv_and_checked_data.mes, recv_and_checked_data.t, recv_and_checked_data.vitesse);
	  printf("Measuring time = %lu micro sec", t2-t1);
	  printf("\n");
	}
      printout_status(status_laser);
      count++;
      sleep(1);

    }

  //printout_data(&l0);
 fail:
  //stoppe mesure laser

  if(Laser_Recv_Stop(&l0)<0)
    return -1;
  else
    printf("Recv l0 stopped\n");

  if(Laser_Recv_Stop(&l1)<0)
    return -1;
  else
    printf("Recv l1 stopped\n");

  if(Laser_Close(&l0)<0)
    return -1;
  else
    printf("Laser l0 closed\n");
  if(Laser_Close(&l1)<0)
    return -1;
  else
    printf("Laser l1 closed\n");

#else
  struct laser_data d;
  count = 0;
  Laser_Init_Simu(&l0);
  Laser_Start_Simu(&l0);
  sleep(5);
  while(count<100)
    {
      Laser_GetData(&l0, NULL, &d);
      usleep(150000);
      printf("measured dist time vel = %lu %lu %ld\n", d.mes, d.t, d.vitesse);
      count++;
    }
  Laser_wait_end_of_simulation(&l0);
#endif//LASER_SIMULATION

  printf("date = %lu\n", GetDate_us());
  return 0;
}
#endif //STAND_ALONE_TEST
#if STAND_ALONE_TEST==2

int main(void)
{
  laser l0, l1;
  struct laser_data d;
  unsigned int laser_status;
  int count = 0;
  unsigned long t0, t1;
  printf("date = %lu\n", GetDate_us());
  init_laser_data(&l1);
  //initialise mesures laser
  if((Laser_Init(&l0, "/dev/ttyUSB0"))<0)
    return -1;
  else
    printf("Laser Initted\n");


  //lance mesures laser
  if(Laser_Start1Laser(&l0))
    return -1;


  else
    {

      //lance reception mesures

      while(count<10)
	{
	  t0 = GetDate_us();
	  if((laser_status = Laser_GetData(&l0, &l1, &d))!=ERR_LASER_FATAL)
	    {
	      t1 = GetDate_us();
	      printf("delta t = %lu\n", t1-t0);
	      printf("\nData measured: mes = %lu, t = %lu, v = %ld\n\n", d.mes, d.t, d.vitesse);
	      usleep(200000);
	    }
	  printout_status(laser_status);
	  if(laser_status == ERR_LASER_FATAL)
	    sleep(1);
	  count++;
	}

      //sleep(5);
    }

  //stoppe mesure laser
  printout_data(&l0);

  if(Laser_Recv_Stop(&l0)<0)
    return -1;
  else
    {
      printf("Recv stopped\n");
    }

  if(Laser_Close(&l0)<0)
    return -1;
  else
    printf("Laser closed\n");

  printf("date = %lu\n", GetDate_us());
  return 0;
}
#endif//STAND_ALONE_TEST
