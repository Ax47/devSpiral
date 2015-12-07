#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <strings.h>

int main(void)
{
  int fd = open("/dev/ttyUSB0", O_RDWR);
  const char * buf = "s0c\n";
  int res;
  struct termios newtio;

      printf("port %d opened->", fd);
      printf("Setting port %d params->", fd);

      if(!isatty(fd))
	{
	  printf("port %d is not a tty\n", fd);
	  return -1;
	}
      /*
      if(tcgetattr(fd, &oldtio) < 0)
	{
	  printf("Failed to get current config of port %d\n", fd);
	  return -1;
	}
      */
      bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

      //setting c_cflags
      // set baud rates
      if(cfsetispeed(&newtio, B19200) < 0 || cfsetospeed(&newtio, B19200) < 0)
	{
	  printf("Impossible to set baud rates of port %d\n", fd);
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

      tcflush( fd, TCIFLUSH );
      if(tcsetattr(fd, TCSANOW, &newtio)<0)
	{
	  printf("Unsable to apply given port settings\n");
	  return -1;
	}
      printf("succeed in setting port %d params\n", fd);
    
  res = write(fd, buf, 4);
  printf("write res = %d", res);
  close(fd);
      return 0;

}
