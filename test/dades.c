#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>                                                  
#include <unistd.h>                                                  
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>


int init( const char *name )
{
   int fd;
   struct termios term;
   speed_t speed;

   fd = open(name, O_RDONLY | O_NOCTTY | O_SYNC);
   if (fd < 0)
      return fd;

   if (tcgetattr(fd, &term) < 0)
      return -1;

   /* Input modes. */
   term.c_iflag &= ~(IXON | IXOFF | IXANY); /* Deactivate software flux control. */
   term.c_iflag &= ~(INLCR | IGNCR | ICRNL); /* Don't ignore or map characters. */
   /* Output modes. */
   term.c_oflag &= ~OPOST; /* Send raw output. */
   /* Control modes. */
   term.c_cflag |= CLOCAL | CREAD; /* Enable reading. */
   term.c_cflag &= ~CRTSCTS; /* Deactivate hardware flux control. */
   /* Local modes. */
   term.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* Raw data reception. */
   /* Control characters. */
   term.c_cc[VMIN] = 0; /* Read at least 0 bytes. */
   term.c_cc[VTIME] = 100; /* 1 second timeout. */
   /* Set the speed. */
   speed = B9600;
   if (cfsetospeed(&term, speed) < 0) /* Try to get fastest possible speed. */
      return -1;
   /* Set attributes. */
   if (tcsetattr(fd, TCSANOW, &term) < 0)
      return -1;

   return fd;
}

int main( int argc, char *argv[] )
{
   int fd;
   int i, n;
   char buf[128];
   uint16_t ui;

   if (argc<2) {
      fprintf(stdout,"Syntax: %s file\n", argv[0]);
      return 1;
   }

   fd = open(argv[1], O_RDONLY | O_NOCTTY | O_SYNC);
   /*fd = init(argv[1]);*/
   if (fd < 0) {
      fprintf(stderr,"Failed to open '%s': %s\n", argv[1], strerror(errno));
      return 1;
   }

   n = 0;
   while (1) {
      i = 0;
      while (i<6) {
         i += read( fd, &buf[i], 6-i );
      }
      ui = (buf[0]<<8) + buf[1];
      fprintf(stdout, "%d %u\n", n, ui );
      n++;
   }

   close(fd);
}
