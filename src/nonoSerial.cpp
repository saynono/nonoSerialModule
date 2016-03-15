/*
 * wiringSerial.c:
 *	Handle a serial port
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************


https://developer.apple.com/library/mac/samplecode/SerialPortSample/Listings/SerialPortSample_SerialPortSample_c.html


 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>


#include <errno.h>
#include <sys/ioctl.h>

// #ifndef TARGET_LINUX

// new shit for Serial usb custom baudrates!
/*
#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/serial/IOSerialKeys.h>
*/
#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#endif

// #endif




#include "nonoSerial.h"

// Hold the original termios attributes so we can reset them
      static struct termios oldoptions;
// #define B1000000 1000000


using namespace nono;
using namespace nono::com;

Serial::Serial(){
  fd = -1;
  deviceName = "-";
}
/*
 * serialOpen:
 *	Open and initialise the serial port, setting all the right
 *	port parameters - or as many as are required - hopefully!
 *********************************************************************************
 */

int Serial::openPort (const char *device, const int baud)
{
  struct termios options ;
  speed_t myBaud = -1;
  int     status;

  switch (baud)
  {
    case     50:	myBaud =     B50 ; break ;
    case     75:	myBaud =     B75 ; break ;
    case    110:	myBaud =    B110 ; break ;
    case    134:	myBaud =    B134 ; break ;
    case    150:	myBaud =    B150 ; break ;
    case    200:	myBaud =    B200 ; break ;
    case    300:	myBaud =    B300 ; break ;
    case    600:	myBaud =    B600 ; break ;
    case   1200:	myBaud =   B1200 ; break ;
    case   1800:	myBaud =   B1800 ; break ;
    case   2400:	myBaud =   B2400 ; break ;
    case   4800:	myBaud =   B4800 ; break ;
    case   9600:	myBaud =   B9600 ; break ;
    case  19200:	myBaud =  B19200 ; break ;
    case  38400:	myBaud =  B38400 ; break ;
    case  57600:	myBaud =  B57600 ; break ;
    case 115200:	myBaud = B115200 ; break ;
    case 230400:	myBaud = B230400 ; break ;
#ifdef __linux__    
    default:      myBaud = baud ; break;
    // case 1000000:  myBaud = B1000000 ; break ;
#endif

    // default:
    //   return -2 ;
  }



    // int success;

    // fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);  
    // if(fd == -1){  
    //   printf("ofSerial: unable to open port");  
    //   return false;  
    // }  
    //       // we want exclusive access, this is different  
    // success = ioctl(fd, TIOCEXCL);  
    // if(success != -1) {  
    //   // this is also different  
    //   success = fcntl(fd, F_SETFL, 0);  
    // } else {  
    //   printf("ofSerial: can't lock port :(");  
    //   return false;  
    // }  
    //             if(success == -1) {  
    //   printf("ofSerial: can't lock filedescriptor :(");  
    //   return false;  
    // }  

    // // struct termios options;  
    // success = tcgetattr(fd,&oldoptions);  
    // if (success == -1) {  
    //   printf("ofSerial: can't get old options");  
    //   return false;  
    // }  
    // options = oldoptions;  
    //             // NOTE: cfsetispeed & cfsetospeed switch statement omitted here for brevity  

    // /*options.c_cflag |= (CLOCAL | CREAD);  
    // options.c_cflag &= ~PARENB;  
    // options.c_cflag &= ~CSTOPB;  
    // options.c_cflag &= ~CSIZE;  
    // options.c_cflag |= CS8;  

    // cfmakeraw(&options);  

    // // set tty attributes (raw-mode in this case)  
    // success = tcsetattr(fd, TCSANOW, &options);  
    // if (success == -1) {  
    //   printf("ofSerial: can't set attibutes on port descriptor");  
    //   return false;  
    // }  
    // speed_t speed = baud;
    // success = ioctl(fd, IOSSIOSPEED, speed);  
    // success = ioctl(fd, IOSSDATALAT, 3); // an arbitrary length of time to wait for the latency  








  // if ((fd = open (device, O_RDWR | O_NONBLOCK)) == -1)
  if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
    return -1 ;

  // fcntl (fd, F_SETFL, O_RDWR) ;


    // Get the current options and save them so we can restore the default settings later.
  if (tcgetattr(fd, &oldoptions) == -1) {
        printf("Error getting tty attributes %s(%d).\n", strerror(errno), errno);
  }

// Get and modify current options:
  tcgetattr (fd, &options) ;

    cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

  tcsetattr (fd, TCSANOW | TCSAFLUSH, &options) ;

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);

  // int currentBaudbaudRate = (int) cfgetospeed(&options);
  // printf("Output baud rate changed to %d\n", (int) cfgetospeed(&options));
  
  // On OS X, starting in Tiger, we can set a custom baud rate, as follows:

  if ((int) cfgetospeed(&options) != baud) {

    int res = setCustomBaudrate(fd,baud);
    if( res == 0 ){
      myBaud = baud;
    }

  }
/*
#ifdef __APPLE__  
  if ((int) cfgetospeed(&options) != baud) {
    speed_t speed = baud;
    if (ioctl(fd,  IOSSIOSPEED, &speed) == -1) {
      printf("ERROR setting custom baudrate!\n");
    }else{
      myBaud = speed;
      printf("Sucessfully altered baudrate to %i\n",(unsigned int)myBaud);
    }
  }
#endif
*/

#ifdef __APPLE__
  unsigned long mics = 1UL;
  int success = ioctl(fd, IOSSDATALAT, &mics);
  if( success < 0 ){
    printf("Error Serial setting read latency %s - %s(%d).\n", device, strerror(errno), errno);
  }
#endif

  std::string strName(device);
  deviceName = strName;

  if( fd != -1 ) printf("Connected to Serial Device: %s at %i baud\n",device,(int)myBaud);
  else printf("ERROR Connecting to Serial Device: %s.\n",device);

  usleep (10000) ;	// 10mS

  return fd ;
}





int Serial::setBaudrate(int fd, speed_t baudrate) {
  int rc;
  struct termios tio;

  rc = tcgetattr(fd, &tio);
  if (rc < 0) {
    printf("ERROR: tcgetattr()\n");
    return -errno;
  }
  cfsetispeed(&tio, baudrate);
  cfsetospeed(&tio, baudrate);

  rc = tcsetattr(fd, TCSANOW, &tio);
  if (rc < 0) {
    printf("ERROR: tcgetattr()\n");
    return -errno;
  }

  return 0;
}


// This part is taken from here: http://cgit.osmocom.org/osmocom-bb/plain/src/shared/libosmocore/src/serial.c

/*! \brief Change current baudrate to a custom one using OS specific method
 *  \param[in] fd File descriptor of the open device
 *  \param[in] baudrate Baudrate as integer
 *  \returns 0 for success or negative errno.
 *
 *  This function might not work on all OS or with all type of serial adapters
 */

int Serial::setCustomBaudrate(int fd, speed_t baudrate) {
#ifdef __linux__
  int rc;
  struct serial_struct ser_info;

  rc = ioctl(fd, TIOCGSERIAL, &ser_info);
  if (rc < 0) {
    printf("ERROR: ioctl(TIOCGSERIAL)\n");
    return -errno;
  }

  ser_info.flags = ASYNC_SPD_CUST | ASYNC_LOW_LATENCY;
  ser_info.custom_divisor = ser_info.baud_base / baudrate;

  rc = ioctl(fd, TIOCSSERIAL, &ser_info);
  if (rc < 0) {
    printf("ERROR: ioctl(TIOCSSERIAL)\n");
    return -errno;
  }

  return setBaudrate(fd, B38400); /* 38400 is a kind of magic ... */
#elif defined(__APPLE__)
#ifndef IOSSIOSPEED
#define IOSSIOSPEED    _IOW('T', 2, speed_t)
#endif
  int rc;

  unsigned int speed = baudrate;
  rc = ioctl(fd, IOSSIOSPEED, &speed);
  if (rc < 0) {
    printf("ERROR: ioctl(IOSSIOSPEED)\n");
    return -errno;
  }
  return 0;
#else
#warning osmo_serial_set_custom_baudrate: unsupported platform
  return -1;
#endif
}


/*
 * serialFlush:
 *	Flush the serial buffers (both tx & rx)
 *********************************************************************************
 */

void Serial::flush ()
{
  tcflush (fd, TCIOFLUSH) ;
}


/*
 * serialClose:
 *	Release the serial port
 *********************************************************************************
 */

void Serial::closePort ()
{
  
    // Block until all written output has been sent from the device.
    // Note that this call is simply passed on to the serial device driver.
    // See tcsendbreak(3) <x-man-page://3/tcsendbreak> for details.

  if (tcdrain(fd) == -1) {
        printf("Error waiting for serial drain.\n");
  }

    // Block until all written output has been sent from the device.
    // Note that this call is simply passed on to the serial device driver.
    // See tcsendbreak(3) <x-man-page://3/tcsendbreak> for details.

  if (tcsetattr(fd, TCSANOW, &oldoptions) == -1) {
    printf("Error resetting tty attributes - %s(%d).\n",
    strerror(errno), errno);

  }
  close (fd) ;
}


/*
 * serialPutchar:
 *	Send a single character to the serial port
 *********************************************************************************
 */

void Serial::writeByte (const unsigned char c)
{
  if( fd == -1 ) return;
  write (fd, &c, 1) ;
}


/*
 * serialPuts:
 *	Send a string to the serial port
 *********************************************************************************
 */

int Serial::writeString (const char *s)
{
  // if( fd == -1 ) return -1;
  return write (fd, s, strlen (s)) ;
}

/*
 * writeBytes:
 *  Send an array of bytes to the serial port
 *********************************************************************************
 */

int Serial::writeBytes (const unsigned char* data, int len )
{
  return write (fd, data, len) ;
}


/*
 * serialPrintf:
 *	Printf over Serial
 *********************************************************************************
 */

// void nono::com::Serial::serialPrintf (const int fd, const char *message, ...)
// {
//   va_list argp ;
//   char buffer [1024] ;

//   va_start (argp, message) ;
//     vsnprintf (buffer, 1023, message, argp) ;
//   va_end (argp) ;

//   serialPuts (fd, buffer) ;
// }


/*
 * serialDataAvail:
 *	Return the number of bytes of data avalable to be read in the serial port
 *********************************************************************************
 */

int Serial::dataAvailable ()
{
  int result ;

  if (ioctl (fd, FIONREAD, &result) == -1)
    return -1 ;

  return result ;
}



/*
 * readBytes:
 *  Read an array of bytes from the serial port
 *********************************************************************************
 */

int Serial::readBytes (unsigned char* data, int len )
{
  return read (fd, data, len) ;
}


/*
 * serialGetchar:
 *	Get a single character from the serial device.
 *	Note: Zero is a valid character and this function will time-out after
 *	10 seconds.
 *********************************************************************************
 */
/*
int Serial::serialGetchar ()
{
  uint8_t x ;

  if (read (fd, &x, 1) != 1)
    return -1 ;

  return ((int)x) & 0xFF ;
}
*/

int Serial::getSerialFD(){
  return fd;
}

bool Serial::isConnected(){
  return fd != -1;
}

const std::string Serial::getDeviceName(){
  return deviceName;
}

