/*

sudo gcc -o Pi_Serial_multi Pi_Serial_multi.cpp -lwiringPi -DRaspberryPi -pedantic -Wall; 
sudo ./Pi_Serial_multi.o

*/
 
//include system librarys
#include <stdio.h> //for printf
#include <stdint.h> //uint8_t definitions
#include <stdlib.h> //for exit(int);
#include <string.h> //for errno
#include <errno.h> //error output

#include <math.h>
#include <algorithm>    // std::max

#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <termios.h>

#include "nonoSerialModule.h"


#define BUFFER_SIZE_UDP 2048

#define HGC_UDP_HEADER_OFF              0xF0
#define HGC_UDP_HEADER_DEMO_MODE        0xF1
#define HGC_UDP_HEADER_REMOTE_MODE      0xF2
#define HGC_UDP_HEADER_QUIT             0xFA
#define HGC_UDP_HEADER_FRAME            0xA0



int udpPortReceiving = 12999;
int udpServer = -1; // receiving

struct sockaddr_in udpClientFront;
struct sockaddr_in udpClientBack;
unsigned char udpBuffer[BUFFER_SIZE_UDP];
unsigned long lastUdpFrameReceived = 0;

static uint64_t epochMilli;

unsigned int frameDur = 1000 / 30;

// unsigned int frameDur = 1000 / 2;
unsigned long timeCur = 0;
unsigned long lastRemoteFrame = 0;

unsigned int frameCnt = 0;


unsigned char dataBuffer[HGC_AMOUNT_LCDS_TOTAL];

int currentMode = HGC_UDP_HEADER_REMOTE_MODE;
// int currentMode = HGC_UDP_HEADER_DEMO_MODE;


using namespace nono::com;

nonoSerialModule serialModule;

//prototypes
int main(void);
void setup(void);
void setupDisplayBuffer(void);
void loop(void);
ssize_t readUdpStream( int udp_fd, unsigned char* buf );
int createUDPServer( int port );
void processUDPData( ssize_t res, unsigned char* buf, int* transtable );
void processPanelData( unsigned char* data, unsigned char* displayBuffer, int* transtable );
void refreshHeader(void);
void processSettingsData( unsigned char* buf, ssize_t res );




unsigned int millis (void)
{
  struct timeval tv ;
  uint64_t now ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ;

  return (uint32_t)(now - epochMilli) ;
}
/*
    // printBinary: function (buf) {
void printBinary( unsigned char* buf , int len ){
  printf("BINARY : \n");
  for( int i=0;i<len*8;i++ ){
    printf("%i",((buf[(int)(i/8)] >> (i%8)) & 0x01) == 1 ? 1:0);
    if( i%8==7 ) printf(" ");
    if( i%48==47 ) printf("\n");
  }
  printf("\n------------------------\n");
}

void printHex( unsigned char* buf , int len ){
  printf("HEX : \n");
  for( int i=0;i<len;i++ ){
    printf("%02X", buf[i] );
    if( i%4==3 ) printf(" ");
    if( i%48==47 ) printf("\n");
  }
  printf("\n------------------------\n");
}
*/

void signalHandler(int sig){

  if( udpServer != -1 ) close( udpServer );
  serialModule.close();
  printf("\nDoor closed... (%i)\n",sig);
  exit(0);

}


// ------------------------------------------------------------------------
// UDP Data
//

int createUDPServer( int port ){

  printf("No 10. UDP Server - Listening on port %i...\n",port);

  int handle = socket( AF_INET, SOCK_DGRAM, 0 );
  if(handle < 0){
    perror("socket");
    exit(1);
  }
       
  int flags = fcntl(handle, F_GETFL);
  flags |= O_NONBLOCK;
  fcntl(handle, F_SETFL, flags);

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_port=htons( port );
  servaddr.sin_addr.s_addr= INADDR_ANY;

  if ( bind( handle, (struct sockaddr*)&servaddr, sizeof(servaddr) ) < 0 ){
    perror("bind");
    exit(1);
  }

  return handle;
}





ssize_t readUDPStream( int udp_fd, unsigned char* buf ){
  ssize_t res = -1;
  res = recvfrom(udp_fd, buf, BUFFER_SIZE_UDP, 0, NULL, NULL);
  return res;
}

void processUDPData( ssize_t res, unsigned char* buf ){

      switch ( buf[0] ) {
        case HGC_UDP_HEADER_OFF:  // Turn off 
          currentMode = HGC_UDP_HEADER_OFF;
          printf("UDP MSG:   TURN OFF COLUMN\n");
          break;
        case HGC_UDP_HEADER_DEMO_MODE:  // Turn off 
          currentMode = HGC_UDP_HEADER_DEMO_MODE;
          printf("UDP MSG:   DEMO MODE\n");
          break;
        case HGC_UDP_HEADER_REMOTE_MODE:  // Turn off 
          currentMode = HGC_UDP_HEADER_REMOTE_MODE;
          printf("UDP MSG:   REMOTE MODE\n");
          break;

        case HGC_UDP_HEADER_FRAME:  // Turn off 
          // memcpy( udp2Buffer,buf+1,FRAME_SIZE);
          // printf("UDP MSG:   NEW FRAME\n");
          lastUdpFrameReceived = millis();
          if( currentMode == HGC_UDP_HEADER_REMOTE_MODE ){
            // printf("Remote Frame. (%i)\n",(int)res);

            memcpy(dataBuffer,buf+1,HGC_AMOUNT_LCDS_TOTAL);

          }
          break;

        case 'Q':  // Quit.
        case 'q':
        case 'X':
        case 'x':
            signalHandler(2);
            return;

        default:
          printf("UDP Unknown opcode! %c\n", buf[0]);
          // keep_processing = false;
          break;
      }
    // }  
}

 
// ------------------------------------------------------------------------
// Setup
//

void setup(){
 

  struct sigaction act;
  act.sa_handler = signalHandler;
  sigemptyset(&act.sa_mask);
  act.sa_flags = 0;
  sigaction(SIGINT, &act, 0);

  memset(dataBuffer,0,HGC_AMOUNT_LCDS_TOTAL);


  printf("%s\n", "Startup!");
  fflush(stdout);

  udpServer = createUDPServer( udpPortReceiving );
  if( udpServer < 0 ){
    printf("Oh no UDP Server possible. :(");
    exit(1); //error
  }

  serialModule.setup();

  printf("All done and setup. May crystals shine now.\n");

}



// ------------------------------------------------------------------------
// Loop
//

void loop(){

  // unsigned long tStart = millis();
/*
  serialDevice1BufferSize = readSerialSignal( serialDevice1, serialDevice1Buffer, serialDevice1BufferSize );
  if( serialDevice1BufferSize > 0 ){
    if( f_settings_sensor_enabled ){
      serialDevice1BufferSize = processSerialData( serialDevice1Buffer, serialDevice1BufferSize, sensorBufferFront, panelToPixelFront );
    }else{
      serialDevice1BufferSize = 0;
    }
    // sendUDPMessageFront( serialDevice1Buffer, len );
  }


  serialDevice2BufferSize = readSerialSignal( serialDevice2, serialDevice2Buffer, serialDevice2BufferSize );
  if( serialDevice2BufferSize > 0 ){
    if( b_settings_sensor_enabled ){
      serialDevice2BufferSize = processSerialData( serialDevice2Buffer, serialDevice2BufferSize, sensorBufferBack, panelToPixelBack );
    }else{
      serialDevice2BufferSize = 0;
    }
    // sendUDPMessageBack( serialDevice2Buffer, len );
  }
*/

  ssize_t res = readUDPStream( udpServer, udpBuffer );
  if( res > -1 ) {
    processUDPData( res, udpBuffer );
  }

  if(millis()-timeCur >= frameDur){

    // printf("Loop (DUR:%i)\n",(unsigned int)(millis()-timeCur));

    timeCur=millis();
    frameCnt ++;

    
    if( currentMode == HGC_UDP_HEADER_DEMO_MODE ){
      // unsigned char val = frameCnt*4 % 0xff;
      unsigned char val = frameCnt%40 > 20 ? 0xff : 00;
      int curLCD = (frameCnt/5)%HGC_AMOUNT_LCDS_TOTAL;
      int timeSomething = frameCnt/50;
      // printf("\n---------------------------------------------------------------------------------------------------\nDemo Frame %i: Send value: %2X\n",frameCnt,val);
      // // bool success = serialModule.testSendToSplitter(val);
      // bool success = serialModule.testSendToSide(val);
      // if( !success ){
      //   printf("Serial: Couldn't send entire data set.");
      // }
      int side;

      for( int i=0;i<HGC_AMOUNT_LCDS_TOTAL;i++ ){
        side = i / HGC_AMOUNT_LCDS_PER_COLUMN;
        dataBuffer[i] = side == (timeSomething%3) ? 0xff : 00;
        if( i == curLCD ) dataBuffer[i] = 0xff - val;
        // dataBuffer[i] = i == frameC ? 0xff : 0;
      }
      // memset(dataBuffer,val,HGC_AMOUNT_LCDS_TOTAL);
      // bool success = serialModule.sendCompleteData(dataBuffer);
      // if( !success ){
      //   printf("Serial: Couldn't send entire data set.");
      // }

    }
    


    serialModule.testSendToSerial( frameCnt );

  }

  // unsigned long tEnd = millis();
  // printf("Dur : %lu \n", (tEnd-tStart) );
}

 
// ------------------------------------------------------------------------
// Main
//

int main(){


#ifdef __linux__
  printf("\n\n=============================\nSerial Output. Linux Version.\n=============================\n\n");
#elif __APPLE__
  printf("\n\n=============================\nSerial Output. Apple Version.\n=============================\n\n");
#endif


  setup();

  while( true ){
    usleep(1000);
    loop();
  } 
  return 0;
}
