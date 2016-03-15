
 


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
#include "nonoHelper.h"



using namespace nono::com;


nonoSerialModule::nonoSerialModule(){
  memset(serialBuffer,0,HGC_SERIAL_BUFFER_SIZE);
  memset(dataBuffer,0,HGC_AMOUNT_LCDS_TOTAL);
  frameCnt = 0;
}


 
// ------------------------------------------------------------------------
// Setup
//

void nonoSerialModule::setup(){

  printf("%s\n", "nonoSerialModule starting...!");


  std::vector<std::string> deviceNames;

#ifdef __linux__

  deviceNames.push_back("/dev/ttyUSB0");
  deviceNames.push_back("/dev/ttyUSB1");
  deviceNames.push_back("/dev/ttyUSB2");

#elif __APPLE__

  // deviceNames.push_back("/dev/tty.usbserial-FTYSF3GQ");
  deviceNames.push_back("/dev/tty.usbserial-FTYXQU1DA");
  
  // deviceNames.push_back("/dev/tty.usbserial-FTYSF3TE");
  // deviceNames.push_back("/dev/tty.usbserial-FTWGGGSN");

#endif

  for( int i=0;i<deviceNames.size();i++){
    serialDevices.push_back( Serial() );
    int fd = serialDevices.back().openPort (deviceNames[i].c_str(), (int)HGC_SERIAL_BAUDRATE);
    if ( fd < 0){
      fprintf (stderr, "Unable to open serial device %i (%s): %s\n", i, deviceNames[i].c_str(), strerror (errno)) ;
    }else{
      struct termios options ;
      tcgetattr (serialDevices[i].getSerialFD(), &options) ;   // Read current options
      options.c_cflag |= CSTOPB ;
      tcsetattr (serialDevices[i].getSerialFD(), TCSANOW, &options) ;   // Set new options
    }
  }

  printf("SerialModule setup up.\n");

}


// ------------------------------------------------------------------------
// Close
//

void nonoSerialModule::close(){

  for( int i=0;i<serialDevices.size();i++){
    serialDevices[i].closePort();
  }
  printf("\nShutting down and sending a dark frame.\n");

}

bool nonoSerialModule::sendCompleteData( unsigned char* data ){
  bool res = true;
  // for( int i=0;i<HGC_SERIAL_DEVICE_CNT;i++){
  //   if( !sendDataToSide( &serialDevices[i], data, (HGC_COLUMN_SIDES)i ) ){
  //     res = false;
  //   }
  // }
  return res;
}


bool nonoSerialModule::sendDataToSide( Serial* serial, unsigned char* data, HGC_COLUMN_SIDES side ){
    
    bool res = true;
    memset( serialBuffer, 0, HGC_SERIAL_BUFFER_SIZE);

    // ModelDefinitions::ColumnSide* sideModel = &ModelDefinitions::allColumns.sides[ serial.side ];

    // int cnt = 0;
    int splitterCnt = 1; // start with 1. 0 is special mode
    // int panelCnt = 0;
    int sideRawDataOffset = side * HGC_AMOUNT_LCDS_PER_PANEL * HGC_AMOUNT_PANELS_PER_COLUMN;
    
    int bufferPos = HGC_SERIAL_HEADER_LENGTH;
    int dataOffset = sideRawDataOffset;
    
    for( int panelID=0;panelID<HGC_AMOUNT_PANELS_PER_COLUMN;panelID++){
        
        serialBuffer[bufferPos] = 0x00;   // DMX Data
        bufferPos += 1;
        
        memcpy( serialBuffer+bufferPos, data+dataOffset, HGC_AMOUNT_LCDS_PER_PANEL );
        bufferPos += 30;
        dataOffset += HGC_AMOUNT_LCDS_PER_PANEL;

        if( panelID!=0 && (panelID%6 == 5  || panelID == HGC_AMOUNT_PANELS_PER_COLUMN-1) ){
            if( !sendDataToSplitter( serial, splitterCnt,(panelID%6)+1 ) ){
              // fprintf (stderr, "Serial Data failed. Side %i, Panel %i\n", side, panelID ) ;
              res = false;
            }
            splitterCnt++;
            bufferPos = HGC_SERIAL_HEADER_LENGTH;
        }

    }
    
//    serial.serial->writeBytes(buffer, cnt);
    return res;
}



bool nonoSerialModule::sendDataToSplitter( Serial* serial, int splitterID, int panelCnt ){
    // printf("-------------- DATA FOR SPLITTER %i (%s) -------------- ", splitterID,serial->getDeviceName().c_str());
    int cnt=0;
    int dataLength = panelCnt * HGC_SERIAL_BYTER_PER_SPLITTER_PORT + 5;
    serialBuffer[cnt++] = HGC_SERIAL_HEADER;          // HEADER
    serialBuffer[cnt++] = dataLength & 0xff;          // PACKET LENGTH LOW
    serialBuffer[cnt++] = (dataLength>>8) & 0xff;     // PACKET LENGTH HIGH
    serialBuffer[cnt++] = splitterID;                 // Splitter ID
    serialBuffer[cnt++] = 0x01;                       // Command
    serialBuffer[cnt++] = panelCnt;                   // Panels count == port per splitter
    serialBuffer[cnt++] = HGC_SERIAL_BYTER_PER_SPLITTER_PORT & 0xff;        // bytes LOW => 30 LCDs + 1 divider
    serialBuffer[cnt++] = (HGC_SERIAL_BYTER_PER_SPLITTER_PORT>>8)&0xff;     // bytes HI <nbytes> is the number of bytes to send to each ports. For the LCD application this is 31 decimal (00+15 nodes x2 LCDs per node)
    serialBuffer[cnt++] = 0x00;                       // <mask> In conjunction with <nports=0> this selects a single port only instead of all ports. Only useful for test/programming.
    serialBuffer[cnt++] = 0x00;                       // <format> This should be 0 for the format used by the LCD nodes. (125kbaud, break to start)
    
    
//    for( int i=0;i<dataLength+HGC_SERIAL_HEADER_LENGTH-5;i++ ){
//        printf(" %02i",buffer[i]);
//    }
//    cout << std::endl;
    int len = dataLength+HGC_SERIAL_HEADER_LENGTH-5;
    int res = serial->writeBytes( serialBuffer, len);
    serial->flush();
//    int res = serial->writeBytes( buffer, HGC_SERIAL_MAX_BUFFER_SIZE);
    // if( serial->isConnected() ){
    //   printf(" --------> DATA PACKET FOR SPLITTER   ID=%i P_COUNT=%i   - (%i,%i) <-----------\n",splitterID, panelCnt, res,len);
    //   printHex(serialBuffer, len);
    // }

    // if( res == len ){
    //   printf("OK. (%i/%i)\n",res,len);
    // }else{
    //   printf("FAILED. (%i/%i)\n",res,len);
    // }
    return res == len;
}

bool nonoSerialModule::testSendToSplitter( unsigned char value, int splitterID, int panelCnt ){
    Serial* serial = &serialDevices[0];
    printf("-------------- TEST DATA FOR SPLITTER %i (%s) -------------- ", splitterID,serial->getDeviceName().c_str());
    int cnt=0;
    int dataLength = panelCnt * HGC_SERIAL_BYTER_PER_SPLITTER_PORT + 5;
    serialBuffer[cnt++] = HGC_SERIAL_HEADER;          // HEADER
    serialBuffer[cnt++] = dataLength & 0xff;          // PACKET LENGTH LOW
    serialBuffer[cnt++] = (dataLength>>8) & 0xff;     // PACKET LENGTH HIGH
    serialBuffer[cnt++] = splitterID;                 // Splitter ID
    serialBuffer[cnt++] = 0x01;                       // Command
    serialBuffer[cnt++] = panelCnt;                   // Panels count == port per splitter
    serialBuffer[cnt++] = HGC_SERIAL_BYTER_PER_SPLITTER_PORT & 0xff;        // bytes LOW => 30 LCDs + 1 divider
    serialBuffer[cnt++] = (HGC_SERIAL_BYTER_PER_SPLITTER_PORT>>8)&0xff;     // bytes HI <nbytes> is the number of bytes to send to each ports. For the LCD application this is 31 decimal (00+15 nodes x2 LCDs per node)
    serialBuffer[cnt++] = 0x00;                       // <mask> In conjunction with <nports=0> this selects a single port only instead of all ports. Only useful for test/programming.
    serialBuffer[cnt++] = 0x00;                       // <format> This should be 0 for the format used by the LCD nodes. (125kbaud, break to start)
    
    for( int i=0;i<panelCnt;i++){
      cnt++;
      memset(serialBuffer+cnt,value,30);
      cnt += 30;
    }
//    for( int i=0;i<dataLength+HGC_SERIAL_HEADER_LENGTH-5;i++ ){
//        printf(" %02i",buffer[i]);
//    }
//    cout << std::endl;
    int len = dataLength+HGC_SERIAL_HEADER_LENGTH-5;
    int res = serial->writeBytes( serialBuffer, len);
    serial->flush();

    if( res == len ){
      printf("OK. (%i/%i)\n",res,len);
    }else{
      printf("FAILED. (%i/%i)\n",res,len);
    }

    if( serial->isConnected() ){
      printf(" --------> DATA PACKET FOR SPLITTER   ID=%i P_COUNT=%i   - (%i,%i) <-----------\n",splitterID, panelCnt, res,len);
      printHex(serialBuffer, len);
    }
    return res == len;  
}


bool nonoSerialModule::testSendToSide( unsigned char value, int side ){
  Serial* serial = &serialDevices[0];
  printf("-------------- TEST DATA FOR SIDE %i (%s) -------------- ", side,serial->getDeviceName().c_str());
  memset(dataBuffer,value,HGC_AMOUNT_LCDS_PER_COLUMN);
  return sendDataToSide( serial, dataBuffer, (HGC_COLUMN_SIDES) side );
}



// ------------------------------------------------------------------------
// Serial Data
//


// ------------------------------------------------------------------------
// Loop
//

void nonoSerialModule::update(){

  // dataBuffer[0] = 0xff;
  // dataBuffer[1] = 0x00;  
  // for( int i=2;i<62;i++ ){
  //   dataBuffer[i] = (100+i+frameCnt)%254;
  // }

  // int res = serialDevices[0].writeBytes(dataBuffer,14);


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


    int res;
    int len;
    len = 30;//FRAME_SIZE;

    // for( int i=0;i<HGC_AMOUNT_LCDS_TOTAL;i++ ){
    //   dataBuffer[i] = 0;
    // }

    dataBuffer[0] = 0xff;
    // res = serialDevices[0].writeBytes( serialBuffer, len );
    // printf("SERIAL SENT : %i/%i    fd:%u   %s\n",res,len,serialDevices[0].getSerialFD(), serialDevices[0].getDeviceName());

    for( int i=0;i<serialDevices.size();i++){
      res = serialDevices[i].writeBytes( dataBuffer, len );
      // res = write( 5, serialBuffer, len );
      if( res != len ){
        printf("ERROR %i SERIAL SENT : %i/%i    fd:%u   %s\n",i,res,len,serialDevices[i].getSerialFD(), serialDevices[i].getDeviceName().c_str());
      }
    }
    
    printf("loop. frame:%i   data sent:%i\n",frameCnt,res);
    frameCnt++;

}
