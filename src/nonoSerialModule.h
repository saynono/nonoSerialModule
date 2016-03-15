#pragma once

#include "nonoSerial.h"

#include <vector>


#define HGC_AMOUNT_LCDS_PER_MODULE    2
#define HGC_AMOUNT_SIDES              3
#define HGC_AMOUNT_PANELS_PER_COLUMN  22
#define HGC_AMOUNT_LCDS_PER_PANEL     30
#define HGC_AMOUNT_LCDS_PER_COLUMN    (HGC_AMOUNT_PANELS_PER_COLUMN*HGC_AMOUNT_LCDS_PER_PANEL)
#define HGC_AMOUNT_LCDS_TOTAL         (HGC_AMOUNT_LCDS_PER_COLUMN*HGC_AMOUNT_SIDES)

#define HGC_SERIAL_BAUDRATE           250000
// #define HGC_SERIAL_BAUDRATE           115200

// #define HGC_SERIAL_DEVICE_CNT         1
#define HGC_SERIAL_BUFFER_SIZE        2048
#define HGC_SERIAL_HEADER             0xC5
#define HGC_SERIAL_HEADER_LENGTH      10
#define HGC_SERIAL_BYTER_PER_SPLITTER_PORT 31


namespace nono
{
  namespace com
  {
    

class nonoSerialModule {

public:


  enum HGC_COLUMN_SIDES{
    HGC_COLUMN_A = 0,
    HGC_COLUMN_B,
    HGC_COLUMN_C
  };


  nonoSerialModule();
  void setup();
  void close();
  bool sendCompleteData( unsigned char* data );  
  bool sendDataToSide( Serial* serial, unsigned char* data, HGC_COLUMN_SIDES side );
  bool sendDataToSplitter( Serial* serial, int splitterID, int panelCnt );
  void update();

  bool testSendToSplitter( unsigned char value, int splitterID=1, int panelCnt=6 );
  bool testSendToSide( unsigned char value, int side=0 );

private:

  unsigned char dataBuffer[HGC_AMOUNT_LCDS_TOTAL];
  unsigned char serialBuffer[HGC_SERIAL_BUFFER_SIZE];
  // Serial serialDevices[HGC_SERIAL_DEVICE_CNT];

  std::vector<Serial> serialDevices;

  int frameCnt;

};

  } // namespace project
} // namespace nono

