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
#define SERIAL_DATA_BUFFER_SIZE       2048


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
  
  bool sendDataToSerial( Serial* serial, unsigned char* data, int size );
  // bool sendDataToSplitter( Serial* serial, int splitterID, int panelCnt );
  // void update();

  bool testSendToSerial( unsigned char value );

private:

  unsigned char dataBuffer[SERIAL_DATA_BUFFER_SIZE];
  unsigned char serialBuffer[SERIAL_DATA_BUFFER_SIZE];
  // Serial serialDevices[HGC_SERIAL_DEVICE_CNT];

  std::vector<Serial> serialDevices;

  int frameCnt;

};

  } // namespace project
} // namespace nono

