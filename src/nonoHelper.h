


// //include system librarys
// #include <stdio.h> //for printf
// #include <stdint.h> //uint8_t definitions
// #include <stdlib.h> //for exit(int);
// #include <string.h> //for errno
// #include <errno.h> //error output

// #include <math.h>
// #include <algorithm>    // std::max

// #include <unistd.h>
// #include <fcntl.h>
// #include <signal.h>

#pragma once

#include <stdio.h> //for printf
#include <stdint.h> //uint8_t definitions
#include <stdlib.h> //for exit(int);
#include <string.h> //for errno
#include <errno.h> //error output

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <termios.h>

/*
static uint64_t epochMilli;

unsigned int millis (void)
{
  struct timeval tv ;
  uint64_t now ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ;

  return (uint32_t)(now - epochMilli) ;
}
*/

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
