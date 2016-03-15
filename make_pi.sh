g++ -c -o nonoSerial.o nonoSerial.cpp -Wall -pedantic;
# g++ -c -o wiringSerial.o wiringSerial.c -Wall -pedantic;
g++ -c -o HGC_SerialModule.o HGC_SerialModule.cpp  -Wall -pedantic;
g++ HGC_SerialModule.o nonoSerial.o -o HGC_SerialModule_pi -lm -Wall -pedantic; 
rm *.o;

#gcc -o HGC_Serial_multi HGC_Serial_multi.cpp -pedantic -Wall; 
