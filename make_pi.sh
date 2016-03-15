
OUTPUTNAME=nonoSerialModule_UDP_pi

projectFolder="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $projectFolder
mkdir tmp

g++ -c -o ./tmp/nonoSerial.o ./src/nonoSerial.cpp -Wall -pedantic;
g++ -c -o ./tmp/nonoSerialModule.o ./src/nonoSerialModule.cpp  -Wall -pedantic;
g++ -c -o ./tmp/nonoSerialModule_UDP_Wrapper.o ./src/nonoSerialModule_UDP_Wrapper.cpp -Wall -pedantic;
g++ ./tmp/nonoSerialModule.o ./tmp/nonoSerialModule_UDP_Wrapper.o ./tmp/nonoSerial.o -o ./bin/$OUTPUTNAME -lm -Wall -pedantic; 

rm -rf ./tmp
