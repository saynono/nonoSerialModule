/*
 * nonoSerial is based on
 * wiringSerial.h:
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
 */


#pragma once

#include <string>


namespace nono
{
	namespace com
	{
		class Serial
		{

		public:
			Serial();
			int   openPort      (const char *device, const int baud) ;
			void  closePort     () ;
			void  flush     () ;
			void  writeByte   (const unsigned char c) ;
			int  writeString       (const char *s) ;
			int  writeBytes		  (const unsigned char* data, int len );
			int  readBytes 		  (unsigned char* data, int len );
			// void  serialPrintf    (const int fd, const char *message, ...) ;
			int  dataAvailable () ;
			// int  serialGetchar   () ;
			int  getSerialFD();
			bool isConnected();
			const std::string getDeviceName();

		private:

			int fd;
			std::string deviceName;
			
		};
  } // namespace com
} // namespace nono


