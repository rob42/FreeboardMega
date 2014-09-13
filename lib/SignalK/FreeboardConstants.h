/*
 * Copyright 2010,2011,2012,2013 Robert Huitema robert@42.co.nz
 *
 * This file is part of FreeBoard. (http://www.42.co.nz/freeboard)
 *
 *  FreeBoard is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.

 *  FreeBoard is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with FreeBoard.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef FREEBOARDCONSTANTS_H_
#define FREEBOARDCONSTANTS_H_

//debug
#define DEBUG true
//enable mux onto Serial.print
#define MUX true

#define SIZE_MAX 	4294967295UL
#define INT16_MIN 	-32768
#define INT16_MAX 	 32767
#define UINT16_MAX 	 65535
#define INT32_MIN 	 (-2147483647L-1)
#define INT32_MAX 	 2147483647L
#define UINT32_MAX   4294967295UL
//uncomment to support different GPS
#define GPS_GENERIC 0
#define GPS_EM_406A 1
#define GPS_MTEK_3329 2
//EM406A pin 3 = RX to arduino TX - pin18, RX to pin19
//GPS pins are Serial1
#define GPS_RX_PIN 19
#define GPS_TX_PIN 18

//Gps STATUS, V=Navigation receiver warning A=Valid
#define GPS_WARN 'V'
#define GPS_VALID 'A'

//SPI
#define MISO_PIN 50
#define M0SI_PIN 51
#define SLCK_PIN 52
#define SS_PIN 53 //should always be OUTPUT as arduino is master only
#define CS_PIN 67 //analog A13

//CAN bus controller
#define CANRX A14
#define CANTX A15

//misc output - AltSoftSerial
//mux NMEA output
#define nmeaRxPin 48
#define nmeaTxPin 46

//autopilot output
#define autopilotRxPin 34
#define autopilotTxPin 36
#define autopilotEngagePin 22

// the number of the buzzer pin

// need reliable efficient interrupts
//Wind speed on pin 3 - INT1 - yellow wire
 //Wind dir on pin 2 - INT0
#define windSpeedPin 3 // pin3
#define windSpeedInterrupt 1 // INT1
#define windDirPin 2 // pin 2
#define windDirInterrupt 0 // INT0
#define logPin 20 // pin20
//#define logInterrupt 3 // INT3
//#define logPin 21 // pin21 //INT 2

//attached device types
#define UID  "UID"
#define IMU  "IMU"
#define MEGA  "MEGA"


#endif
