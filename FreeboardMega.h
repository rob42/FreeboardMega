/*
 * Copyright 2010,2011,2012,2013 Robert Huitema robert@42.co.nz
 *
 * This file is part of Freeboard. (http://www.42.co.nz/freeboard)
 *
 *  Freeboard is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.

 *  Freeboard is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with Freeboard.  If not, see <http://www.gnu.org/licenses/>.
 */

// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef FREEBOARDMEGA_H_
#define FREEBOARDMEGA_H_

typedef unsigned char byte;

#include "Arduino.h"
//add your includes for the project  here
#include "FreeboardConstants.h"
#include <EEPROM.h>
#include <MultiSerial.h>
#include <FlexiTimer2.h>
#include "Alarm.h"
#include "Seatalk.h"
#include "Wind.h"
#include "Gps.h"
#include "Anchor.h"
#include "NmeaSerial.h"
#include "Autopilot.h"
#include <stream_json_reader.h>
#include <MemoryFree.h>
#include "NmeaRelay.h"
#include "FreeboardModel.h"

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif



//add your function definitions for the project  here
void check_mem();
int freeRam();
void readWDS();
void readWDD();

void calculate();
void process(char * s, char parser);
byte getChecksum(char* str);

//Do not add code below this line
#endif /* FREEBOARDMEGA_H_*/
