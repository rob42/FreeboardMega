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
/*
 * SignalkHelper.cpp
 *
 *  Created on: Aug 2, 2014
 *      Author: robert
 */

#include "SignalkHelper.h"

 const char SignalkHelper::j_vessels[] PROGMEM = "vessels";
 const char  SignalkHelper::j_self[] PROGMEM = "self";
 const char  SignalkHelper::j_arduino[] PROGMEM = "_arduino";
 const char  SignalkHelper::j_airPressure[] PROGMEM = "airPressure";
 const char  SignalkHelper::j_airPressureChangeRateAlarm[] PROGMEM = "airPressureChangeRateAlarm";
 const char  SignalkHelper::j_airTemp[] PROGMEM = "airTemp";
 const char  SignalkHelper::j_alarm[] PROGMEM = "alarm";
 const char  SignalkHelper::j_alarmHeadingXte[] PROGMEM = "alarmHeadingXte";
 const char  SignalkHelper::j_alarmRadius[] PROGMEM = "alarmRadius";
 const char  SignalkHelper::j_alarms[] PROGMEM = "alarms";
 const char  SignalkHelper::j_altitude[] PROGMEM = "altitude";
 const char  SignalkHelper::j_anchor[] PROGMEM = "anchor";
 const char  SignalkHelper::j_anchorAlarmMethod[] PROGMEM = "anchorAlarmMethod";
 const char  SignalkHelper::j_anchorAlarmState[] PROGMEM = "anchorAlarmState";
 const char  SignalkHelper::j_autopilot[] PROGMEM = "autopilot";
 const char  SignalkHelper::j_autopilotAlarmMethod[] PROGMEM = "autopilotAlarmMethod";
 const char  SignalkHelper::j_autopilotAlarmState[] PROGMEM = "autopilotAlarmState";
 const char  SignalkHelper::j_average[] PROGMEM = "average";
 const char  SignalkHelper::j_backlash[] PROGMEM = "backlash";
 const char  SignalkHelper::j_baud0[] PROGMEM = "baud0";
 const char  SignalkHelper::j_baud1[] PROGMEM = "baud1";
 const char  SignalkHelper::j_baud2[] PROGMEM = "baud2";
 const char  SignalkHelper::j_baud3[] PROGMEM = "baud3";
 const char  SignalkHelper::j_baud4[] PROGMEM = "baud4";
 const char  SignalkHelper::j_baud5[] PROGMEM = "baud5";
 const char  SignalkHelper::j_baudRate[] PROGMEM = "baudRate";
 const char  SignalkHelper::j_bearingActual[] PROGMEM = "bearingActual";
 const char  SignalkHelper::j_bearingDirect[] PROGMEM = "bearingDirect";
 const char  SignalkHelper::j_belowKeel[] PROGMEM = "belowKeel";
 const char  SignalkHelper::j_belowSurface[] PROGMEM = "belowSurface";
 const char  SignalkHelper::j_belowTransducer[] PROGMEM = "belowTransducer";
 const char  SignalkHelper::j_status[] PROGMEM = "status";
 const char  SignalkHelper::j_courseOverGroundMagnetic[] PROGMEM = "courseOverGroundMagnetic";
 const char  SignalkHelper::j_courseOverGroundTrue[] PROGMEM = "courseOverGroundTrue";
 const char  SignalkHelper::j_courseRequired[] PROGMEM = "courseRequired";
 const char  SignalkHelper::j_currentDirection[] PROGMEM = "currentDirection";
 const char  SignalkHelper::j_currentRadius[] PROGMEM = "currentRadius";
 const char  SignalkHelper::j_currentRoute[] PROGMEM = "currentRoute";
 const char  SignalkHelper::j_currentSpeed[] PROGMEM = "currentSpeed";
 const char  SignalkHelper::j_deadZone[] PROGMEM = "deadZone";
 const char  SignalkHelper::j_decode[] PROGMEM = "decode";
 const char  SignalkHelper::j_depth[] PROGMEM = "depth";
 const char  SignalkHelper::j_destination[] PROGMEM = "destination";
 const char  SignalkHelper::j_directionApparent[] PROGMEM = "directionApparent";
 const char  SignalkHelper::j_directionChangeAlarm[] PROGMEM = "directionChangeAlarm";
 const char  SignalkHelper::j_directionTrue[] PROGMEM = "directionTrue";
 const char  SignalkHelper::j_drift[] PROGMEM = "drift";
 const char  SignalkHelper::j_east[] PROGMEM = "east";
 const char  SignalkHelper::j_engineAlarmMethod[] PROGMEM = "engineAlarmMethod";
 const char  SignalkHelper::j_engineAlarmState[] PROGMEM = "engineAlarmState";
 const char  SignalkHelper::j_environment[] PROGMEM = "environment";
 const char  SignalkHelper::j_eta[] PROGMEM = "eta";
 const char  SignalkHelper::j_factor[] PROGMEM = "factor";
 const char  SignalkHelper::j_fireAlarmMethod[] PROGMEM = "fireAlarmMethod";
 const char  SignalkHelper::j_fireAlarmState[] PROGMEM = "fireAlarmState";
 const char  SignalkHelper::j_gain[] PROGMEM = "gain";
 const char  SignalkHelper::j_gasAlarmMethod[] PROGMEM = "gasAlarmMethod";
 const char  SignalkHelper::j_gasAlarmState[] PROGMEM = "gasAlarmState";
 const char  SignalkHelper::j_genericAlarmMethod[] PROGMEM = "genericAlarmMethod";
 const char  SignalkHelper::j_genericAlarmState[] PROGMEM = "genericAlarmState";
 const char  SignalkHelper::j_gps[] PROGMEM = "gps";
 const char  SignalkHelper::j_gpsAlarmMethod[] PROGMEM = "gpsAlarmMethod";
 const char  SignalkHelper::j_gpsAlarmState[] PROGMEM = "gpsAlarmState";
 const char  SignalkHelper::j_headingMagnetic[] PROGMEM = "headingMagnetic";
 const char  SignalkHelper::j_headingSource[] PROGMEM = "headingSource";
 const char  SignalkHelper::j_headingTrue[] PROGMEM = "headingTrue";
 const char  SignalkHelper::j_heightHigh[] PROGMEM = "heightHigh";
 const char  SignalkHelper::j_heightLow[] PROGMEM = "heightLow";
 const char  SignalkHelper::j_heightNow[] PROGMEM = "heightNow";
 const char  SignalkHelper::j_humidity[] PROGMEM = "humidity";
 const char  SignalkHelper::j_last[] PROGMEM = "last";
 const char  SignalkHelper::j_lastFix[] PROGMEM = "lastFix";
 const char  SignalkHelper::j_lastTime[] PROGMEM = "lastTime";
 const char  SignalkHelper::j_lastUpdate[] PROGMEM = "lastUpdate";
 const char  SignalkHelper::j_latitude[] PROGMEM = "latitude";
 const char  SignalkHelper::j_level1[] PROGMEM = "level1";
 const char  SignalkHelper::j_level2[] PROGMEM = "level2";
 const char  SignalkHelper::j_level3[] PROGMEM = "level3";
 const char  SignalkHelper::j_upper[] PROGMEM = "upper";
 const char  SignalkHelper::j_lower[] PROGMEM = "lower";
 const char  SignalkHelper::j_longitude[] PROGMEM = "longitude";
 const char  SignalkHelper::j_magneticVariation[] PROGMEM = "magneticVariation";
 const char  SignalkHelper::j_max[] PROGMEM = "max";
 const char  SignalkHelper::j_maxDriveAmps[] PROGMEM = "maxDriveAmps";
 const char  SignalkHelper::j_maxDriveRate[] PROGMEM = "maxDriveRate";
 const char  SignalkHelper::j_maxRadius[] PROGMEM = "maxRadius";
 const char  SignalkHelper::j_maydayAlarmMethod[] PROGMEM = "maydayAlarmMethod";
 const char  SignalkHelper::j_maydayAlarmState[] PROGMEM = "maydayAlarmState";
 const char  SignalkHelper::j_mobAlarmMethod[] PROGMEM = "mobAlarmMethod";
 const char  SignalkHelper::j_mobAlarmState[] PROGMEM = "mobAlarmState";
 const char  SignalkHelper::j_mode[] PROGMEM = "mode";
 const char  SignalkHelper::j_model[] PROGMEM = "model";
 const char  SignalkHelper::j_navigation[] PROGMEM = "navigation";
 const char  SignalkHelper::j_next[] PROGMEM = "next";
 const char  SignalkHelper::j_nextEta[] PROGMEM = "nextEta";
 const char  SignalkHelper::j_north[] PROGMEM = "north";
 const char  SignalkHelper::j_offcourse[] PROGMEM = "offcourse";
 const char  SignalkHelper::j_panpanAlarmMethod[] PROGMEM = "panpanAlarmMethod";
 const char  SignalkHelper::j_panpanAlarmState[] PROGMEM = "panpanAlarmState";
 const char  SignalkHelper::j_pitch[] PROGMEM = "pitch";
 const char  SignalkHelper::j_portLock[] PROGMEM = "portLock";
 const char  SignalkHelper::j_position[] PROGMEM = "position";
 const char  SignalkHelper::j_powerAlarmMethod[] PROGMEM = "powerAlarmMethod";
 const char  SignalkHelper::j_powerAlarmState[] PROGMEM = "powerAlarmState";
 const char  SignalkHelper::j_radarAlarmMethod[] PROGMEM = "radarAlarmMethod";
 const char  SignalkHelper::j_radarAlarmState[] PROGMEM = "radarAlarmState";
 const char  SignalkHelper::j_radiusDeg[] PROGMEM = "radiusDeg";
 const char  SignalkHelper::j_rateOfTurn[] PROGMEM = "rateOfTurn";
 const char  SignalkHelper::j_roll[] PROGMEM = "roll";
 const char  SignalkHelper::j_route[] PROGMEM = "route";
 const char  SignalkHelper::j_rudderAngle[] PROGMEM = "rudderAngle";
 const char  SignalkHelper::j_rudderAngleTarget[] PROGMEM = "rudderAngleTarget";
 const char  SignalkHelper::j_rudderCommand[] PROGMEM = "rudderCommand";
 const char  SignalkHelper::j_salinity[] PROGMEM = "salinity";
 const char  SignalkHelper::j_seatalk[] PROGMEM = "seatalk";
 const char  SignalkHelper::j_serial[] PROGMEM = "serial";
 const char  SignalkHelper::j_set[] PROGMEM = "set";
 const char  SignalkHelper::j_silentInterval[] PROGMEM = "silentInterval";
 const char  SignalkHelper::j_snooze[] PROGMEM = "snooze";
 const char  SignalkHelper::j_south[] PROGMEM = "south";
 const char  SignalkHelper::j_speedAlarm[] PROGMEM = "speedAlarm";
 const char  SignalkHelper::j_speedApparent[] PROGMEM = "speedApparent";
const char  SignalkHelper::j_speedOverGround[] PROGMEM = "speedOverGround";
const char  SignalkHelper::j_speedThroughWater[] PROGMEM = "speedThroughWater";
const char  SignalkHelper::j_speedTrue[] PROGMEM = "speedTrue";
const char  SignalkHelper::j_starboardLock[] PROGMEM = "starboardLock";
const char  SignalkHelper::j_startTime[] PROGMEM = "startTime";
const char  SignalkHelper::j_state[] PROGMEM = "state";
const char  SignalkHelper::j_steering[] PROGMEM = "steering";
const char  SignalkHelper::j_surfaceToTransducer[] PROGMEM = "surfaceToTransducer";
const char  SignalkHelper::j_targetHeadingMagnetic[] PROGMEM = "targetHeadingMagnetic";
const char  SignalkHelper::j_targetHeadingNorth[] PROGMEM = "targetHeadingNorth";
const char  SignalkHelper::j_tide[] PROGMEM = "tide";
const char  SignalkHelper::j_timeHigh[] PROGMEM = "timeHigh";
const char  SignalkHelper::j_timeLow[] PROGMEM = "timeLow";
const char  SignalkHelper::j_transducerToKeel[] PROGMEM = "transducerToKeel";
const char  SignalkHelper::j_utc[] PROGMEM = "utc";
const char  SignalkHelper::j_waterTemp[] PROGMEM = "waterTemp";
const char  SignalkHelper::j_waypoint[] PROGMEM = "waypoint";
const char  SignalkHelper::j_west[] PROGMEM = "west";
const char  SignalkHelper::j_wind[] PROGMEM = "wind";
const char  SignalkHelper::j_windAlarmMethod[] PROGMEM = "windAlarmMethod";
const char  SignalkHelper::j_windAlarmState[] PROGMEM = "windAlarmState";
const char  SignalkHelper::j_xte[] PROGMEM = "xte";
const char  SignalkHelper::j_zeroOffset[] PROGMEM = "zeroOffset";


SignalkHelper::SignalkHelper() {

}

int SignalkHelper::findInArray(const char *array[], const char* value) {

	for (unsigned int i = 0; i < (sizeof(array) / sizeof(array[0])); i++) {
		if ((strcmp(array[i], value) == 0)) {
			return i;
		}
	}
	return -1;
}
unsigned long SignalkHelper::hash(const char *str) {
	unsigned long hash = 5381;
	int c;

	while ((c = *str++))
		hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

	return hash;
}

void SignalkHelper::openMessage(HardwareSerial* serial){
	serial->print("{");
}
void SignalkHelper::closeMessage(HardwareSerial* serial){
	serial->println("}");
}
void SignalkHelper::openBranch(HardwareSerial* serial, const char* key){
	serial->print("\"");
	print_PROGMEM(serial, key);
	serial->print("\":{");
}
void SignalkHelper::closeBranch(HardwareSerial* serial, bool last){
	serial->print("}");
	if(!last)serial->print(",");
}

void SignalkHelper::printValue(HardwareSerial* serial, const char* key, const float value, bool last){
	serial->print("\"");
	print_PROGMEM(serial, key);
	serial->print("\":");
	if( value!=value){
		serial->print("null");
	}else{
		serial->print(value,DEC);
	}
	if(!last)serial->print(",");
}
void SignalkHelper::printValue(HardwareSerial* serial, const char* key, const unsigned long value, bool last){
	serial->print("\"");
	print_PROGMEM(serial, key);
	serial->print("\":");
	if(value==SIZE_MAX){
			serial->print("null");
		}else{
			serial->print(value,DEC);
		}
	if(!last)serial->print(",");
}

void SignalkHelper::printValue(HardwareSerial* serial, const char* key, const int value, bool last){
	serial->print("\"");
	print_PROGMEM(serial, key);
	serial->print("\":");
	if(value==SIZE_MAX){
			serial->print("null");
		}else{
			serial->print(value,DEC);
		}
	if(!last)serial->print(",");
}
void SignalkHelper::printValue(HardwareSerial* serial, const char* key, const long value, bool last){
	serial->print("\"");
	print_PROGMEM(serial, key);
	serial->print("\":");
	if(value==SIZE_MAX){
			serial->print("null");
		}else{
			serial->print(value,DEC);
		}
	if(!last)serial->print(",");
}
void SignalkHelper::printValue(HardwareSerial* serial, const char* key, const bool value, bool last){
	serial->print("\"");
	print_PROGMEM(serial, key);
	serial->print("\":");
	if(value){
			serial->print("true");
	}else{
		serial->print("false");
	}
	if(!last)serial->print(",");
}
void SignalkHelper::printValue(HardwareSerial* serial, const char* key,  const char* value, bool last){
	serial->print("\"");
	print_PROGMEM(serial, key);
	serial->print("\":\"");
	serial->print(value);
	serial->print("\"");
	if(!last)serial->print(",");
}
void SignalkHelper::printValue(HardwareSerial* serial, const char* key,  const char value, bool last){
	serial->print("\"");
	print_PROGMEM(serial, key);
	//serial->print(key);
	serial->print("\":");
	serial->print(value);
	if(!last)serial->print(",");
}

void SignalkHelper::print_PROGMEM(HardwareSerial* serial, const char* key){
	//PROGMEM
	char c;
	while((c = pgm_read_byte(key++))){
	   serial->write(c);
	}
}

byte SignalkHelper::getChecksum(char* str) {
		byte cs = 0; //clear any old checksum
		for (unsigned int n = 1; n < strlen(str) - 1; n++) {
			cs ^= str[n]; //calculates the checksum
		}
		return cs;
	}

int SignalkHelper::getNumericType(unsigned long key){
	switch (key) {
		case NAVIGATION_DESTINATION_ETA: return TYPE_UNSIGNED_LONG;break;
		case STEERING_AUTOPILOT_GAIN: return TYPE_INT;break;
		case _ARDUINO_GPS_MODEL: return TYPE_INT;break;
		case _ARDUINO_GPS_LASTFIX: return TYPE_UNSIGNED_LONG;break;
		case _ARDUINO_GPS_UTC: return TYPE_UNSIGNED_LONG;break;
		case _ARDUINO_GPS_DECODE: return TYPE_BOOL;break;
		case _ARDUINO_AUTOPILOT_BAUDRATE: return TYPE_UNSIGNED_LONG;break;
		case _ARDUINO_AUTOPILOT_OFFCOURSE: return TYPE_DOUBLE;break;
		case _ARDUINO_AUTOPILOT_RUDDERCOMMAND: return TYPE_DOUBLE;break;
		case _ARDUINO_ALARM_LAST: return TYPE_UNSIGNED_LONG;break;
		case _ARDUINO_ALARM_SNOOZE: return TYPE_UNSIGNED_LONG;break;
		case _ARDUINO_WIND_LASTUPDATE: return TYPE_UNSIGNED_LONG;break;
		case ALARMS_SILENTINTERVAL: return TYPE_INT;break;
		case _ARDUINO_GPS_STATUS: return TYPE_CHAR;break;
		//all char array, which we know anyway, or float
		default: return TYPE_FLOAT;

	/* - all char array, which we know, or float
	 * case ALARMS_ANCHORALARMMETHOD: return TYPE_CHAR_ARRAY;break;
		case ALARMS_ANCHORALARMSTATE: return TYPE_CHAR_ARRAY;break;
		case ALARMS_AUTOPILOTALARMMETHOD: return TYPE_CHAR_ARRAY;break;
		case ALARMS_AUTOPILOTALARMSTATE: return TYPE_CHAR_ARRAY;break;
		case ALARMS_ENGINEALARMMETHOD: return TYPE_CHAR_ARRAY;break;
		case ALARMS_ENGINEALARMSTATE: return TYPE_CHAR_ARRAY;break;
		case ALARMS_FIREALARMMETHOD: return TYPE_CHAR_ARRAY;break;
		case ALARMS_FIREALARMSTATE: return TYPE_CHAR_ARRAY;break;
		case ALARMS_GASALARMMETHOD: return TYPE_CHAR_ARRAY;break;
		case ALARMS_GASALARMSTATE: return TYPE_CHAR_ARRAY;break;
		case ALARMS_GPSALARMMETHOD: return TYPE_CHAR_ARRAY;break;
		case ALARMS_GPSALARMSTATE: return TYPE_CHAR_ARRAY;break;
		case ALARMS_MAYDAYALARMMETHOD: return TYPE_CHAR_ARRAY;break;
		case ALARMS_MAYDAYALARMSTATE: return TYPE_CHAR_ARRAY;break;
		case ALARMS_PANPANALARMMETHOD: return TYPE_CHAR_ARRAY;break;
		case ALARMS_PANPANALARMSTATE: return TYPE_CHAR_ARRAY;break;
		case ALARMS_POWERALARMMETHOD: return TYPE_CHAR_ARRAY;break;
		case ALARMS_POWERALARMSTATE: return TYPE_CHAR_ARRAY;break;
		case ALARMS_WINDALARMMETHOD: return TYPE_CHAR_ARRAY;break;
		case ALARMS_WINDALARMSTATE: return TYPE_CHAR_ARRAY;break;
		case ALARMS_GENERICALARMMETHOD: return TYPE_CHAR_ARRAY;break;
		case ALARMS_GENERICALARMSTATE: return TYPE_CHAR_ARRAY;break;
		case ALARMS_RADARALARMMETHOD: return TYPE_CHAR_ARRAY;break;
		case ALARMS_RADARALARMSTATE: return TYPE_CHAR_ARRAY;break;
		case ALARMS_MOBALARMMETHOD: return TYPE_CHAR_ARRAY;break;
		case ALARMS_MOBALARMSTATE: return TYPE_CHAR_ARRAY;break;
		case NAVIGATION_STATE: return TYPE_CHAR_ARRAY;break;
		case STEERING_AUTOPILOT_STATE: return TYPE_CHAR_ARRAY;break;
		case STEERING_AUTOPILOT_MODE: return TYPE_CHAR_ARRAY;break;
		case STEERING_AUTOPILOT_HEADINGSOURCE: return TYPE_CHAR_ARRAY;break;

	 * these are now all float so = 0
		case NAVIGATION_COURSEOVERGROUNDMAGNETIC:return TYPE_FLOAT;break;
		case NAVIGATION_COURSEOVERGROUNDTRUE: return TYPE_FLOAT;break;
		case NAVIGATION_MAGNETICVARIATION: return TYPE_FLOAT;break;
		case NAVIGATION_DESTINATION_LONGITUDE: return TYPE_FLOAT;break;
		case NAVIGATION_DESTINATION_LATITUDE: return TYPE_FLOAT;break;

		case NAVIGATION_HEADINGMAGNETIC: return TYPE_FLOAT;break;
		case NAVIGATION_HEADINGTRUE: return TYPE_FLOAT;break;
		case NAVIGATION_POSITION_LONGITUDE: return TYPE_FLOAT;break;
		case NAVIGATION_POSITION_LATITUDE: return TYPE_FLOAT;break;
		case NAVIGATION_POSITION_ALTITUDE: return TYPE_FLOAT;break;
		case NAVIGATION_PITCH: return TYPE_FLOAT;break;
		case NAVIGATION_RATEOFTURN: return TYPE_FLOAT;break;
		case NAVIGATION_ROLL: return TYPE_FLOAT;break;
		case NAVIGATION_SPEEDOVERGROUND: return TYPE_FLOAT;break;
		case NAVIGATION_SPEEDTHROUGHWATER: return TYPE_FLOAT;break;

		case NAVIGATION_ANCHOR_ALARMRADIUS: return TYPE_FLOAT;break;
		case NAVIGATION_ANCHOR_MAXRADIUS: return TYPE_FLOAT;break;
		case NAVIGATION_ANCHOR_CURRENTRADIUS: return TYPE_FLOAT;break;
		case NAVIGATION_ANCHOR_POSITION_ALTITUDE: return TYPE_FLOAT;break;
		case NAVIGATION_ANCHOR_POSITION_LATITUDE: return TYPE_FLOAT;break;
		case NAVIGATION_ANCHOR_POSITION_LONGITUDE: return TYPE_FLOAT;break;


		case STEERING_AUTOPILOT_TARGETHEADINGNORTH: return TYPE_FLOAT;break;
		case STEERING_AUTOPILOT_TARGETHEADINGMAGNETIC: return TYPE_FLOAT;break;
		case STEERING_AUTOPILOT_ALARMHEADINGXTE: return TYPE_FLOAT;break;

		case STEERING_AUTOPILOT_DEADZONE: return TYPE_FLOAT;break;
		case STEERING_AUTOPILOT_BACKLASH: return TYPE_FLOAT;break;

		case STEERING_AUTOPILOT_MAXDRIVEAMPS: return TYPE_FLOAT;break;
		case STEERING_AUTOPILOT_MAXDRIVERATE: return TYPE_FLOAT;break;
		case STEERING_AUTOPILOT_PORTLOCK: return TYPE_FLOAT;break;
		case STEERING_AUTOPILOT_STARBOARDLOCK: return TYPE_FLOAT;break;
		case STEERING_RUDDERANGLE: return TYPE_FLOAT;break;
		case STEERING_RUDDERANGLETARGET: return TYPE_FLOAT;break;



		case ENVIRONMENT_WIND_DIRECTIONAPPARENT: return TYPE_FLOAT;break;
		case ENVIRONMENT_WIND_DIRECTIONCHANGEALARM: return TYPE_FLOAT;break;
		case ENVIRONMENT_WIND_DIRECTIONTRUE: return TYPE_FLOAT;break;
		case ENVIRONMENT_WIND_SPEEDALARM: return TYPE_FLOAT;break;
		case ENVIRONMENT_WIND_SPEEDTRUE: return TYPE_FLOAT;break;
		case ENVIRONMENT_WIND_SPEEDAPPARENT: return TYPE_FLOAT;break;
		case ENVIRONMENT_AIRPRESSURECHANGERATEALARM: return TYPE_FLOAT;break;
		case ENVIRONMENT_AIRPRESSURE: return TYPE_FLOAT;break;
		case ENVIRONMENT_WATERTEMP: return TYPE_FLOAT;break;


		case _ARDUINO_SERIAL_BAUD0: return TYPE_FLOAT;break;
		case _ARDUINO_SERIAL_BAUD1: return TYPE_FLOAT;break;
		case _ARDUINO_SERIAL_BAUD2: return TYPE_FLOAT;break;
		case _ARDUINO_SERIAL_BAUD3: return TYPE_FLOAT;break;
		case _ARDUINO_SERIAL_BAUD4: return TYPE_FLOAT;break;
		case _ARDUINO_SERIAL_BAUD5: return TYPE_FLOAT;break;
		case _ARDUINO_ALARM_LEVEL1_UPPER: return TYPE_FLOAT;break;
		case _ARDUINO_ALARM_LEVEL1_LOWER: return TYPE_FLOAT;break;
		case _ARDUINO_ALARM_LEVEL2_UPPER: return TYPE_FLOAT;break;
		case _ARDUINO_ALARM_LEVEL2_LOWER: return TYPE_FLOAT;break;
		case _ARDUINO_ALARM_LEVEL3_UPPER: return TYPE_FLOAT;break;
		case _ARDUINO_ALARM_LEVEL3_LOWER: return TYPE_FLOAT;break;
		case _ARDUINO_SEATALK: return TYPE_FLOAT;break;
		case _ARDUINO_WIND_ZEROOFFSET: return TYPE_FLOAT;break;
		case _ARDUINO_WIND_LASTUPDATE: return TYPE_FLOAT;break;

		case _ARDUINO_WIND_AVERAGE: return TYPE_FLOAT;break;
		case _ARDUINO_WIND_FACTOR: return TYPE_FLOAT;break;
		case _ARDUINO_WIND_MAX: return TYPE_FLOAT;break;

		case _ARDUINO_ANCHOR_RADIUSDEG: return TYPE_FLOAT;break;
		case _ARDUINO_ANCHOR_NORTH: return TYPE_FLOAT;break;
		case _ARDUINO_ANCHOR_SOUTH: return TYPE_FLOAT;break;
		case _ARDUINO_ANCHOR_EAST: return TYPE_FLOAT;break;
		case _ARDUINO_ANCHOR_WEST: return TYPE_FLOAT;break;
		*/
	}
return TYPE_FLOAT;
}
