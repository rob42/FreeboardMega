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
 * SignalkHelper.h
 *
 *Holds all the global model vars
 *
 *  Created on: Mar 28, 2012
 *      Author: robert
 */

#ifndef SIGNALKHELPER_H_
#define SIGNALKHELPER_H_
#define DEBUG true
#include "Arduino.h"

#define SIZE_MAX 	4294967295UL
#define INT16_MIN 	-32768
#define INT16_MAX 	 32767
#define UINT16_MAX 	 65535
#define INT32_MIN 	 (-2147483647L-1)
#define INT32_MAX 	 2147483647L
#define UINT32_MAX   4294967295UL
#define TYPE_FLOAT  0
#define TYPE_BOOL  1
#define TYPE_INT  2
#define TYPE_UNSIGNED_LONG  3
#define TYPE_DOUBLE  4 //same as float on mega
#define TYPE_CHAR_ARRAY 5
#define TYPE_CHAR 6
//#include "StreamJsonReader.h"
//#include "FreeboardConstants.h"
#define  NAVIGATION_COURSEOVERGROUNDMAGNETIC 177632231UL
#define  NAVIGATION_COURSEOVERGROUNDTRUE 1495033343UL
#define  NAVIGATION_MAGNETICVARIATION 3733725560UL
#define  NAVIGATION_DESTINATION_LONGITUDE 250998974UL
#define  NAVIGATION_DESTINATION_LATITUDE 2077664687UL
#define  NAVIGATION_DESTINATION_ETA 3951685101UL
#define  NAVIGATION_HEADINGMAGNETIC 390698939UL
#define  NAVIGATION_HEADINGTRUE 1129234387UL
#define  NAVIGATION_POSITION_LONGITUDE 4257320113UL
#define  NAVIGATION_POSITION_LATITUDE 2329218882UL
#define  NAVIGATION_POSITION_ALTITUDE 2999027170UL
#define  NAVIGATION_PITCH 2242652731UL
#define  NAVIGATION_RATEOFTURN 1051834157UL
#define  NAVIGATION_ROLL 588639420UL
#define  NAVIGATION_SPEEDOVERGROUND 3375602335UL
#define  NAVIGATION_SPEEDTHROUGHWATER 2011270296UL
#define  NAVIGATION_STATE 2246585668UL
#define  NAVIGATION_ANCHOR_ALARMRADIUS 3204687649UL
#define  NAVIGATION_ANCHOR_MAXRADIUS 3688841722UL
#define  NAVIGATION_ANCHOR_CURRENTRADIUS 2152304183UL
#define  NAVIGATION_ANCHOR_POSITION_ALTITUDE 3067744843UL
#define  NAVIGATION_ANCHOR_POSITION_LATITUDE 2397936555UL
#define  NAVIGATION_ANCHOR_POSITION_LONGITUDE 2230036026UL

#define  STEERING_AUTOPILOT_STATE 1414591300UL
#define  STEERING_AUTOPILOT_MODE 3296408520UL
#define  STEERING_AUTOPILOT_TARGETHEADINGNORTH 1763947237UL
#define  STEERING_AUTOPILOT_TARGETHEADINGMAGNETIC 542499234UL
#define  STEERING_AUTOPILOT_ALARMHEADINGXTE 2225422449UL
#define  STEERING_AUTOPILOT_HEADINGSOURCE 1161926884UL
#define  STEERING_AUTOPILOT_DEADZONE 3691686605UL
#define  STEERING_AUTOPILOT_BACKLASH 3570446012UL
#define  STEERING_AUTOPILOT_GAIN 3296177826UL
#define  STEERING_AUTOPILOT_MAXDRIVEAMPS 3211458388UL
#define  STEERING_AUTOPILOT_MAXDRIVERATE 3212056367UL
#define  STEERING_AUTOPILOT_PORTLOCK 430477553UL
#define  STEERING_AUTOPILOT_STARBOARDLOCK 4221374702UL
#define  STEERING_RUDDERANGLE 1770208225UL
#define  STEERING_RUDDERANGLETARGET 1589626152UL

#define  ALARMS_ANCHORALARMMETHOD 2783435100UL
#define  ALARMS_ANCHORALARMSTATE 1003034460UL
#define  ALARMS_AUTOPILOTALARMMETHOD 1469117826UL
#define  ALARMS_AUTOPILOTALARMSTATE 3305916098UL
#define  ALARMS_ENGINEALARMMETHOD 3992095383UL
#define  ALARMS_ENGINEALARMSTATE 2991918391UL
#define  ALARMS_FIREALARMMETHOD 133189895UL
#define  ALARMS_FIREALARMSTATE 662422951UL
#define  ALARMS_GASALARMMETHOD 3929525820UL
#define  ALARMS_GASALARMSTATE 3380473916UL
#define  ALARMS_GPSALARMMETHOD 2429115595UL
#define  ALARMS_GPSALARMSTATE 2814404843UL
#define  ALARMS_MAYDAYALARMMETHOD 67311014UL
#define  ALARMS_MAYDAYALARMSTATE 1441329766UL
#define  ALARMS_PANPANALARMMETHOD 1695331519UL
#define  ALARMS_PANPANALARMSTATE 3963523679UL
#define  ALARMS_POWERALARMMETHOD 3887776462UL
#define  ALARMS_POWERALARMSTATE 1296800398UL
#define  ALARMS_SILENTINTERVAL 357415143UL
#define  ALARMS_WINDALARMMETHOD 1899960179UL
#define  ALARMS_WINDALARMSTATE 3449122451UL
#define  ALARMS_GENERICALARMMETHOD 3742841118UL
#define  ALARMS_GENERICALARMSTATE 1422558942UL
#define  ALARMS_RADARALARMMETHOD 1862050059UL
#define  ALARMS_RADARALARMSTATE 2927371563UL
#define  ALARMS_MOBALARMMETHOD 1690466719UL
#define  ALARMS_MOBALARMSTATE 2011118399UL

#define  ENVIRONMENT_WIND_DIRECTIONAPPARENT 3267532580UL
#define  ENVIRONMENT_WIND_DIRECTIONCHANGEALARM 4209206332UL
#define  ENVIRONMENT_WIND_DIRECTIONTRUE 1752542857UL
#define  ENVIRONMENT_WIND_SPEEDALARM 2559123686UL
#define  ENVIRONMENT_WIND_SPEEDTRUE 1900346521UL
#define  ENVIRONMENT_WIND_SPEEDAPPARENT 531262772UL
#define  ENVIRONMENT_AIRPRESSURECHANGERATEALARM 2231147356UL
#define  ENVIRONMENT_AIRPRESSURE 2568414557UL
#define  ENVIRONMENT_WATERTEMP 1380637633UL

#define  _ARDUINO_GPS_MODEL 260479757UL
#define  _ARDUINO_SERIAL_BAUD0 1806063390UL
#define  _ARDUINO_SERIAL_BAUD1 1806063391UL
#define  _ARDUINO_SERIAL_BAUD2 1806063392UL
#define  _ARDUINO_SERIAL_BAUD3 1806063393UL
#define  _ARDUINO_SERIAL_BAUD4 1806063394UL
#define  _ARDUINO_SERIAL_BAUD5 1806063395UL
#define  _ARDUINO_ALARM_LEVEL1_UPPER 2104985698UL
#define  _ARDUINO_ALARM_LEVEL1_LOWER 2094284095UL
#define  _ARDUINO_ALARM_LEVEL2_UPPER 3396453667UL
#define  _ARDUINO_ALARM_LEVEL2_LOWER 3385752064UL
#define  _ARDUINO_ALARM_LEVEL3_UPPER 392954340UL
#define  _ARDUINO_ALARM_LEVEL3_LOWER 382252737UL
#define  _ARDUINO_SEATALK 3540059849UL
#define  _ARDUINO_WIND_ZEROOFFSET 1490126571UL
#define  _ARDUINO_WIND_LASTUPDATE 3391315227UL
#define  _ARDUINO_GPS_LASTFIX 110834493UL
#define  _ARDUINO_GPS_UTC 1376688456UL
#define  _ARDUINO_GPS_STATUS 246548288UL
#define  _ARDUINO_GPS_DECODE 3936761728UL
#define  _ARDUINO_AUTOPILOT_BAUDRATE 2707722171UL
#define  _ARDUINO_AUTOPILOT_OFFCOURSE 3534384255UL
#define  _ARDUINO_AUTOPILOT_RUDDERCOMMAND 210883000UL
#define  _ARDUINO_ALARM_LAST 3882620915UL
#define  _ARDUINO_WIND_AVERAGE 916641727UL
#define  _ARDUINO_WIND_FACTOR 3452243523UL
#define  _ARDUINO_WIND_MAX 891796138UL
#define  _ARDUINO_ALARM_SNOOZE 2215576829UL
#define  _ARDUINO_ANCHOR_RADIUSDEG 439059557UL
#define  _ARDUINO_ANCHOR_NORTH 3020492120UL
#define  _ARDUINO_ANCHOR_SOUTH 3026424992UL
#define  _ARDUINO_ANCHOR_EAST 3214803994UL
#define  _ARDUINO_ANCHOR_WEST 3215455216UL



class SignalkHelper {
public:

	SignalkHelper();

	int findInArray(const char *array[], const char* value);
	int getNumericType(unsigned long key);

	unsigned long hash(const char *str);


	void openMessage(HardwareSerial* serial);;
	void closeMessage(HardwareSerial* serial);;
	void openBranch(HardwareSerial* serial, const char* key);
	void closeBranch(HardwareSerial* serial, bool last);

	void printValue(HardwareSerial* serial, const char* key, const float value, bool last);
	void printValue(HardwareSerial* serial, const char* key, const unsigned long value, bool last);
	void printValue(HardwareSerial* serial, const char* key, const int value, bool last);
	void printValue(HardwareSerial* serial, const char* key, const long value, bool last);
	void printValue(HardwareSerial* serial, const char* key, const bool value, bool last);
	void printValue(HardwareSerial* serial, const char* key,  const char* value, bool last);
	void printValue(HardwareSerial* serial, const char* key,  const char value, bool last);
	void print_PROGMEM(HardwareSerial* serial, const char* key);
	byte getChecksum(char* str);

	static const char j_vessels[] PROGMEM;
	static const char j_self[] PROGMEM;
	static const char j_arduino[] PROGMEM;
	static const char j_airPressure[] PROGMEM;
	static const char j_airPressureChangeRateAlarm[] PROGMEM;
	static const char j_airTemp[] PROGMEM;
	static const char j_alarm[] PROGMEM;
	static const char j_alarmHeadingXte[] PROGMEM;
	static const char j_alarmRadius[] PROGMEM;
	static const char j_alarms[] PROGMEM;
	static const char j_altitude[] PROGMEM;
	static const char j_anchor[] PROGMEM;
	static const char j_anchorAlarmMethod[] PROGMEM;
	static const char j_anchorAlarmState[] PROGMEM;
	static const char j_autopilot[] PROGMEM;
	static const char j_autopilotAlarmMethod[] PROGMEM;
	static const char j_autopilotAlarmState[] PROGMEM;
	static const char j_average[] PROGMEM;
	static const char j_backlash[] PROGMEM;
	static const char j_baud0[] PROGMEM;
	static const char j_baud1[] PROGMEM;
	static const char j_baud2[] PROGMEM;
	static const char j_baud3[] PROGMEM;
	static const char j_baud4[] PROGMEM;
	static const char j_baud5[] PROGMEM;
	static const char j_baudRate[] PROGMEM;
	static const char j_bearingActual[] PROGMEM;
	static const char j_bearingDirect[] PROGMEM;
	static const char j_belowKeel[] PROGMEM;
	static const char j_belowSurface[] PROGMEM;
	static const char j_belowTransducer[] PROGMEM;
	static const char j_status[] PROGMEM;
	static const char j_courseOverGroundMagnetic[] PROGMEM;
	static const char j_courseOverGroundTrue[] PROGMEM;
	static const char j_courseRequired[] PROGMEM;
	static const char j_currentDirection[] PROGMEM;
	static const char j_currentRadius[] PROGMEM;
	static const char j_currentRoute[] PROGMEM;
	static const char j_currentSpeed[] PROGMEM;
	static const char j_deadZone[] PROGMEM;
	static const char j_decode[] PROGMEM;
	static const char j_depth[] PROGMEM;
	static const char j_destination[] PROGMEM;
	static const char j_directionApparent[] PROGMEM;
	static const char j_directionChangeAlarm[] PROGMEM;
	static const char j_directionTrue[] PROGMEM;
	static const char j_drift[] PROGMEM;
	static const char j_east[] PROGMEM;
	static const char j_engineAlarmMethod[] PROGMEM;
	static const char j_engineAlarmState[] PROGMEM;
	static const char j_environment[] PROGMEM;
	static const char j_eta[] PROGMEM;
	static const char j_factor[] PROGMEM;
	static const char j_fireAlarmMethod[] PROGMEM;
	static const char j_fireAlarmState[] PROGMEM;
	static const char j_gain[] PROGMEM;
	static const char j_gasAlarmMethod[] PROGMEM;
	static const char j_gasAlarmState[] PROGMEM;
	static const char j_genericAlarmMethod[] PROGMEM;
	static const char j_genericAlarmState[] PROGMEM;
	static const char j_gps[] PROGMEM;
	static const char j_gpsAlarmMethod[] PROGMEM;
	static const char j_gpsAlarmState[] PROGMEM;
	static const char j_headingMagnetic[] PROGMEM;
	static const char j_headingSource[] PROGMEM;
	static const char j_headingTrue[] PROGMEM;
	static const char j_heightHigh[] PROGMEM;
	static const char j_heightLow[] PROGMEM;
	static const char j_heightNow[] PROGMEM;
	static const char j_humidity[] PROGMEM;
	static const char j_last[] PROGMEM;
	static const char j_lastFix[] PROGMEM;
	static const char j_lastTime[] PROGMEM;
	static const char j_lastUpdate[] PROGMEM;
	static const char j_latitude[] PROGMEM;
	static const char j_level1[] PROGMEM;
	static const char j_level2[] PROGMEM;
	static const char j_level3[] PROGMEM;
	static const char j_upper[] PROGMEM;
	static const char j_lower[] PROGMEM;
	static const char j_longitude[] PROGMEM;
	static const char j_magneticVariation[] PROGMEM;
	static const char j_max[] PROGMEM;
	static const char j_maxDriveAmps[] PROGMEM;
	static const char j_maxDriveRate[] PROGMEM;
	static const char j_maxRadius[] PROGMEM;
	static const char j_maydayAlarmMethod[] PROGMEM;
	static const char j_maydayAlarmState[] PROGMEM;
	static const char j_mobAlarmMethod[] PROGMEM;
	static const char j_mobAlarmState[] PROGMEM;
	static const char j_mode[] PROGMEM;
	static const char j_model[] PROGMEM;
	static const char j_navigation[] PROGMEM;
	static const char j_next[] PROGMEM;
	static const char j_nextEta[] PROGMEM;
	static const char j_north[] PROGMEM;
	static const char j_offcourse[] PROGMEM;
	static const char j_panpanAlarmMethod[] PROGMEM;
	static const char j_panpanAlarmState[] PROGMEM;
	static const char j_pitch[] PROGMEM;
	static const char j_portLock[] PROGMEM;
	static const char j_position[] PROGMEM;
	static const char j_powerAlarmMethod[] PROGMEM;
	static const char j_powerAlarmState[] PROGMEM;
	static const char j_radarAlarmMethod[] PROGMEM;
	static const char j_radarAlarmState[] PROGMEM;
	static const char j_radiusDeg[] PROGMEM;
	static const char j_rateOfTurn[] PROGMEM;
	static const char j_roll[] PROGMEM;
	static const char j_route[] PROGMEM;
	static const char j_rudderAngle[] PROGMEM;
	static const char j_rudderAngleTarget[] PROGMEM;
	static const char j_rudderCommand[] PROGMEM;
	static const char j_salinity[] PROGMEM;
	static const char j_seatalk[] PROGMEM;
	static const char j_serial[] PROGMEM;
	static const char j_set[] PROGMEM;
	static const char j_silentInterval[] PROGMEM;
	static const char j_snooze[] PROGMEM;
	static const char j_south[] PROGMEM;
	static const char j_speedAlarm[] PROGMEM;
	static const char j_speedApparent[] PROGMEM;
	static const char j_speedOverGround[] PROGMEM;
	static const char j_speedThroughWater[] PROGMEM;
	static const char j_speedTrue[] PROGMEM;
	static const char j_starboardLock[] PROGMEM;
	static const char j_startTime[] PROGMEM;
	static const char j_state[] PROGMEM;
	static const char j_steering[] PROGMEM;
	static const char j_surfaceToTransducer[] PROGMEM;
	static const char j_targetHeadingMagnetic[] PROGMEM;
	static const char j_targetHeadingNorth[] PROGMEM;
	static const char j_tide[] PROGMEM;
	static const char j_timeHigh[] PROGMEM;
	static const char j_timeLow[] PROGMEM;
	static const char j_transducerToKeel[] PROGMEM;
	static const char j_utc[] PROGMEM;
	static const char j_waterTemp[] PROGMEM;
	static const char j_waypoint[] PROGMEM;
	static const char j_west[] PROGMEM;
	static const char j_wind[] PROGMEM;
	static const char j_windAlarmMethod[] PROGMEM;
	static const char j_windAlarmState[] PROGMEM;
	static const char j_xte[] PROGMEM;
	static const char j_zeroOffset[] PROGMEM;
private:


};


#endif /* SIGNALKHELPER_H_ */
