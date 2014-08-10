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
/*
 * FreeboardModel.cpp
 *
 *  Created on: Mar 28, 2012
 *      Author: robert
 */

#include "FreeboardModel.h"

FreeboardModel::FreeboardModel() {
	alarmLast = 0; //millis time of last beep state change
	alarmSnooze = 0; //5 minute alarm snooze
	//unsigned long alarmTriggered ; //true if any alarm is triggered - derived

	//anchor
	//float anchorRadius; //anchor alarm radius in meters
	anchorState.anchorRadiusDeg = 0.0; //anchor alarm radius in decimal degrees, eg 1deg = 60NM.

	//anchor alarm data
	//float anchorLat; // variable for reading the anchor latitude
	//float anchorLon; // variable for reading the anchor longitude
	anchorState.anchorDistance = 0.0;
	anchorState.anchorMaxDistance = 0.0;
	//bool anchorAlarmOn; //flag to turn anchor alarm on/off toggle
	anchorState.anchorAlarmTriggered = false; //set to true to trigger anchor alarm

	//a box around the anchor, shrinks with every GPS reading to home in on the anchor itself
	anchorState.anchorN = 90.0;
	anchorState.anchorS = -90.0;
	anchorState.anchorE = 180.0;
	anchorState.anchorW = -180.0;

	//autopilot
	autopilotState.autopilotOn = false;
	//disengage the autopilot if we reboot!!
	//Dont want to go screaming off on wrong course.
	autopilotState.autopilotReference = AUTOPILOT_COMPASS;
	autopilotState.autopilotTargetHeading = 0; //Setpoint
	autopilotState.autopilotRudderCommand = 33; //Output (rudder central)
	//bool autopilotAlarmOn;
	autopilotState.autopilotAlarmTriggered = false;
	autopilotState.autopilotAlarmMaxXTError = 100; // +/- meters cross track error
	autopilotState.autopilotAlarmMaxWindError = 10; // +/- wind angle change, for over 1 minute
	autopilotState.autopilotAlarmMaxCourseError = 10; // +/- course error, for over 1 minute
	autopilotState.autopilotSpeed=128;
	//compass
	magneticHeading = 0;
	declination = 0;
	//gps
	gpsState.gpsDecode = false; //flag to indicate a new sentence was decoded.
	gpsState.gpsLastFix = 0; //time of last good gps fix.
	gpsState.gpsUtc = 0; // decimal value of UTC term in last full GPRMC sentence
	gpsState.gpsStatus = 'V'; //  status character in last full GPRMC sentence ('A' or 'V')
	gpsState.gpsLatitude = 0.0; // signed degree-decimal value of latitude terms in last full GPRMC sentence
	gpsState.gpsLongitude = 0.0; // signed degree-decimal value of longitude terms in last full GPRMC sentence
	//float gpsSpeedUnit; //unit multiplier for gpsSpeed. 1.0 = KNT,1.1507794	=MPH, see nmea.h
	gpsState.gpsSpeed = 0.0; // speed-on-ground term in last full GPRMC sentence
	gpsState.gpsCourse = 0.0; // track-angle-made-good term in last full GPRMC sentence
	//bool gpsAlarmOn; //true to engage alarm
	gpsState.gpsAlarmTriggered = false; //set to true to trigger gps alarm
	//double gpsAlarmFixTime; //max time in millis without fix

	//seatalk
	//volatile bool radarAlarmOn; //set to true to enable radar alarm
	radarAlarmTriggered = false; //set to true to trigger radar alarm
	//volatile bool mobAlarmOn; //set to true to enable mob alarm
	mobAlarmTriggered = false; //set to true to trigger MOB alarm
	lvl1AlarmTriggered = false;
	config.lvl1UpperLimit = 1025; //0-1024 analogue range - higher is slower to alarm, eg more lvl1
	config.lvl1LowerLimit=-1; //0-1024 analogue range - lower is slower to alarm, eg less lvl1
	lvl2AlarmTriggered =false; //set to true to trigger lvl2 alarm
	config.lvl2UpperLimit= 1025; //0-1024 analogue range - higher is slower to alarm, eg more lvl2
	config.lvl2LowerLimit=-1; //0-1024 analogue range - lower is slower to alarm, eg less lvl2
	lvl3AlarmTriggered =false; //set to true to trigger lvl3 alarm
	config.lvl3UpperLimit= 1025; //0-1024 analogue range - higher is slower to alarm, eg more lvl3
	config.lvl3LowerLimit=-1; //0-1024 analogue range - lower is slower to alarm, eg less lvl3
	//wind
	windState.windLastUpdate = 0;
	windState.windAverage = 0.0;
	//windState.windFactor=0;
	windState.windMax = 0;
	windState.windApparentDir = 0;
	windState.windTrueDir = 0;
	//int windAlarmSpeed;
	//bool windAlarmOn;
	windState.windAlarmTriggered = false;

	//struct Configuration{
	config.anchorLat = 0.0;
	config.anchorLon = 0.0;
	config.anchorRadius = 40.0;
	config.anchorAlarmOn = false;
	config.autopilotAlarmOn = false;
	config.autopilotDeadZone = 0;
	config.autopilotSlack = 0;
	config.gpsSpeedUnit = KTS;
	config.gpsAlarmOn = false;
	config.gpsAlarmFixTime = 1000l * 60 * 5; //5 min
	config.radarAlarmOn = false;
	config.mobAlarmOn = false;
	config.windAlarmSpeed = 99;
	config.windAlarmOn = false;
	config.windFactor = 10000.0;
	config.windZeroOffset = 0;
	//ver6
	config.gpsModel = GPS_EM_406A;
	config.serialBaud = 38400l;
	config.serialBaud1 = 38400l;
	config.serialBaud2 = 9600l; //seatalk?
	config.serialBaud3 = 9600l; //16 bytes
	config.serialBaud4 = 9600l; //16 bytes
	config.autopilotBaud = 9600l; //16 bytes
	config.seaTalk = false;
	//}config;

//we change this if we change the struct so we can tell before reloading incompatible versions
	version = EEPROM_VER;
}

template<class T> int writeObject(HardwareSerial& ser, T& value, char name) {
	unsigned char* p = (unsigned char*) (void*) &value;
	unsigned int i;
	char checksum = '0';
	ser.write('@');
	ser.write(name);
	for (i = 0; i < sizeof(value); i++) {
		ser.write(*p++);
		checksum = checksum ^ *p;
	}
	ser.write(checksum);
	ser.write('\n');
	return i;

}

int FreeboardModel::writeSimple(HardwareSerial& ser) {
	//ArduIMU output format
	//!!VER:1.9,RLL:-0.52,PCH:0.06,YAW:80.24,IMUH:253,MGX:44,MGY:-254,MGZ:-257,MGH:80.11,LAT:-412937350,LON:1732472000,ALT:14,COG:116,SOG:0,FIX:1,SAT:5,TOW:22504700,

	ser.print(F("!!VER:1.9,"));
	ser.print(F("UID:MEGA,APX:"));
	ser.print(autopilotState.autopilotOn);
	ser.print(F(",APS:"));
	ser.print(autopilotState.autopilotReference);
	//if autopilot on, send autopilot data
	if (autopilotState.autopilotOn) {
		ser.print(F(",APT:"));
		ser.print(getAutopilotTargetHeading());
		ser.print(F(",APC:"));
		ser.print(getAutopilotCurrentHeading());
		ser.print(F(",APR:"));
		ser.print(autopilotState.autopilotRudderCommand - 33.0); // 0-66 in model
	}
	//if anchor alarm on, send data
	ser.print(F(",AAX:"));
	ser.print(config.anchorAlarmOn);
	ser.print(F(",AAR:"));
	ser.print(config.anchorRadius);
	if (config.anchorAlarmOn) {
		ser.print(F(",AAN:"));
		ser.print(config.anchorLat);
		ser.print(F(",AAE:"));
		ser.print(config.anchorLon);
		ser.print(F(",AAD:"));
		ser.print(getAnchorDistance());
	}
	//if wind alarm on, send data
	ser.print(F(",WSX:"));
	ser.print(config.windAlarmOn);
	ser.print(F(",WSK:"));
	ser.print(config.windAlarmSpeed);

	ser.println(F(","));
	return 0;
}
/*
 * Write out the config to serial
 */
int FreeboardModel::writeConfig(HardwareSerial& ser) {
	//ArduIMU output format
	//!!VER:1.9,RLL:-0.52,PCH:0.06,YAW:80.24,IMUH:253,MGX:44,MGY:-254,MGZ:-257,MGH:80.11,LAT:-412937350,LON:1732472000,ALT:14,COG:116,SOG:0,FIX:1,SAT:5,TOW:22504700,

	ser.print(F("!!VER:1.9,"));
	ser.print(F("UID:MEGA,APX:"));

	ser.print(F(",WZJ:"));
	ser.print(getWindZeroOffset());
	ser.print(F(",GPS:"));
	ser.print(getGpsModel());
	ser.print(F(",SB0:"));
	ser.print(getSerialBaud());
	ser.print(F(",SB1:"));
	ser.print(getSerialBaud1());
	ser.print(F(",SB2:"));
	ser.print(getSerialBaud2());
	ser.print(F(",SB3:"));
	ser.print(getSerialBaud3());
	ser.print(F(",STK:"));
	ser.print(getSeaTalk());

	ser.print(F(",LU1:"));
	ser.print(getLvl1UpperLimit());
	ser.print(F(",LL1:"));
	ser.print(getLvl1LowerLimit());

	ser.print(F(",LU2:"));
	ser.print(getLvl2UpperLimit());
	ser.print(F(",LL2:"));
	ser.print(getLvl2LowerLimit());

	ser.print(F(",LU3:"));
	ser.print(getLvl3UpperLimit());
	ser.print(F(",LL3:"));
	ser.print(getLvl3LowerLimit());

	ser.println(F(","));
	return 0;
}
int FreeboardModel::sendData(HardwareSerial& ser, char name) {
	if (CONFIG_T == name) {
		return writeObject(ser, config, name);
	}
	if (DYNAMIC_T == name) {
		return writeObject(ser, gpsState, name);
	}
	if (SIMPLE_T == name) {
		return writeSimple(ser);
	}
	return -1;
}

template<class T> int readObject(HardwareSerial& ser, T& value, char name) {
	unsigned char* p = (unsigned char*) (void*) &value;
	unsigned int i;
	char checksum = '0';
	for (i = 0; i < sizeof(value); i++) {
		*p++ = ser.read();
		checksum = checksum ^ *p;
	}
	//TODO: make sure this actually works
	if (ser.read() == checksum) {
		Serial.print(F("Checksum valid"));
	} else {
		Serial.print(F("Checksum invalid"));
	}

	return i;
}

int FreeboardModel::receiveData(HardwareSerial& ser, char name) {
	if (CONFIG_T == name) {
		return readObject(ser, config, name);
	}
	return -1;
}

template<class T> int EEPROM_writeAnything(int ee, T& value) {
	unsigned char* p = (unsigned char*) (void*) &value;
	unsigned int i;
	for (i = 0; i < sizeof(value); i++)
		EEPROM.write(ee++, *p++);
	return i;
}
//saving
template<class T> int EEPROM_readAnything(int ee, T& value) {
	unsigned char* p = (unsigned char*) (void*) &value;
	unsigned int i;
	for (i = 0; i < sizeof(value); i++)
		*p++ = EEPROM.read(ee++);
	return i;
}
void FreeboardModel::saveConfig() {
	//write out a current version
	EEPROM_writeAnything(0, version);
	//write data
	EEPROM_writeAnything(EEPROM_DATA, config);
}

void FreeboardModel::readConfig() {
	//check versions here
	int ver;
	EEPROM_readAnything(0, ver);
	if (ver != version) {
		//save a default config, since we cant read the old one safely
		saveConfig();
	}

	//now we know its compatible
	EEPROM_readAnything(EEPROM_DATA, config);

}

//accessors
bool FreeboardModel::isWindAlarmTriggered() {
	return windState.windAlarmTriggered;
}

unsigned long FreeboardModel::getAlarmLast() {
	return alarmLast;
}

unsigned long FreeboardModel::getAlarmSnooze() {
	return alarmSnooze;
}

float FreeboardModel::getAnchorDistance() {
	return anchorState.anchorDistance;
}

float FreeboardModel::getAnchorE() {
	return anchorState.anchorE;
}

float FreeboardModel::getAnchorLat() {
	return config.anchorLat;
}

float FreeboardModel::getAnchorLon() {
	return config.anchorLon;
}

float FreeboardModel::getAnchorMaxDistance() {
	return anchorState.anchorMaxDistance;
}

float FreeboardModel::getAnchorN() {
	return anchorState.anchorN;
}

float FreeboardModel::getAnchorRadius() {
	return config.anchorRadius;
}

float FreeboardModel::getAnchorRadiusDeg() {
	return anchorState.anchorRadiusDeg;
}

float FreeboardModel::getAnchorS() {
	return anchorState.anchorS;
}

float FreeboardModel::getAnchorW() {
	return anchorState.anchorW;
}

/*
 * Returns -179 to +180 as the degrees off course
 */
double FreeboardModel::getAutopilotOffCourse() {
	//get degrees between
	autopilotState.autopilotOffCourse = getAutopilotTargetHeading() - getAutopilotCurrentHeading();
	autopilotState.autopilotOffCourse += (autopilotState.autopilotOffCourse > 180) ? -360 : (autopilotState.autopilotOffCourse < -180) ? 360 : 0;

	return autopilotState.autopilotOffCourse;
}

char FreeboardModel::getAutopilotReference() {
	return autopilotState.autopilotReference;
}

double FreeboardModel::getAutopilotAlarmMaxCourseError() {
	return autopilotState.autopilotAlarmMaxCourseError;
}

double FreeboardModel::getAutopilotAlarmMaxWindError() {
	return autopilotState.autopilotAlarmMaxWindError;
}

double FreeboardModel::getAutopilotAlarmMaxXtError() {
	return autopilotState.autopilotAlarmMaxXTError;
}

double FreeboardModel::getAutopilotRudderCommand() {
	return autopilotState.autopilotRudderCommand;
}

double FreeboardModel::getAutopilotTargetHeading() {
	return autopilotState.autopilotTargetHeading;
}
double FreeboardModel::getAutopilotCurrentHeading() {
	if (autopilotState.autopilotReference == AUTOPILOT_WIND) {
		return windState.windApparentDir;
	}
	//default option - compass
	return magneticHeading;
}

int FreeboardModel::getAutopilotDeadZone() {
	return this->config.autopilotDeadZone;
}
int FreeboardModel::getAutopilotSlack() {
	return this->config.autopilotSlack;
}
long FreeboardModel::getAutopilotSpeed() {
	return this->config.autopilotSpeed;
}

long FreeboardModel::getGpsAlarmFixTime() {
	return config.gpsAlarmFixTime;
}

float FreeboardModel::getGpsCourse() {
	return gpsState.gpsCourse;
}

unsigned long FreeboardModel::getGpsLastFix() {
	return gpsState.gpsLastFix;
}

float FreeboardModel::getGpsLatitude() {
	return gpsState.gpsLatitude;
}

float FreeboardModel::getGpsLongitude() {
	return gpsState.gpsLongitude;
}

float FreeboardModel::getGpsSpeed() {
	return gpsState.gpsSpeed;
}

float FreeboardModel::getGpsSpeedUnit() {
	return config.gpsSpeedUnit;
}

char FreeboardModel::getGpsStatus() {
	return gpsState.gpsStatus;
}

float FreeboardModel::getGpsUtc() {
	return gpsState.gpsUtc;
}

float FreeboardModel::getMagneticHeading() {
	return this->magneticHeading;
}
float FreeboardModel::getDeclination() {
	return declination;
}

volatile bool FreeboardModel::isMobAlarmTriggered() {
	return mobAlarmTriggered;
}
volatile bool FreeboardModel::isLvl1AlarmTriggered() {
	return lvl1AlarmTriggered;
}
int FreeboardModel::getLvl1UpperLimit(){
	return config.lvl1UpperLimit;
}
void FreeboardModel::setLvl1UpperLimit(int lvl1UpperLimit){
	this->config.lvl1UpperLimit=lvl1UpperLimit;
}

int FreeboardModel::getLvl1LowerLimit(){
	return config.lvl1LowerLimit;
}
void FreeboardModel::setLvl1LowerLimit(int lvl1LowerLimit){
	this->config.lvl1LowerLimit=lvl1LowerLimit;
}
volatile bool FreeboardModel::isLvl2AlarmTriggered() {
	return lvl2AlarmTriggered;
}
int FreeboardModel::getLvl2UpperLimit(){
	return config.lvl2UpperLimit;
}
void FreeboardModel::setLvl2UpperLimit(int lvl2UpperLimit){
	this->config.lvl2UpperLimit=lvl2UpperLimit;
}

int FreeboardModel::getLvl2LowerLimit(){
	return config.lvl2LowerLimit;
}
void FreeboardModel::setLvl2LowerLimit(int lvl2LowerLimit){
	this->config.lvl2LowerLimit=lvl2LowerLimit;
}

volatile bool FreeboardModel::isLvl3AlarmTriggered() {
	return lvl3AlarmTriggered;
}
int FreeboardModel::getLvl3UpperLimit(){
	return config.lvl3UpperLimit;
}
void FreeboardModel::setLvl3UpperLimit(int lvl3UpperLimit){
	this->config.lvl3UpperLimit=lvl3UpperLimit;
}

int FreeboardModel::getLvl3LowerLimit(){
	return config.lvl3LowerLimit;
}
void FreeboardModel::setLvl3LowerLimit(int lvl3LowerLimit){
	this->config.lvl3LowerLimit=lvl3LowerLimit;
}


volatile bool FreeboardModel::isRadarAlarmTriggered() {
	return radarAlarmTriggered;
}

int FreeboardModel::getWindZeroOffset() {
	return config.windZeroOffset;
}

int FreeboardModel::getWindAlarmSpeed() {
	return config.windAlarmSpeed;
}

int FreeboardModel::getWindApparentDir() {
	return windState.windApparentDir;
}

int FreeboardModel::getWindTrueDir() {
	return windState.windTrueDir;
}

float FreeboardModel::getWindAverage() {
	return windState.windAverage;
}

float FreeboardModel::getWindFactor() {
	return config.windFactor;
}

unsigned long FreeboardModel::getWindLastUpdate() {
	return windState.windLastUpdate;
}

int FreeboardModel::getWindMax() {
	return windState.windMax;
}

bool FreeboardModel::isAnchorAlarmOn() {
	return config.anchorAlarmOn;
}

bool FreeboardModel::isAnchorAlarmTriggered() {
	return anchorState.anchorAlarmTriggered;
}

bool FreeboardModel::isAutopilotAlarmOn() {
	return config.autopilotAlarmOn;
}

bool FreeboardModel::isAutopilotAlarmTriggered() {
	return autopilotState.autopilotAlarmTriggered;
}

bool FreeboardModel::isGpsAlarmOn() {
	return config.gpsAlarmOn;
}

bool FreeboardModel::isGpsAlarmTriggered() {
	return gpsState.gpsAlarmTriggered;
}

bool FreeboardModel::isGpsDecode() {
	return gpsState.gpsDecode;
}

bool FreeboardModel::isWindAlarmOn() {
	return config.windAlarmOn;
}

void FreeboardModel::setAlarmLast(unsigned long alarmLast) {
	this->alarmLast = alarmLast;
}

void FreeboardModel::setAlarmSnooze(unsigned long alarmSnooze) {
	this->alarmSnooze = alarmSnooze;
}

void FreeboardModel::setAnchorAlarmOn(bool anchorAlarmOn) {
	this->config.anchorAlarmOn = anchorAlarmOn;
}

void FreeboardModel::setAnchorAlarmTriggered(bool anchorAlarmTriggered) {
	this->anchorState.anchorAlarmTriggered = anchorAlarmTriggered;
}

void FreeboardModel::setAnchorDistance(float anchorDistance) {
	this->anchorState.anchorDistance = anchorDistance;
}

void FreeboardModel::setAnchorE(float anchorE) {
	this->anchorState.anchorE = anchorE;
}

void FreeboardModel::setAnchorLat(float anchorLat) {
	this->config.anchorLat = anchorLat;
}

void FreeboardModel::setAnchorLon(float anchorLon) {
	this->config.anchorLon = anchorLon;
}

void FreeboardModel::setAnchorMaxDistance(float anchorMaxDistance) {
	this->anchorState.anchorMaxDistance = anchorMaxDistance;
}

void FreeboardModel::setAnchorN(float anchorN) {
	this->anchorState.anchorN = anchorN;
}

void FreeboardModel::setAnchorRadius(float anchorRadius) {
	this->config.anchorRadius = anchorRadius;
}

void FreeboardModel::setAnchorRadiusDeg(float anchorRadiusDeg) {
	this->anchorState.anchorRadiusDeg = anchorRadiusDeg;
}

void FreeboardModel::setAnchorS(float anchorS) {
	this->anchorState.anchorS = anchorS;
}

void FreeboardModel::setAnchorW(float anchorW) {
	this->anchorState.anchorW = anchorW;
}

void FreeboardModel::setAutopilotReference(char autopilotReference) {

	if (autopilotReference != AUTOPILOT_WIND && autopilotReference != AUTOPILOT_COMPASS) return;
	this->autopilotState.autopilotReference = autopilotReference;
	if (autopilotState.autopilotReference == AUTOPILOT_WIND) {
		autopilotState.autopilotTargetHeading = windState.windApparentDir;
	}
	if (autopilotState.autopilotReference == AUTOPILOT_COMPASS) {
		autopilotState.autopilotTargetHeading = magneticHeading;
	}
	//and netralise the rudder position too.
	this->autopilotState.autopilotRudderCommand = 33;
}

void FreeboardModel::setAutopilotAlarmMaxCourseError(double autopilotAlarmMaxCourseError) {
	this->autopilotState.autopilotAlarmMaxCourseError = autopilotAlarmMaxCourseError;
}

void FreeboardModel::setAutopilotAlarmMaxWindError(double autopilotAlarmMaxWindError) {
	this->autopilotState.autopilotAlarmMaxWindError = autopilotAlarmMaxWindError;
}

void FreeboardModel::setAutopilotAlarmMaxXtError(double autopilotAlarmMaxXtError) {
	autopilotState.autopilotAlarmMaxXTError = autopilotAlarmMaxXtError;
}

void FreeboardModel::setAutopilotAlarmOn(bool autopilotAlarmOn) {
	this->config.autopilotAlarmOn = autopilotAlarmOn;
}

void FreeboardModel::setAutopilotAlarmTriggered(bool autopilotAlarmTriggered) {
	this->autopilotState.autopilotAlarmTriggered = autopilotAlarmTriggered;
}

void FreeboardModel::setAutopilotRudderCommand(double autopilotRudderCommand) {
	this->autopilotState.autopilotRudderCommand = autopilotRudderCommand;
}
void FreeboardModel::setAutopilotSpeed(long autopilotSpeed) {
	this->autopilotState.autopilotSpeed = autopilotSpeed;
}

/**
 * For magnetic it will be 0-360degM
 * For wind it will be -180 to +180 from bow.
 * Since this matches with target heading, and we convert in autopilot to 0-360, alls good?
 */
void FreeboardModel::setAutopilotTargetHeading(double autopilotTargetHeading) {
	//make this 0-360 range only
	this->autopilotState.autopilotTargetHeading = (double) (((int) autopilotTargetHeading + 360) % 360);
}

void FreeboardModel::setGpsAlarmFixTime(long gpsAlarmFixTime) {
	this->config.gpsAlarmFixTime = gpsAlarmFixTime;
}

void FreeboardModel::setGpsAlarmOn(bool gpsAlarmOn) {
	this->config.gpsAlarmOn = gpsAlarmOn;
}

void FreeboardModel::setGpsAlarmTriggered(bool gpsAlarmTriggered) {
	this->gpsState.gpsAlarmTriggered = gpsAlarmTriggered;
}

void FreeboardModel::setGpsCourse(float gpsCourse) {
	this->gpsState.gpsCourse = gpsCourse;
}

void FreeboardModel::setGpsDecode(bool gpsDecode) {
	this->gpsState.gpsDecode = gpsDecode;
}

void FreeboardModel::setGpsLastFix(unsigned long gpsLastFix) {
	this->gpsState.gpsLastFix = gpsLastFix;
}

void FreeboardModel::setGpsLatitude(float gpsLatitude) {
	this->gpsState.gpsLatitude = gpsLatitude;
}

void FreeboardModel::setGpsLongitude(float gpsLongitude) {
	this->gpsState.gpsLongitude = gpsLongitude;
}

void FreeboardModel::setGpsSpeed(float gpsSpeed) {
	this->gpsState.gpsSpeed = gpsSpeed;
}

void FreeboardModel::setGpsSpeedUnit(float gpsSpeedUnit) {
	this->config.gpsSpeedUnit = gpsSpeedUnit;
}

void FreeboardModel::setGpsStatus(char gpsStatus) {
	this->gpsState.gpsStatus = gpsStatus;
}

void FreeboardModel::setGpsUtc(float gpsUtc) {
	this->gpsState.gpsUtc = gpsUtc;
}

void FreeboardModel::setMagneticHeading(float magneticHeading) {
	this->magneticHeading = magneticHeading;
}

void FreeboardModel::setDeclination(float declination) {
	this->declination = declination;
}

void FreeboardModel::setMobAlarmTriggered(volatile bool mobAlarmTriggered) {
	this->mobAlarmTriggered = mobAlarmTriggered;
}
void FreeboardModel::setLvl1AlarmTriggered(volatile bool lvl1AlarmTriggered) {
	this->lvl1AlarmTriggered = lvl1AlarmTriggered;
}
void FreeboardModel::setLvl2AlarmTriggered(volatile bool lvl2AlarmTriggered) {
	this->lvl2AlarmTriggered = lvl2AlarmTriggered;
}
void FreeboardModel::setLvl3AlarmTriggered(volatile bool lvl3AlarmTriggered) {
	this->lvl3AlarmTriggered = lvl3AlarmTriggered;
}

void FreeboardModel::setRadarAlarmTriggered(volatile bool radarAlarmTriggered) {
	this->radarAlarmTriggered = radarAlarmTriggered;
}

void FreeboardModel::setWindZeroOffset(int windZeroOffset) {
	this->config.windZeroOffset = windZeroOffset;
}

void FreeboardModel::setWindAlarmOn(bool windAlarmOn) {
	this->config.windAlarmOn = windAlarmOn;
}

void FreeboardModel::setWindAlarmSpeed(int windAlarmSpeed) {
	this->config.windAlarmSpeed = windAlarmSpeed;
}
/* 0-360deg off the bow clockwise.*/
void FreeboardModel::setWindApparentDir(int windApparentDir) {
	this->windState.windApparentDir = windApparentDir;
}
void FreeboardModel::setWindTrueDir(int windTrueDir) {
	this->windState.windTrueDir = windTrueDir;
}

void FreeboardModel::setWindAverage(float windAverage) {
	this->windState.windAverage = windAverage;
}

void FreeboardModel::setWindFactor(float windFactor) {
	this->config.windFactor = windFactor;
}

void FreeboardModel::setWindLastUpdate(unsigned long windLastUpdate) {
	this->windState.windLastUpdate = windLastUpdate;
}

volatile bool FreeboardModel::isAlarmTriggered() {
	return windState.windAlarmTriggered && radarAlarmTriggered && gpsState.gpsAlarmTriggered && anchorState.anchorAlarmTriggered
			&& autopilotState.autopilotAlarmTriggered && mobAlarmTriggered && lvl1AlarmTriggered && lvl2AlarmTriggered && lvl3AlarmTriggered;
}

volatile bool FreeboardModel::isMobAlarmOn() {
	return config.mobAlarmOn;
}

volatile bool FreeboardModel::isRadarAlarmOn() {
	return config.radarAlarmOn;
}

void FreeboardModel::setMobAlarmOn(volatile bool mobAlarmOn) {
	this->config.mobAlarmOn = mobAlarmOn;
}

bool FreeboardModel::isAutopilotOn() {
	return autopilotState.autopilotOn;
}

void FreeboardModel::setAutopilotOn(bool autopilotOn) {
	//this is potentally dangerous, since we dont want the boat diving off on an old target heading.
	//ALWAYS reset target heading to current magnetic or wind dir here
	setAutopilotReference(getAutopilotReference());
	this->autopilotState.autopilotOn = autopilotOn;
}

void FreeboardModel::setAutopilotDeadZone(int deadZone) {
	this->config.autopilotDeadZone = deadZone;
}

void FreeboardModel::setAutopilotSlack(int slack) {
	this->config.autopilotSlack = slack;
}

void FreeboardModel::setRadarAlarmOn(volatile bool radarAlarmOn) {
	this->config.radarAlarmOn = radarAlarmOn;
}

void FreeboardModel::setWindMax(int windMax) {
	this->windState.windMax = windMax;
}

void FreeboardModel::setWindAlarmTriggered(bool windAlarmTriggered) {
	this->windState.windAlarmTriggered = windAlarmTriggered;
}

short FreeboardModel::getGpsModel() {
	return this->config.gpsModel;
}
void FreeboardModel::setGpsModel(short gpsModel) {
	this->config.gpsModel = gpsModel;
}
long FreeboardModel::getSerialBaud() {
	return this->config.serialBaud;
}
void FreeboardModel::setSerialBaud(long serialBaud) {
	this->config.serialBaud = serialBaud;
}
long FreeboardModel::getSerialBaud1() {
	return this->config.serialBaud1;
}
void FreeboardModel::setSerialBaud1(long serialBaud1) {
	this->config.serialBaud1 = serialBaud1;
}
long FreeboardModel::getSerialBaud2() {
	return this->config.serialBaud2;
}
void FreeboardModel::setSerialBaud2(long serialBaud2) {
	this->config.serialBaud2 = serialBaud2;
}
long FreeboardModel::getSerialBaud3() {
	return this->config.serialBaud3;
}
void FreeboardModel::setSerialBaud3(long serialBaud3) {
	this->config.serialBaud3 = serialBaud3;
}
long FreeboardModel::getSerialBaud4() {
	return this->config.serialBaud4;
}
void FreeboardModel::setSerialBaud4(long serialBaud4) {
	this->config.serialBaud4 = serialBaud4;
}
long FreeboardModel::getSerialBaud5() {
	return this->config.serialBaud5;
}
void FreeboardModel::setSerialBaud5(long serialBaud5) {
	this->config.serialBaud5 = serialBaud5;
}
long FreeboardModel::getAutopilotBaud() {
	return this->config.autopilotBaud;
}
void FreeboardModel::setAutopilotBaud(long autopilotBaud) {
	this->config.autopilotBaud = autopilotBaud;
}
bool FreeboardModel::getSeaTalk() {
	return this->config.seaTalk;
}
void FreeboardModel::setSeaTalk(bool seaTalk) {
	this->config.seaTalk = seaTalk;
}
