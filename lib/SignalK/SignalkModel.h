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
 * SignalkModel.h
 *
 *Holds all the global model vars
 *
 *  Created on: Mar 28, 2012
 *      Author: robert
 */

#ifndef SIGNALKMODEL_H_
#define SIGNALKMODEL_H_
#define DEBUG true
#include "Arduino.h"
#include "FreeboardConstants.h"
#include <SignalkHelper.h>


typedef enum {ALRM_MESSAGE,ALRM_SOUND,ALRM_SMS,ALRM_EMAIL,ALRM_DSC} AlarmMethodType;
static const char *AlarmMethodString[] = {"message", "sound","sms","email","dsc",};
typedef enum {ALRM_DISABLED,ALRM_ENABLED,ALRM_FIRING,ALRM_SILENT} AlarmStateType;
static const char *AlarmStateString[] = {"disabled", "enabled", "firing", "silent",};

typedef enum {AP_OFF,AP_ON,AP_ALARM} AutopilotStateType;
static const char *AutopilotStateString[] = {"off", "on", "alarm",};
typedef enum {AP_POWERSAVE,AP_NORMAL,AP_ACCURATE} AutopilotModeType;
static const char *AutopilotModeString[] = { "powersave", "normal", "accurate",};

typedef enum {NAV_UNDERWAY_ENGINE,NAV_ANCHORED,NAV_NOT_UNDER_COMMAND, NAV_RESTRICTED_MANOUVERABILITY, NAV_CONSTRAINED_BY_DRAFT,
	NAV_MOORED,NAV_AGROUND,NAV_FISHING,NAV_SAILING,NAV_NOT_DEFINED} NavigationStateType;
static const char *NavigationStateString[] = {
		"Under way using engine",
		"At anchor",
        "Not under command",
        "Restricted manoeuverability",
        "Constrained by her draught",
        "Moored",
        "Aground",
        "Engaged in Fishing",
        "Under way sailing",
        "Not defined (example)",
};

typedef enum {AP_COMPASS,AP_WIND,AP_GPS} AutopilotHeadingSourceType;
static const char *AutopilotHeadingSourceString[] = { "compass", "wind", "gps",};


typedef struct {
	float longitude;
	float latitude;
	float altitude;
} Position ;

typedef struct {
		int upper;
		int lower;
	}LevelStruct;

class SignalkModel: public virtual SignalkHelper {
public:

	SignalkModel();

	void setValue(char* attribute, bool value);
	void setValue(char* attribute, char* value);
	void setValue(char* attribute, char value);
	void setValue(char* attribute, float value);
	void setValue(char* attribute, long value);
	void setValue(char* attribute, unsigned long value);
	void setValue(char* attribute, int value);

	void setValue(unsigned long key, bool value);
	void setValue(unsigned long key, char* value);
	void setValue(unsigned long key, char value);
	void setValue(unsigned long key, float value);
	void setValue(unsigned long key, double value);
	void setValue(unsigned long key, unsigned long value);
	void setValue(unsigned long key, int value);

	bool getValueBool(unsigned long key);
	const char* getValueCharArray(unsigned long key);
	char getValueChar(unsigned long key);
	float getValueFloat(unsigned long key);
	double getValueDouble(unsigned long key);
	unsigned long getValueLong(unsigned long key);
	int getValueInt(unsigned long key);
	volatile bool isAlarmTriggered() ;
	volatile bool isAutopilotOn() ;
	volatile bool isAlarmTriggered(unsigned long key);
	volatile bool isAlarmOn(unsigned long key);


	void printVesselWrapper(HardwareSerial* serial);
	void printSteeringBranch(HardwareSerial* serial, bool last);
	void printNavigationBranch(HardwareSerial* serial, bool last);
	void printEnvironmentBranch(HardwareSerial* serial, bool last);
	void printAlarmBranch(HardwareSerial* serial, bool last);
	void printAnchorBranch(HardwareSerial* serial, bool last);
	void printPositionBranch(HardwareSerial* serial, bool last);
	void printAutopilotBranch(HardwareSerial* serial, bool last);
	void printWindBranch(HardwareSerial* serial, bool last);
	void printConfigBranch(HardwareSerial* serial, bool last);


private:

	struct Navigation {
			float courseOverGroundMagnetic;
			float courseOverGroundTrue;
			struct CurrentRoute {
				float bearingActual;
				float bearingDirect;
				float courseRequired;
				unsigned long eta;
				char* route;
				unsigned long startTime;
				struct Waypoint {
					unsigned long lastTime;
					char last[20];
					unsigned long nextEta;
					char next[20];
					float xte;
				} waypoint;
			} currentRoute ;
			float magneticVariation;
			struct Destination {
				unsigned long eta;
				float longitude;
				float latitude;
				float altitude;
			} destination;
			float drift;
			//gnss
			float headingMagnetic;
			float headingTrue;
			Position  position ;
			float pitch;
			float rateOfTurn;
			float roll;
			float set;
			float speedOverGround;
			float speedThroughWater;
			NavigationStateType state;
			struct Anchor {
				Position  position ;
				float alarmRadius;
				float maxRadius;
				float currentRadius;
			}anchor;
		} navigation;


	struct Steering {
		float rudderAngle;
		float rudderAngleTarget;
		struct Autopilot {
			AutopilotStateType state;
			AutopilotModeType mode;
			float targetHeadingNorth;
			float targetHeadingMagnetic;
			float alarmHeadingXte;
			AutopilotHeadingSourceType headingSource;
			float deadZone;
			float backlash;
			int gain;
			float maxDriveAmps;
			float maxDriveRate;
			float portLock;
			float starboardLock;
		} autopilot ;
	} steering ;
	struct Alarms {
		AlarmMethodType anchorAlarmMethod;
		AlarmStateType anchorAlarmState;
		AlarmMethodType autopilotAlarmMethod;
		AlarmStateType autopilotAlarmState;
		AlarmMethodType  engineAlarmMethod;
		AlarmStateType engineAlarmState;
		AlarmMethodType  fireAlarmMethod;
		AlarmStateType fireAlarmState;
		AlarmMethodType  gasAlarmMethod;
		AlarmStateType gasAlarmState;
		AlarmMethodType  gpsAlarmMethod;
		AlarmStateType gpsAlarmState;
		AlarmMethodType  maydayAlarmMethod;
		AlarmStateType maydayAlarmState;
		AlarmMethodType  panpanAlarmMethod;
		AlarmStateType panpanAlarmState;
		AlarmMethodType  powerAlarmMethod;
		AlarmStateType powerAlarmState;
		int  silentInterval;
		AlarmMethodType windAlarmMethod;
		AlarmStateType windAlarmState;
		AlarmMethodType genericAlarmMethod;
		AlarmStateType genericAlarmState;
		AlarmMethodType radarAlarmMethod;
		AlarmStateType radarAlarmState;
		AlarmMethodType mobAlarmMethod;
		AlarmStateType mobAlarmState;
	} alarms ;

	struct WindStruct {
					float directionApparent;
					float directionChangeAlarm;
					float directionTrue;
					float speedAlarm;
					float speedTrue;
					float speedApparent;
				} wind;
		struct TideStruct {
			float heightHigh;
			float heightNow;
			float heightLow;
			unsigned long timeLow;
			unsigned long timeHigh;
		} tide ;

		struct DepthStruct {
			float belowKeel;
			float belowTransducer;
			float belowSurface;
			float transducerToKeel;
			float surfaceToTransducer;
		} depth;

	struct EnvironmentStruct {
		float airPressureChangeRateAlarm;
		float airPressure;
		float airTemp;
		float currentDirection;
		float currentSpeed;
		float humidity;
		float salinity;
		float waterTemp;
		DepthStruct depth;
		TideStruct tide;
		WindStruct wind;
	} environment ;


	struct ConfigStruct {
		struct GpsStruct {
			bool decode;
			int model;
			unsigned long lastFix;
			unsigned long utc;
			char status;
		}gps;
		struct SerialStruct{
			unsigned long baud0;//console
			unsigned long baud1;//GPS
			unsigned long baud2;//NMEA1 or Seatalk
			unsigned long baud3;//NMEA2
			unsigned long baud4;//NMEA3 - SPI-2
			unsigned long baud5;//NMEA talker - AltSoftSerial
		}serial;
		struct AlarmStruct{
			LevelStruct level1;
			LevelStruct level2;
			LevelStruct level3;
			unsigned long snooze;
			unsigned long last;
		}alarm;
		struct AnchorStruct{
			float radiusDeg;
			float north;
			float south;
			float east;
			float west;
		}anchor;
		bool seatalk;
		struct WindStruct{
			unsigned long lastUpdate;
			float average;
			float factor;
			float max;
			float zeroOffset;
		}wind;
		struct AutopilotStruct{
			long baudRate;//autopilot - SPI-1
			double offcourse;
			double rudderCommand;
		}autopilot;

	}_arduino;
};


#endif /* SIGNALKMODEL_H_ */
