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
 * Alarm.cpp
 *
 *  Created on: 19/12/2010
 *      Author: robert
 */

#include "Alarm.h"

Alarm::Alarm(SignalkModel* model) {
	this->model=model;
	alarmBeepState=false;
	model->setValue(_ARDUINO_ALARM_LAST,0);
	pinMode(alarmPin0, OUTPUT); //main MOSFET pin
	pinMode(alarmPin1, OUTPUT);
	pinMode(alarmPin2, OUTPUT);
	pinMode(alarmPin3, OUTPUT);

}

Alarm::~Alarm() {

}

bool Alarm::isAlarmTriggered() {
	return model->isAlarmTriggered() && model->getValueLong(_ARDUINO_ALARM_SNOOZE) < millis() ;
}


/* Take action if alarms are triggered*/
void Alarm::checkAlarms() {
	if (isAlarmTriggered()) {
		//alarm beeps on off on off
		//once in the alarm state, hitting any button will give a 5 minute respite from the beeping, eg snooze
		if ((millis() - model->getValueLong(_ARDUINO_ALARM_LAST)) > 1000UL) {
			digitalWrite(alarmPin0, alarmBeepState);
			digitalWrite(alarmPin1, alarmBeepState);
			digitalWrite(alarmPin2, alarmBeepState);
			digitalWrite(alarmPin3, alarmBeepState);
			alarmBeepState = !alarmBeepState;
			model->setValue(_ARDUINO_ALARM_LAST, (unsigned long)millis());
			//model->setAlarmSnooze(0); //5 minute alarm snooze
		}
	} else {
		//no alarm
		digitalWrite(alarmPin0, LOW);
		digitalWrite(alarmPin1, LOW);
		digitalWrite(alarmPin2, LOW);
		digitalWrite(alarmPin3, LOW);
	}
}




