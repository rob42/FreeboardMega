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
 * Levels.cpp
 *
 *  Created on: 19/12/2010
 *      Author: robert
 */

#include "Levels.h"

Levels::Levels(SignalkModel* model) {
	this->model=model;

}

Levels::~Levels() {

}



void Levels::checkLvlAlarms(){
	//check lvl* alarm val, 0 is low, 1024 is high
	int lvl1 = analogRead(lvl1Pin);
	int lvl2 = analogRead(lvl2Pin);
	int lvl3 = analogRead(lvl3Pin);
			if ( (lvl1 < model->getValueInt(_ARDUINO_ALARM_LEVEL1_LOWER) || lvl1 >model->getValueInt(_ARDUINO_ALARM_LEVEL1_UPPER))
				|| ( lvl2 < model->getValueInt(_ARDUINO_ALARM_LEVEL2_LOWER)  || lvl2 >model->getValueInt(_ARDUINO_ALARM_LEVEL2_UPPER))
				|| ( lvl3 < model->getValueInt(_ARDUINO_ALARM_LEVEL3_LOWER)  || lvl3 >model->getValueInt(_ARDUINO_ALARM_LEVEL3_UPPER))) {

			model->setValue(ALARMS_GENERICALARMSTATE,true);
		} else {
			model->setValue(ALARMS_GENERICALARMSTATE,false);
		}
}
