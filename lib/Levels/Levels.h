/*
 * Copyright 2012,2013 Robert Huitema robert@42.co.nz
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
 * Alarm.h
 *
 *  Created on: 19/12/2010
 *      Author: robert
 */

#ifndef LEVELS_H_
#define LEVELS_H_

#include "Arduino.h"

#define lvl3Pin A12 // analogue pin A12
#define lvl1Pin A10
#define lvl2Pin A11

#include <SignalkModel.h>

class Levels {
public:
	Levels(SignalkModel* model);
	virtual ~Levels();
	void checkLvlAlarms();

private:
	SignalkModel* model;

};

#endif /* LEVELS_H_ */
