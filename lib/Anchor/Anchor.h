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
 * Anchor.h
 *
 *  Created on: 17/12/2010
 *      Author: robert
 */

#ifndef ANCHOR_H_
#define ANCHOR_H_


// 0.00053995680 nautical miles per meter
//decimal lat lon is in degrees, and we have 60 NM per degree so degrees per meter
#define  LLMTRS .00000899928
#define  MINMTRS	LLMTRS*5.0

#include "Arduino.h"
#include <PString.h>
#include <Gps.h>
#include <SignalkModel.h>

//extern void saveAnchorAlarmState(bool anchorAlarmOn) ;
//extern void saveAnchorAlarmLat(float anchorLat) ;
//extern void saveAnchorAlarmLon(float anchorLon) ;
//extern void saveAnchorAlarmRadius(float anchorRadius);
// read the last anchor alarm values
//extern bool getAnchorAlarmState() ;
//extern float getAnchorAlarmLat();
//extern float getAnchorAlarmLon();
//extern float getAnchorAlarmRadius();



//anchor

class Anchor {

public:
	Anchor(SignalkModel* model);
	virtual ~Anchor();



	void setAnchorRadius(float radius);
	float getAnchorRadius();
	void setAnchorPoint();

	void checkAnchor();
private:

	void updateAnchorBox(float laty, float lonx);
	void resetAnchorBox(float laty, float lonx);

	bool inAnchorBox(float laty, float lonx);



	SignalkModel* model;
	static const unsigned long MAX_SINCE_LAST_GPS_FIX = 300000;

};

#endif /* ANCHOR_H_ */
