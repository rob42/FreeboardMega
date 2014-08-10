/*
 * Copyright 2012,2013 Robert Huitema robert@42.co.nz
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
 * Anchor.h
 *
 *  Created on: 17/12/2010
 *      Author: robert
 */

#ifndef ANCHOR_H_
#define ANCHOR_H_

#include "Arduino.h"

#include <PString.h>
#include "Gps.h"
#include "FreeboardModel.h"

extern void saveAnchorAlarmState(bool anchorAlarmOn) ;
extern void saveAnchorAlarmLat(float anchorLat) ;
extern void saveAnchorAlarmLon(float anchorLon) ;
extern void saveAnchorAlarmRadius(float anchorRadius);
// read the last anchor alarm values
extern bool getAnchorAlarmState() ;
extern float getAnchorAlarmLat();
extern float getAnchorAlarmLon();
extern float getAnchorAlarmRadius();



//anchor

class Anchor {

public:
	Anchor(FreeboardModel* model);
	virtual ~Anchor();



	void setAnchorRadius(float radius);
	float getAnchorRadius();
	void setAnchorPoint();

	void checkAnchor();
private:

	void updateAnchorBox(float laty, float lonx);
	void resetAnchorBox(float laty, float lonx);

	bool inAnchorBox(float laty, float lonx);



	FreeboardModel* model;
	static const unsigned long MAX_SINCE_LAST_GPS_FIX = 300000;

};

#endif /* ANCHOR_H_ */
