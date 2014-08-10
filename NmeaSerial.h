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
 * NmeaSerial.h
 *
 *  Created on: 23/12/2010
 *      Author: robert
 */

#ifndef NMEASERIAL_H_
#define NMEASERIAL_H_

#include "Arduino.h"
#include <PString.h>
#include <AltSoftSerial.h>
#include "FreeboardConstants.h"
#include "FreeboardModel.h"
#include "FreeboardMega.h"

class NmeaSerial: AltSoftSerial {
public:
	NmeaSerial(FreeboardModel* model):model(model){};
	virtual ~NmeaSerial();
	void printNmea(char* sentence);
	void printWindNmea();
	void printTrueHeading();
	void begin(long speed);
private:
	FreeboardModel* model;
	char windSentence [30];
	char trueHeadingSentence [20];
	byte cs;
};


#endif /* NMEASERIAL_H_ */
