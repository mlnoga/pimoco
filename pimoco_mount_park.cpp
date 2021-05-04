/*
    PiMoCo: Raspberry Pi Telescope Mount and Focuser Control
    Copyright (C) 2021 Markus Noga

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


#include "pimoco_mount.h"
#include <libindi/indilogger.h>


bool PimocoMount::SetParkPosition(double Axis1Value, double Axis2Value) {
	SetAxis1Park(Axis1Value);
	SetAxis2Park(Axis2Value);
   	LOGF_INFO("Setting park position to HA %f Dec %f", GetAxis1Park(), GetAxis2Park());
	WriteParkData();
	return true;
}

bool PimocoMount::SetCurrentPark() {
	double localHaHours, decDegrees;
	if(!stepperHA.getPositionHours(&localHaHours) || !stepperDec.getPositionDegrees(&decDegrees))
		return false;
	return SetParkPosition(localHaHours, decDegrees);
}

bool PimocoMount::SetDefaultPark() {
	return SetParkPosition(-6, 90);
}

bool PimocoMount::Park() {
	double localHaHours=GetAxis1Park(), decDegrees=GetAxis2Park();
   	LOGF_INFO("Parking at HA %f Dec %f", localHaHours, decDegrees);
	if(!stepperHA.setTargetPositionHours(localHaHours) || !stepperDec.setTargetPositionDegrees(decDegrees) ) {
		LOG_ERROR("Parking");
		return false;
	}
 
    TrackState = SCOPE_PARKING;
    manualSlewArcsecPerSecRA=manualSlewArcsecPerSecDec=0;
    guiderActiveRA=guiderActiveDec=false;
    // 	SetParked() and thus WriteParkData() happens in TimerHit() once the scope has reached position
  	return true;
}

bool PimocoMount::UnPark() {
	SetParked(false); // Updates TrackState to idle and calls WriteParkData(). However, Indi does not restore the prior tracking state of the scope after parking
    manualSlewArcsecPerSecRA=manualSlewArcsecPerSecDec=0;
    guiderActiveRA=guiderActiveDec=false;
	return true;
}

