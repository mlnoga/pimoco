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
#include <libindi/indicom.h>  // for rangeHA etc.


bool PimocoMount::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) {
	if(command==MOTION_STOP) {
		manualSlewArcsecPerSecDec=0;
		return applyTracking(false, true);
	}

	double xSidereal=SlewRatesN[IUFindOnSwitchIndex(&SlewRateSP)].value;
	if(dir==DIRECTION_SOUTH)  // DIRECTION_NORTH is positive on the Dec axis
		xSidereal=-xSidereal;
	double arcsecPerSec=xSidereal * trackRates[0];
	if(stepperDec.getDebugLevel()>=Stepper::TMC_DEBUG_DEBUG)
		LOGF_DEBUG("Moving %s at %.1fx sidereal rate (%.2f arcsec/s)", (xSidereal>=0 ? "north" : "south"), abs(xSidereal), abs(arcsecPerSec));

	if(!applyLimits(0, arcsecPerSec))
		return false;

	if(!stepperDec.setTargetVelocityArcsecPerSec(arcsecPerSec)) {
		LOG_ERROR("MoveNS");
		return false;
	}

	manualSlewArcsecPerSecDec=arcsecPerSec;
    guiderActiveDec=false;  // avoid leftover guider pulse overriding manual movement on this axis
	return true;
}


bool PimocoMount::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) {
	if(command==MOTION_STOP) {
		manualSlewArcsecPerSecRA=0;
		return applyTracking(true, false);
	}

	double xSidereal=SlewRatesN[IUFindOnSwitchIndex(&SlewRateSP)].value;
	if(dir==DIRECTION_EAST)
		xSidereal=-xSidereal;  // DIRECTION_WEST for RA is positive on the HA axis
	double arcsecPerSec=xSidereal * trackRates[0];
	if(stepperHA.getDebugLevel()>=Stepper::TMC_DEBUG_DEBUG)
		LOGF_DEBUG("Moving %s at %.1fx sidereal rate (%.2f arcsec/s)", (xSidereal>=0 ? "west" : "east"), abs(xSidereal), abs(arcsecPerSec));

	if(!applyLimits(arcsecPerSec, 0))
		return false;

	if(!stepperHA.setTargetVelocityArcsecPerSec(arcsecPerSec)) {
		LOG_ERROR("MoveWE");
		return false;
	}

	manualSlewArcsecPerSecRA=arcsecPerSec;
    guiderActiveRA=false;  // avoid leftover guider pulse overriding manual movement on this axis
	return true;
}


bool PimocoMount::SetSlewRate(int index) {
	return true;
}
