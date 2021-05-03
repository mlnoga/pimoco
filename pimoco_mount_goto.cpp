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


bool PimocoMount::Sync(double equRA, double equDec) {
    LOGF_INFO("Syncing to equatorial position RA %f Dec", equRA, equDec);

    double deviceHA, deviceDec;
    deviceFromEquatorial(&deviceHA, &deviceDec, equRA, equDec, getPierSide());

    return SyncDeviceHADec(deviceHA, deviceDec);
}


bool PimocoMount::SyncDeviceHADec(double deviceHA, double deviceDec) {
   	LOGF_INFO("Syncing to device position HA %f Dec %f", deviceHA, deviceDec);

	if(!stepperHA.syncPositionHours(deviceHA) || !stepperDec.syncPositionDegrees(deviceDec) ) {
		LOG_ERROR("Syncing position");
		return false;
	}
	return true;
}


bool PimocoMount::Goto(double equRA, double equDec, TelescopePierSide equPS, bool forcePierSide) {
    if(!checkLimitsPosAlt(equRA, equDec)) {
        LOGF_ERROR("Goto RA %f Dec %f outside mount altitude limits [%f, %f]", 
                   equRA, equDec, AltLimitsN[0].value, AltLimitsN[1].value);
        return false;
    }

    double deviceHA, deviceDec;
    deviceFromEquatorial(&deviceHA, &deviceDec, equRA, equDec, equPS);

    if(!checkLimitsPosHA(deviceHA, deviceDec)) {
        if(forcePierSide) {
            LOGF_ERROR("Goto RA %f Dec %f %s outside mount HA limits [%f, %f]",
                       equRA, equDec, getPierSideStr(equPS), HALimitsN[0].value, HALimitsN[1].value);
            return false;
        } else { // try meridian flip
            equPS= (equPS==PIER_WEST) ? PIER_EAST : PIER_WEST;
            deviceFromEquatorial(&deviceHA, &deviceDec, equRA, equDec, equPS);

            if(!checkLimitsPosHA(deviceHA, deviceDec))  {
                LOGF_ERROR("Goto RA %f Dec %f outside mount HA limits [%f, %f] on both pier sides",
                           equRA, equDec, HALimitsN[0].value, HALimitsN[1].value);
                return false;
            }
        }
    }

   	LOGF_INFO("Goto equatorial RA %f Dec %f %s device HA %f Dec %f", equRA, equDec, getPierSideStr(equPS), deviceHA, deviceDec);

 	if(TrackState==SCOPE_TRACKING)
 		wasTrackingBeforeSlew=true;
 	else if(TrackState==SCOPE_IDLE)
 		wasTrackingBeforeSlew=false;
 	else
 		; // don't touch

	if(!stepperHA.setTargetPositionHours(deviceHA, wasTrackingBeforeSlew ? stepperHA.arcsecPerSecToNative(getTrackRateRA()) : 0) || 
	   !stepperDec.setTargetPositionDegrees(deviceDec, wasTrackingBeforeSlew ? stepperDec.arcsecPerSecToNative(getTrackRateDec()) : 0) ) {
		LOG_ERROR("Goto");
		return false;
	}

	// cache target equatorial ra/dec/pier side for periodic reissue of HA-based device targets as time proceeds
	gotoTargetRA =equRA;  
	gotoTargetDec=equDec;
    gotoTargetPS =equPS;
  	manualSlewArcsecPerSecRA=manualSlewArcsecPerSecDec=0;
	guiderActiveRA=guiderActiveDec=false;

	SetTimer(100);  // Workaround: increase polling frequency to continuously update HA target during slew

    TrackState = SCOPE_SLEWING;
  	return true;
}


bool PimocoMount::Goto(double equRA, double equDec) {
    return Goto(equRA, equDec, getPierSide(), true);
}
