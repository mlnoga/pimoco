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


bool PimocoMount::Sync(double ra, double dec) {
	double last=getLocalSiderealTime();
   	double ha  =rangeHA(last - ra); 

   	LOGF_INFO("Syncing position to RA %f (HA %f) Dec %f", ra, ha, dec);

	if(!stepperHA.syncPositionHours(ha) || !stepperDec.syncPositionDegrees(dec) ) {
		LOG_ERROR("Syncing position");
		return false;
	}
	return true;
}


bool PimocoMount::SyncHADec(double ha, double dec) {
	double last=getLocalSiderealTime();
   	double ra  =rangeHA(last - ha); 

   	LOGF_INFO("Syncing position to RA %f (HA %f) Dec %f", ra, ha, dec);

	if(!stepperHA.syncPositionHours(ha) || !stepperDec.syncPositionDegrees(dec) ) {
		LOG_ERROR("Syncing position");
		return false;
	}
	return true;
}


bool PimocoMount::Goto(double ra, double dec) {
    // check alt limits, abort if target lies outside
	if(!checkLimitsPos(ra, dec)) {
   		LOGF_ERROR("Goto RA %f Dec %f outside mount altitude limits [%f, %f]", 
   			       ra, dec, AltLimitsN[0].value, AltLimitsN[1].value);
   		return false;
   	}

   	// convert RA to hour angle to prepare goto
	double last=getLocalSiderealTime();
   	double ha  =rangeHA(last - ra); 
   	LOGF_INFO("Goto RA %f (HA %f) Dec %f", ra, ha, dec);

 	if(TrackState==SCOPE_TRACKING)
 		wasTrackingBeforeSlew=true;
 	else if(TrackState==SCOPE_IDLE)
 		wasTrackingBeforeSlew=false;
 	else
 		; // don't touch

	if(!stepperHA.setTargetPositionHours(ha, wasTrackingBeforeSlew ? stepperHA.arcsecPerSecToNative(getTrackRateRA()) : 0) || 
	   !stepperDec.setTargetPositionDegrees(dec, wasTrackingBeforeSlew ? stepperDec.arcsecPerSecToNative(getTrackRateDec()) : 0) ) {
		LOG_ERROR("Goto");
		return false;
	}

	// cache target ra/dec for periodic reissue of ha-based target given progressing time
	gotoTargetRA=ra;
	gotoTargetDec=dec;
  	manualSlewArcsecPerSecRA=manualSlewArcsecPerSecDec=0;
	guiderActiveRA=guiderActiveDec=false;

	SetTimer(100);  // Workaround: increase polling frequency to continuously update HA target during slew

    TrackState = SCOPE_SLEWING;
  	return true;
}
