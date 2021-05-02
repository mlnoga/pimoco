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
#include <time.h>


void PimocoMount::TimerHit() {
	if(!isConnected())
		return;

	bool rc;
	if(guiderActiveRA || guiderActiveDec) 
		rc=guiderTimerHit(); 	// guiding is time-critical, avoid general scope updates
	else
		rc=ReadScopeStatus();

	if(!rc) {
        EqNP.s = IPS_ALERT;
        IDSetNumber(&EqNP, nullptr);
    }

	uint32_t pollingPeriod=getNextTimerInterval();
    SetTimer(pollingPeriod);
}


uint32_t PimocoMount::getNextTimerInterval() {
	if(TrackState==SCOPE_TRACKING && (guiderActiveRA || guiderActiveDec)) {
		// if guiding, closest guider timeout determines the delay 
		uint64_t ms=getGuiderTimerInterval();
		if(ms>0)
			return (uint32_t) ms;
		guiderTimerHit();
		return getNextTimerInterval();		
	} else if(TrackState==SCOPE_SLEWING) {
	    // during gotos, continuously update the HA target position based on system time		
		return 100; 
	} else
		return getCurrentPollingPeriod();
}


bool PimocoMount::ReadScopeStatus() {
	// get local hour angle and declination angle from scope
	double localHaHours, decDegrees;
	if(!stepperHA.getPositionHours(&localHaHours) || !stepperDec.getPositionDegrees(&decDegrees))
		return false;

 	// calculate RA and Dec
   	double last=getLocalSiderealTime();
   	double raHours=range24(last - localHaHours); 
   	decDegrees=rangeDecNative(decDegrees);

    // update pier side based on native Dec position 
    TelescopePierSide pierSide=(abs(decDegrees)<=90) ? PIER_EAST : PIER_WEST;
    setPierSide(pierSide);

	switch(TrackState) {
        case SCOPE_IDLE:
        	if(manualSlewArcsecPerSecRA!=0 || manualSlewArcsecPerSecDec!=0)
        		if(!applyLimitsPosSpeed(manualSlewArcsecPerSecRA, manualSlewArcsecPerSecDec))
        			return false;
        	break;

        case SCOPE_SLEWING:
    		if(!stepperHA.hasReachedTargetPos()) {
	        	// while HA axis is moving, reissue HA goto command updated with current time
				double last=getLocalSiderealTime();
	   			double ha  =rangeHA(last - gotoTargetRA); 
				if(!stepperHA.setTargetPositionHours(ha, wasTrackingBeforeSlew ? stepperHA.arcsecPerSecToNative(getTrackRateRA()) : 0 )) {
					LOG_ERROR("Updating goto HA target");
					return false;
				}
        	} else if(stepperDec.hasReachedTargetPos()) {
        		// physical axis tracking has been re-enabled by the ISRs already
        		// restore tracking state visible to INDI once both axes have reached target
        		LOGF_INFO("Goto reached target position RA %f Dec %f", gotoTargetRA, gotoTargetDec);
				manualSlewArcsecPerSecRA=manualSlewArcsecPerSecDec=0;
				guiderActiveRA=guiderActiveDec=false;
        		TrackState=wasTrackingBeforeSlew ? SCOPE_TRACKING : SCOPE_IDLE;
        	}
        	break;

        case SCOPE_TRACKING:
        	if(manualSlewArcsecPerSecRA!=0 || manualSlewArcsecPerSecDec!=0) {
        		if(!applyLimitsPosSpeed(manualSlewArcsecPerSecRA, manualSlewArcsecPerSecDec))
        			return false;
        	} else
			    applyTracking();
        	break;

        case SCOPE_PARKING:
        	if(stepperHA.hasReachedTargetPos() && stepperDec.hasReachedTargetPos())
	        	SetParked(true); // updates TrackState and calls WriteParkData()
        	break;

        case SCOPE_PARKED:
        	break;  // do nothing 
   	}

    NewRaDec(raHours, decDegrees); 

	return true;
}

