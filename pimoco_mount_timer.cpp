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
	double deviceHA, deviceDec, equRA, equDec; // hour angle and RA in hours, declination in degrees
    TelescopePierSide equPS;

	if(!stepperHA.getPositionHours(&deviceHA) || !stepperDec.getPositionDegrees(&deviceDec))
		return false;

    equatorialFromDevice(&equRA, &equDec, &equPS, deviceHA, deviceDec);
    NewRaDec(equRA, equDec); 
    setPierSide(equPS);

	switch(TrackState) {
        case SCOPE_IDLE:
        	if(manualSlewArcsecPerSecRA!=0 || manualSlewArcsecPerSecDec!=0)
        		if(!applyLimitsPosSpeed(manualSlewArcsecPerSecRA, manualSlewArcsecPerSecDec))
        			return false;
        	break;

        case SCOPE_SLEWING:
    		if(!stepperHA.hasReachedTargetPos()) {
	        	// while HA axis is moving, recalculate HA target based on current time
                double deviceHA, deviceDec;
                bool valid=deviceFromEquatorial(&deviceHA, &deviceDec, gotoTargetRA, gotoTargetDec, gotoTargetPS);
                if(!valid) {
                	// deal with edge case that goto target has moved too far beyond the meridian 
                	// since the goto command was issued, which can be healed with a flip.
         			auto newTargetPS= (gotoTargetPS==PIER_WEST) ? PIER_EAST : PIER_WEST;
                	valid=deviceFromEquatorial(&deviceHA, &deviceDec, gotoTargetRA, gotoTargetDec, newTargetPS);
                	if(!valid) {
	                	Abort();
	                	break;                		
                	}
                }

                // check current HA position
                double curDeviceHA;
                if(!stepperHA.getPositionHours(&curDeviceHA)) {
                    LOG_ERROR("HA: Reading position");
                    return false;
                }
                double absHADistArcsec=abs(deviceHA-curDeviceHA)*60*60;
                if(stepperHA.getDebugLevel()>=Stepper::TMC_DEBUG_DEBUG)
                    LOGF_DEBUG("HA: target %f device %f abs_distance_arcsec %f", deviceHA, curDeviceHA, absHADistArcsec);

                // reissue goto command if HA difference is above threshold
                if(absHADistArcsec>=0.25) {
     				if(!stepperHA.setTargetPositionHours(deviceHA, wasTrackingBeforeSlew ? stepperHA.arcsecPerSecToNative(getTrackRateRA()) : 0 )) {
	   				    LOG_ERROR("HA: Updating goto target");
		  			 return false;
				    }
                }
        	} else if(stepperDec.hasReachedTargetPos()) {
        		// physical axis tracking has been re-enabled by the ISRs already
        		// restore tracking state visible to INDI once both axes have reached target
        		LOGF_INFO("Goto reached target RA %f Dec %f pier %s device HA %f Dec %f", equRA, equDec, getPierSideStr(equPS), deviceHA, deviceDec);
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


	return true;
}

