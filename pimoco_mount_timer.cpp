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
#include <libnova/julian_day.h>
#include <libnova/sidereal_time.h>

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
	if(DeviceCoordNP.s!=EqNP.s || TimeNP.s!=EqNP.s || AltAzNP.s!=EqNP.s) {
        DeviceCoordNP.s = TimeNP.s = AltAzNP.s = EqNP.s;
        IDSetNumber(&DeviceCoordNP, nullptr);
        IDSetNumber(&TimeNP, nullptr);
        IDSetNumber(&AltAzNP, nullptr);
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
	    // during gotos, when close continuously update the HA target position based on system time
	    double distLimit=gotoSpeedupPollingDegrees*0.001*(double)getCurrentPollingPeriod();
    	if((abs(EqN[0].value-gotoTargetRA)*15 < distLimit) && 
    	   (abs(EqN[1].value-gotoTargetDec)   < distLimit)    )
    		return 100; 
    	// fallthrough
	}
	return getCurrentPollingPeriod();
}


bool PimocoMount::ReadScopeStatus() {
	// update device coordinates
	double deviceHA, deviceDec; // hour angle in hours, declination in degrees
	if(!stepperHA.getPositionHours(&deviceHA) || !stepperDec.getPositionDegrees(&deviceDec))
	   	return false;
	DeviceCoordN[0].value=deviceHA;
	DeviceCoordN[1].value=deviceDec;

	// check device HA limits
	double arcsecPerSecHA=getArcsecPerSecHA();
	if(!checkLimitsHA(deviceHA, arcsecPerSecHA)) {
		Abort();
	   	return false;
	}

	// update time
	double jd=ln_get_julian_from_sys();
	double lst=range24(ln_get_apparent_sidereal_time(jd) - (360.0 - LocationN[LOCATION_LONGITUDE].value) / 15.0);
	TimeN[0].value=jd;
	TimeN[1].value=lst;

	// update equatorial coordinates
	double equRA, equDec; // RA in hours, declination in degrees
    TelescopePierSide equPS;
    equatorialFromDevice(&equRA, &equDec, &equPS, deviceHA, deviceDec, lst);
    NewRaDec(equRA, equDec); 
    setPierSide(equPS);

    // update horizon coordinates
    double horAlt, horAz;
    horizonFromEquatorial(&horAlt, &horAz, equRA, equDec, jd);
    AltAzN[0].value=horAlt;
    AltAzN[1].value=horAz;
	AltAzNP.s=IPS_OK;

	// check device Alt limits
	double arcsecPerSecDec=getArcsecPerSecDec();
	if(!checkLimitsAlt(horAlt, deviceHA, deviceDec, arcsecPerSecHA, arcsecPerSecDec, jd, lst)) {
		Abort();
	   	return false;
	}

	switch(TrackState) {
        case SCOPE_IDLE:
        	break; // do nothing

        case SCOPE_SLEWING:
    		if(!stepperHA.hasReachedTargetPos()) {
	        	// while HA axis is moving, recalculate HA target based on current time
                double targetDevHA, targetDevDec;
                bool valid=deviceFromEquatorial(&targetDevHA, &targetDevDec, gotoTargetRA, gotoTargetDec, gotoTargetPS);
                if(!valid) {
                	// deal with edge case that goto target has moved too far beyond the meridian 
                	// since the goto command was issued, which can be healed with a flip.
         			auto newTargetPS= (gotoTargetPS==PIER_WEST) ? PIER_EAST : PIER_WEST;
                	valid=deviceFromEquatorial(&targetDevHA, &targetDevDec, gotoTargetRA, gotoTargetDec, newTargetPS);
                	if(!valid) {
	                	Abort();
					   	return false;
                	}
                }

                // reissue goto command if HA difference is above threshold
                double absHADistArcsec=abs(targetDevHA-deviceHA)*60*60;
                if(absHADistArcsec>=0.25) {
     				if(!stepperHA.setTargetPositionHours(targetDevHA, wasTrackingBeforeSlew ? stepperHA.arcsecPerSecToNative(getTrackRateRA()) : 0 )) {
	   					LOG_ERROR("HA: Updating goto target");
	   					Abort();
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
        	break; // do nothing

        case SCOPE_PARKING:
        	if(stepperHA.hasReachedTargetPos() && stepperDec.hasReachedTargetPos())
	        	SetParked(true); // updates TrackState and calls WriteParkData()
        	break;

        case SCOPE_PARKED:
        	break;  // do nothing 
   	}

   	return true;
}
