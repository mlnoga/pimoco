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
#include <libnova/julian_day.h>
#include <libnova/sidereal_time.h>


const double PimocoMount::gotoSpeedupPollingDegrees=5;


bool PimocoMount::Sync(double equRA, double equDec) {
    LOGF_INFO("Syncing to RA %f Dec %f", equRA, equDec);

    double deviceHA, deviceDec;
    bool valid=deviceFromEquatorial(&deviceHA, &deviceDec, equRA, equDec, getPierSide());
    if(!valid) {
        LOGF_ERROR("Syncing position: invalid device HA %f Dec %f", deviceHA, deviceDec);
        return false;
    }

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
    // calculate horizontal alt/az coordinates of the target
    double jd=ln_get_julian_from_sys();
    double lst=range24(ln_get_apparent_sidereal_time(jd) - (360.0 - LocationN[LOCATION_LONGITUDE].value) / 15.0);
    double horAlt, horAz;
    horizonFromEquatorial(&horAlt, &horAz, equRA, equDec, jd);

    if(!checkLimitsAlt(horAlt)) {
        LOGF_ERROR("Goto RA %f Dec %f outside mount altitude limits [%f, %f]", 
                   equRA, equDec, AltLimitsN[0].value, AltLimitsN[1].value);
        return false;
    }

    // Deal with Indi idiom: Goto to same coordinates requests meridian flip
    if(!forcePierSide) {
	    double distRA=equRA-EqN[0].value, distDec=equDec-EqN[1].value;
    	double distArcsec=sqrt(distRA*distRA*(360.0/24.0)*(360.0/24.0) + distDec*distDec)*60.0*60.0;
    	if(distArcsec<=0.5) {
    		equPS = (equPS==PIER_WEST) ? PIER_EAST : PIER_WEST;
    		LOGF_INFO("Distance %.1f arcsec, flip requested", distArcsec);
    	}
	}

    // Calculate device hour angle and declination coordinates of the target 
    double deviceHA, deviceDec;
    bool valid=deviceFromEquatorial(&deviceHA, &deviceDec, equRA, equDec, equPS, lst);

    if(!valid) {
        if(forcePierSide) {
            LOGF_ERROR("Goto RA %f Dec %f pier %s device HA %f Dec %f outside mount HA limits [%f, %f]",
                       equRA, equDec, getPierSideStr(equPS), deviceHA, deviceDec, HALimitsN[0].value, HALimitsN[1].value);
            return false;
        } else { // try meridian flip
            LOGF_WARN("Goto RA %f Dec %f pier %s device HA %f Dec %f outside mount HA limits [%f, %f], trying other side",
                       equRA, equDec, getPierSideStr(equPS), deviceHA, deviceDec, HALimitsN[0].value, HALimitsN[1].value);

            equPS= (equPS==PIER_WEST) ? PIER_EAST : PIER_WEST;
            bool valid=deviceFromEquatorial(&deviceHA, &deviceDec, equRA, equDec, equPS, lst);

            if(!valid)  {
                LOGF_ERROR("Goto RA %f Dec %f pier %s device HA %f Dec %f outside mount HA limits [%f, %f]",
                           equRA, equDec, getPierSideStr(equPS), deviceHA, deviceDec, HALimitsN[0].value, HALimitsN[1].value);
                return false;
            }
        }
    }

   	LOGF_INFO("Goto RA %f Dec %f pier %s device HA %f Dec %f", equRA, equDec, getPierSideStr(equPS), deviceHA, deviceDec);

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

    double distLimit=gotoSpeedupPollingDegrees*0.001*(double)getCurrentPollingPeriod();
    if((abs(EqN[0].value-gotoTargetRA)*15 < distLimit) && 
       (abs(EqN[1].value-gotoTargetDec)   < distLimit)    )
    	SetTimer(100);  // Workaround: when close, increase polling frequency to continuously update HA target during slew

    TrackState = SCOPE_SLEWING;
  	return true;
}


bool PimocoMount::Goto(double equRA, double equDec) {
    return Goto(equRA, equDec, getPierSide(), false);
}
