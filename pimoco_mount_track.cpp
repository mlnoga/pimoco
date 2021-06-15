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
#include "pimoco_time.h"
#include <libindi/indilogger.h>
#include <libindi/indicom.h>  // for rangeHA etc.


bool PimocoMount::Abort() {
	LOG_INFO("Aborting all motion");
	if(!stepperHA .setTargetVelocityArcsecPerSec(0) ||
  	   !stepperDec.setTargetVelocityArcsecPerSec(0)    ) {
		LOG_ERROR("Aborting all motion");
		return false;
	}

	manualSlewArcsecPerSecRA=manualSlewArcsecPerSecDec=0;
	guiderActiveRA=guiderActiveDec=false;
	if(TrackState!=SCOPE_PARKED)
		TrackState=SCOPE_IDLE;
	return true;
}


bool PimocoMount::SetTrackEnabled(bool enabled) {
	LOG_INFO(enabled ? "Enabling tracking" : "Disabling tracking");
	TrackState=enabled ? SCOPE_TRACKING : SCOPE_IDLE; 
	// caller resets this if return code is false
	return applyTracking();
}


bool PimocoMount::SetTrackMode(uint8_t mode) {
	if(mode>TRACK_KING) {
		LOGF_ERROR("Invalid tracking mode %d", mode);
		return false;
	}
	double rateRA=getTrackRateRA(), rateDec=getTrackRateDec();
	LOGF_INFO("Selecting %s tracking (mode %d) with rate RA %.4f Dec %.4f arcsec/s", 
	       	  trackRateLabels[mode], mode, rateRA, rateDec);

	return applyTracking();
}


bool PimocoMount::syncTrackRate() {
	// get time and position
	Timestamp ts;
	uint64_t now=ts.ms();
	double deviceHA, deviceDec;
	if(!stepperHA.getPositionHours(&deviceHA) || !stepperDec.getPositionDegrees(&deviceDec))
		return false;

	// update tracking rate if prior sync is available
	if(SyncTrackRateS[0].s==ISS_ON) {
		uint64_t deltaMs=now - syncTrackRateMs;
		double arcsecPerSecRA =(deviceHA  - syncTrackRateHA )*1000.0*60.0*60.0*15.0/(double) deltaMs; // HA is in hours
		double arcsecPerSecDec=(deviceDec - syncTrackRateDec)*1000.0*60.0*60.0     /(double) deltaMs; // Dec is in degrees
		SetTrackRate(arcsecPerSecRA, arcsecPerSecDec);
	    SyncTrackRateSP.s=IPS_OK;
	} else
	    SyncTrackRateSP.s=IPS_BUSY;

	// update sync position and and trigger GUI update
	syncTrackRateHA =deviceHA;
	syncTrackRateDec=deviceDec;
	syncTrackRateMs =now;
	SyncTrackRateS[0].s=ISS_ON;
    IDSetSwitch(&SyncTrackRateSP, nullptr);
    return true;
}


bool PimocoMount::SetTrackRate(double rateRA, double rateDec) {
	trackRateCustomRA =rateRA;
	trackRateCustomDec=rateDec;
	LOGF_INFO("Setting custom tracking rate to RA %.3f Dec %.3f arcsec/s", rateRA, rateDec);

	return applyTracking();
}


double PimocoMount::getArcsecPerSecHA() {
    if(TrackState==SCOPE_IDLE && manualSlewArcsecPerSecRA!=0) 
        return manualSlewArcsecPerSecRA;
    else if(TrackState==SCOPE_TRACKING && manualSlewArcsecPerSecRA!=0)
        return manualSlewArcsecPerSecRA;
    else if(TrackState==SCOPE_TRACKING)
        return getTrackRateRA();
    return 0; 
}


double PimocoMount::getArcsecPerSecDec() {
    if(TrackState==SCOPE_IDLE && manualSlewArcsecPerSecDec!=0) 
        return manualSlewArcsecPerSecDec;
    else if(TrackState==SCOPE_TRACKING && manualSlewArcsecPerSecDec!=0)
        return manualSlewArcsecPerSecDec;
    else if(TrackState==SCOPE_TRACKING)
        return getTrackRateDec();
    return 0; 
}


bool PimocoMount::applyTracking(bool updateRA, bool updateDec) {
	double rateRA =(TrackState==SCOPE_TRACKING) ? getTrackRateRA()  : 0;
	double rateDec=(TrackState==SCOPE_TRACKING) ? getTrackRateDec() : 0;

	if(!applyLimits(rateRA, rateDec))
		return false;

	if((updateRA  && !stepperHA .setTargetVelocityArcsecPerSec(rateRA )) ||
  	   (updateDec && !stepperDec.setTargetVelocityArcsecPerSec(rateDec))    ) {
		LOG_ERROR("Setting tracking speed");
		return false;
	}		

	return true;
}
