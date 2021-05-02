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
	// caller sets TrackState=SCOPE_TRACKING, no need to do it here
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


bool PimocoMount::SetTrackRate(double rateRA, double rateDec) {
	trackRateCustomRA =rateRA;
	trackRateCustomDec=rateDec;
	LOGF_INFO("Setting custom tracking rate to RA %.3f Dec %.3f arcsec/s", rateRA, rateDec);

	return applyTracking();
}


bool PimocoMount::applyTracking(bool updateRA, bool updateDec) {
	double rateRA =(TrackState==SCOPE_TRACKING) ? getTrackRateRA()  : 0;
	double rateDec=(TrackState==SCOPE_TRACKING) ? getTrackRateDec() : 0;

	if(!checkMotionAgainstMountLimitsAndStopIfReached(rateRA, rateDec))
		return false;

	if((updateRA  && !stepperHA .setTargetVelocityArcsecPerSec(rateRA )) ||
  	   (updateDec && !stepperDec.setTargetVelocityArcsecPerSec(rateDec))    ) {
		LOG_ERROR("Setting tracking speed");
		return false;
	}		

	return true;
}
