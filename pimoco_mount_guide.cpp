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


IPState PimocoMount::GuideNorth(uint32_t ms) {
	if(TrackState!=SCOPE_TRACKING) {
		LOG_ERROR("Can only guide while tracking");
		return IPS_ALERT;
	} else if(ms>GuiderMaxPulseN[0].value && GuiderMaxPulseN[0].value>0) {
		LOGF_WARN("Restricting guider pulse of %d ms to maximum of %d ms.", ms, GuiderMaxPulseN[0].value);
		ms=GuiderMaxPulseN[0].value;
	}

	// Indi: North is defined as DEC+
	double arcsecPerSec=getTrackRateDec() + GuiderSpeedN[0].value * trackRates[0];
	if(!stepperDec.setTargetVelocityArcsecPerSec(arcsecPerSec))
		return IPS_ALERT;

	guiderActiveDec=true;
	guiderTimeoutDec=getTimeMillis()+ms;

	if(!guiderActiveRA || guiderTimeoutDec<=guiderTimeoutRA)
		SetTimer(ms);

	//LOGF_INFO("Guide north %d ms speed %f", ms, arcsecPerSec);

	return IPS_BUSY;
}

IPState PimocoMount::GuideSouth(uint32_t ms) {
	if(TrackState!=SCOPE_TRACKING) {
		LOG_ERROR("Can only guide while tracking");
		return IPS_ALERT;
	} else if(ms>GuiderMaxPulseN[0].value && GuiderMaxPulseN[0].value>0) {
		LOGF_WARN("Restricting guider pulse of %d ms to maximum of %d ms.", ms, GuiderMaxPulseN[0].value);
		ms=GuiderMaxPulseN[0].value;
	}

	// Indi: South is defined as DEC-
	double arcsecPerSec=getTrackRateDec() - GuiderSpeedN[0].value * trackRates[0];
	if(!stepperDec.setTargetVelocityArcsecPerSec(arcsecPerSec))
		return IPS_ALERT;

	guiderActiveDec=true;
	guiderTimeoutDec=getTimeMillis()+ms;

	if(!guiderActiveRA || guiderTimeoutDec<=guiderTimeoutRA)
		SetTimer(ms);

	//LOGF_INFO("Guide south %d ms speed %f", ms, arcsecPerSec);

	return IPS_BUSY;
}

IPState PimocoMount::GuideEast(uint32_t ms) {
	if(TrackState!=SCOPE_TRACKING) {
		LOG_ERROR("Can only guide while tracking");
		return IPS_ALERT;
	} else if(ms>GuiderMaxPulseN[0].value && GuiderMaxPulseN[0].value>0) {
		LOGF_WARN("Restricting guider pulse of %d ms to maximum of %d ms.", ms, GuiderMaxPulseN[0].value);
		ms=GuiderMaxPulseN[0].value;
	}

	// Indi: East is defined as RA+, so HA-
	double arcsecPerSec=getTrackRateRA() - GuiderSpeedN[0].value * trackRates[0];
	if(!stepperHA.setTargetVelocityArcsecPerSec(arcsecPerSec))
		return IPS_ALERT;

	guiderActiveRA=true;
	guiderTimeoutRA=getTimeMillis()+ms;

	if(!guiderActiveDec || guiderTimeoutRA<=guiderTimeoutDec)
		SetTimer(ms);

	//LOGF_INFO("Guide east %d ms speed %f", ms, arcsecPerSec);

	return IPS_BUSY;
}

IPState PimocoMount::GuideWest(uint32_t ms) {
	if(TrackState!=SCOPE_TRACKING) {
		LOG_ERROR("Can only guide while tracking");
		return IPS_ALERT;
	} else if(ms>GuiderMaxPulseN[0].value && GuiderMaxPulseN[0].value>0) {
		LOGF_WARN("Restricting guider pulse of %d ms to maximum of %d ms.", ms, GuiderMaxPulseN[0].value);
		ms=GuiderMaxPulseN[0].value;
	}

	// Indi: West is defined as RA-, so HA+
	double arcsecPerSec=getTrackRateRA() + GuiderSpeedN[0].value * trackRates[0];
	if(!stepperHA.setTargetVelocityArcsecPerSec(arcsecPerSec))
		return IPS_ALERT;

	guiderActiveRA=true;
	guiderTimeoutRA=getTimeMillis()+ms;

	if(!guiderActiveDec || guiderTimeoutRA<=guiderTimeoutDec)
		SetTimer(ms);

	//LOGF_INFO("Guide west %d ms speed %f", ms, arcsecPerSec);

	return IPS_BUSY;
}
