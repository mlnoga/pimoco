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

#define CDRIVER_VERSION_MAJOR	1
#define CDRIVER_VERSION_MINOR	0


// Singleton instance
PimocoMount mount;

const double PimocoMount::trackRates[]={
    15.041067, // TRACK_SIDEREAL
    15.0,      // TRACK_SOLAR
    14.685,    // TRACK_LUNAR
    15.041067, // TRACK_CUSTOM
    15.0369,   // King tracking rate, not defined in INDI standard
};

const char *PimocoMount::trackRateNames[]={
    "SIDEREAL", // TRACK_SIDEREAL
    "SOLAR",    // TRACK_SOLAR
    "LUNAR",    // TRACK_LUNAR
    "CUSTOM",   // TRACK_CUSTOM
    "KING",     // King tracking rate, not defined in INDI standard
};

const char *PimocoMount::trackRateLabels[]={
    "Sidereal", // TRACK_SIDEREAL
    "Solar",    // TRACK_SOLAR
    "Lunar",    // TRACK_LUNAR
    "Custom",   // TRACK_CUSTOM
    "King",     // King tracking rate, not defined in INDI standard
};

const char *PimocoMount::HA_TAB="Hour angle";
const char *PimocoMount::DEC_TAB="Declination";


// C function interface redirecting to singleton
//
extern "C" {

void ISGetProperties(const char *dev) {
	mount.ISGetProperties(dev);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n) {
	mount.ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
	mount.ISNewNumber(dev, name, values, names, n);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
	mount.ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) {
	mount.ISNewText(dev, name, texts, names, n);
}

void ISSnoopDevice(XMLEle *root) {
	mount.ISSnoopDevice(root);
}

} // extern "C"


// Public class members
//

PimocoMount::PimocoMount() : stepperHA(getDeviceName(), HA_DIAG0_PIN), stepperDec(getDeviceName(), DEC_DIAG0_PIN),
    spiDeviceFilenameHA("/dev/spidev1.0"), spiDeviceFilenameDec("/dev/spidev1.1") {
	setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);

	SetTelescopeCapability(
       		TELESCOPE_CAN_GOTO |
            TELESCOPE_CAN_SYNC |
            TELESCOPE_CAN_PARK |
            TELESCOPE_CAN_ABORT |
            TELESCOPE_HAS_TIME |
            TELESCOPE_HAS_LOCATION |
            // TELESCOPE_HAS_PIER_SIDE |
            // TELESCOPE_HAS_PEC |
            TELESCOPE_HAS_TRACK_MODE |
            TELESCOPE_CAN_CONTROL_TRACK |
            TELESCOPE_HAS_TRACK_RATE |
            //TELESCOPE_HAS_PIER_SIDE_SIMULATION |
            //TELESCOPE_CAN_TRACK_SATELLITE |
            0, NUM_SLEW_RATES);

	setTelescopeConnection(CONNECTION_NONE);

    SetParkDataType(PARK_HA_DEC);
}

PimocoMount::~PimocoMount() {
	stepperHA.stop();
	stepperDec.stop();
}

const char *PimocoMount::getDefaultName() {
	return "Pimoco mount";
}

bool PimocoMount::initProperties() {
	if(!INDI::Telescope::initProperties()) 
		return false;
 	initGuiderProperties(getDeviceName(), GUIDE_TAB); // returns void, cannot fail

	// Create the four standard tracking modes
	AddTrackMode(trackRateNames[TRACK_SIDEREAL], trackRateLabels[TRACK_SIDEREAL], true);
	AddTrackMode(trackRateNames[TRACK_SOLAR],    trackRateLabels[TRACK_SOLAR],    false);
	AddTrackMode(trackRateNames[TRACK_LUNAR],    trackRateLabels[TRACK_LUNAR],    false);
	AddTrackMode(trackRateNames[TRACK_CUSTOM],   trackRateLabels[TRACK_CUSTOM],   false);

	// Initialize stepper properties
	stepperHA .initProperties( HAMotorN, & HAMotorNP,  HAMSwitchS, & HAMSwitchSP, HARampN, & HARampNP,  
							  "HA_MOTOR", "Motor", "HA_MSWITCH", "Switches", "HA_RAMP", "Ramp", HA_TAB);
	stepperDec.initProperties(DecMotorN, &DecMotorNP, DecMSwitchS, &DecMSwitchSP, DecRampN, &DecRampNP, 
							  "DEC_MOTOR", "Motor", "DEC_MSWITCH", "Switches", "DEC_RAMP", "Ramp", DEC_TAB);

	// Initialize mount properties
	IUFillNumber(&SlewRatesN[0], SlewRateS[0].name, SlewRateS[0].label, "%.1f", 0, 1600, 16, 0.5);
	IUFillNumber(&SlewRatesN[1], SlewRateS[1].name, SlewRateS[1].label, "%.1f", 0, 1600, 16, 16);
	IUFillNumber(&SlewRatesN[2], SlewRateS[2].name, SlewRateS[2].label, "%.1f", 0, 1600, 16, 250);
	IUFillNumber(&SlewRatesN[3], SlewRateS[3].name, SlewRateS[3].label, "%.1f", 0, 1600, 16, 1000);
	IUFillNumberVector(&SlewRatesNP, SlewRatesN, NUM_SLEW_RATES, getDeviceName(), "SLEW_RATES", "Slew rates [x sidereal]", MOTION_TAB, IP_RW, 0, IPS_IDLE);

	// load configuration data from file, as there is no device with own storage
	loadConfig(true, HAMotorNP.name);
	loadConfig(true, HAMSwitchSP.name);
	loadConfig(true, HARampNP.name);

	loadConfig(true, DecMotorNP.name);
	loadConfig(true, DecMSwitchSP.name);
	loadConfig(true, DecRampNP.name);

	loadConfig(true, SlewRatesNP.name);

    addDebugControl();
    return true;
}

bool PimocoMount::updateProperties() {
	if(!INDI::Telescope::updateProperties())
		return false;

	if(!stepperHA .updateProperties(this,  HAMotorN, & HAMotorNP,  HAMSwitchS, & HAMSwitchSP,  HARampN, & HARampNP))
		return false;
	if(!stepperDec.updateProperties(this, DecMotorN, &DecMotorNP, DecMSwitchS, &DecMSwitchSP, DecRampN, &DecRampNP))
		return false;

	if(isConnected()) {
	    defineProperty(&SlewRatesNP);
	} else {
	    deleteProperty(SlewRatesNP.name);
	}

	return true;
}

bool PimocoMount::ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
    	return INDI::Telescope::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);

    // if(strcmp(name, ...)) { } else

	return INDI::Telescope::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

bool PimocoMount::ISUpdateNumber(INumberVectorProperty *NP, double values[], char *names[], int n, bool res) 
{
	if(res) {
		IUUpdateNumber(NP, values, names, n);
		saveConfig(true, NP->name);
	}
	NP->s=res ? IPS_OK : IPS_ALERT;
	IDSetNumber(NP, NULL);
	return res;
}

bool PimocoMount::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
		return INDI::Telescope::ISNewNumber(dev, name, values, names, n);

	int res;
	if((res=stepperHA.ISNewNumber(&HAMotorNP, &HARampNP, name, values, names, n)) > 0) {
		saveConfig(true, HAMotorNP.name);
		saveConfig(true, HARampNP.name);
		return true;
	} else if(res==0)
		return false;

	if((res=stepperDec.ISNewNumber(&DecMotorNP, &DecRampNP, name, values, names, n)) > 0) {
		saveConfig(true, DecMotorNP.name);
		saveConfig(true, DecRampNP.name);
		return true;
	} else if(res==0)
		return false;
    
	if(!strcmp(name, SlewRatesNP.name)) {
        return ISUpdateNumber(&SlewRatesNP, values, names, n, true);		
	}

	if(!strcmp(name, GuideNSNP.name) || !strcmp(name, GuideWENP.name)) {
		processGuiderProperties(name, values, names, n); // does not return a status
		return true;
	}

	return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool PimocoMount::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
		return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);

	int res;
	if((res=stepperHA.ISNewSwitch(&HAMSwitchSP, name, states, names, n)) >0) {
		saveConfig(true, HAMSwitchSP.name);
		return true;
	} else if(res==0)
		return false;

	if((res=stepperDec.ISNewSwitch(&DecMSwitchSP, name, states, names, n)) >0) {
		saveConfig(true, DecMSwitchSP.name);
		return true;
	} else if(res==0)
		return false;

	return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool PimocoMount::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
		return INDI::Telescope::ISNewText(dev, name, texts, names, n);

    // if(strcmp(name, ...)) { } else

	return INDI::Telescope::ISNewText(dev, name, texts, names, n);
}

bool PimocoMount::ISSnoopDevice(XMLEle *root) {
	return INDI::Telescope::ISSnoopDevice(root);
}

// Protected class members
//

bool PimocoMount::saveConfigItems(FILE *fp){
    INDI::Telescope::saveConfigItems(fp);

    IUSaveConfigNumber(fp, &HAMotorNP);
    IUSaveConfigSwitch(fp, &HAMSwitchSP);
    IUSaveConfigNumber(fp, &HARampNP);

    IUSaveConfigNumber(fp, &DecMotorNP);
    IUSaveConfigSwitch(fp, &DecMSwitchSP);
    IUSaveConfigNumber(fp, &DecRampNP);

    IUSaveConfigNumber(fp, &SlewRatesNP);

    return true;
}

bool PimocoMount::Connect() {
	LOGF_INFO("Attempting connection to HA on %s", spiDeviceFilenameHA);
	if(!stepperHA.open(spiDeviceFilenameHA)) {
		LOGF_WARN("Connection to HA on %s failed", spiDeviceFilenameHA);
		return false;
	}
	LOGF_INFO("Connection to HA on %s successful", spiDeviceFilenameHA);

	LOGF_INFO("Attempting connection to Dec on %s", spiDeviceFilenameDec);
	if(!stepperDec.open(spiDeviceFilenameDec)) {
		LOGF_WARN("Connection to Dec on %s failed", spiDeviceFilenameDec);
		return false;
	}
	if(!ReadScopeStatus())
		return false;
	LOGF_INFO("Connection to Dec on %s successful", spiDeviceFilenameDec);

	uint32_t pp=getPollingPeriod();
	if (pp > 0)
		SetTimer(pp);

	return true;
}

bool PimocoMount::Disconnect() {
	if(!stepperHA.close() || !stepperDec.close()) {
		LOG_WARN("Error closing connection");
		return false;
	}
	LOG_INFO("Successfully closed connection");
	return true;
}

bool PimocoMount::Handshake() {
	return true;
}

void PimocoMount::TimerHit() {
	if(!isConnected())
		return;

	if(guiderActiveRA || guiderActiveDec) 
		guiderTimerHit(); 	// guiding is more time-critical, go first
	else
		ReadScopeStatus();
	
	uint32_t pollingPeriod=getNextTimerInterval();
    SetTimer(pollingPeriod);
}

void PimocoMount::guiderTimerHit() {
	struct timespec now;
	clock_gettime(CLOCK_REALTIME, &now);

	if(guiderActiveRA  & isLessThanOrEqual(&guiderTimeoutRA,  &now)) {
		stepperHA.setTargetVelocityArcsecPerSec(getTrackRateRA());
		guiderActiveRA=false;
		GuideComplete(AXIS_RA);
	}

	if(guiderActiveDec & isLessThanOrEqual(&guiderTimeoutDec, &now)) {
		stepperDec.setTargetVelocityArcsecPerSec(getTrackRateDec());
		guiderActiveDec=false;
		GuideComplete(AXIS_DE);
	}
}

bool PimocoMount::ReadScopeStatus() {
	// get local hour angle and declination angle from scope
	double localHaHours, decDegrees;
	if(!stepperHA.getPositionHours(&localHaHours) || !stepperDec.getPositionDegrees(&decDegrees))
		return false;

	// update pier side based on hour angle (basic approach) 
	//auto pierSide=(localHaHours<0) ? PIER_WEST : PIER_EAST;
	//setPierSide(pierSide);

 	// calculate RA and Dec
   	double last=getLocalSiderealTime();
   	double raHours=range24(last - localHaHours); 
   	decDegrees=rangeDec(decDegrees);

	// update scope status
	auto  haStatus=stepperHA .getStatus();
	auto decStatus=stepperDec.getStatus();
	switch(TrackState) {
        case SCOPE_IDLE:
        	break; // do nothing

        case SCOPE_SLEWING:
    		if(!stepperHA.hasReachedTargetPos()) {
	        	// while HA axis is moving, reissue HA goto command updated with current time
				double last=getLocalSiderealTime();
	   			double ha  =rangeHA(last - gotoTargetRA); 
				if(!stepperHA.setTargetPositionHours(ha, wasTrackingBeforeGoto ? stepperHA.arcsecPerSecToNative(getTrackRateRA()) : 0 )) {
					LOG_ERROR("Updating goto HA target");
					return false;
				}
        	} else if(stepperDec.hasReachedTargetPos()) {
        		// restore tracking state visible to INDI once both axes have reached target
        		LOGF_INFO("Goto reached target position RA %f Dec %f", gotoTargetRA, gotoTargetDec);
        		TrackState=wasTrackingBeforeGoto ? SCOPE_TRACKING : SCOPE_IDLE;
        	}
        	break;

        case SCOPE_TRACKING:
        	break; // FIXME correct fractional tracking speed issues

        case SCOPE_PARKING:
        	if((haStatus & Stepper::TMC_POSITION_REACHED) && (decStatus & Stepper::TMC_POSITION_REACHED))
	        	SetParked(true); // updates TrackState and logs
        	break;

        case SCOPE_PARKED:
        	break;  // do nothing 
   	}

    NewRaDec(raHours, decDegrees); 

	return true;
}

uint32_t PimocoMount::getNextTimerInterval() {
	if(TrackState==SCOPE_TRACKING && (guiderActiveRA || guiderActiveDec)) {
		// if guiding, closest guider timeout determines the delay 
		long ms;
		if(guiderActiveRA && (!guiderActiveDec || isLessThanOrEqual(&guiderTimeoutRA, &guiderTimeoutDec)))
			ms=getMsUntil(&guiderTimeoutRA);
		else
			ms=getMsUntil(&guiderTimeoutDec);
		if(ms>0)
			return (uint32_t) ms;
		guiderTimerHit();
		return getNextTimerInterval();		
	} else if(TrackState==SCOPE_SLEWING) {
	    // if slewing, perform reasonably fast updates of HA target position as time passes		
		return 100; 
	} else
		return getCurrentPollingPeriod();
}

bool PimocoMount::Abort() {
	LOG_INFO("Aborting all motion");
	if(!SetTrackEnabled(false))
		return false;
	if(TrackState!=SCOPE_PARKED)
		TrackState=SCOPE_IDLE;
	return true;
}


bool PimocoMount::SetTrackEnabled(bool enabled) {
	if(enabled) {
		uint8_t trackMode=getTrackMode();
		double trackRateRA=getTrackRateRA(), trackRateDec=getTrackRateDec();
		LOGF_INFO("Enabling %s tracking mode (%d) with RA rate %.4f and Dec rate %.4f arcsec/s", trackRateLabels[trackMode], trackMode, trackRateRA, trackRateDec);		
		if(!stepperHA .setTargetVelocityArcsecPerSec(trackRateRA ) ||
	  	   !stepperDec.setTargetVelocityArcsecPerSec(trackRateDec) 	  ) {
			LOG_ERROR("Enabling tracking");
			return false;
		}
	} else {
		LOG_INFO("Disabling tracking");		
		if(!stepperHA .setTargetVelocityArcsecPerSec(0) ||
	  	   !stepperDec.setTargetVelocityArcsecPerSec(0)    ) {
			LOG_ERROR("Disabling tracking");
			return false;
		}
	}
	// caller sets TrackState=SCOPE_TRACKING, no need to do it here
	return true;
}


bool PimocoMount::SetTrackEnabledRA() {
	uint8_t trackMode=getTrackMode();
	double trackRateRA=getTrackRateRA();
	LOGF_INFO("Enabling %s tracking mode (%d) for RA with rate %.4f arcsec/s", trackRateLabels[trackMode], trackMode, trackRateRA);		
	if(!stepperHA .setTargetVelocityArcsecPerSec(trackRateRA ) ) {
		LOG_ERROR("Enabling RA tracking");
		return false;
	}
	return true;
}


bool PimocoMount::SetTrackEnabledDec() {
	uint8_t trackMode=getTrackMode();
	double trackRateDec=getTrackRateDec();
	LOGF_INFO("Enabling %s tracking mode (%d) for Dec with rate %.4f arcsec/s", trackRateLabels[trackMode], trackMode, trackRateDec);		
	if(!stepperDec.setTargetVelocityArcsecPerSec(trackRateDec) ) {
		LOG_ERROR("Enabling Dec tracking");
		return false;
	}
	return true;
}


bool PimocoMount::SetTrackMode(uint8_t mode) {
	if(mode>TRACK_CUSTOM) {
		LOGF_ERROR("Invalid tracking mode %d", mode);
		return false;
	}

	uint8_t trackMode=getTrackMode();
	double trackRateRA=getTrackRateRA(), trackRateDec=getTrackRateDec();
	LOGF_INFO("Selecting %s tracking mode (%d) with RA rate %.4f and Dec rate %.4f arcsec/s", trackRateLabels[trackMode], trackMode, trackRateRA, trackRateDec);		

	if(TrackState==SCOPE_TRACKING)
		if(!stepperHA .setTargetVelocityArcsecPerSec(trackRateRA ) ||
	  	   !stepperDec.setTargetVelocityArcsecPerSec(trackRateDec) 	  ) {
			LOG_ERROR("Enabling tracking");
			return false;
		}
	return true;
}


bool PimocoMount::SetTrackRate(double raRate, double deRate) {
	LOGF_INFO("Setting custom tracking rate to RA %.3f Dec %.3f arcsec/s", raRate, deRate);
	trackRateCustomRA =raRate;
	trackRateCustomDec=deRate;

	// update device if tracking with custom rate
	if(TrackState==SCOPE_TRACKING && getTrackMode()==TRACK_CUSTOM)
		if(!stepperHA .setTargetVelocityArcsecPerSec(raRate) ||
		   !stepperDec.setTargetVelocityArcsecPerSec(deRate)    ) {
			LOGF_ERROR("Error setting custom tracking rate to RA %.3f Dec %.3f arcsec/s", raRate, deRate);
			return false;
		}

	return true;
}


bool PimocoMount::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) {
	double xSidereal=0;
	if(command==MOTION_START) {
		xSidereal=SlewRatesN[IUFindOnSwitchIndex(&SlewRateSP)].value;
		if(dir==DIRECTION_SOUTH)  // DIRECTION_NORTH is positive on the Dec axis
			xSidereal=-xSidereal;
	}
	double arcsecPerSec=xSidereal * trackRates[0];

	LOGF_INFO("Moving %s at %.1fx sidereal rate (%.2f arcsec/s)", (xSidereal>=0 ? "north" : "south"), abs(xSidereal), abs(arcsecPerSec));
	if(!stepperDec.setTargetVelocityArcsecPerSec(arcsecPerSec)) {
		LOG_ERROR("MoveNS");
		return false;
	}
	return true;
}


bool PimocoMount::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) {
	double xSidereal=0;
	if(command==MOTION_START) {
		xSidereal=SlewRatesN[IUFindOnSwitchIndex(&SlewRateSP)].value;
		if(dir==DIRECTION_EAST)
			xSidereal=-xSidereal;  // DIRECTION_WEST for RA is positive on the HA axis
	}
	double arcsecPerSec=xSidereal * trackRates[0];

	LOGF_INFO("Moving %s at %.1fx sidereal rate (%.2f arcsec/s)", (xSidereal>=0 ? "west" : "east"), abs(xSidereal), abs(arcsecPerSec));
	if(!stepperHA.setTargetVelocityArcsecPerSec(arcsecPerSec)) {
		LOG_ERROR("MoveWE");
		return false;
	}
	return true;
}


bool PimocoMount::SetSlewRate(int index) {
	return true;
}


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


bool PimocoMount::Goto(double ra, double dec) {
	double last=getLocalSiderealTime();
   	double ha  =rangeHA(last - ra); 

   	LOGF_INFO("Goto RA %f (HA %f) Dec %f", ra, ha, dec);

 	if(TrackState==SCOPE_TRACKING)
 		wasTrackingBeforeGoto=true;
 	else if(TrackState==SCOPE_IDLE)
 		wasTrackingBeforeGoto=false;
 	else
 		; // don't touch

	if(!stepperHA.setTargetPositionHours(ha, wasTrackingBeforeGoto ? stepperHA.arcsecPerSecToNative(getTrackRateRA()) : 0) || 
	   !stepperDec.setTargetPositionDegrees(dec, wasTrackingBeforeGoto ? stepperDec.arcsecPerSecToNative(getTrackRateDec()) : 0) ) {
		LOG_ERROR("Goto");
		return false;
	}

	// cache target ra/dec for periodic reissue of ha-based target given progressing time
	gotoTargetRA=ra;
	gotoTargetDec=dec;
 
	SetTimer(100);  // Workaround: increase polling frequency to continuously update HA target during slew

    TrackState = SCOPE_SLEWING;
  	return true;
}


bool PimocoMount::SetParkPosition(double Axis1Value, double Axis2Value) {
   	parkPositionHA =Axis1Value;
	parkPositionDec=Axis2Value;
   	LOGF_INFO("Setting park position to HA %f Dec %f", parkPositionHA, parkPositionDec);
	return true;
}

bool PimocoMount::SetCurrentPark() {
	double localHaHours, decDegrees;
	if(!stepperHA.getPositionHours(&localHaHours) || !stepperDec.getPositionDegrees(&decDegrees))
		return false;
	parkPositionHA =localHaHours;
	parkPositionDec=decDegrees;
   	LOGF_INFO("Setting park position to current position HA %f Dec %f", parkPositionHA, parkPositionDec);
	return true;
}

bool PimocoMount::SetDefaultPark() {
   	LOG_ERROR("There is no default park position for this mount, please set a custom one");
	return false;
}

bool PimocoMount::Park() {
   	LOGF_INFO("Parking at HA %f Dec %f", parkPositionHA, parkPositionDec);
	if(!stepperHA.setTargetPositionHours(parkPositionHA) || !stepperDec.setTargetPositionDegrees(parkPositionDec) ) {
		LOG_ERROR("Parking");
		return false;
	}
 
    TrackState = SCOPE_PARKING;
  	return true;
}

bool PimocoMount::UnPark() {
	SetParked(false); // updates TrackState and logs
	return true;
}

IPState PimocoMount::GuideNorth(uint32_t ms) {
	if(TrackState!=SCOPE_TRACKING)
		return IPS_ALERT; // can only guide if tracking

	// Indi: North is defined as DEC+
	double arcsecPerSec=getTrackRateDec() + SlewRatesN[0].value * trackRates[0];
	if(!stepperDec.setTargetVelocityArcsecPerSec(arcsecPerSec))
		return IPS_ALERT;

	guiderActiveDec=true;
	setToNowPlusMs(&guiderTimeoutDec, ms);

	if(!guiderActiveRA || isLessThanOrEqual(&guiderTimeoutDec, &guiderTimeoutRA))
		SetTimer(ms);

	return IPS_BUSY;
}

IPState PimocoMount::GuideSouth(uint32_t ms) {
	if(TrackState!=SCOPE_TRACKING)
		return IPS_ALERT; // can only guide if tracking

	// Indi: South is defined as DEC-
	double arcsecPerSec=getTrackRateDec() - SlewRatesN[0].value * trackRates[0];
	if(!stepperDec.setTargetVelocityArcsecPerSec(arcsecPerSec))
		return IPS_ALERT;

	guiderActiveDec=true;
	setToNowPlusMs(&guiderTimeoutDec, ms);

	if(!guiderActiveRA || isLessThanOrEqual(&guiderTimeoutDec, &guiderTimeoutRA))
		SetTimer(ms);

	return IPS_BUSY;
}

IPState PimocoMount::GuideEast(uint32_t ms) {
	if(TrackState!=SCOPE_TRACKING)
		return IPS_ALERT; // can only guide if tracking

	// Indi: East is defined as RA+, so HA-
	double arcsecPerSec=getTrackRateRA() - SlewRatesN[0].value * trackRates[0];
	if(!stepperHA.setTargetVelocityArcsecPerSec(arcsecPerSec))
		return IPS_ALERT;

	guiderActiveRA=true;
	setToNowPlusMs(&guiderTimeoutRA, ms);

	if(!guiderActiveDec || isLessThanOrEqual(&guiderTimeoutRA, &guiderTimeoutDec))
		SetTimer(ms);

	return IPS_BUSY;
}

IPState PimocoMount::GuideWest(uint32_t ms) {
	if(TrackState!=SCOPE_TRACKING)
		return IPS_ALERT; // can only guide if tracking

	// Indi: West is defined as RA-, so HA+
	double arcsecPerSec=getTrackRateRA() + SlewRatesN[0].value * trackRates[0];
	if(!stepperHA.setTargetVelocityArcsecPerSec(arcsecPerSec))
		return IPS_ALERT;

	guiderActiveRA=true;
	setToNowPlusMs(&guiderTimeoutRA, ms);

	if(!guiderActiveDec || isLessThanOrEqual(&guiderTimeoutRA, &guiderTimeoutDec))
		SetTimer(ms);

	return IPS_BUSY;
}

double PimocoMount::getLocalSiderealTime() {
   	struct ln_date lnDate;
   	ln_get_date_from_sys(&lnDate);
   	double julianDay=ln_get_julian_day(&lnDate);
   	double gast=ln_get_apparent_sidereal_time(julianDay);
 	double observerLongitudeDegreesEastPositive=LocationN[LOCATION_LONGITUDE].value;
 	double last=gast + observerLongitudeDegreesEastPositive/15.0;
 	return last;	
}

void PimocoMount::setToNowPlusMs(struct timespec *ts, uint32_t ms) {
	clock_gettime(CLOCK_REALTIME, ts);
	ts->tv_nsec+=((long) ms)*1000;
	if(ts->tv_nsec>=1000000000) {
		ts->tv_sec+=1;
		ts->tv_nsec-=1000000000;
	} 
}

long PimocoMount::getMsUntil(const struct timespec *ts) {
	struct timespec now;
	clock_gettime(CLOCK_REALTIME, &now);

	auto secDiff=ts->tv_sec - now.tv_sec;
	if(ts->tv_nsec>=now.tv_nsec) {
		auto nsecDiff=ts->tv_nsec - now.tv_nsec;
		return ((long)secDiff)*1000 + ((long)nsecDiff)/1000000;
	} else {
		secDiff--;
		auto nsecDiff=1000000000 + ts->tv_nsec - now.tv_nsec;
		return ((long)secDiff)*1000 + ((long)nsecDiff)/1000000;
	}
}