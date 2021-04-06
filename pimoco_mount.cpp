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

// Name of the mount tab
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

PimocoMount::PimocoMount() : stepperHA(getDeviceName()), stepperDec(getDeviceName()),
    spiDeviceFilenameHA("/dev/spidev1.1"), spiDeviceFilenameDec("/dev/spidev1.2") {
	setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);

	SetTelescopeCapability(
       		TELESCOPE_CAN_GOTO |
            TELESCOPE_CAN_SYNC |
            TELESCOPE_CAN_PARK |
            TELESCOPE_CAN_ABORT |
            TELESCOPE_HAS_TIME |
            TELESCOPE_HAS_LOCATION |
            TELESCOPE_HAS_PIER_SIDE |
            // TELESCOPE_HAS_PEC |
            TELESCOPE_HAS_TRACK_MODE |
            TELESCOPE_CAN_CONTROL_TRACK |
            TELESCOPE_HAS_TRACK_RATE |
            //TELESCOPE_HAS_PIER_SIDE_SIMULATION |
            //TELESCOPE_CAN_TRACK_SATELLITE |
            0, 4);

	setTelescopeConnection(CONNECTION_NONE);
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

	stepperHA .initProperties( HAMotorN, & HAMotorNP,  HAMSwitchS, & HAMSwitchSP, HARampN, & HARampNP,  
							  "HA_MOTOR", "Motor", "HA_MSWITCH", "Switches", "HA_RAMP", "Ramp", HA_TAB);
	stepperDec.initProperties(DecMotorN, &DecMotorNP, DecMSwitchS, &DecMSwitchSP, DecRampN, &DecRampNP, 
							  "DEC_MOTOR", "Motor", "DEC_MSWITCH", "Switches", "DEC_RAMP", "Ramp", DEC_TAB);

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
		// ...
	} else {
		// ...
	}

	return true;
}

void PimocoMount::ISGetProperties(const char *dev) {
	INDI::Telescope::ISGetProperties(dev);

	// load from configuration on init
	loadConfig(true, HAMotorNP.name);
	loadConfig(true, HAMSwitchSP.name);
	loadConfig(true, HARampNP.name);

	loadConfig(true, DecMotorNP.name);
	loadConfig(true, DecMSwitchSP.name);
	loadConfig(true, DecRampNP.name);
}

bool PimocoMount::ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
    	return INDI::Telescope::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);

    // if(strcmp(name, ...)) { } else

	return INDI::Telescope::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

bool PimocoMount::ISUpdateNumber(INumberVectorProperty *NP, double values[], char *names[], int n, bool res) 
{
	if(res)
		IUUpdateNumber(NP, values, names, n);               
	NP->s=res ? IPS_OK : IPS_ALERT;
	IDSetNumber(NP, NULL);
	return res;
}

bool PimocoMount::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
		return INDI::Telescope::ISNewNumber(dev, name, values, names, n);

	int res=stepperHA.ISNewNumber(&HAMotorNP, &HARampNP, name, values, names, n);
	if(res>=0)
		return res>0;
	res=stepperDec.ISNewNumber(&DecMotorNP, &DecRampNP, name, values, names, n);
	if(res>=0)
		return res>0;
    
	return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool PimocoMount::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
		return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);

	int res=stepperHA.ISNewSwitch(&HAMSwitchSP, name, states, names, n);
	if(res>=0)
		return res>0;
	res=stepperDec.ISNewSwitch(&DecMSwitchSP, name, states, names, n);
	if(res>=0)
		return res>0;

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

bool PimocoMount::saveConfigItems(FILE *fp){
    INDI::Telescope::saveConfigItems(fp);

    IUSaveConfigNumber(fp, &HAMotorNP);
    IUSaveConfigSwitch(fp, &HAMSwitchSP);
    IUSaveConfigNumber(fp, &HARampNP);

    IUSaveConfigNumber(fp, &DecMotorNP);
    IUSaveConfigSwitch(fp, &DecMSwitchSP);
    IUSaveConfigNumber(fp, &DecRampNP);

    return true;
}

void PimocoMount::TimerHit() {
	if(!isConnected())
		return;

	ReadScopeStatus();
	
    SetTimer(getCurrentPollingPeriod());
}


bool PimocoMount::ReadScopeStatus() {
	if (!isConnected())
		return false;

	// get local hour angle and declination angle from scope
	double localHaHours, decDegrees;
	if(!stepperHA.getPositionHours(&localHaHours) || !stepperDec.getPositionDegrees(&decDegrees))
		return false;

	// update scope status
	auto  haStatus=stepperHA.getStatus();
	auto decStatus=stepperDec.getStatus();
	switch(TrackState) {
        case SCOPE_IDLE:
        	break; // FIXME

        case SCOPE_SLEWING:
        	break; // FIXME

        case SCOPE_TRACKING:
        	break; // FIXME

        case SCOPE_PARKING:
        	if((haStatus & Stepper::TMC_POSITION_REACHED) && (decStatus & Stepper::TMC_POSITION_REACHED))
        		TrackState=SCOPE_PARKED;
        	break;

        case SCOPE_PARKED:
        	break;  // FIXME 
   	}

   	// calculate local sidereal time
   	struct ln_date lnDate;
   	ln_get_date_from_sys(&lnDate);
   	double julianDay=ln_get_julian_day(&lnDate);
   	double gast=ln_get_apparent_sidereal_time(julianDay);
 	double observerLongitudeDegreesEastPositive=LocationN[LOCATION_LONGITUDE].value;
 	double last=gast + observerLongitudeDegreesEastPositive/15.0;

 	// calculate and update RA and Dec
   	double raHours=rangeHA(last - localHaHours); 
 	LOGF_INFO("lha %f gast %f obsLong %f last %f raHours %f", localHaHours, gast, observerLongitudeDegreesEastPositive, last, raHours);
   	decDegrees=rangeDec(decDegrees);
    NewRaDec(raHours, decDegrees); 

	return true;
}

// Protected class members
//

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

