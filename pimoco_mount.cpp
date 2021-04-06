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

	stepperHA .initProperties( HACurrentMaN, & HACurrentMaNP,  HARampN, & HARampNP,  
							  "HA_CURRENT", "Current", "HA_RAMP", "Ramp", HA_TAB);
	stepperDec.initProperties(DecCurrentMaN, &DecCurrentMaNP, DecRampN, &DecRampNP, 
							  "DEC_CURRENT", "Current", "DEC_RAMP", "Ramp", DEC_TAB);

    addDebugControl();
    return true;
}

bool PimocoMount::updateProperties() {
	if(!INDI::Telescope::updateProperties())
		return false;

	if(!stepperHA .updateProperties(this,  HACurrentMaN, & HACurrentMaNP,  HARampN, & HARampNP))
		return false;
	if(!stepperDec.updateProperties(this, DecCurrentMaN, &DecCurrentMaNP, DecRampN, &DecRampNP))
		return false;

	if(isConnected()) {
		// ...
	} else {
		// ...
	}

	return true;
}

void PimocoMount::ISGetProperties(const char *dev) {
	return INDI::Telescope::ISGetProperties(dev);
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

	int res=stepperHA.ISNewNumber(&HACurrentMaNP, &HARampNP, name, values, names, n);
	if(res>=0)
		return res>0;
	res=stepperDec.ISNewNumber(&DecCurrentMaNP, &DecRampNP, name, values, names, n);
	if(res>=0)
		return res>0;
    
	return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool PimocoMount::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
		return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);

    // if(strcmp(name, ...)) { } else

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

void PimocoMount::TimerHit() {
	if(!isConnected())
		return;

	// update state from device
	auto pos=0;
	if(!stepperHA.getPosition(&pos)) {
		LOG_ERROR("Error reading HA position");
	    // FocusAbsPosNP.s = IPS_ALERT;
	} else {
	    /* auto status=stepper.getStatus();
	    if((FocusAbsPosNP.s==IPS_BUSY) && (status&Stepper::TMC_STAND_STILL))
	    	LOGF_INFO("Focuser has reached position %u", pos);

	    FocusAbsPosN[0].value = pos;
	    FocusAbsPosNP.s = (status & Stepper::TMC_STAND_STILL) ? IPS_OK : IPS_BUSY;
	    FocusRelPosNP.s = FocusAbsPosNP.s;
	    IDSetNumber(&FocusAbsPosNP, NULL);		
	    IDSetNumber(&FocusRelPosNP, NULL); */		
	}

	if(!stepperDec.getPosition(&pos)) {
		LOG_ERROR("Error reading Dec position");
	    // FocusAbsPosNP.s = IPS_ALERT;
	} else {
	    /* auto status=stepper.getStatus();
	    if((FocusAbsPosNP.s==IPS_BUSY) && (status&Stepper::TMC_STAND_STILL))
	    	LOGF_INFO("Focuser has reached position %u", pos);

	    FocusAbsPosN[0].value = pos;
	    FocusAbsPosNP.s = (status & Stepper::TMC_STAND_STILL) ? IPS_OK : IPS_BUSY;
	    FocusRelPosNP.s = FocusAbsPosNP.s;
	    IDSetNumber(&FocusAbsPosNP, NULL);		
	    IDSetNumber(&FocusRelPosNP, NULL); */		
	}
	
    SetTimer(getCurrentPollingPeriod());
}


bool PimocoMount::ReadScopeStatus() {
	return false;
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