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
#include <math.h> // for round()

#define CDRIVER_VERSION_MAJOR	1
#define CDRIVER_VERSION_MINOR	0


// Singleton instance
PimocoMount mount;

// Name of the mount tab
const char *PimocoMount::MOUNT_TAB="Mount";


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

	// HA controls
	uint32_t currentHwMaxMaHA;
	stepperHA.getHardwareMaxCurrent(&currentHwMaxMaHA);

	IUFillNumber(&HACurrentMaN[0], "HOLD", "Hold [mA]", "%.0f", 0, currentHwMaxMaHA, currentHwMaxMaHA/100, 0);
	IUFillNumber(&HACurrentMaN[1], "RUN",  "Run [mA]",  "%.0f", 0, currentHwMaxMaHA, currentHwMaxMaHA/100, 0);
	IUFillNumberVector(&HACurrentMaNP, HACurrentMaN, 2, getDeviceName(), "HA_CURRENT", "HA Current", MOUNT_TAB, IP_RW, 0, IPS_IDLE);

	IUFillNumber(&HARampN[0], "VSTART",    "VStart [usteps/t]",     "%.0f", 0, (1ul<<18)-1,   ((1ul<<18)-1)/99,   0);
	IUFillNumber(&HARampN[1], "A1",        "A1 [usteps/ta^2]",      "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&HARampN[2], "V1",        "V1 [usteps/t]",         "%.0f", 0, (1ul<<20)-1,   ((1ul<<20)-1)/99,   0);
	IUFillNumber(&HARampN[3], "AMAX",      "AMax [usteps/ta^2]",    "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&HARampN[4], "VMAX",      "VMax [usteps/t]",       "%.0f", 0, (1ul<<23)-512, ((1ul<<23)-512)/99, 0);
	IUFillNumber(&HARampN[5], "DMAX",      "DMax [usteps/ta^2]",    "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&HARampN[6], "D1",        "DMax [usteps/ta^2]",    "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&HARampN[7], "VSTOP",     "VStop [usteps/t]",      "%.0f", 0, (1ul<<18)-1,   ((1ul<<18)-1)/99,   0);
	IUFillNumber(&HARampN[8], "TZEROWAIT", "TZeroWait [512 t_clk]", "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumberVector(&HARampNP, HARampN, 9, getDeviceName(), "HA_RAMP", "HA Ramp", MOUNT_TAB, IP_RW, 0, IPS_IDLE);


	// Decl controls
	uint32_t currentHwMaxMaDec;
	stepperDec.getHardwareMaxCurrent(&currentHwMaxMaDec);

	IUFillNumber(&DecCurrentMaN[0], "HOLD", "Hold [mA]", "%.0f", 0, currentHwMaxMaDec, currentHwMaxMaDec/100, 0);
	IUFillNumber(&DecCurrentMaN[1], "RUN",  "Run [mA]",  "%.0f", 0, currentHwMaxMaDec, currentHwMaxMaDec/100, 0);
	IUFillNumberVector(&DecCurrentMaNP, DecCurrentMaN, 2, getDeviceName(), "DEC_CURRENT", "Dec Current", MOUNT_TAB, IP_RW, 0, IPS_IDLE);

	IUFillNumber(&DecRampN[0], "VSTART",    "VStart [usteps/t]",     "%.0f", 0, (1ul<<18)-1,   ((1ul<<18)-1)/99,   0);
	IUFillNumber(&DecRampN[1], "A1",        "A1 [usteps/ta^2]",      "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&DecRampN[2], "V1",        "V1 [usteps/t]",         "%.0f", 0, (1ul<<20)-1,   ((1ul<<20)-1)/99,   0);
	IUFillNumber(&DecRampN[3], "AMAX",      "AMax [usteps/ta^2]",    "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&DecRampN[4], "VMAX",      "VMax [usteps/t]",       "%.0f", 0, (1ul<<23)-512, ((1ul<<23)-512)/99, 0);
	IUFillNumber(&DecRampN[5], "DMAX",      "DMax [usteps/ta^2]",    "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&DecRampN[6], "D1",        "DMax [usteps/ta^2]",    "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&DecRampN[7], "VSTOP",     "VStop [usteps/t]",      "%.0f", 0, (1ul<<18)-1,   ((1ul<<18)-1)/99,   0);
	IUFillNumber(&DecRampN[8], "TZEROWAIT", "TZeroWait [512 t_clk]", "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumberVector(&DecRampNP, DecRampN, 9, getDeviceName(), "DEC_RAMP", "Dec Ramp", MOUNT_TAB, IP_RW, 0, IPS_IDLE);

    addDebugControl();
    return true;
}

bool PimocoMount::updateProperties() {
	if(!INDI::Telescope::updateProperties())
		return false;

	if(isConnected()) {
		// HA currents
		defineProperty(&HACurrentMaNP);
		uint32_t currentHoldMa, currentRunMa;
		if(!stepperHA.getHoldCurrent(&currentHoldMa) || !stepperHA.getRunCurrent(&currentRunMa)) {
		    HACurrentMaNP.s = IPS_ALERT;
		    IDSetNumber(&HACurrentMaNP, NULL);				
			return false;
		} else {
		    HACurrentMaN[0].value = currentHoldMa;
		    HACurrentMaN[1].value = currentRunMa;
		    HACurrentMaNP.s = IPS_OK;
		    IDSetNumber(&HACurrentMaNP, NULL);
	    }				

		// HA ramp
	    defineProperty(&HARampNP);
	    uint32_t vstart, a1, v1, amax, vmax, dmax, d1, vstop, tzerowait;
	    if(!stepperHA.getVStart(&vstart) || !stepperHA.getA1(&a1) || !stepperHA.getV1(&v1) || !stepperHA.getAMax(&amax) ||
	       !stepperHA.getMaxGoToSpeed(&vmax) || 
	       !stepperHA.getDMax(&dmax) || !stepperHA.getD1(&d1) || !stepperHA.getVStop(&vstop) || !stepperHA.getTZeroWait(&tzerowait)) {
	       HARampNP.s = IPS_ALERT;
	       IDSetNumber(&HARampNP, NULL);
	       return false;	
	    } else {
	    	HARampN[0].value=vstart;
	    	HARampN[1].value=a1;
	    	HARampN[2].value=v1;
	    	HARampN[3].value=amax;
	    	HARampN[4].value=vmax;
	    	HARampN[5].value=dmax;
	    	HARampN[6].value=d1;
	    	HARampN[7].value=vstop;
	    	HARampN[8].value=tzerowait;
	    	IDSetNumber(&HARampNP, NULL);
	    }

		// Dec currents
		defineProperty(&DecCurrentMaNP);
		if(!stepperDec.getHoldCurrent(&currentHoldMa) || !stepperDec.getRunCurrent(&currentRunMa)) {
		    DecCurrentMaNP.s = IPS_ALERT;
		    IDSetNumber(&DecCurrentMaNP, NULL);				
			return false;
		} else {
		    DecCurrentMaN[0].value = currentHoldMa;
		    DecCurrentMaN[1].value = currentRunMa;
		    DecCurrentMaNP.s = IPS_OK;
		    IDSetNumber(&DecCurrentMaNP, NULL);
	    }				

		// Dec ramp
	    defineProperty(&DecRampNP);
	    if(!stepperDec.getVStart(&vstart) || !stepperDec.getA1(&a1) || !stepperDec.getV1(&v1) || !stepperDec.getAMax(&amax) ||
	       !stepperDec.getMaxGoToSpeed(&vmax) || 
	       !stepperDec.getDMax(&dmax) || !stepperDec.getD1(&d1) || !stepperDec.getVStop(&vstop) || !stepperDec.getTZeroWait(&tzerowait)) {
	       DecRampNP.s = IPS_ALERT;
	       IDSetNumber(&DecRampNP, NULL);
	       return false;	
	    } else {
	    	DecRampN[0].value=vstart;
	    	DecRampN[1].value=a1;
	    	DecRampN[2].value=v1;
	    	DecRampN[3].value=amax;
	    	DecRampN[4].value=vmax;
	    	DecRampN[5].value=dmax;
	    	DecRampN[6].value=d1;
	    	DecRampN[7].value=vstop;
	    	DecRampN[8].value=tzerowait;
	    	IDSetNumber(&DecRampNP, NULL);
	    }

	} else {
		deleteProperty(HACurrentMaNP.name);
		deleteProperty(HARampNP.name);
		deleteProperty(HACurrentMaNP.name);
		deleteProperty(HARampNP.name);
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

    if(!strcmp(name, HACurrentMaNP.name)) { 
        bool res=stepperHA.setHoldCurrent((uint32_t) round(values[0])) && 
                 stepperHA.setRunCurrent ((uint32_t) round(values[1]))    ;
        return ISUpdateNumber(&HACurrentMaNP, values, names, n, res);
    } else if(!strcmp(name, HARampNP.name)) {
    	bool res=stepperHA.setVStart((uint32_t) round(values[0])) &&
    			 stepperHA.setA1((uint32_t) round(values[1])) &&
    			 stepperHA.setV1((uint32_t) round(values[2])) &&
    			 stepperHA.setAMax((uint32_t) round(values[3])) &&
    			 stepperHA.setMaxGoToSpeed((uint32_t) round(values[4])) &&
    			 stepperHA.setDMax((uint32_t) round(values[5])) &&
    			 stepperHA.setD1((uint32_t) round(values[6])) &&
    			 stepperHA.setVStop((uint32_t) round(values[7])) &&
    			 stepperHA.setTZeroWait((uint32_t) round(values[8]))    ;
        return ISUpdateNumber(&HARampNP, values, names, n, res);
    } else if(!strcmp(name, DecCurrentMaNP.name)) { 
        bool res=stepperDec.setHoldCurrent((uint32_t) round(values[0])) && 
                 stepperDec.setRunCurrent ((uint32_t) round(values[1]))    ;
        return ISUpdateNumber(&DecCurrentMaNP, values, names, n, res);
    } else if(!strcmp(name, DecRampNP.name)) {
    	bool res=stepperDec.setVStart((uint32_t) round(values[0])) &&
    			 stepperDec.setA1((uint32_t) round(values[1])) &&
    			 stepperDec.setV1((uint32_t) round(values[2])) &&
    			 stepperDec.setAMax((uint32_t) round(values[3])) &&
    			 stepperDec.setMaxGoToSpeed((uint32_t) round(values[4])) &&
    			 stepperDec.setDMax((uint32_t) round(values[5])) &&
    			 stepperDec.setD1((uint32_t) round(values[6])) &&
    			 stepperDec.setVStop((uint32_t) round(values[7])) &&
    			 stepperDec.setTZeroWait((uint32_t) round(values[8]))    ;
        return ISUpdateNumber(&DecRampNP, values, names, n, res);
    }
    
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
