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


#include "pimoco_focuser.h"
#include <libindi/indilogger.h>
#include <math.h> // for round()

#define CDRIVER_VERSION_MAJOR	1
#define CDRIVER_VERSION_MINOR	0


// Singleton instance
PimocoFocuser focuser;

// C function interface redirecting to singleton
//
extern "C" {

void ISGetProperties(const char *dev) {
	focuser.ISGetProperties(dev);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n) {
	focuser.ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
	focuser.ISNewNumber(dev, name, values, names, n);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
	focuser.ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) {
	focuser.ISNewText(dev, name, texts, names, n);
}

void ISSnoopDevice(XMLEle *root) {
	focuser.ISSnoopDevice(root);
}

} // extern "C"


// Public class members
//

PimocoFocuser::PimocoFocuser() : spiDeviceFilename("/dev/spidev1.0") {
	setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);
    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | 
    	              FOCUSER_CAN_REVERSE  | FOCUSER_CAN_SYNC     ); // | FOCUSER_HAS_VARIABLE_SPEED);	
	setSupportedConnections(CONNECTION_NONE);
}

PimocoFocuser::~PimocoFocuser() {
	stepper.stop();
}

const char *PimocoFocuser::getDefaultName() {
	return "Pimoco focuser";
}

bool PimocoFocuser::initProperties() {
	if(!INDI::Focuser::initProperties()) 
		return false;

	FocusMaxPosN[0].max=2000000000;
	IUUpdateMinMax(&FocusMaxPosNP);

	uint32_t currentHwMaxMa;
	stepper.getHardwareMaxCurrent(&currentHwMaxMa);

	IUFillNumber(&CurrentMaN[0], "HOLD", "Hold [mA]", "%.0f", 0, currentHwMaxMa, currentHwMaxMa/100, 0);
	IUFillNumber(&CurrentMaN[1], "RUN",  "Run [mA]",  "%.0f", 0, currentHwMaxMa, currentHwMaxMa/100, 0);
	IUFillNumberVector(&CurrentMaNP, CurrentMaN, 2, getDeviceName(), "CURRENT", "Current", FOCUS_TAB, IP_RW, 0, IPS_IDLE);

	IUFillNumber(&RampN[0], "VSTART",    "VStart [usteps/t]",     "%.0f", 0, (1ul<<18)-1,   ((1ul<<18)-1)/99,   0);
	IUFillNumber(&RampN[1], "A1",        "A1 [usteps/ta^2]",      "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&RampN[2], "V1",        "V1 [usteps/t]",         "%.0f", 0, (1ul<<20)-1,   ((1ul<<20)-1)/99,   0);
	IUFillNumber(&RampN[3], "AMAX",      "AMax [usteps/ta^2]",    "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&RampN[4], "VMAX",      "VMax [usteps/t]",       "%.0f", 0, (1ul<<23)-512, ((1ul<<23)-512)/99, 0);
	IUFillNumber(&RampN[5], "DMAX",      "DMax [usteps/ta^2]",    "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&RampN[6], "D1",        "DMax [usteps/ta^2]",    "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&RampN[7], "VSTOP",     "VStop [usteps/t]",      "%.0f", 0, (1ul<<18)-1,   ((1ul<<18)-1)/99,   0);
	IUFillNumber(&RampN[8], "TZEROWAIT", "TZeroWait [512 t_clk]", "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumberVector(&RampNP, RampN, 9, getDeviceName(), "RAMP", "Ramp", FOCUS_TAB, IP_RW, 0, IPS_IDLE);

    addDebugControl();
    return true;
}

bool PimocoFocuser::updateProperties() {
	if(!INDI::Focuser::updateProperties())
		return false;

	if(isConnected()) {
		// currents
		defineProperty(&CurrentMaNP);
		uint32_t currentHoldMa, currentRunMa;
		if(!stepper.getHoldCurrent(&currentHoldMa) || !stepper.getRunCurrent(&currentRunMa)) {
		    CurrentMaNP.s = IPS_ALERT;
		    IDSetNumber(&CurrentMaNP, NULL);				
			return false;
		} else {
		    CurrentMaN[0].value = currentHoldMa;
		    CurrentMaN[1].value = currentRunMa;
		    CurrentMaNP.s = IPS_OK;
		    IDSetNumber(&CurrentMaNP, NULL);
	    }				

		// ramp
	    defineProperty(&RampNP);
	    uint32_t vstart, a1, v1, amax, vmax, dmax, d1, vstop, tzerowait;
	    if(!stepper.getVStart(&vstart) || !stepper.getA1(&a1) || !stepper.getV1(&v1) || !stepper.getAMax(&amax) ||
	       !stepper.getMaxGoToSpeed(&vmax) || 
	       !stepper.getDMax(&dmax) || !stepper.getD1(&d1) || !stepper.getVStop(&vstop) || !stepper.getTZeroWait(&tzerowait)) {
	       RampNP.s = IPS_ALERT;
	       IDSetNumber(&RampNP, NULL);
	       return false;	
	    } else {
	    	RampN[0].value=vstart;
	    	RampN[1].value=a1;
	    	RampN[2].value=v1;
	    	RampN[3].value=amax;
	    	RampN[4].value=vmax;
	    	RampN[5].value=dmax;
	    	RampN[6].value=d1;
	    	RampN[7].value=vstop;
	    	RampN[8].value=tzerowait;
	    	IDSetNumber(&RampNP, NULL);
	    }
	} else {
		deleteProperty(CurrentMaNP.name);
		deleteProperty(RampNP.name);
	}
	return true;
}

void PimocoFocuser::ISGetProperties(const char *dev) {
	return INDI::Focuser::ISGetProperties(dev);
}

bool PimocoFocuser::ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
    	return INDI::Focuser::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);

    // if(strcmp(name, ...)) { } else

	return INDI::Focuser::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

bool PimocoFocuser::ISUpdateNumber(INumberVectorProperty *NP, double values[], char *names[], int n, bool res) 
{
	if(res)
		IUUpdateNumber(NP, values, names, n);               
	NP->s=res ? IPS_OK : IPS_ALERT;
	IDSetNumber(NP, NULL);
	return res;
}

bool PimocoFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
		return INDI::Focuser::ISNewNumber(dev, name, values, names, n);

    if(!strcmp(name, CurrentMaNP.name)) { 
        bool res=stepper.setHoldCurrent((uint32_t) round(values[0])) && 
                 stepper.setRunCurrent ((uint32_t) round(values[1]))    ;
        return ISUpdateNumber(&CurrentMaNP, values, names, n, res);
    } else if(!strcmp(name, RampNP.name)) {
    	bool res=stepper.setVStart((uint32_t) round(values[0])) &&
    			 stepper.setA1((uint32_t) round(values[1])) &&
    			 stepper.setV1((uint32_t) round(values[2])) &&
    			 stepper.setAMax((uint32_t) round(values[3])) &&
    			 stepper.setMaxGoToSpeed((uint32_t) round(values[4])) &&
    			 stepper.setDMax((uint32_t) round(values[5])) &&
    			 stepper.setD1((uint32_t) round(values[6])) &&
    			 stepper.setVStop((uint32_t) round(values[7])) &&
    			 stepper.setTZeroWait((uint32_t) round(values[8]))    ;
        return ISUpdateNumber(&RampNP, values, names, n, res);
    }
    
	return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

bool PimocoFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
		return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);

    // if(strcmp(name, ...)) { } else

	return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool PimocoFocuser::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
		return INDI::Focuser::ISNewText(dev, name, texts, names, n);

    // if(strcmp(name, ...)) { } else

	return INDI::Focuser::ISNewText(dev, name, texts, names, n);
}

bool PimocoFocuser::ISSnoopDevice(XMLEle *root) {
	return INDI::Focuser::ISSnoopDevice(root);
}

void PimocoFocuser::TimerHit() {
	if(!isConnected())
		return;

	// update state from device
	auto pos=0;
	if(!stepper.getPosition(&pos)) {
		LOG_ERROR("Error reading position");
	    FocusAbsPosNP.s = IPS_ALERT;
	} else {
	    auto status=stepper.getStatus();
	    if((FocusAbsPosNP.s==IPS_BUSY) && (status&Stepper::TMC_STAND_STILL))
	    	LOGF_INFO("Focuser has reached position %u", pos);

	    FocusAbsPosN[0].value = pos;
	    FocusAbsPosNP.s = (status & Stepper::TMC_STAND_STILL) ? IPS_OK : IPS_BUSY;
	    FocusRelPosNP.s = FocusAbsPosNP.s;
	    IDSetNumber(&FocusAbsPosNP, NULL);		
	    IDSetNumber(&FocusRelPosNP, NULL);		
	}
	
    SetTimer(getCurrentPollingPeriod());
}

// Protected class members
//

bool PimocoFocuser::Connect() {
	if(!stepper.open(spiDeviceFilename)) {
		LOGF_WARN("Connection on %s failed", spiDeviceFilename);
		return false;
	}
	LOGF_INFO("Connection on %s successful", spiDeviceFilename);

	uint32_t pp=getPollingPeriod();
	if (pp > 0)
		SetTimer(pp);

	return true;
}

bool PimocoFocuser::Disconnect() {
	if(!stepper.close()) {
		LOG_WARN("Error closing connection");
		return false;
	}
	LOG_INFO("Successfully closed connection");
	return true;
}

bool PimocoFocuser::Handshake() {
	return true;
}

IPState PimocoFocuser::MoveAbsFocuser(uint32_t targetTicks) {
	if(!stepper.setTargetPosition(targetTicks)) {
		LOG_ERROR("Error setting focuser target position");
		return IPS_ALERT;
	}
	return IPS_BUSY;
}

IPState PimocoFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks) {
	int32_t pos;
	if(!stepper.getPosition(&pos)) {
		LOG_ERROR("Error getting current focuser position");
		return IPS_ALERT;
	}

	int32_t targetPos=(dir==FOCUS_OUTWARD) ? pos + ticks : pos - ticks;
	LOGF_INFO("Focuser is moving by %c%u to target position %u", (dir==FOCUS_OUTWARD)?'+':'-', ticks, targetPos);
	if(!stepper.setTargetPosition(targetPos)) {
		LOG_ERROR("Error setting focuser target position");
		return IPS_ALERT;
	}
	return IPS_BUSY;
}

bool PimocoFocuser::AbortFocuser() {
	LOG_INFO("Stopping focuser motion");
	if(!stepper.stop()) {
		LOG_ERROR("Error stopping focuser motion");
		return false;
	}
	return true;
}

bool PimocoFocuser::ReverseFocuser(bool enabled) {
	LOGF_INFO("Setting direction of focuser motion to %s", enabled ? "reversed" : "normal");
	if(!stepper.setInvertMotor(enabled)) {
		LOG_ERROR("Error setting direction of focuser motion");
		return false;
	}
	return true;
}

bool PimocoFocuser::SetFocuserSpeed(int speed) {
	LOGF_INFO("Setting focuser max speed to %d", speed);
	if(!stepper.setMaxGoToSpeed((uint32_t) speed)) {
		LOG_ERROR("Error setting focuser max speed");
		return false;
	}
	return true;
}

bool PimocoFocuser::SyncFocuser(uint32_t ticks) {
	LOGF_INFO("Syncing focuser position to %u", ticks);
	if(!stepper.syncPosition((int32_t) ticks)) {
		LOG_ERROR("Error syncing focuser position");
		return false;
	}
	return true;
}