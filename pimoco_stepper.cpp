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


#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <cstdio>
#include <math.h> // for round(), M_PI
#include <sys/time.h>  // for gettimeofday() etc.
#include <libindi/indilogger.h> // for LOG_..., LOGF_... macros

#include "pimoco_stepper.h"
#include "pimoco_time.h"


const uint32_t Stepper::defaultHardwareMaxCurrent_mA=3100; // default for TMC5160-BOB
const int32_t  Stepper::defaultMinPosition=-1000ul*1000ul*256ul;
const int32_t  Stepper::defaultMaxPosition= 1000ul*1000ul*256ul;
const int32_t  Stepper::defaultMaxGoToSpeed=100000;
const double   Stepper::defaultStepsPerRev =400;
const double   Stepper::defaultGearRatio   =3*144;


Stepper::Stepper(const char *theIndiDeviceName) : TMC5160(theIndiDeviceName), minPosition(defaultMinPosition), maxPosition(defaultMaxPosition),
				     maxGoToSpeed(defaultMaxGoToSpeed), hardwareMaxCurrent_mA(defaultHardwareMaxCurrent_mA),
				     stepsPerRev(defaultStepsPerRev), gearRatio(defaultGearRatio) {
}


Stepper::~Stepper() {
	stop();
}

bool Stepper::open(const char *deviceName) {
	if(!TMC5160::open(deviceName))
		return false;

	// Stop device, just in case it was left running
	if(!stop())
		return false;
	usleep(100*1000l);

	// Clear reset, undervoltage and driver error flags if present
	if(!setGStat(0x07))
		return false;

	// Setup Diagnosis 0 output to provide interrupts based on ramp function
	//
	if(!setDiag0EnableError(0))
		return false;
	if(!setDiag0EnableTemp(0))
		return false;
	if(!setDiag0EnableInterruptStep(0))
		return false;

	// Set motor current parameters
	//
	if(!setRunCurrent(800))
		return false;
	if(!setHoldCurrent(800)) // Hold current = run current for StealthChop configuration
		return false;
	if(!setIHoldDelay(10))
		return false;
	if(!setTPowerDown(10))
		return false;

	if(!setInvertMotor(0)) 		// FIXME: should come from stored config data
		return false;

	if(!setRegister(TMCR_SW_MODE, 0))  // Disable all switches and StallGuard for now
		return false;

	// Set a default ramp (used in auto-tuning)
	//
	if(!setVStart(10))
		return false;
	if(!setA1(7500*3/2))
		return false;
	if(!setV1(250000*3/2))
		return false;
	if(!setAMax(2500*3/2))
		return false;
	if(!setVMax(300000*3/2))
		return false;
	if(!setMaxGoToSpeed(300000*3/2))
		return false;
	if(!setDMax(2500*3/2))
		return false;
	if(!setD1(7500*3/2))
		return false;
	if(!setVStop(10))
		return false;
	if(!setTZeroWait(100))
		return false;

	// Set PWM parameters for stealth chop
	//
	if(!setPWMFrequencyDivider(0))  // set PWM frequency divider to 0=2/1024 f_clk, i.e. 29 KHz at 15 MHz clock. Should be in 20/40 KHz range
		return false;
	if(!setPWMEnableStealthChop(1))
		return false;
	if(!setPWMAutoscale(1))         // stealth chop current regulator
		return false;
	if(!setPWMAutoGradient(1))      // stealth chop gradient regulator
		return false;

	// Set chopper parameters
	//
	if(!setChopperMode(0))          // If above the threshold, move to spread cycle mode
		return false;
	if(!setTPWMThreshold(0))        // Disable threshold to use only StealthChop during calibration
		return false;
	if(!setChopperMicroRes(0))      // full 256 microsteps for internal operation
		return false;
	if(!setChopperTOff(5))
		return false;
	if(!setChopperTBlank(2))
		return false;
	if(!setChopperHStart(4))
		return false;
	if(!setChopperHEnd(0))
		return false;

	if(!chopperAutoTuneStealthChop(500, 3000))
		return false;

	// now that configuration is complete, set hold current to proper target 
	if(!setHoldCurrent(100)) 
		return false;

	// FIXME
	//
	
	// // if performance isn't good up to VMax, enable switch from StealthChop to SpreadCycle above threshold speed (e.g. 16x sidereal?)
	// if(!setTPWMThreshold(tbd))
	//	return false;
	// // if that results in coil overshoot during deceleration
	// if(!setPWMLimit(tbd))
	//	return false;

	// configure coolstep load adaptive current control
	// configure high velocity mode with switch to fullstep, and enable DCStep to avoid lost steps when too fast

	if(debugLevel>=TMC_DEBUG_DEBUG)
		LOGF_DEBUG("Successfully initialized device %s", deviceName!=NULL ? deviceName : "NULL");
	
	return true;
}


bool Stepper::close() {
	bool res1=stop();
	bool res2=TMC5160::close();
	return res1 && res2;
}


bool Stepper::chopperAutoTuneStealthChop(uint32_t secondSteps, uint32_t timeoutMs) {
	// keep track of starting position
	int32_t startPos;
	if(!getPosition(&startPos))
		return false;
	if(debugLevel>=TMC_DEBUG_DEBUG)
		LOGF_DEBUG("Current position is %'+d", startPos);

	uint32_t microRes;
	if(!getChopperMicroRes(&microRes))
		return false;
	uint32_t fullStep=256>>microRes; 

	// move a single full step
	int32_t targetPos=startPos+(int32_t)fullStep;
	if(!setTargetPositionBlocking(targetPos, timeoutMs))
		return false;

	// wait >=130 ms, then device automatically sets PWM_OFS_AUTO
	usleep(140000l);

	// move given amount of full steps (should be few 100s), while device automatically updates PWM_GRAD_AUTO
	targetPos+=secondSteps*(int32_t)fullStep;
	if(!setTargetPositionBlocking(targetPos, timeoutMs))
		return false;

	// return to starting position
	return setTargetPositionBlocking(startPos, timeoutMs);
}


bool Stepper::setTargetSpeed(int32_t value) {
	if(debugLevel>=TMC_DEBUG_DEBUG)
		LOGF_DEBUG("Setting target speed to %'+d", value);

	// FIXME: min/max position limits are not checked when setting a speed.
	// Would need a background timer with e.g. speed-based 1s lookahead.

	return setRegister(TMCR_RAMPMODE, value>=0 ? 1 : 2) &&    // select velocity mode and sign
	       setRegister(TMCR_VMAX, value>=0 ? value : -value); // set absolute target speed to initiate movement
}


bool Stepper::setTargetVelocityArcsecPerSec(double arcsecPerSec) {
	if(stepsPerRev==0 || gearRatio==0 || clockHz==0) {
		LOGF_ERROR("Zero value detected: %d steps/rev %d gear ratio %d Hz clock", stepsPerRev, gearRatio, clockHz);
		return false;
	}

	uint32_t ustepsPerRev=256*stepsPerRev*gearRatio;
	double ustepsPerArcsec=((double) ustepsPerRev) * (1.0 / (360.0*60.0*60.0));

	uint32_t stepperChipTimeScaler=(1ul<<24);
	double stepperTimeUnit=((double)stepperChipTimeScaler)/((double)clockHz);

	double ustepsPerT=arcsecPerSec * ustepsPerArcsec * stepperTimeUnit;
	int32_t ustepsPerTRounded=round(ustepsPerT);

	if(debugLevel>=TMC_DEBUG_DEBUG)
		LOGF_DEBUG("Setting target velocity to %f arcsec/sec * %f usteps/arcsec * %f s/stepper_t = %f usteps/stepper_t, rounded to %d",
			       arcsecPerSec, ustepsPerArcsec, stepperTimeUnit, ustepsPerT, ustepsPerTRounded);

	return setTargetSpeed(ustepsPerTRounded);
}


// Stops all current movement. Returns true on success, else false
bool Stepper::stop() {
	uint32_t xactual;
	if(!getRegister(TMCR_XACTUAL, &xactual))
		return false;
	return setTargetPosition(xactual);
}


bool Stepper::getPositionInUnits(double *result, double full) {
	int32_t pos;
	if(!getPosition(&pos))
		return false;
	*result=(full * (double) pos) / (microsteps * stepsPerRev * gearRatio);
	return true;
}


bool Stepper::syncPosition(int32_t value) {
	if(value<minPosition || value>maxPosition) {
		LOGF_ERROR("Unable to sync to position %'+d outside defined limits [%'+d, %'+d]", value, minPosition, maxPosition);
		return false;
	}

	if(debugLevel>=TMC_DEBUG_DEBUG)
		LOGF_DEBUG("Syncing current position to %'+d", value);

	// Syncing in positioning mode moves the axis, so we temporarily enter holding mode
	uint32_t rm;
	if(!getRegister(TMCR_RAMPMODE, &rm))
		return false;
	if(!setRegister(TMCR_RAMPMODE, 3))
		return false;
	if(!setRegister(TMCR_XACTUAL, value))
		return false;
	if(rm!=0) 
		return setRegister(TMCR_RAMPMODE, rm);
	return setRegister(TMCR_RAMPMODE, 0) &&                  // select absolute positioning mode
		   setRegister(TMCR_VMAX, maxGoToSpeed) &&           // restore max speed in case setTargetSpeed() overwrote it
	       setRegister(TMCR_XTARGET, (uint32_t) value);      // set target position to initiate movement
}


// Syncs current position in given units for full circle
bool Stepper::syncPositionInUnits(double value, double full) {
	int32_t stepsValue=round(value * (microsteps * stepsPerRev * gearRatio) / full);
	return syncPosition(stepsValue);
}


bool Stepper::setTargetPosition(int32_t value) {
	if(value<minPosition || value>maxPosition) {
		LOGF_ERROR("Unable to set target position %'+d outside defined limits [%'+d, %'+d]", value, minPosition, maxPosition);
		return false;
	}

	if(debugLevel>=TMC_DEBUG_DEBUG)
		LOGF_DEBUG("Setting target position to %'+d", value);

	return setRegister(TMCR_RAMPMODE, 0) &&                  // select absolute positioning mode
		   setRegister(TMCR_VMAX, maxGoToSpeed) &&           // restore max speed in case setTargetSpeed() overwrote it
	       setRegister(TMCR_XTARGET, (uint32_t) value);      // set target position to initiate movement
}


bool Stepper::setTargetPositionBlocking(int32_t value, uint32_t timeoutMs) {
	Timestamp start;

	if(!setTargetPosition(value))
		return false;

	// wait until position reached or timeout occurs
	for(int i=0; ; i++) {
		usleep(1000l);  // 1 ms
		int32_t pos;
		if(!getPosition(&pos))
			return false;

		if(pos==value) {
			Timestamp now;
			uint64_t elapsedMs=now.msSince(start);
			if(debugLevel>=TMC_DEBUG_DEBUG)
				LOGF_DEBUG("Reached target position at %'+d after %d polls in %llus %llums", value, i, elapsedMs/1000, elapsedMs%1000);
			if(!setTargetPositionReachedEvent(1)) // clear event flag
				return false;			
			return true;  // position reached
		}

		Timestamp now;
		uint64_t elapsedMs=now.msSince(start);
		if(timeoutMs>0 && elapsedMs>(uint64_t) timeoutMs)
			return false; // timeout
	}
}


bool Stepper::setMinPosition(int32_t value) {
	int32_t currentPos;
	if(!getPosition(&currentPos))
		return false;
	if(currentPos<value) {
		LOGF_ERROR("Unable to set minimum position limit %'+d above current position %'+d", value, currentPos);
		return false;
	}

	if(debugLevel>=TMC_DEBUG_DEBUG)
		LOGF_DEBUG("Setting minimum position limit to %'+d", value);
	minPosition=value; 
	return true; 
}


bool Stepper::setMaxPosition(int32_t value) { 
	int32_t currentPos;
	if(!getPosition(&currentPos))
		return false;
	if(currentPos>value) {
		LOGF_ERROR("Unable to set maximum position limit %'+d below current position %'+d", value, currentPos);
		return false;
	}

	if(debugLevel>=TMC_DEBUG_DEBUG)
		LOGF_DEBUG("Setting maximum position limit to %'+d", value);
	maxPosition=value; 
	return true; 
}


bool Stepper::getSoftwareMaxCurrent(uint32_t *result_mA) {
	uint32_t gcs;
	if(!getGlobalCurrentScaler(&gcs))
		return false;
	if(gcs==0)
		gcs=256;
	*result_mA=(gcs*hardwareMaxCurrent_mA + 128) / 256;
	return true;
}

// Gets motor run current in mA. Must be called after setHardwareMaxCurrent(). Returns true on success, else false
bool Stepper::getRunCurrent(uint32_t *result_mA) {
	uint32_t softwareMaxCurrent_mA;
	if(!getSoftwareMaxCurrent(&softwareMaxCurrent_mA))
		return false;

	// scale down to desired hold current value with local current scaler
	uint32_t cs;
	if(!getIRun(&cs))
		return false;
	cs++;
	*result_mA=(cs*softwareMaxCurrent_mA + 16) / 32;
	return true;
}

// Sets motor run current in mA. Must be called after setHardwareMaxCurrent(). Returns true on success, else false
bool Stepper::setRunCurrent(uint32_t value_mA, bool bestPerformanceHint) {
	// cache the current hold current setting, as we are about to change the global scaler 
	uint32_t holdCurrent_mA;
	if(!getHoldCurrent(&holdCurrent_mA))
		return false;

	// calculate appropriate global current scaler
	uint32_t gcs=(256*value_mA + hardwareMaxCurrent_mA/2)/hardwareMaxCurrent_mA;
	if(gcs>=256)  // hard limit with 8-bit wraparound
		gcs=0;

	// apply operational limits and/or performance hints, see datasheet p.36
	uint32_t lowerBound = bestPerformanceHint ? 129 : 32; 
	if(gcs<=lowerBound)
		gcs=lowerBound;

	if(!setGlobalCurrentScaler(gcs))
		return false;

	// calculate resulting max software current
	uint32_t softwareMaxCurrent_mA;
	if(!getSoftwareMaxCurrent(&softwareMaxCurrent_mA))
		return false;

	// calculate local current scaler value for the target value 
	uint32_t cs=(32*value_mA + softwareMaxCurrent_mA/2)/softwareMaxCurrent_mA;
	if(cs>=32)
		cs=32;
	if(cs>0)
		cs--;
	if(!setIRun(cs))
		return false;

	uint32_t resulting_mA;
	if(!getRunCurrent(&resulting_mA))
		return false;
	if(debugLevel>=TMC_DEBUG_DEBUG)
		LOGF_DEBUG("Setting run current %dmA with global scaler %d and iRun %d, resulting in %dmA", value_mA, gcs, cs, resulting_mA);

	// restore the hold current setting
	return setHoldCurrent(holdCurrent_mA, true);
}


// Gets motor hold current in mA. Must be called after setHardwareMaxCurrent(). Returns true on success, else false
bool Stepper::getHoldCurrent(uint32_t *result_mA) {
	uint32_t softwareMaxCurrent_mA;
	if(!getSoftwareMaxCurrent(&softwareMaxCurrent_mA))
		return false;

	// then scale down to desired hold current value with local current scaler
	uint32_t cs;
	if(!getIHold(&cs))
		return false;
	cs++;
	*result_mA=(cs*softwareMaxCurrent_mA + 16) / 32;
	return true;
}


// Sets motor hold current in mA. Must be called after setHardwareMaxCurrent(). Returns true on success, else false
bool Stepper::setHoldCurrent(uint32_t value_mA, bool suppressDebugOutput) {
	uint32_t softwareMaxCurrent_mA;
	if(!getSoftwareMaxCurrent(&softwareMaxCurrent_mA))
		return false;

	uint32_t cs=(32*value_mA + softwareMaxCurrent_mA/2)/softwareMaxCurrent_mA;
	if(cs>=32)
		cs=32;
	// then calculate local current scaler value for the target value 
	if(cs>0)
		cs--;
	if(!setIHold(cs))
		return false;

	uint32_t resulting_mA;
	if(!getHoldCurrent(&resulting_mA))
		return false;
	if(!suppressDebugOutput && debugLevel>=TMC_DEBUG_DEBUG)
		LOGF_DEBUG("Setting hold current %dmA with iHold %d, resulting in %dmA", value_mA, cs, resulting_mA);
	return true;
}


void Stepper::initProperties(INumber *MotorN, INumberVectorProperty *MotorNP, 
						     ISwitch *MSwitchS, ISwitchVectorProperty *MSwitchSP, 
	                         INumber *RampN, INumberVectorProperty *RampNP, 
	                         const char *motorVarName, const char *motorUILabel,
	                         const char *mSwitchVarName, const char *mSwitchUILabel,
	                         const char *rampVarName, const char *rampUILabel, 
	                         const char *tabName) {

	uint32_t currentHwMaxMa;
	getHardwareMaxCurrent(&currentHwMaxMa);

	IUFillNumber(&MotorN[0], "STEPS", "Steps/rev [1]",     "%.0f", 0, 1000, 10, 0);
	IUFillNumber(&MotorN[1], "GEAR",  "Gear ratio [1:n]",  "%.0f", 0, 1000, 10, 0);
	IUFillNumber(&MotorN[2], "HOLD",  "Hold current [mA]", "%.0f", 0, currentHwMaxMa, currentHwMaxMa/100, 0);
	IUFillNumber(&MotorN[3], "RUN",   "Run current [mA]",  "%.0f", 0, currentHwMaxMa, currentHwMaxMa/100, 0);
	IUFillNumber(&MotorN[4], "CLOCK", "Clock [Hz]",        "%.0f", 8000000, 16000000, 100000, 9600000);
	IUFillNumberVector(MotorNP, MotorN, 5, getDeviceName(), motorVarName, motorUILabel, tabName, IP_RW, 0, IPS_IDLE);

	IUFillSwitch(&MSwitchS[0], "INVERT", "Invert axis", ISS_OFF);
	IUFillSwitchVector(MSwitchSP, MSwitchS, 1, getDeviceName(), mSwitchVarName, mSwitchUILabel, tabName, IP_RW, ISR_NOFMANY, 0, IPS_IDLE);

	IUFillNumber(&RampN[0], "VSTART",    "VStart [usteps/t]",     "%.0f", 0, (1ul<<18)-1,   ((1ul<<18)-1)/99,   0);
	IUFillNumber(&RampN[1], "A1",        "A1 [usteps/ta^2]",      "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&RampN[2], "V1",        "V1 [usteps/t]",         "%.0f", 0, (1ul<<20)-1,   ((1ul<<20)-1)/99,   0);
	IUFillNumber(&RampN[3], "AMAX",      "AMax [usteps/ta^2]",    "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&RampN[4], "VMAX",      "VMax [usteps/t]",       "%.0f", 0, (1ul<<23)-512, ((1ul<<23)-512)/99, 0);
	IUFillNumber(&RampN[5], "DMAX",      "DMax [usteps/ta^2]",    "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&RampN[6], "D1",        "DMax [usteps/ta^2]",    "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumber(&RampN[7], "VSTOP",     "VStop [usteps/t]",      "%.0f", 0, (1ul<<18)-1,   ((1ul<<18)-1)/99,   0);
	IUFillNumber(&RampN[8], "TZEROWAIT", "TZeroWait [512 t_clk]", "%.0f", 0, (1ul<<16)-1,   ((1ul<<16)-1)/99,   0);
	IUFillNumberVector(RampNP, RampN, 9, getDeviceName(), rampVarName, rampUILabel, tabName, IP_RW, 0, IPS_IDLE);
}


bool Stepper::updateProperties(INDI::DefaultDevice *iDevice,
							   INumber *MotorN, INumberVectorProperty *MotorNP, 
						       ISwitch *MSwitchS, ISwitchVectorProperty *MSwitchSP, 
	                           INumber *RampN, INumberVectorProperty *RampNP) {
	if(iDevice->isConnected()) {
		// Motor settings
		iDevice->defineProperty(MotorNP);
		uint32_t currentHoldMa, currentRunMa;
		if(!getHoldCurrent(&currentHoldMa) || !getRunCurrent(&currentRunMa)) {
		    MotorNP->s = IPS_ALERT;
		    IDSetNumber(MotorNP, NULL);				
			return false;
		} else {
		    MotorN[0].value = stepsPerRev;
		    MotorN[1].value = gearRatio;
		    MotorN[2].value = currentHoldMa;
		    MotorN[3].value = currentRunMa;
		    MotorN[4].value = clockHz;
		    MotorNP->s = IPS_OK;
		    IDSetNumber(MotorNP, NULL);
	    }				

	    // Motor switches
	    iDevice->defineProperty(MSwitchSP);
	    uint32_t invert;
	    if(!getInvertMotor(&invert)) {
			MSwitchSP->s=IPS_ALERT;
			IDSetSwitch(MSwitchSP, NULL);
			return false;	    	
	    } else {
	    	MSwitchS[0].s=(invert>0) ? ISS_ON : ISS_OFF;
	    	MSwitchSP->s=IPS_OK;
	    	IDSetSwitch(MSwitchSP, NULL);
	    }

		// Ramp settings
	    iDevice->defineProperty(RampNP);
	    uint32_t vstart, a1, v1, amax, vmax, dmax, d1, vstop, tzerowait;
	    if(!getVStart(&vstart) || !getA1(&a1) || !getV1(&v1) || !getAMax(&amax) ||
	       !getMaxGoToSpeed(&vmax) || 
	       !getDMax(&dmax) || !getD1(&d1) || !getVStop(&vstop) || !getTZeroWait(&tzerowait)) {
	       RampNP->s = IPS_ALERT;
	       IDSetNumber(RampNP, NULL);
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
	    	IDSetNumber(RampNP, NULL);
	    }
	} else {
		iDevice->deleteProperty(MotorNP->name);
		iDevice->deleteProperty(MSwitchSP->name);
		iDevice->deleteProperty(RampNP->name);
	}
	return true;
}


int Stepper::ISNewNumber(INumberVectorProperty *MotorNP, INumberVectorProperty *RampNP,
                         const char *name, double values[], char *names[], int n) {
    if(!strcmp(name, MotorNP->name)) { 
        bool res=setStepsPerRev(values[0]) &&
        		 setGearRatio(values[1]) &&
        	     setHoldCurrent((uint32_t) round(values[2])) && 
                 setRunCurrent ((uint32_t) round(values[3])) &&
                 setClockHz(values[4]) ;
        return ISUpdateNumber(MotorNP, values, names, n, res) ? 1 : 0;
    } else if(!strcmp(name, RampNP->name)) {
    	bool res=setVStart((uint32_t) round(values[0])) &&
    			 setA1((uint32_t) round(values[1])) &&
    			 setV1((uint32_t) round(values[2])) &&
    			 setAMax((uint32_t) round(values[3])) &&
    			 setMaxGoToSpeed((uint32_t) round(values[4])) &&
    			 setDMax((uint32_t) round(values[5])) &&
    			 setD1((uint32_t) round(values[6])) &&
    			 setVStop((uint32_t) round(values[7])) &&
    			 setTZeroWait((uint32_t) round(values[8]))    ;
        return ISUpdateNumber(RampNP, values, names, n, res) ? 1 : 0; 
    }
    
    return -1;
}

bool Stepper::ISUpdateNumber(INumberVectorProperty *NP, double values[], char *names[], int n, bool res) {
	if(res)
		IUUpdateNumber(NP, values, names, n);               
	NP->s=res ? IPS_OK : IPS_ALERT;
	IDSetNumber(NP, NULL);
	return res;
}

int Stepper::ISNewSwitch(ISwitchVectorProperty *MSwitchSP, 
                         const char *name, ISState *states, char *names[], int n) {
    if(!strcmp(name, MSwitchSP->name)) { 
        bool res=setInvertMotor(states[0]==ISS_ON ? 1 : 0);
		if(res)
			IUUpdateSwitch(MSwitchSP, states, names, n);
		MSwitchSP->s=res ? IPS_OK : IPS_ALERT;
		IDSetSwitch(MSwitchSP, NULL);
		return res ? 1 : 0;
    }    
    return -1;
}
