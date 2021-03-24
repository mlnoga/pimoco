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
#include <sys/time.h>  // for gettimeofday() etc.

#include "pimoco_stepper.h"
#include "pimoco_time.h"


const uint32_t Stepper::defaultHardwareMaxCurrent_mA=3100; // default for TMC5160-BOB
const int32_t  Stepper::defaultMinPosition=-1000ul*1000ul*256ul;
const int32_t  Stepper::defaultMaxPosition= 1000ul*1000ul*256ul;
const int32_t  Stepper::defaultMaxGoToSpeed=100000;


Stepper::Stepper() : TMC5160(), minPosition(defaultMinPosition), maxPosition(defaultMaxPosition),
				     maxGoToSpeed(defaultMaxGoToSpeed), hardwareMaxCurrent_mA(defaultHardwareMaxCurrent_mA) {
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
	if(!setRunCurrent(900))
		return false;
	if(!setHoldCurrent(900)) // Hold current = run current for StealthChop configuration
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
	if(!setA1(1000))
		return false;
	if(!setV1(50000))
		return false;
	if(!setAMax(500))
		return false;
	if(!setVMax(200000))
		return false;
	if(!setDMax(700))
		return false;
	if(!setD1(1400))
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
	if(!setHoldCurrent(300)) 
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

	if(debugLevel>=TMC_DEBUG_ACTIONS) 
		fprintf(debugFile, "Successfully initialized device %s\n", deviceName!=NULL ? deviceName : "NULL");
	
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
	if(debugLevel>=TMC_DEBUG_ACTIONS) 
		fprintf(debugFile, "Current position is %'+d\n", startPos);

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
	if(debugLevel>=TMC_DEBUG_ACTIONS) 
		fprintf(debugFile, "Setting target speed to %'+d\n", value);

	// FIXME: min/max position limits are not checked when setting a speed.
	// Would need a background timer with e.g. speed-based 1s lookahead.

	return setRegister(TMCR_RAMPMODE, value>=0 ? 1 : 2) &&    // select velocity mode and sign
	       setRegister(TMCR_VMAX, value>=0 ? value : -value); // set absolute target speed to initiate movement
}


bool Stepper::syncPosition(int32_t value) {
	if(value<minPosition || value>maxPosition) {
		if(debugLevel>=TMC_DEBUG_ERRORS) 
			fprintf(debugFile, "Unable to sync to position %'+d outside defined limits [%'+d, %'+d]\n", 
				    value, minPosition, maxPosition);
		return false;
	}

	if(debugLevel>=TMC_DEBUG_ACTIONS) 
		fprintf(debugFile, "Syncing current position to %'+d\n", value);

	// Syncing in positioning mode moves the axis, so we temporarily enter holding mode
	uint32_t rm;
	if(!getRegister(TMCR_RAMPMODE, &rm))
		return false;
	if(!setRegister(TMCR_RAMPMODE, 3))
		return false;
	if(!setRegister(TMCR_XACTUAL, value))
		return false;
	return setRegister(TMCR_RAMPMODE, rm);
}


bool Stepper::setTargetPosition(int32_t value) {
	if(value<minPosition || value>maxPosition) {
		if(debugLevel>=TMC_DEBUG_ERRORS) 
			fprintf(debugFile, "Unable to set target position %'+d outside defined limits [%'+d, %'+d]\n", 
				    value, minPosition, maxPosition);
		return false;
	}

	if(debugLevel>=TMC_DEBUG_ACTIONS) 
		fprintf(debugFile, "Setting target position to %'+d\n", value);

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
			if(debugLevel>=TMC_DEBUG_ACTIONS) {
				Timestamp now;
				uint64_t elapsedMs=now.msSince(start);
				fprintf(debugFile, "Reached target position at %'+d after %d polls in %llus %llums\n", 
					    value, i, elapsedMs/1000, elapsedMs%1000);
			}
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
		if(debugLevel>=TMC_DEBUG_ERRORS) 
			fprintf(debugFile, "Unable to set minimum position limit %'+d above current position %'+d\n", 
				    value, currentPos);
		return false;
	}

	if(debugLevel>=TMC_DEBUG_ACTIONS)
		fprintf(debugFile, "Setting minimum position limit to %'+d\n", value);
	minPosition=value; 
	return true; 
}


bool Stepper::setMaxPosition(int32_t value) { 
	int32_t currentPos;
	if(!getPosition(&currentPos))
		return false;
	if(currentPos>value) {
		if(debugLevel>=TMC_DEBUG_ERRORS) 
			fprintf(debugFile, "Unable to set maximum position limit %'+d below current position %'+d\n", 
				    value, currentPos);
		return false;
	}

	if(debugLevel>=TMC_DEBUG_ACTIONS)
		fprintf(debugFile, "Setting maximum position limit to %'+d\n", value);
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
	if(debugLevel>=TMC_DEBUG_ACTIONS) 
		fprintf(debugFile, "Setting run current %dmA with global scaler %d and iRun %d, resulting in %dmA\n", 
			    value_mA, gcs, cs, resulting_mA);

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
	if(!suppressDebugOutput && debugLevel>=TMC_DEBUG_ACTIONS) 
		fprintf(debugFile, "Setting hold current %dmA with iRun %d, resulting in %dmA\n", 
			    value_mA, cs, resulting_mA);
	return true;
}

