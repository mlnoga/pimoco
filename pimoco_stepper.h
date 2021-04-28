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

#pragma once

#ifndef PIMOCO_STEPPER_H
#define PIMOCO_STEPPER_H

#include "pimoco_tmc5160.h"
#include <libindi/indidevapi.h>
#include <math.h> // for M_PI

// Higher-level stepper functions, building on the underlying implementation of TMC5160 stepper registers
class Stepper : public TMC5160 {
public:
	// Creates a TMC5160 stepper connected via SPI, with optional physical connector pin for diag0 interrupt (negative=unused)
	Stepper(const char *theIndiDeviceName, int diag0Pin=-1);

	// Destroys this TMC5160 stepper connected via SPI. Stops device motion for safety's sake
	~Stepper();

	bool open(const char *device);

	bool close();

	// Sets the target velocity in arcseconds per second of the controlled object. Returns immediately. Returns true on success, else false
	bool setTargetVelocityArcsecPerSec(double arcsecPerSec);

	// Stops all current movement. Returns true on success, else false
	bool stop();

	// Returns the current position in the variable pointed to by result. Returns true on success, else false	
	bool getPosition(int32_t *result) { return getRegister(TMCR_XACTUAL, (uint32_t*) result); }

	// Gets current position in radians, based on usteps, steps and gear ratio
	bool getPositionRadians(double *result) { return getPositionInUnits(result, 2.0*M_PI); }

	// Gets current position in degrees, based on usteps, steps and gear ratio
	bool getPositionDegrees(double *result) { return getPositionInUnits(result, 360.0); }

	// Gets current position in hours, based on usteps, steps and gear ratio
	bool getPositionHours(double *result) { return getPositionInUnits(result, 24.0); }

	// Gets current position in given units for full circle
	bool getPositionInUnits(double *result, double full);

	// Syncs the current position on device to the given value. Temporarily enters holding mode to avoid moving the axis, 
	// then restores previous ramping mode afterwards. Returns true on success, else false	
	bool syncPosition(int32_t value);

	// Syncs current position in radians, based on usteps, steps and gear ratio
	bool syncPositionRadians(double value) { return syncPositionInUnits(value, 2.0*M_PI); }

	// Syncs current position in degrees, based on usteps, steps and gear ratio
	bool syncPositionDegrees(double value) { return syncPositionInUnits(value, 360.0); }

	// Syncs current position in hours, based on usteps, steps and gear ratio
	bool syncPositionHours(double value) { return syncPositionInUnits(value, 24.0); }

	// Syncs current position in given units for full circle
	bool syncPositionInUnits(double value, double full);

	// Gets the target position for gotos. Returns true on success, else false
	bool getTargetPosition(int32_t *result) { return getRegister(TMCR_XTARGET, (uint32_t*) result); }

	// Sets the target position, initiating a non-blocking go-to. If restoreSpeed is nonzero, restores the given speed once position is reached.
	// Returns immediately. Returns true on success, else false
	bool setTargetPosition(int32_t value, int32_t restoreSpeed=0);

	// Sets the target position and performs a blocking go-to with optional timeout (0=no timeout). Returns when position reached, or timeout occurs. Returns true on success, else false
	bool setTargetPositionBlocking(int32_t value, uint32_t timeoutMs=0);

	// Sets the target position in radians, initiating a non-blocking go-to. Returns immediately. Returns true on success, else false
	bool setTargetPositionRadians(double value, int32_t restoreSpeed=0) { return setTargetPositionInUnits(value, 2.0*M_PI, restoreSpeed); }

	// Sets the target position in degrees, initiating a non-blocking go-to. Returns immediately. Returns true on success, else false
	bool setTargetPositionDegrees(double value, int32_t restoreSpeed=0) { return setTargetPositionInUnits(value, 360.0, restoreSpeed); }

	// Sets the target position in hours, initiating a non-blocking go-to. Returns immediately. Returns true on success, else false
	bool setTargetPositionHours(double value, int32_t restoreSpeed=0) { return setTargetPositionInUnits(value, 24.0, restoreSpeed); }

	// Syncs current position in given units for full circle
	bool setTargetPositionInUnits(double value, double full, int32_t restoreSpeed=0);

	// Converts radians to native steps
	int32_t radiansToNative(double value) {	return unitsToNative(value, 2*M_PI); }

	// Converts degrees to native steps
	int32_t degreesToNative(double value) {	return unitsToNative(value, 360.0); }

	// Converts hours to native steps
	int32_t hoursToNative(double value) { return unitsToNative(value, 24.0); }

	// Converts given full circle units to native steps
	int32_t unitsToNative(double value, double full) {	return round(value * (microsteps * stepsPerRev * gearRatio) / full); }

	// Converts given speed in arcecs/sec to native step speed units
	int32_t arcsecPerSecToNative(double arcsecPerSec);

	// Get minimum position limit. Returns true on success, else false
	bool getMinPosition(int32_t *result) { *result=minPosition; return true; }

	// Set minimum position limit. Returns true on success, else false
	bool setMinPosition(int32_t value);

	// Get maximum position limit. Returns true on success, else false
	bool getMaxPosition(int32_t *result) { *result=maxPosition; return true; }

	// Set maximum position limit. Returns true on success, else false
	bool setMaxPosition(int32_t value);

	// Gets maximal motor speed for gotos. In units of 2^24/f_clk. Returns true on success, else false
	bool getMaxGoToSpeed(uint32_t *result) { *result=maxGoToSpeed;  return true; }

	// Sets maximal motor speed. In units of 2^24/f_clk. Effective on next goto. Returns true on success, else false
	bool setMaxGoToSpeed(uint32_t value) { maxGoToSpeed=value; return true; }

	// Get maximum current supported by the hardware based on the chosen sense resistor. See datasheet section 9, p.74. Returns true on success, else false
	bool getHardwareMaxCurrent(uint32_t *result_mA) { *result_mA=hardwareMaxCurrent_mA; return true; }

	// Set maximum current supported by the hardware based on the chosen sense resistor. See datasheet section 9, p.74. Returns true on success, else false
	bool setHardwareMaxCurrent(uint32_t value_mA) { hardwareMaxCurrent_mA=value_mA; return true; }

	// Get maximum current supported by the software based on hardware limits and chosen global current scaler. See datasheet section 9, p.74. Returns true on success, else false
	bool getSoftwareMaxCurrent(uint32_t *result_mA);

	// Gets motor run current in mA. Must be called after setHardwareMaxCurrent(). Returns true on success, else false
	bool getRunCurrent(uint32_t *result_mA);

	// Sets motor run current in mA. Must be called after setHardwareMaxCurrent(). Returns true on success, else false
	bool setRunCurrent(uint32_t value_mA, bool bestPerformanceHint=true);

	// Gets motor hold current in mA. Must be called after setHardwareMaxCurrent(). Returns true on success, else false
	bool getHoldCurrent(uint32_t *result_mA);

	// Sets motor hold current in mA. Must be called after setHardwareMaxCurrent(). Returns true on success, else false
	bool setHoldCurrent(uint32_t value_mA, bool suppressDebugOutput=false);

	// Gets number of microsteps per step. Always succeeds
	bool getMicrosteps(double *result) { *result=microsteps; return true; }

	// Sets steps per revolution. Always succeeds
	bool setStepsPerRev(double value) { stepsPerRev=value; return true; }

	// Gets steps per revolution. Always succeeds
	bool getStepsPerRev(double *result) { *result=stepsPerRev; return true; }

	// Sets gear ratio. Motor must turn X times for one full turn of the controlled object. Always succeeds
	bool setGearRatio(double value) { gearRatio=value; return true; }

	// Gets gear ratio. Motor must turn X times for one full turn of the controlled object. Always succeeds
	bool getGearRatio(double *result) { *result=gearRatio; return true; }

	// Sets clock in Hz.  Does not change physical setting, used for physical position/speed/accel calculations only. Always succeeds
	bool setClockHz(uint32_t value) { clockHz=value; return true; }

	// Gets glock in Hz.  Does not change physical setting, used for physical position/speed/accel calculations only. Always succeeds
	bool getClockHz(uint32_t *result) { *result=clockHz; return true; }


	// Indi UI
	//

	// Initialize INDI UI properties
	void initProperties(INumber *MotorN, INumberVectorProperty *MotorNP,
						ISwitch *MSwitchS, ISwitchVectorProperty *MSwitchSP, 
	                    INumber *RampN, INumberVectorProperty *RampNP, 
                        const char *motorVarName, const char *motorUILabel,
                        const char *mSwitchVarName, const char *mSwitchUILabel,
                        const char *rampVarName, const char *rampUILabel, 
	                    const char *tabName);

	// Update INDI UI properties based on connection status. Returns true on success, else false
	bool updateProperties(INDI::DefaultDevice *iDevice,
					      INumber *MotorN, INumberVectorProperty *MotorNP, 
						  ISwitch *MSwitchS, ISwitchVectorProperty *MSwitchSP, 
	                      INumber *RampN, INumberVectorProperty *RampNP);

	// Update stepper setting number based on new values coming from UI. 
	// Returns 1 if successful, 0 if unsuccessful, -1 if handler not applicable for this name
	int ISNewNumber(INumberVectorProperty *MotorNP, INumberVectorProperty *RampNP,
                    const char *name, double values[], char *names[], int n);

    // Updates number vector property with the given values if res is true and display status IPS_OK, else display status IPS_ALERT. Returns res for convenience. 
    bool ISUpdateNumber(INumberVectorProperty *NP, double values[], char *names[], int n, bool res);

    // Update stepper setting switch based on new values coming from UI.
	// Returns 1 if successful, 0 if unsuccessful, -1 if handler not applicable for this name
	int ISNewSwitch(ISwitchVectorProperty *MSwitchSP, 
                    const char *name, ISState *states, char *names[], int n);

public:
	enum {
		MOTORN_SIZE = 5,
		MSWITCHS_SIZE = 4,
		RAMPN_SIZE = 17,
	};

protected:
	// Runs automatic chopper tuning procedure, as per TMC5160A datasheet section 7.1, p.57ff
	bool chopperAutoTuneStealthChop(uint32_t secondSteps, uint32_t timeoutMs);

	// Minimum position, in microsteps
	int32_t minPosition;

	// Maximum position, in microsteps
	int32_t maxPosition;

	// Maximum speed for GoTos. Stored separately as setTargetSpeed() overwrites VMAX on the device
	uint32_t maxGoToSpeed;

	// Maximal current supported by hardware based on the chosen sense resistor. See datasheet section 9, p.74
	uint32_t hardwareMaxCurrent_mA;

	// Microsteps per full motor step
	const double   microsteps=256;

	// Motor steps per full revolution of the motor shaft
	double   stepsPerRev;

	// Gear ratio 1:x between a motor shaft revolution and a revolution of the controlled object.
	// Motor must turn X times for one full turn of the controlled object. 
	double   gearRatio;

	// Stepper clock in Hz. Does not change physical setting, used for physical position/speed/accel calculations only.
	uint32_t clockHz;


protected:	
	// Default maximal current supported by TMC5160-BOB. See datasheet section 9, p.74
	static const uint32_t defaultHardwareMaxCurrent_mA;

	// Default minimal position. Based on arbitrary scope with gear ratio 1:1000, 1000 steps/rev and 256 microsteps 
	static const int32_t defaultMinPosition;

	// Default maximal position. Based on arbitrary scope with gear ratio 1:1000, 1000 steps/rev and 256 microsteps 
	static const int32_t defaultMaxPosition;

	// Default maximal go-to speed 
	static const int32_t defaultMaxGoToSpeed;

	// Default motor steps per full revolution of the motor shaft
	static const double   defaultStepsPerRev;

	// Default gear ratio 1:x between a motor shaft revolution and a revolution of the controlled object.
	// Motor must turn X times for one full turn of the controlled object. 
	static const double   defaultGearRatio;

 };

#endif // PIMOCO_STEPPER_H
