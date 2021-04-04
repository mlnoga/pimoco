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


#ifndef PIMOCO_STEPPER_H
#define PIMOCO_STEPPER_H

#include "pimoco_tmc5160.h"

// Higher-level stepper functions, building on the underlying implementation of TMC5160 stepper registers
class Stepper : public TMC5160 {
public:
	// Creates a TMC5160 stepper connected via SPI
	Stepper();

	// Destroys this TMC5160 stepper connected via SPI. Stops device motion for safety's sake
	~Stepper();

	bool open(const char *device);

	bool close();

	// Returns the current speed in the variable pointed to by result. Returns true on success, else false	
	bool getSpeed(int32_t *result) { return getRegister(TMCR_VACTUAL, (uint32_t*) result); }

	// Sets the target speed to the given number of microsteps per second. Returns immediately. Returns true on success, else false
	bool setTargetSpeed(int32_t value);

	// Stops all current movement. Returns true on success, else false
	bool stop();

	// Returns the current position in the variable pointed to by result. Returns true on success, else false	
	bool getPosition(int32_t *result) { return getRegister(TMCR_XACTUAL, (uint32_t*) result); }

	// Syncs the current position on device to the given value. Temporarily enters holding mode to avoid moving the axis, 
	// then restores previous ramping mode afterwards. Returns true on success, else false	
	bool syncPosition(int32_t value);

	// Gets the target position for gotos. Returns true on success, else false
	bool getTargetPosition(int32_t *result) { return getRegister(TMCR_XTARGET, (uint32_t*) result); }

	// Sets the target position, initiating a non-blocking go-to. Returns immediately. Returns true on success, else false
	bool setTargetPosition(int32_t value);

	// Sets the target position and performs a blocking go-to with optional timeout (0=no timeout). Returns when position reached, or timeout occurs. Returns true on success, else false
	bool setTargetPositionBlocking(int32_t value, uint32_t timeoutMs=0);

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

protected:	
	// Default maximal current supported by TMC5160-BOB. See datasheet section 9, p.74
	static const uint32_t defaultHardwareMaxCurrent_mA;

	// Default minimal position. Based on arbitrary scope with gear ratio 1:1000, 1000 steps/rev and 256 microsteps 
	static const int32_t defaultMinPosition;

	// Default maximal position. Based on arbitrary scope with gear ratio 1:1000, 1000 steps/rev and 256 microsteps 
	static const int32_t defaultMaxPosition;

	// Default maximal go-to speed 
	static const int32_t defaultMaxGoToSpeed;
};

#endif // PIMOCO_STEPPER_H
