
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


#ifndef PIMOCO_TMC5160_H
#define PIMOCO_TMC5160_H

#include <stdint.h>

// A TMC5160 stepper connected via SPI
class TMC5160SPI {
protected:
	// Device register addresses
	enum TMCRegisters : uint8_t {
		// general configuration 
		//
		TMCR_GCONF = 0x00,
		TMCR_GSTAT = 0x01,
		TMCR_IFCNT = 0x02,
		TMCR_SLAVECONF = 0x03,
		TMCR_IOIN = 0x04,   // on read (shared register)
		TMCR_OUTPUT = 0x04, // on write (shared register)
		TMCR_X_COMPARE = 0x05,
		TMCR_OPT_PROG = 0x06,
		TMCR_OPT_READ = 0x07,
		TMCR_FACTORY_CONF = 0x08,
		TMCR_SHORT_CONF = 0x09,
		TMCR_DRV_CONF = 0x0a,
		TMCR_GLOBAL_SCALER = 0x0b,
		TMCR_OFFSET_READ = 0x0c,

		// velocity-dependent driver feature control
		//
		TMCR_IHOLD_IRUN = 0x10,
		TMCR_TPOWER_DOWN = 0x11,
		TMCR_TSTEP = 0x12,
		TMCR_TPWMTHRS = 0x13,
		TMCR_TCOOLTHRS = 0x14,
		TMCR_THIGH = 0x15,

		// ramp generator 
		//
		TMCR_RAMPMODE = 0x20,
		TMCR_XACTUAL = 0x21,
		TMCR_VACTUAL = 0x22,
		TMCR_VSTART = 0x23,
		TMCR_A1 = 0x24,
		TMCR_V1 = 0x25,
		TMCR_AMAX = 0x26,
		TMCR_VMAX = 0x27,
		TMCR_DMAX = 0x28,
		TMCR_D1   = 0x2a,
		TMCR_VSTOP = 0x2b,
		TMCR_TZEROWAIT = 0x2c,
		TMCR_XTARGET = 0x2d,

		// ramp generator feature control
		//
		TMCR_VDCMIN = 0x33,
		TMCR_SW_MODE = 0x34,
		TMCR_RAMP_STAT = 0x35,
		TMCR_XLATCH = 0x36,

		// encoder registers
		//
		TMCR_ENCMODE = 0x38,
		TMCR_X_ENC = 0x39,
		TMCR_ENC_CONST = 0x3a,
		TMCR_ENC_STATUS = 0x3b,
		TMCR_ENC_LATCH = 0x3c,
		TMCR_ENC_DEVIATION = 0x3d,

		// motor driver registers
		//
		TMCR_MSLUT0 = 0x60,
		TMCR_MSLUT1 = 0x61,
		TMCR_MSLUT2 = 0x62,
		TMCR_MSLUT3 = 0x63,
		TMCR_MSLUT4 = 0x64,
		TMCR_MSLUT5 = 0x65,
		TMCR_MSLUT6 = 0x66,
		TMCR_MSLUT7 = 0x67,
		TMCR_MSLUTSEL = 0x68,
		TMCR_MSLUTSTART = 0x69,
		TMCR_MSCNT = 0x6a,
		TMCR_MSCURACT = 0x6b,
		TMCR_CHOPCONF = 0x6c,
		TMCR_COOLCONF = 0x6d,
		TMCR_DCCTRL = 0x6e,
		TMCR_DRV_STATUS = 0x6f,
		TMCR_PWMCONF = 0x70,
		TMCR_PWM_SCALE = 0x71,
		TMCR_PWM_AUTO = 0x72,
		TMCR_LOST_STEPS = 0x73,

		TMCR_NUM_REGISTERS = 0x080,
	};

	// Supported modes on a device register (none, read, write, both)
	enum TMCRegisterModes : int {
		TMCRM_NONE = 0,
		TMCRM_R    = 1,
		TMCRM_W    = 2,
		TMCRM_RW   = 3,
	};

	// Metadata about TMC registers
	struct TMCRegisterMetaData {
		const char *name;
		enum TMCRegisterModes mode;
	};

	// Device status bit flags
	enum TMCStatusFlags : uint8_t {
		TMC_RESET            = ((uint8_t)1)<<0,
		TMC_DRIVER_ERROR     = ((uint8_t)1)<<1,
		TMC_STALL_GUARD      = ((uint8_t)1)<<2,
		TMC_STAND_STILL      = ((uint8_t)1)<<3,
		TMC_VELOCITY_REACHED = ((uint8_t)1)<<4,
		TMC_POSITION_REACHED = ((uint8_t)1)<<5,
		TMC_STOP_L           = ((uint8_t)1)<<6,
		TMC_STOP_R           = ((uint8_t)1)<<7
	};

public:
	// Driver debugging level
	enum DriverDebugLevel : int {
		TMC_DEBUG_ERRORS    = 0,
		TMC_DEBUG_ACTIONS   = 1,
		TMC_DEBUG_REGISTERS = 2,
		TMC_DEBUG_PACKETS   = 3,
	};


	// Creates a TMC5160 stepper connected via SPI
	TMC5160SPI();

	// Destroys this TMC5160 stepper connected via SPI. Stops device motion for safety's sake
	~TMC5160SPI();

	// Opens and initializes a TMC5160 SPI device. Returns true on success, else false
	bool open(const char *device);

	// Returns true if the device is connected, else false
	bool isConnected() { return fd>=0; }

	// Returns the current speed in the variable pointed to by result. Returns true on success, else false	
	bool getSpeed(int32_t *result) { return getRegister(TMCR_VACTUAL, (uint32_t*) result); }

	// Sets the target speed to the given number of microsteps per second. Returns immediately. Returns true on success, else false
	bool setTargetSpeed(int32_t value);

	// Stops all current movement. Returns true on success, else false
	bool stop() { return setTargetSpeed(0); }

	// Returns the current position in the variable pointed to by result. Returns true on success, else false	
	bool getPosition(int32_t *result) { return getRegister(TMCR_XACTUAL, (uint32_t*) result); }

	// Syncs the current position on device to the given value. Temporarily enters holding mode to avoid moving the axis, 
	// then restores previous ramping mode afterwards. Returns true on success, else false	
	bool syncPosition(int32_t value);

	// Sets the target position, initiating a non-blocking go-to. Returns immediately. Returns true on success, else false
	bool setTargetPosition(int32_t value);

	// Sets the target position and performs a blocking go-to with optional timeout (0=no timeout). Returns when position reached, or timeout occurs. Returns true on success, else false
	bool setTargetPositionBlocking(int32_t value, uint32_t timeoutMs=0);

	// Gets maximal motor speed for gotos. In units of 2^24/f_clk. Returns true on success, else false
	bool getMaxGoToSpeed(uint32_t *result) { *result=maxGoToSpeed;  return true; }

	// Sets maximal motor speed. In units of 2^24/f_clk. Effective on next goto. Returns true on success, else false
	bool getMaxGoToSpeed(uint32_t value) { maxGoToSpeed=value; return true; }

	// Returns the device status flags from the latest command	
	enum TMCStatusFlags getStatus() { return deviceStatus; }

	// Sets driver debugging level
	void setDebugLevel(enum DriverDebugLevel value) { debugLevel=value; }

	// Gets driver debugging level	
	enum DriverDebugLevel getDebugLevel() { return debugLevel; }

	// Sets file for debug output
	void setDebugFile(FILE *f) { debugFile=f; }

	// Gets file for debug output	
	FILE *getDebugFile() { return debugFile; }


	// General configuration settings
	//

	// Gets motor inversion flag 0/1 from device. Returns true on success, else false
	bool getInvertMotor(uint32_t *result) { return getRegisterBits(TMCR_GCONF, result, 4, 1); }

	// Sets motor inversion flag 0/1 on device. Returns true on success, else false
	bool setInvertMotor(uint32_t value) { return setRegisterBits(TMCR_GCONF, value, 4, 1); }

	// Gets diagnosis 0 enable on error flag 0/1 from device. Returns true on success, else false
	bool getDiag0EnableError(uint32_t *result) { return getRegisterBits(TMCR_GCONF, result, 5, 1); }

	// Sets diagnosis 0 enable on error flag 0/1 on device. Returns true on success, else false
	bool setDiag0EnableError(uint32_t value) { return setRegisterBits(TMCR_GCONF, value, 5, 1); }

	// Gets diagnosis 0 enable on overtemperature flag 0/1 from device. Returns true on success, else false
	bool getDiag0EnableTemp(uint32_t *result) { return getRegisterBits(TMCR_GCONF, result, 6, 1); }

	// Sets diagnosis 0 enable on overtemperature flag 0/1 on device. Returns true on success, else false
	bool setDiag0EnableTemp(uint32_t value) { return setRegisterBits(TMCR_GCONF, value, 6, 1); }

	// Gets diagnosis 0 enable on interrupt=0 or step=1 flag from device. Returns true on success, else false
	bool getDiag0EnableInterruptStep(uint32_t *result) { return getRegisterBits(TMCR_GCONF, result, 7, 1); }

	// Sets diagnosis 0 enable on interrupt=0 or step=1 on device. Returns true on success, else false
	bool setDiag0EnableInterruptStep(uint32_t value) { return setRegisterBits(TMCR_GCONF, value, 7, 1); }

	// Gets PWM enable flag 0/1 from device. Returns true on success, else false
	bool getPWMEnableStealthChop(uint32_t *result) { return getRegisterBits(TMCR_GCONF, result, 2, 1); }

	// Sets PWM enable flag 0/1 on device. Returns true on success, else false
	bool setPWMEnableStealthChop(uint32_t value) { return setRegisterBits(TMCR_GCONF, value, 2, 1); }

	// Gets global status flags from device. Returns true on success, else false
	bool getGStat(uint32_t *result) { return getRegister(TMCR_GSTAT, result); }

	// Sets global status flags on device, e.g. to clear device status flags. Returns true on success, else false
	bool setGStat(uint32_t value) { return setRegister(TMCR_GSTAT, value); }


	// PWM settings
	//

	// Gets PWM autoscaling flag 0/1 from device. Returns true on success, else false
	bool getPWMAutoscale(uint32_t *result) { return getRegisterBits(TMCR_PWMCONF, result, 18, 1); }

	// Sets PWM autoscaling flag 0/1 on device. Returns true on success, else false
	bool setPWMAutoscale(uint32_t value) { return setRegisterBits(TMCR_PWMCONF, value, 18, 1); }

	// Gets PWM automatic gradient flag 0/1 from device. Returns true on success, else false
	bool getPWMAutoGradient(uint32_t *result) { return getRegisterBits(TMCR_PWMCONF, result, 19, 1); }

	// Sets PWM automatic gradient flag 0/1 on device. Returns true on success, else false
	bool setPWMAutoGradient(uint32_t value) { return setRegisterBits(TMCR_PWMCONF, value, 19, 1); }

	// Gets PWM frequency divider from device. 0=2/1024 clk, 1=2/683, 2=2/512, 3=2/510. Returns true on success, else false
	bool getPWMFrequencyDivider(uint32_t *result) { return getRegisterBits(TMCR_PWMCONF, result, 17, 2); }

	// Sets PWM frequency divider on device. 0=2/1024 clk, 1=2/683, 2=2/512, 3=2/510. Returns true on success, else false
	bool setPWMFrequencyDivider(uint32_t value) { return setRegisterBits(TMCR_PWMCONF, value, 17, 2); }

	// Gets PWM autoscale amplitude limit from device. See datasheet. Returns true on success, else false
	bool getPWMLimit(uint32_t *result) { return getRegisterBits(TMCR_PWMCONF, result, 28, 4); }

	// Sets PWM autoscale amplitude limit on device. See datasheet. Returns true on success, else false
	bool setPWMLimit(uint32_t value) { return setRegisterBits(TMCR_PWMCONF, value, 28, 4); }


	// Chopper settings
	//

	// Gets chopper mode from device. 0=SpreadCycle, 1=constant off time. Returns true on success, else false
	bool getChopperMode(uint32_t *result) { return getRegisterBits(TMCR_CHOPCONF, result, 14, 1); }

	// Sets chopper mode on device. 0=SpreadCycle, 1=constant off time. Returns true on success, else false
	bool setChopperMode(uint32_t value) { return setRegisterBits(TMCR_CHOPCONF, value, 14, 1); }

	// Gets chopper enable high velocity fullstep mode 0/1 from device. Returns true on success, else false
	bool getChopperHighVelFullstep(uint32_t *result) { return getRegisterBits(TMCR_CHOPCONF, result, 18, 1); }

	// Sets chopper enable high velocity fullstep mode 0/1 on device. Returns true on success, else false
	bool setChopperHighVelFullstep(uint32_t value) { return setRegisterBits(TMCR_CHOPCONF, value, 18, 1); }

	// Gets chopper enable high velocity mode 0/1 from device. Returns true on success, else false
	bool getChopperHighVel(uint32_t *result) { return getRegisterBits(TMCR_CHOPCONF, result, 19, 1); }

	// Sets chopper enable high velocity mode 0/1 on device. Returns true on success, else false
	bool setChopperHighVel(uint32_t value) { return setRegisterBits(TMCR_CHOPCONF, value, 19, 1); }


	// Gets chopper micro step resolution from device. 0=native 256, 1=128, 2=64, 3=32, 4=16, 5=8, 6=4, 7=2 8=full step. Returns true on success, else false
	bool getChopperMicroRes(uint32_t *result) { return getRegisterBits(TMCR_CHOPCONF, result, 24, 4); }

	// Sets chopper micro step resolution on device. 0=native 256, 1=128, 2=64, 3=32, 4=16, 5=8, 6=4, 7=2 8=full step. Returns true on success, else false
	bool setChopperMicroRes(uint32_t value) { return setRegisterBits(TMCR_CHOPCONF, value, 24, 4); }

	// Gets chopper off time and driver enable from device. 24+32*TOFF clocks. Returns true on success, else false
	bool getChopperTOff(uint32_t *result) { return getRegisterBits(TMCR_CHOPCONF, result, 0, 4); }

	// Sets chopper off time and driver enable on device. 24+32*TOFF clocks. Returns true on success, else false
	bool setChopperTOff(uint32_t value) { return setRegisterBits(TMCR_CHOPCONF, value, 0, 4); }

	// Gets chopper blank time select from device. 24+32*TOFF clocks. Returns true on success, else false
	bool getChopperTBlank(uint32_t *result) { return getRegisterBits(TMCR_CHOPCONF, result, 15, 2); }

	// Sets chopper blank time select on device. 24+32*TOFF clocks. Returns true on success, else false
	bool setChopperTBlank(uint32_t value) { return setRegisterBits(TMCR_CHOPCONF, value, 15, 2); }

	// Gets chopper hysteresis start value from device. 24+32*TOFF clocks. Returns true on success, else false
	bool getChopperHStart(uint32_t *result) { return getRegisterBits(TMCR_CHOPCONF, result, 4, 3); }

	// Sets chopper hysteresis start value on device. 24+32*TOFF clocks. Returns true on success, else false
	bool setChopperHStart(uint32_t value) { return setRegisterBits(TMCR_CHOPCONF, value, 4, 3); }

	// Gets chopper hysteresis end value from device. 24+32*TOFF clocks. Returns true on success, else false
	bool getChopperHEnd(uint32_t *result) { return getRegisterBits(TMCR_CHOPCONF, result, 7, 4); }

	// Sets chopper hysteresis end value on device. 24+32*TOFF clocks. Returns true on success, else false
	bool setChopperHEnd(uint32_t value) { return setRegisterBits(TMCR_CHOPCONF, value, 7, 4); }

	// Runs automatic chopper tuning procedure, as per TMC5160A datasheet section 7.1, p.57ff
	bool chopperAutoTuneStealthChop(uint32_t secondSteps, uint32_t timeoutMs);


	// Velocity dependent configuration settings
	//

	// Gets global current scaler from device, from 0..255, where 0 counts as 256. Returns true on success, else false
	bool getGlobalCurrentScaler(uint32_t *result) { return getRegister(TMCR_GLOBAL_SCALER, result); }

	// Sets global current scaler on device, from 0..255, where 0 counts as 256. Returns true on success, else false
	bool setGlobalCurrentScaler(uint32_t value) { return setRegister(TMCR_GLOBAL_SCALER, value); }

	// Gets hold motor current from device. 0=1/32 ... 31=32/32. Returns true on success, else false
	bool getIHold(uint32_t *result) { return getRegisterBits(TMCR_IHOLD_IRUN, result, 0, 5); }

	// Sets hold motor current on device. 0=1/32 ... 31=32/32. Returns true on success, else false
	bool setIHold(uint32_t value) { return setRegisterBits(TMCR_IHOLD_IRUN, value, 0, 5); }

	// Gets run motor current from device. 0=1/32 ... 31=32/32. Returns true on success, else false
	bool getIRun(uint32_t *result) { return getRegisterBits(TMCR_IHOLD_IRUN, result, 8, 5); }

	// Sets run motor current on device. 0=1/32 ... 31=32/32. Returns true on success, else false
	bool setIRun(uint32_t value) { return setRegisterBits(TMCR_IHOLD_IRUN, value, 8, 5); }

	// Gets ramp time for powering down motor current from device. In 2^18 clocks. Works after TPowerdown. Returns true on success, else false
	bool getIHoldDelay(uint32_t *result) { return getRegisterBits(TMCR_IHOLD_IRUN, result, 16, 4); }

	// Sets ramp time for powering down motor current on device. In 2^18 clocks. Works after TPowerdown. Returns true on success, else false
	bool setIHoldDelay(uint32_t value) { return setRegisterBits(TMCR_IHOLD_IRUN, value, 16, 4); }

	// Gets motor powerdown delay from device. In 2^18 clocks. Works before IHoldDelay. Returns true on success, else false
	bool getTPowerDown(uint32_t *result) { return getRegister(TMCR_TPOWER_DOWN, result); }

	// Sets motor powerdown delay on device. In 2^18 clocks. Works before IHoldDelay. Returns true on success, else false
	bool setTPowerDown(uint32_t value) { return setRegister(TMCR_TPOWER_DOWN, value); }

	// Gets actual measured time between two 1/256 microsteps measured in clocks. Returns true on success, else false
	bool getTStep(uint32_t *result) { return getRegister(TMCR_TSTEP, result); }

	// Setter omitted intentionally, this is a read only property

	// Gets step time threshold for StealthChop voltage PWM mode from device. PWM is on iff getTStep() is >= this. Returns true on success, else false
	bool getTPWMThreshold(uint32_t *result) { return getRegister(TMCR_TPWMTHRS, result); }

	// Sets step time threshold for StealthChop voltage PWM mode on device. PWM is on iff getTStep() is >= this. Returns true on success, else false
	bool setTPWMThreshold(uint32_t value) { return setRegister(TMCR_TPWMTHRS, value); }

	// Gets step time threshold for CoolStep and StallGuard mode from device. CoolStep is on iff this >= getTStep() >= getTHigh(). Returns true on success, else false
	bool getTCoolThreshold(uint32_t *result) { return getRegister(TMCR_TCOOLTHRS, result); }

	// Sets step time threshold for  for CoolStep and StallGuard mode on device. CoolStep is on iff this >= getTStep() >= getTHigh(). Returns true on success, else false
	bool setTCoolThreshold(uint32_t value) { return setRegister(TMCR_TCOOLTHRS, value); }

	// Gets step time threshold for high-speed voltage PWM mode from device. High-speed is on iff getTStep() <= this. Returns true on success, else false
	bool getTHighThreshold(uint32_t *result) { return getRegister(TMCR_THIGH, result); }

	// Sets step time threshold for high-speed mode on device. High-speed is on iff getTStep() <= this. Returns true on success, else false
	bool setTHighThreshold(uint32_t value) { return setRegister(TMCR_THIGH, value); }


	// Ramp configuration settings
	//

	// Gets initial motor speed when starting from standstill. In units of 2^24/f_clk. Returns true on success, else false
	bool getVStart(uint32_t *result) { return getRegister(TMCR_VSTART, result); }

	// Sets initial motor speed when starting from standstill. In units of 2^24/f_clk. Returns true on success, else false
	bool setVStart(uint32_t value) { return setRegister(TMCR_VSTART, value); }

	// Gets initial motor acceleration after starting from standstill. In units of 2^41/f_clk^2. Returns true on success, else false
	bool getA1(uint32_t *result) { return getRegister(TMCR_A1, result); }

	// Sets initial motor acceleration after starting from standstill. In units of 2^41/f_clk^2. Returns true on success, else false
	bool setA1(uint32_t value) { return setRegister(TMCR_A1, value); }

	// Gets motor speed for switchover to max acceleration. In units of 2^24/f_clk. Returns true on success, else false
	bool getV1(uint32_t *result) { return getRegister(TMCR_V1, result); }

	// Sets motor speed for switchover to max acceleration. In units of 2^24/f_clk. Returns true on success, else false
	bool setV1(uint32_t value) { return setRegister(TMCR_V1, value); }

	// Gets maximal motor acceleration. In units of 2^41/f_clk^2. Returns true on success, else false
	bool getAMax(uint32_t *result) { return getRegister(TMCR_AMAX, result); }

	// Sets maximal motor acceleration. In units of 2^41/f_clk^2. Returns true on success, else false
	bool setAMax(uint32_t value) { return setRegister(TMCR_AMAX, value); }

	// Gets maximal motor speed. In units of 2^24/f_clk. Returns true on success, else false
	bool getVMax(uint32_t *result) { return getRegister(TMCR_VMAX, result); }

	// Sets maximal motor speed. In units of 2^24/f_clk. Returns true on success, else false
	bool setVMax(uint32_t value) { return setRegister(TMCR_VMAX, value); }

	// Gets maximal motor deceleration. In units of 2^41/f_clk^2. Returns true on success, else false
	bool getDMax(uint32_t *result) { return getRegister(TMCR_DMAX, result); }

	// Sets maximal motor deceleration. In units of 2^41/f_clk^2. Returns true on success, else false
	bool setDMax(uint32_t value) { return setRegister(TMCR_DMAX, value); }

	// Gets final motor acceleration for deceleration to stop. In units of 2^41/f_clk^2. Returns true on success, else false
	bool getD1(uint32_t *result) { return getRegister(TMCR_D1, result); }

	// Sets final motor acceleration for deceleration to stop. In units of 2^41/f_clk^2. Returns true on success, else false
	bool setD1(uint32_t value) { return setRegister(TMCR_D1, value); }

	// Gets final motor speed before stopping. In units of 2^24/f_clk. Must be greater or equal than VStart. Do not set to 0 in positioning mode, minimum 10 recommended. Returns true on success, else false
	bool getVStop(uint32_t *result) { return getRegister(TMCR_VSTOP, result); }

	// Sets final  motor speed before stopping. In units of 2^24/f_clk. Must be greater or equal than VStart. Do not set to 0 in positioning mode, minimum 10 recommended. Returns true on success, else false
	bool setVStop(uint32_t value) { return setRegister(TMCR_VSTOP, value); }

	// Gets waiting time between movements in opposite directions. In units of 512*t_clk. Returns true on success, else false
	bool getTZeroWait(uint32_t *result) { return getRegister(TMCR_TZEROWAIT, result); }

	// Gets waiting time between movements in opposite directions. In units of 512*t_clk. Returns true on success, else false
	bool setTZeroWait(uint32_t value) { return setRegister(TMCR_TZEROWAIT, value); }

	// Get register name. Drops the 0x80 flag used for setting registers
	static const char *getRegisterName(uint8_t address) { return registerMetaData[address & 0x007f].name; }

	// Returns true if a register can be read from in hardware. Drops the 0x80 flag used for setting registers.
	static bool canReadRegister(uint8_t address) { return registerMetaData[address & 0x007f].mode & TMCRM_R; }

	// Returns true if a register can be written to in harwdare. Drops the 0x80 flag used for setting registers
	static bool canWriteRegister(uint8_t address) { return registerMetaData[address & 0x007f].mode & TMCRM_W; }


protected:
	// Gets bits from a register value from the device. For convenience, uses a driver-side cache for write-only registers. 
	// Updates spiStatus if successfully read from device. Fails if the register is undefined. Returns true on success, else false.
	bool getRegisterBits(uint8_t address, uint32_t *result, uint32_t firstBit, uint32_t numBits);

	// Sets bits in a register value on the device. For convenience, uses a driver-side cache for write-only registers. 
	// Other bits left unchanged. Updates spiStatus if successful. Fails if the register is not writeable. Returns true on success, else false
	bool setRegisterBits(uint8_t address, uint32_t value, uint32_t firstBit, uint32_t numBits);

	// Gets a register value from the device. For convenience, uses a driver-side cache for write-only registers.
	// Updates spiStatus if successfully read from device. Fails if the register is undefined. Returns true on success, else false
	bool getRegister(uint8_t address, uint32_t *result);

	// Sets a register on device to the given value. For convenience, uses a driver-side cache for write-only registers.
	// Updates spiStatus if successful. Fails if the register is not writeable. Returns true on success, else false
	bool setRegister(uint8_t address, uint32_t value);

	// Sends the given number of raw bytes to the device, then retrieves the same number of bytes. Returns true on success, else false
	bool sendReceiveRaw(const uint8_t *tx, uint8_t *rx, uint32_t numBytes);

	// Prints a packet to stdout with given prefix and suffix (if non-NULL). Returns true on success, else false
	static bool printPacket(FILE *f, const uint8_t *data, uint32_t numBytes, bool isTX, const char *prefix, const char *suffix);

	// Prints a register get/set command
	static void printRegister(FILE *f, uint8_t address, uint32_t value, uint8_t status, const char *prefix, const char *suffix);

public:
	// Prints status flags
	static void printStatus(FILE *f, uint8_t status);


protected:
	// File descriptor for the SPI device
	int fd;

	// Device status returned by the last SPI datagram
	TMCStatusFlags deviceStatus;

	// Maximum speed for GoTos. Stored separately as setTargetSpeed() overwrites VMAX on the device
	uint32_t maxGoToSpeed;

	// Last value written to write-only register TMCR_IHOLD_IRUN
	uint32_t cachedRegisterValues[TMCR_NUM_REGISTERS];

	// Debug level
	enum DriverDebugLevel debugLevel;

	// File for debug output 
	FILE *debugFile;


public:
	// Default SPI device
	static const char *defaultDevice;

protected:	
	// Default SPI mode settings
	static const uint8_t defaultSPIMode;

	// Default SPI bit settings
	static const uint8_t defaultSPIBits;

	// Default SPI maximum speed in Hertz
	static const uint32_t defaultSPIMaxSpeedHz;

	// Default SPI delay in microseconds
	static const uint32_t defaultSPIDelayUsec;

	// Table of register metadata (names etc.) 
	static const TMCRegisterMetaData registerMetaData[];

	// Table of status flag names
	static const char *statusFlagNames[];
};

#endif // PIMOCO_TMC5160_H
