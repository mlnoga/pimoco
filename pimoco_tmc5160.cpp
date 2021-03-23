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

#include "pimoco_tmc5160.h"


const char    *TMC5160SPI::defaultDevice="/dev/spidev1.0";
const uint8_t  TMC5160SPI::defaultSPIMode=SPI_MODE_0;
const uint8_t  TMC5160SPI::defaultSPIBits=8;
const uint32_t TMC5160SPI::defaultSPIMaxSpeedHz=500000;
const uint32_t TMC5160SPI::defaultSPIDelayUsec=0;

const char     *TMC5160SPI::registerNames[]={
	"GCONF", // 0x00
	"GSTAT", // 0x01
	"IFCNT", // 0x02
	"SLAVECONF", // 0x03
	"IOIN_or_OUTPUT", // 0x04
	"X_COMPARE", // 0x05
	"OPT_PROG", // 0x06
	"OPT_READ", // 0x07
	"FACTORY_CONF", // 0x08
	"SHORT_CONF", // 0x09
	"DRV_CONF", // 0x0a
	"GLOBAL_SCALER", // 0x0b
	"OFFSET_READ", // 0x0c
	"UNUSED", // 0x0d
	"UNUSED", // 0x0e
	"UNUSED", // 0x0f
	"IHOLD_IRUN", // 0x10
	"TPOWER_DOWN", // 0x11
	"TSTEP", // 0x12
	"TPWMTHRS", // 0x13
	"TCOOLTHRS", // 0x14
	"THIGH", // 0x15
	"UNUSED", // 0x16
	"UNUSED", // 0x17
	"UNUSED", // 0x18
	"UNUSED", // 0x19
	"UNUSED", // 0x1a
	"UNUSED", // 0x1b
	"UNUSED", // 0x1c
	"UNUSED", // 0x1d
	"UNUSED", // 0x1e
	"UNUSED", // 0x1f
	"RAMPMODE", // 0x20
	"XACTUAL", // 0x21
	"VACTUAL", // 0x22
	"VSTART", // 0x23
	"A1", // 0x24
	"V1", // 0x25
	"AMAX", // 0x26
	"VMAX", // 0x27
	"DMAX", // 0x28
	"UNUSED", // 0x29
	"D1", // = 0x2a
	"VSTOP", // 0x2b
	"TZEROWAIT", // 0x2c
	"XTARGET", // 0x2d
	"UNUSED", // 0x2e
	"UNUSED", // 0x2f
	"UNUSED", // 0x30
	"UNUSED", // 0x31
	"UNUSED", // 0x32
	"VDCMIN", // 0x33
	"SW_MODE", // 0x34
	"RAMP_STAT", // 0x35
	"XLATCH", // 0x36
	"UNUSED", // 0x07
	"ENCMODE", // 0x38
	"X_ENC", // 0x39
	"ENC_CONST", // 0x3a
	"ENC_STATUS", // 0x3b
	"ENC_LATCH", // 0x3c
	"ENC_DEVIATION", // 0x3d
	"UNUSED", // 0x3e
	"UNUSED", // 0x3f
	"UNUSED", // 0x40
	"UNUSED", // 0x41
	"UNUSED", // 0x42
	"UNUSED", // 0x43
	"UNUSED", // 0x44
	"UNUSED", // 0x45
	"UNUSED", // 0x46
	"UNUSED", // 0x47
	"UNUSED", // 0x48
	"UNUSED", // 0x49
	"UNUSED", // 0x4a
	"UNUSED", // 0x4b
	"UNUSED", // 0x4c
	"UNUSED", // 0x4d
	"UNUSED", // 0x4e
	"UNUSED", // 0x4f
	"UNUSED", // 0x50
	"UNUSED", // 0x51
	"UNUSED", // 0x52
	"UNUSED", // 0x53
	"UNUSED", // 0x54
	"UNUSED", // 0x55
	"UNUSED", // 0x56
	"UNUSED", // 0x57
	"UNUSED", // 0x58
	"UNUSED", // 0x59
	"UNUSED", // 0x5a
	"UNUSED", // 0x5b
	"UNUSED", // 0x5c
	"UNUSED", // 0x5d
	"UNUSED", // 0x5e
	"UNUSED", // 0x5f
	"MSLUT0", // 0x60
	"MSLUT1", // 0x61
	"MSLUT2", // 0x62
	"MSLUT3", // 0x63
	"MSLUT4", // 0x64
	"MSLUT5", // 0x65
	"MSLUT6", // 0x66
	"MSLUT7", // 0x67
	"MSLUTSEL", // 0x68
	"MSLUTSTART", // 0x69
	"MSCNT", // 0x6a
	"MSCURACT", // 0x6b
	"CHOPCONF", // 0x6c
	"COOLCONF", // 0x6d
	"DCCTRL", // 0x6e
	"DRV_STATUS", // 0x6f
	"PWMCONF", // 0x70
	"PWM_SCALE", // 0x71
	"PWM_AUTO", // 0x72
	"LOST_STEPS", // 0x73
	"UNUSED", // 0x74
	"UNUSED", // 0x75
	"UNUSED", // 0x76
	"UNUSED", // 0x77
	"UNUSED", // 0x78
	"UNUSED", // 0x79
	"UNUSED", // 0x7a
	"UNUSED", // 0x7b
	"UNUSED", // 0x7c
	"UNUSED", // 0x7d
	"UNUSED", // 0x7e
	"UNUSED", // 0x7f
};

TMC5160SPI::TMC5160SPI() : fd(-1), deviceStatus((enum TMCStatusFlags) 0), maxGoToSpeed(100000) {
}


TMC5160SPI::~TMC5160SPI() {
	if(fd>=0) {
		stop();
		close(fd);
	}
}

bool TMC5160SPI::open(const char *deviceName) {
	if(fd>=0) {
		stop();
		close(fd);
	}

	fd=::open(deviceName, O_RDWR);
	if(fd<0)
		return false;

	// Set SPI mode, bits per word and speed
	//
	if(ioctl(fd, SPI_IOC_WR_MODE, &defaultSPIMode)<0)
		return false;
	if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &defaultSPIBits)<0)
		return false;
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &defaultSPIMaxSpeedHz)<0)
		return false;

	// Stop device, just in case it was left running
	if(!stop())
		return false;
	usleep(100*1000l);

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
	if(!setGlobalCurrentScaler(0))  // full global current with 0 = 256/256 of nominal power
		return false;
	if(!setIRun(31))  // Run current: 31=32/32 or 100% of global current
		return false;
	if(!setIHold(31)) // Hold current = run current for StealthChop configuration
		return false;
	if(!setIHoldDelay(10))
		return false;
	if(!setTPowerDown(10))
		return false;

	if(!setInvertMotor(0))
		return false;

	// Set a default ramp (used in auto-tuning)
	//
	if(!setVStart(0))
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

	chopperAutoTuneStealthChop(500, 1000);

	// now that configuration is complete, set hold current to proper target 
	if(!setIHold(10)) // 33% hold current: 10=11/32 of global current
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

	return true;
}


bool TMC5160SPI::chopperAutoTuneStealthChop(uint32_t secondSteps, uint32_t timeoutMs) {
	// keep track of starting position
	int32_t startPos;
	if(!getPosition(&startPos))
		return false;

	uint32_t microRes;
	if(!getChopperMicroRes(&microRes))
		return false;
	uint32_t fullStep=256>>microRes; 

	// move a single full step
	int32_t targetPos=startPos+(int32_t)fullStep;
	if(!setPositionBlocking(targetPos, timeoutMs))
		return false;

	// wait >=130 ms, then device automatically sets PWM_OFS_AUTO
	usleep(140000l);

	// move given amount of full steps (should be few 100s), while device automatically updates PWM_GRAD_AUTO
	targetPos+=secondSteps*(int32_t)fullStep;
	if(!setPositionBlocking(targetPos, timeoutMs))
		return false;

	// return to starting position
	return setPositionBlocking(startPos, timeoutMs);
}


bool TMC5160SPI::setSpeed(int32_t value) {
	return setRegister(TMCR_RAMPMODE, value>=0 ? 1 : 2) &&    // select velocity mode and sign
	       setRegister(TMCR_VMAX, value>=0 ? value : -value); // set absolute target speed to initiate movement
}


bool TMC5160SPI::setPosition(int32_t value) {
	return setRegister(TMCR_RAMPMODE, 0) &&                  // select absolute positioning mode
		   setRegister(TMCR_VMAX, maxGoToSpeed) &&           // restore max speed in case setSpeed() overwrote it
	       setRegister(TMCR_XTARGET, (uint32_t) value);      // set target position to initiate movement
}


bool TMC5160SPI::setPositionBlocking(int32_t value, uint32_t timeoutMs) {
	if(!setPosition(value))
		return false;

	// wait until in position
	for(uint32_t i=0; i<timeoutMs; i++) {
		usleep(1000l);  // 1 ms
		int32_t pos;
		if(!getPosition(&pos))
			return false;
		if(pos==value)
			return true;
	}
	return false; // timeout
}


bool TMC5160SPI::getRegisterBits(uint8_t address, uint32_t *result, uint32_t firstBit, uint32_t numBits) {
	uint32_t tmp;
	if(!getRegister(address, &tmp))
		return false;
	uint32_t mask=(uint32_t) ( (((uint64_t)1)<<numBits)-1 );
	*result=(tmp>>firstBit) & mask;
	return true;
}


bool TMC5160SPI::setRegisterBits(uint8_t address, uint32_t value, uint32_t firstBit, uint32_t numBits) {
	uint32_t tmp;
	if(!getRegister(address, &tmp))
		return false;
	uint32_t mask=(uint32_t) ( (((uint64_t)1)<<numBits)-1 );
	tmp=(tmp & ~(mask<<firstBit)) | ((value&mask)<<firstBit);
	return setRegister(address, tmp);
}


bool TMC5160SPI::getRegister(uint8_t address, uint32_t *result) {
	uint8_t tx[5]={(uint8_t) (address & 0x007f),0,0,0,0};
	uint8_t rx[5];

	// Per the datasheet, raw send/receive returns the value requested with the PREVIOUS transfer.
	// As SPI is not performance critical for our application, we simply send read requests twice. 
	for(int i=0; i<2; i++)
		if(!sendReceiveRaw(tx,rx,5))
			return false;

	deviceStatus=(enum TMCStatusFlags) rx[0];
	*result=(((uint32_t) rx[1])<<24) | (((uint32_t) rx[2])<<16) | 
	        (((uint32_t) rx[3])<<8)  |  ((uint32_t) rx[4]); 
	return true;
}


bool TMC5160SPI::setRegister(uint8_t address, uint32_t value) {
	uint8_t tx[5]={(uint8_t) (address | 0x0080), 
		           (uint8_t) ((value>>24)&0x00ff), (uint8_t) ((value>>16)&0x00ff), 
		           (uint8_t) ((value>>8)&0x00ff),  (uint8_t) ((value>>0)&0x00ff)   };
	uint8_t rx[5];
	if(!sendReceiveRaw(tx,rx,5))
		return false;

	deviceStatus=(enum TMCStatusFlags) rx[0];
	// result[1..4] contains the value from the previous send/receive command. Ignoring that.
	return true;
}


bool TMC5160SPI::sendReceiveRaw(const uint8_t *tx, uint8_t *rx, uint32_t len) {
	struct spi_ioc_transfer tr = {
		.tx_buf        = (unsigned long) tx,
		.rx_buf        = (unsigned long) rx,
		.len           = len,
		.speed_hz      = defaultSPIMaxSpeedHz,
		.delay_usecs   = defaultSPIDelayUsec,
		.bits_per_word = defaultSPIBits,
	};

	prettyPrint(tx, len, "TX:", NULL);

	int res=ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

	prettyPrint(rx, len, "RX:", NULL);
	printf(" Return: %d\n", res);

	return res>=0;
}

bool TMC5160SPI::prettyPrint(const uint8_t *data, uint32_t numBytes, const char *prefix, const char *suffix) {
	if(data==NULL || numBytes==0)
		return false;
	if(prefix!=NULL)
		printf("%s", prefix);

	const char *opName=data[0]<=0x0080 ? "get" : "set";
	const char *regName=registerNames[data[0] & 0x007f];
	printf(" (%s %s)", opName, regName);

	for(uint32_t i=0; i<numBytes; i++)
		printf(" %02X",data[i]);

	if(suffix!=NULL)
		printf("%s", suffix);
	return true;
}
