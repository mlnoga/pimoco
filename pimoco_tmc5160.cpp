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


#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "pimoco_tmc5160.h"


static const char    *TMC5160SPI::defaultDevice="/dev/spidev1.1";
static const uint8_t  TMC5160SPI::defaultSPIMode=SPI_CS_LOW;
static const uint8_t  TMC5160SPI::defaultSPIBits=8;
static const uint32_t TMC5160SPI::defaultSPIMaxSpeedHz=500000;
static const uint32_t TMC5160SPI::defaultSPIDelayUsec=0;


TMC5160SPI::TMC5160SPI() : fd(-1), statusFlags(0), maxGoToSpeed(100000) {
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

	fd=open(deviceName, O_RDWR);
	return fd>=0;

	// Set SPI mode, bits per word and speed
	//
	if(ioctl(fd, SPI_IOC_WR_MODE, &defaultMode)<0)
		return false;
	if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &defaultBits)<0)
		return false;
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &defaultMaxSpeedHz)<0)
		return false;

	// Stop device, just in case it was left running
	stop();
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
	if(!setGlobalCurrentScaler(TMCR_GLOBAL_SCALER, 0))  // full global current with 0 = 256/256 of nominal power
		return false;
	if(!setIRun(31))  // Run current: 31=32/32 or 100% of global current
		return false;
	if(!setIHold(31)) // Hold current = run current for StealthChop configuration
		return false;
	if(!setIHoldDelay(10))
		return false;
	if(!setTPowerDown(10))
		return false;

	if(!setMotorInvert(0))
		return false;

	// Set a default ramp (used in auto-tuning)
	//
	if(!setVstart(0))
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
	uint32_t startPos;
	if(!getPosition(&startPos))
		return false;

	uint32_t microRes;
	if(!getChopperMicroRes(&microRes))
		return false;
	fullStep=256>>microRes; 

	// move a single full step
	uint32_t targetPos=startPos+fullStep;
	if(!setPositionBlocking(targetPos, timeoutMs))
		return false;

	// wait >=130 ms, then device automatically sets PWM_OFS_AUTO
	usleep(140000l);

	// move given amount of full steps (should be few 100s), while device automatically updates PWM_GRAD_AUTO
	targetPos+=secondSteps*fullStep;
	if(!setPositionBlocking(targetPos, timeoutMs))
		return false;

	// return to starting position
	return setPositionBlocking(startPos, timeoutMs)
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


bool TMC5160SPI::setPositionBlocking(uint32_t value, uint32_t timeoutMs) {
	if(!setPosition(value))
		return false;

	// wait until in position
	for(int i=0; i<timeoutMs; i++) {
		usleep(1000l);  // 1 ms
		uint32_t pos;
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
}


bool TMC5160SPI::setRegisterBits(uint8_t address, uint32_t value, uint32_t firstBit, uint32_t numBits) {
	uint32_t tmp;
	if(!getRegister(address, &tmp))
		return false;
	uint32_t mask=(uint32_t) ( (((uint64_t)1)<<numBits)-1 );
	tmp=(tmp & ~(mask<<firstBit)) | ((value&mask)<<firstBit)
	return setRegister(address, tmp);
}


bool TMC5160SPI::getRegister(uint8_t address, uint32_t *value) {
	uint8_t tx[5]={address & 0x007f,0,0,0,0};
	uint8_t rx[5];

	// Per the datasheet, raw send/receive returns the value requested with the PREVIOUS transfer.
	// As SPI is not performance critical for our application, we simply send read requests twice. 
	for(int i=0; i<2; i++)
		if(!spiSendReceiveRaw(tx,rx,5))
			return false;

	deviceStatus=result[0];
	*result=(((uint32_t) result[1])<<24) | (((uint32_t) result[2])<<16) | 
	        (((uint32_t) result[3])<<8)  |  ((uint32_t) result[4]); 
	return true;
}


bool TMC5160SPI::setRegister(uint8_t address, uint32_t value) {
	uint8_t tx[5]={address | 0x0080, (uint8_t) ((value>>24)&0x00ff), (uint8_t) ((value>>16)&0x00ff), 
		                             (uint8_t) ((value>>8)&0x00ff),  (uint8_t) ((value>>0)&0x00ff)   };
	uint8_t rx[5];
	if(!spiSendReceiveRaw(tx,rx,5))
		return false;

	deviceStatus=result[0];
	// result[1..4] contains the value from the previous send/receive command. Ignoring that.
	return true;
}


bool TMC5160SPI::spiSendReceiveRaw(const uint8_t *tx, uint8_t *rx, in len) {
	struct spi_ioc_transfer tr = {
		.tx_buf        = (unsigned long) tx,
		.rx_buf        = (unsigned long) rx,
		.len           = len,
		.delay_usecs   = defaultSPIDelayUsec,
		.speed_hz      = defaultSPISpeed,
		.bits_per_word = defaultSPIBits,
	};

	return ioctl(fd, SPI_IOC_MESSAGE(1), &tr)>=0;
}
