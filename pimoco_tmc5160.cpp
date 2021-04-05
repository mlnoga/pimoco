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
#include <libindi/indilogger.h> // for LOG_..., LOGF_... macros

#include "pimoco_tmc5160.h"


const TMC5160::TMCRegisterMetaData TMC5160::registerMetaData[]={
	{ "GCONF",          TMC5160::TMCRM_RW   }, // 0x00
	{ "GSTAT",          TMC5160::TMCRM_RW   }, // 0x01
	{ "IFCNT",          TMC5160::TMCRM_R    }, // 0x02
	{ "SLAVECONF",      TMC5160::TMCRM_W    }, // 0x03
	{ "IOIN_or_OUTPUT", TMC5160::TMCRM_RW   }, // 0x04
	{ "X_COMPARE",      TMC5160::TMCRM_W    }, // 0x05
	{ "OPT_PROG",       TMC5160::TMCRM_W    }, // 0x06
	{ "OPT_READ",       TMC5160::TMCRM_R    }, // 0x07
	{ "FACTORY_CONF",   TMC5160::TMCRM_RW   }, // 0x08
	{ "SHORT_CONF",     TMC5160::TMCRM_W    }, // 0x09
	{ "DRV_CONF",       TMC5160::TMCRM_W    }, // 0x0a
	{ "GLOBAL_SCALER",  TMC5160::TMCRM_W    }, // 0x0b
	{ "OFFSET_READ",    TMC5160::TMCRM_R    }, // 0x0c
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x0d
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x0e
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x0f
	{ "IHOLD_IRUN",     TMC5160::TMCRM_W    }, // 0x10
	{ "TPOWER_DOWN",    TMC5160::TMCRM_W    }, // 0x11
	{ "TSTEP",          TMC5160::TMCRM_R    }, // 0x12
	{ "TPWMTHRS",       TMC5160::TMCRM_W    }, // 0x13
	{ "TCOOLTHRS",      TMC5160::TMCRM_W    }, // 0x14
	{ "THIGH",          TMC5160::TMCRM_W    }, // 0x15
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x16
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x17
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x18
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x19
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x1a
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x1b
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x1c
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x1d
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x1e
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x1f
	{ "RAMPMODE",       TMC5160::TMCRM_RW   }, // 0x20
	{ "XACTUAL",        TMC5160::TMCRM_RW   }, // 0x21
	{ "VACTUAL",        TMC5160::TMCRM_R    }, // 0x22
	{ "VSTART",         TMC5160::TMCRM_W    }, // 0x23
	{ "A1",             TMC5160::TMCRM_W    }, // 0x24
	{ "V1",             TMC5160::TMCRM_W    }, // 0x25
	{ "AMAX",           TMC5160::TMCRM_W    }, // 0x26
	{ "VMAX",           TMC5160::TMCRM_W    }, // 0x27
	{ "DMAX",           TMC5160::TMCRM_W    }, // 0x28
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x29
	{ "D1",             TMC5160::TMCRM_W    }, // 0x2a
	{ "VSTOP",          TMC5160::TMCRM_W    }, // 0x2b
	{ "TZEROWAIT",      TMC5160::TMCRM_W    }, // 0x2c
	{ "XTARGET",        TMC5160::TMCRM_RW   }, // 0x2d
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x2e
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x2f
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x30
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x31
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x32
	{ "VDCMIN",         TMC5160::TMCRM_W    }, // 0x33
	{ "SW_MODE",        TMC5160::TMCRM_RW   }, // 0x34
	{ "RAMP_STAT",      TMC5160::TMCRM_RW   }, // 0x35
	{ "XLATCH",         TMC5160::TMCRM_R    }, // 0x36
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x07
	{ "ENCMODE",        TMC5160::TMCRM_RW   }, // 0x38
	{ "X_ENC",          TMC5160::TMCRM_RW   }, // 0x39
	{ "ENC_CONST",      TMC5160::TMCRM_W    }, // 0x3a
	{ "ENC_STATUS",     TMC5160::TMCRM_RW   }, // 0x3b
	{ "ENC_LATCH",      TMC5160::TMCRM_R    }, // 0x3c
	{ "ENC_DEVIATION",  TMC5160::TMCRM_W    }, // 0x3d
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x3e
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x3f
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x40
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x41
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x42
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x43
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x44
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x45
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x46
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x47
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x48
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x49
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x4a
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x4b
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x4c
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x4d
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x4e
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x4f
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x50
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x51
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x52
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x53
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x54
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x55
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x56
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x57
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x58
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x59
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x5a
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x5b
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x5c
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x5d
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x5e
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x5f
	{ "MSLUT0",         TMC5160::TMCRM_W    }, // 0x60
	{ "MSLUT1",         TMC5160::TMCRM_W    }, // 0x61
	{ "MSLUT2",         TMC5160::TMCRM_W    }, // 0x62
	{ "MSLUT3",         TMC5160::TMCRM_W    }, // 0x63
	{ "MSLUT4",         TMC5160::TMCRM_W    }, // 0x64
	{ "MSLUT5",         TMC5160::TMCRM_W    }, // 0x65
	{ "MSLUT6",         TMC5160::TMCRM_W    }, // 0x66
	{ "MSLUT7",         TMC5160::TMCRM_W    }, // 0x67
	{ "MSLUTSEL",       TMC5160::TMCRM_W    }, // 0x68
	{ "MSLUTSTART",     TMC5160::TMCRM_W    }, // 0x69
	{ "MSCNT",          TMC5160::TMCRM_R    }, // 0x6a
	{ "MSCURACT",       TMC5160::TMCRM_R    }, // 0x6b
	{ "CHOPCONF",       TMC5160::TMCRM_RW   }, // 0x6c
	{ "COOLCONF",       TMC5160::TMCRM_W    }, // 0x6d
	{ "DCCTRL",         TMC5160::TMCRM_W    }, // 0x6e
	{ "DRV_STATUS",     TMC5160::TMCRM_R    }, // 0x6f
	{ "PWMCONF",        TMC5160::TMCRM_W    }, // 0x70
	{ "PWM_SCALE",      TMC5160::TMCRM_R    }, // 0x71
	{ "PWM_AUTO",       TMC5160::TMCRM_R    }, // 0x72
	{ "LOST_STEPS",     TMC5160::TMCRM_R    }, // 0x73
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x74
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x75
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x76
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x77
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x78
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x79
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x7a
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x7b
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x7c
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x7d
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x7e
	{ "UNDEFINED",      TMC5160::TMCRM_NONE }, // 0x7f
};

const char *TMC5160::statusFlagNames[]={
		"RESET",             // Bit 0
		"DRIVER_ERROR",      // Bit 1
		"STALL_GUARD",       // Bit 2
		"STAND_STILL",       // Bit 3
		"VELOCITY_REACHED",  // Bit 4
		"POSITION_REACHED",  // Bit 5
		"STOP_L",            // Bit 6
		"STOP_R",            // Bit 7
};

TMC5160::TMC5160(const char *theIndiDeviceName) : SPI(theIndiDeviceName), deviceStatus((enum TMCStatusFlags) 0) {
    for(int i=0; i<(int) TMCR_NUM_REGISTERS; i++)
    	cachedRegisterValues[i]=0;
}


bool TMC5160::getRegisterBits(uint8_t address, uint32_t *result, uint32_t firstBit, uint32_t numBits) {
	uint32_t tmp;
	if(!getRegister(address, &tmp))
		return false;
	uint32_t mask=(uint32_t) ( (((uint64_t)1)<<numBits)-1 );
	*result=(tmp>>firstBit) & mask;
	return true;
}


bool TMC5160::setRegisterBits(uint8_t address, uint32_t value, uint32_t firstBit, uint32_t numBits) {
	uint32_t tmp;
	if(!getRegister(address, &tmp))
		return false;
	uint32_t mask=(uint32_t) ( (((uint64_t)1)<<numBits)-1 ) << firstBit;
	int tmp2=(tmp & ~mask) | ((value<<firstBit) & mask);
	if(debugLevel>=TMC_DEBUG_REGISTERS)
		LOGF_DEBUG("    old %08x value %08x firstBit %d numBits %d mask %08x new %08x", tmp, value, firstBit, numBits, mask, tmp2);
	return setRegister(address, tmp2);
}


bool TMC5160::getRegister(uint8_t address, uint32_t *result) {
	// use driver-side cache for write-only registers, fail for undefined registers
	if(!canReadRegister(address)) {
		if(!canWriteRegister(address)) {
			const int bufsize=1023;
			char buffer[bufsize+1]={0};
			printRegister(buffer, bufsize, address, 0, deviceStatus, "get", "error register is undefined");
			LOG_ERROR(buffer);
			return false;
		}   
		*result=cachedRegisterValues[address & (TMCR_NUM_REGISTERS-1)];
		if(debugLevel>=TMC_DEBUG_REGISTERS) {
			const int bufsize=1023;
			char buffer[bufsize+1]={0};
			printRegister(buffer, bufsize, address, *result, deviceStatus, "get", "cached");
			LOG_DEBUG(buffer);
		}
		return true;
	}

	uint8_t tx[5]={(uint8_t) (address & (TMCR_NUM_REGISTERS-1)),0,0,0,0};
	uint8_t rx[5];

	// Per the datasheet, raw send/receive returns the value requested with the PREVIOUS transfer.
	// As SPI is not performance critical for our application, we simply send read requests twice. 
	for(int i=0; i<2; i++)
		if(!sendReceive(tx,rx,5)) {
			const int bufsize=1023;
			char buffer[bufsize+1]={0};
			printRegister(buffer, bufsize, address, *result, rx[0], "get", "error");
			LOG_ERROR(buffer);
			return false;
		}

	deviceStatus=(enum TMCStatusFlags) rx[0];

	*result=(((uint32_t) rx[1])<<24) | (((uint32_t) rx[2])<<16) | 
	        (((uint32_t) rx[3])<<8)  |  ((uint32_t) rx[4]); 
	if(debugLevel>=TMC_DEBUG_REGISTERS) {
		const int bufsize=1023;
		char buffer[bufsize+1]={0};
		printRegister(buffer, bufsize, address, *result, rx[0], "get", NULL);
		LOG_DEBUG(buffer);
	}

	return true;
}


bool TMC5160::setRegister(uint8_t address, uint32_t value) {
	if(!canWriteRegister(address)) {
		const int bufsize=1023;
		char buffer[bufsize+1]={0};
		printRegister(buffer, bufsize, address, value, 0, "SET", "error register not writeable");
		LOG_ERROR(buffer);
		return false;
	}

	uint8_t tx[5]={(uint8_t) (address | 0x0080), 
		           (uint8_t) ((value>>24)&0x00ff), (uint8_t) ((value>>16)&0x00ff), 
		           (uint8_t) ((value>>8)&0x00ff),  (uint8_t) ((value>>0)&0x00ff)   };
	uint8_t rx[5];
	if(!sendReceive(tx,rx,5)) {
		if(debugLevel>=TMC_DEBUG_REGISTERS) {
			const int bufsize=1023;
			char buffer[bufsize+1]={0};
			printRegister(buffer, bufsize, address, value, rx[0], "SET", "error");
			LOG_DEBUG(buffer);
		}
		return false;
	}

	// Per the datasheet, raw send/receive returns the value requested with the PREVIOUS transfer.
	// As SPI is not performance critical for our application, we simply send a dummy read request.
	// Returned data must be identical to the originally set data 
	uint8_t tx2[5]={0,0,0,0,0};
	if(!sendReceive(tx2,rx,5) || rx[1]!=tx[1] || rx[2]!=tx[2] || rx[3]!=tx[3] || rx[4]!=tx[4]) {
		if(debugLevel>=TMC_DEBUG_REGISTERS) {
			const int bufsize=1023;
			char buffer[bufsize+1]={0};
			printRegister(buffer, bufsize, address, value, rx[0], "get", "error");
			LOG_DEBUG(buffer);
		}
		return false;
	}

	deviceStatus=(enum TMCStatusFlags) rx[0];
	cachedRegisterValues[address & (TMCR_NUM_REGISTERS-1)]=value;

	if(debugLevel>=TMC_DEBUG_REGISTERS) {
		const int bufsize=1023;
		char buffer[bufsize+1]={0};
		printRegister(buffer, bufsize, address, value, rx[0], "SET", NULL);
		LOG_DEBUG(buffer);
	}
	return true;
}


bool TMC5160::sendReceive(const uint8_t *tx, uint8_t *rx, uint32_t len) {
	const int bufsize=1023;
	char buffer[bufsize+1]={0};
	int bufpos=0;
	int remaining=bufsize-bufpos;

	if(debugLevel>=TMC_DEBUG_PACKETS) {
		int numPrinted=printPacket(buffer + bufpos, remaining, tx, len, true, "TX", NULL);
		bufpos+=numPrinted;
		remaining-=numPrinted;
		numPrinted=snprintf(buffer + bufpos, remaining, "  ");
		bufpos+=numPrinted;
		remaining-=numPrinted;
	}

	bool res=SPI::sendReceive(tx, rx, len);

	if(debugLevel>=TMC_DEBUG_PACKETS) {
		int numPrinted=printPacket(buffer + bufpos, remaining, rx, len, false, "RX", NULL);
		bufpos+=numPrinted;
		remaining-=numPrinted;
		numPrinted=snprintf(buffer + bufpos, remaining, "   Return %s", res?"true":"false");
		bufpos+=numPrinted;
		remaining-=numPrinted;
		LOG_DEBUG(buffer);
	}

	return res;
}


int TMC5160::printRegister(char *buffer, int bufsize, uint8_t address, uint32_t value, uint8_t status, const char *prefix, const char *suffix) {
	int bufpos=0;
	int remaining=bufsize-bufpos;

	const char *regName=getRegisterName(address);
	if(prefix!=NULL) {
		int numPrinted=snprintf(buffer + bufpos, remaining, "%s ", prefix);
		bufpos+=numPrinted;
		remaining-=numPrinted;
	}
	int numPrinted=snprintf(buffer + bufpos, remaining, "'%-14s'@0x%04x = %'+14d (0x%08x) ", regName, address, value, value);
	bufpos+=numPrinted;
	remaining-=numPrinted;

	numPrinted=printStatus(buffer + bufpos, remaining, (enum TMCStatusFlags) status);
	bufpos+=numPrinted;
	remaining-=numPrinted;

	if(suffix!=NULL) {
		numPrinted=snprintf(buffer + bufpos, remaining, " %s", suffix);
		bufpos+=numPrinted;
		remaining-=numPrinted;
	}
	return bufpos;
}


int TMC5160::printPacket(char *buffer, int bufsize, const uint8_t *data, uint32_t numBytes, bool isTX, const char *prefix, const char *suffix) {
	int bufpos=0;
	int remaining=bufsize-bufpos;

	if(data==NULL || numBytes==0)
		return false;
	if(prefix!=NULL) {
		int numPrinted=snprintf(buffer + bufpos, remaining, "%s ", prefix);
		bufpos+=numPrinted;
		remaining-=numPrinted;
	}

	if(isTX) {
		const char *opName=data[0]<0x0080 ? "get" : "SET";
		const char *regName=getRegisterName(data[0]);
		int numPrinted=snprintf(buffer + bufpos, remaining, "%s '%-14s'", opName, regName);
		bufpos+=numPrinted;
		remaining-=numPrinted;
	} else {
		int numPrinted=printStatus(buffer + bufpos, remaining, (enum TMCStatusFlags) data[0]);
		bufpos+=numPrinted;
		remaining-=numPrinted;
	}

	for(uint32_t i=0; i<numBytes; i++) {
		int numPrinted=snprintf(buffer + bufpos, remaining, " %02X",data[i]);
		bufpos+=numPrinted;
		remaining-=numPrinted;
	}

	if(suffix!=NULL) {
		int numPrinted=snprintf(buffer + bufpos, remaining, " %s", suffix);
		bufpos+=numPrinted;
		remaining-=numPrinted;
	}

	return bufpos;
}


int TMC5160::printStatus(char *buffer, int bufsize, enum TMCStatusFlags status) {
	int bufpos=0;
	int remaining=bufsize-bufpos;

	const char *separator="";
	int numPrinted=snprintf(buffer+bufpos, remaining, "[");
	bufpos+=numPrinted;
	remaining-=numPrinted;
	for(uint32_t i=0; i<8; i++)
		if( ((uint8_t)status) & (1u<<i) ) {
			numPrinted=snprintf(buffer + bufpos, remaining, "%s%s", separator, statusFlagNames[i]);
			bufpos+=numPrinted;
			remaining-=numPrinted;
			separator=" ";
		}
	numPrinted=snprintf(buffer + bufpos, remaining, "]");	
	bufpos+=numPrinted;
	remaining-=numPrinted;
	return bufpos;
}
