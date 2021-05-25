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
#include <libindi/indilogger.h> // for LOG_..., LOGF_... macros

#include "pimoco_spi.h"

const char    *SPI::defaultSPIDevice="/dev/spidev0.0";
const uint8_t  SPI::defaultSPIMode=SPI_MODE_3;
const uint8_t  SPI::defaultSPIBits=8;
const uint32_t SPI::defaultSPIMaxSpeedHz=4000000;
const uint32_t SPI::defaultSPIDelayUsec=0;


bool SPI::open(const char *deviceName) {
	if(fd>=0)
		close();

	if(debugLevel>=TMC_DEBUG_DEBUG)
		LOGF_DEBUG("Device %s: opening", deviceName!=NULL ? deviceName : "NULL");

	fd=::open(deviceName, O_RDWR);
	if(fd<0) {
		LOGF_ERROR("Device %s: opening: %s", deviceName!=NULL ? deviceName : "NULL", strerror(errno));
		return false;
	}

	// Set SPI mode, bits per word and speed
	//
	if(ioctl(fd, SPI_IOC_WR_MODE, &defaultSPIMode)<0) {
		LOGF_ERROR("Device %s: Setting SPI mode 0x%02x: %s", deviceName!=NULL ? deviceName : "NULL", defaultSPIMode, strerror(errno));
		return false;
	}
	if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &defaultSPIBits)<0) {
		LOGF_ERROR("Device %s: Setting SPI bits to %d: %s", deviceName!=NULL ? deviceName : "NULL", defaultSPIBits, strerror(errno));
		return false;
	}
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &defaultSPIMaxSpeedHz)<0) {
		LOGF_ERROR("Device %s: Setting SPI speed to %d: %s", deviceName!=NULL ? deviceName : "NULL", defaultSPIMaxSpeedHz, strerror(errno));
		return false;
	}

	return true;
}


bool SPI::close() {
	if(fd>=0) {
		if(debugLevel>=TMC_DEBUG_DEBUG)
			LOGF_DEBUG("Shutting down existing device with file descriptor %d", fd);
		::close(fd);
		fd=-1;
	}
	return true;
}


bool SPI::sendReceive(const uint8_t *tx, uint8_t *rx, uint32_t len) {
	if((len==0) || ((len%5)!=0)) {
		LOGF_ERROR("SPI send/receive: length %d not a nonzero multiple of 5", len);
		return false;		
	}

	const uint32_t numTransfers=len/5;
	struct spi_ioc_transfer tr[numTransfers];
	for(uint32_t t=0; t<numTransfers; t++)
		tr[t] = {
			.tx_buf        = (unsigned long) &tx[5*t],
			.rx_buf        = (unsigned long) &rx[5*t],
			.len           = 5,
			.speed_hz      = defaultSPIMaxSpeedHz,
			.delay_usecs   = defaultSPIDelayUsec,
			.bits_per_word = defaultSPIBits,
			.cs_change     = ((t==(numTransfers-1)) ? (uint8_t) 0 : (uint8_t) 0xff),
			.pad           = 0,
		};
	int res=ioctl(fd, SPI_IOC_MESSAGE(numTransfers), tr);

	if(res<0) 
		LOGF_ERROR("SPI send/receive: %s", strerror(errno));
	return res>=0;
}
