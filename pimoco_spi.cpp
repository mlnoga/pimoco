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

#include "pimoco_spi.h"

const char    *SPI::defaultSPIDevice="/dev/spidev1.0";
const uint8_t  SPI::defaultSPIMode=SPI_MODE_0;
const uint8_t  SPI::defaultSPIBits=8;
const uint32_t SPI::defaultSPIMaxSpeedHz=500000;
const uint32_t SPI::defaultSPIDelayUsec=0;


bool SPI::open(const char *deviceName) {
	if(fd>=0)
		close();

	if(debugLevel>=TMC_DEBUG_ACTIONS) 
		fprintf(debugFile, "Opening device %s\n", deviceName!=NULL ? deviceName : "NULL");

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

	return true;
}


bool SPI::close() {
	if(fd>=0) {
		if(debugLevel>=TMC_DEBUG_ACTIONS) 
			fprintf(debugFile, "Shutting down existing device with file descriptor %d\n", fd);
		::close(fd);
		fd=-1;
	}
	return true;
}


bool SPI::sendReceive(const uint8_t *tx, uint8_t *rx, uint32_t len) {
	struct spi_ioc_transfer tr = {
		.tx_buf        = (unsigned long) tx,
		.rx_buf        = (unsigned long) rx,
		.len           = len,
		.speed_hz      = defaultSPIMaxSpeedHz,
		.delay_usecs   = defaultSPIDelayUsec,
		.bits_per_word = defaultSPIBits,
	};

	return ioctl(fd, SPI_IOC_MESSAGE(1), &tr)>=0;
}
