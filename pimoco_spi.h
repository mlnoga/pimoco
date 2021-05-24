
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

#ifndef PIMOCO_SPI_H
#define PIMOCO_SPI_H

#include <stdint.h>

// A SPI device
class SPI {
public:
	// Driver debugging level
	enum DriverDebugLevel : int {
		TMC_DEBUG_ERROR     = 0,
		TMC_DEBUG_WARNING   = 1,
		TMC_DEBUG_INFO      = 2,
		TMC_DEBUG_DEBUG     = 3,
		TMC_DEBUG_REGISTERS = 4,
		TMC_DEBUG_PACKETS   = 5,
	};

	// Creates device connected via SPI
	SPI(const char *theIndiDeviceName) : fd(-1), debugLevel(TMC_DEBUG_DEBUG), indiDeviceName(theIndiDeviceName) { }

	// Destroys this device connected via SPI
	~SPI() { close(); }

	// Opens and initializes an SPI device. Returns true on success, else false
	bool open(const char *device);

	// Closes a device. Returns true on success, else false
	bool close();

	// Returns true if the device is connected, else false
	bool isConnected() const { return fd>=0; }

	// Sends the given number of bytes to the device, then retrieves the same number of bytes.  Number of bytes must be dividable by 5.
	// Returns true on success, else false
	virtual bool sendReceive(const uint8_t *tx, uint8_t *rx, uint32_t numBytes);

	// Gets driver debugging level	
	enum DriverDebugLevel getDebugLevel() const { return debugLevel; }

	// Sets driver debugging level
	void setDebugLevel(enum DriverDebugLevel value) { debugLevel=value; }

	// Get Indi device name. Used by logging macros
	const char *getDeviceName() const { return indiDeviceName; }


protected:
	// File descriptor for the SPI device
	int fd;

	// Debug level
	enum DriverDebugLevel debugLevel;

	// INDI device name. Used by logging macros
	const char *indiDeviceName;

public:
	// Default SPI device
	static const char *defaultSPIDevice;

protected:
	// Default SPI mode settings
	static const uint8_t defaultSPIMode;

	// Default SPI bit settings
	static const uint8_t defaultSPIBits;

	// Default SPI maximum speed in Hertz
	static const uint32_t defaultSPIMaxSpeedHz;

	// Default SPI delay in microseconds
	static const uint32_t defaultSPIDelayUsec;
};

#endif // PIMOCO_SPI_H