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


#include "pimoco_mount.h"
#include <libindi/indilogger.h>
#include <libindi/indicom.h>  // for rangeHA etc.
#include <libnova/julian_day.h>
#include <libnova/sidereal_time.h>
#include <libnova/transform.h>
#include <time.h>

#define CDRIVER_VERSION_MAJOR	1
#define CDRIVER_VERSION_MINOR	0


// Singleton instance
PimocoMount mount;

const double PimocoMount::trackRates[]={
    15.041067, // TRACK_SIDEREAL
    15.0,      // TRACK_SOLAR
    14.685,    // TRACK_LUNAR
    15.041067, // TRACK_CUSTOM
    15.0369,   // King tracking rate, not defined in INDI standard
};

const char *PimocoMount::trackRateNames[]={
    "TRACK_SIDEREAL", // TRACK_SIDEREAL
    "TRACK_SOLAR",    // TRACK_SOLAR
    "TRACK_LUNAR",    // TRACK_LUNAR
    "TRACK_CUSTOM",   // TRACK_CUSTOM
    "TRACK_KING",     // King tracking rate, not defined in INDI standard
};

const char *PimocoMount::trackRateLabels[]={
    "Sidereal", // TRACK_SIDEREAL
    "Solar",    // TRACK_SOLAR
    "Lunar",    // TRACK_LUNAR
    "Custom",   // TRACK_CUSTOM
    "King",     // King tracking rate, not defined in INDI standard
};

const char *PimocoMount::HA_TAB="Hour angle";
const char *PimocoMount::DEC_TAB="Declination";


// C function interface redirecting to singleton
//
extern "C" {

void ISGetProperties(const char *dev) {
	mount.ISGetProperties(dev);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n) {
	mount.ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
	mount.ISNewNumber(dev, name, values, names, n);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
	mount.ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) {
	mount.ISNewText(dev, name, texts, names, n);
}

void ISSnoopDevice(XMLEle *root) {
	mount.ISSnoopDevice(root);
}

} // extern "C"


// Public class members
//

PimocoMount::PimocoMount() : stepperHA(getDeviceName(), HA_DIAG0_PIN), stepperDec(getDeviceName(), DEC_DIAG0_PIN),
    spiDeviceFilenameHA("/dev/spidev1.0"), spiDeviceFilenameDec("/dev/spidev1.1") {
	setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);

	SetTelescopeCapability(
       		TELESCOPE_CAN_GOTO |
            TELESCOPE_CAN_SYNC |
            TELESCOPE_CAN_PARK |
            TELESCOPE_CAN_ABORT |
            TELESCOPE_HAS_TIME |
            TELESCOPE_HAS_LOCATION |
            TELESCOPE_HAS_PIER_SIDE |
            // TELESCOPE_HAS_PEC |
            TELESCOPE_HAS_TRACK_MODE |
            TELESCOPE_CAN_CONTROL_TRACK |
            TELESCOPE_HAS_TRACK_RATE |
            //TELESCOPE_HAS_PIER_SIDE_SIMULATION |
            //TELESCOPE_CAN_TRACK_SATELLITE |
            0, NUM_SLEW_RATES);

	setTelescopeConnection(CONNECTION_NONE);

    SetParkDataType(PARK_HA_DEC);
}

PimocoMount::~PimocoMount() {
	stepperHA.stop();
	stepperDec.stop();
}

const char *PimocoMount::getDefaultName() {
	return "Pimoco mount";
}


// Protected class members
//

bool PimocoMount::Connect() {
	LOGF_INFO("Attempting connection to HA on %s", spiDeviceFilenameHA);
	if(!stepperHA.open(spiDeviceFilenameHA)) {
		LOGF_WARN("Connection to HA on %s failed", spiDeviceFilenameHA);
		return false;
	}
	LOGF_INFO("Connection to HA on %s successful", spiDeviceFilenameHA);

	LOGF_INFO("Attempting connection to Dec on %s", spiDeviceFilenameDec);
	if(!stepperDec.open(spiDeviceFilenameDec)) {
		LOGF_WARN("Connection to Dec on %s failed", spiDeviceFilenameDec);
		return false;
	}
	if(!ReadScopeStatus())
		return false;
	LOGF_INFO("Connection to Dec on %s successful", spiDeviceFilenameDec);

	// Restore park status. Must be performed after connection
	if(isParked())
		SyncDeviceHADec(GetAxis1Park(), GetAxis2Park());

	uint32_t pp=getPollingPeriod();
	if (pp > 0)
		SetTimer(pp);

	return true;
}

bool PimocoMount::Disconnect() {
	if(!stepperHA.close() || !stepperDec.close()) {
		LOG_WARN("Error closing connection");
		return false;
	}
	LOG_INFO("Successfully closed connection");
	return true;
}

bool PimocoMount::Handshake() {
	return true;
}

double PimocoMount::getLocalSiderealTime(double julianDay) {
   	double gast=ln_get_apparent_sidereal_time(julianDay);
 	double observerLongitudeDegreesEastPositive=LocationN[LOCATION_LONGITUDE].value;
 	double last=gast + observerLongitudeDegreesEastPositive/15.0;
 	return last;	
}

double PimocoMount::getLocalSiderealTime() {
   	double julianDay=ln_get_julian_from_sys();
   	return getLocalSiderealTime(julianDay);
}

uint64_t PimocoMount::getTimeMillis() {
	struct timespec now;
	clock_gettime(CLOCK_REALTIME, &now);

	return ((uint64_t)now.tv_sec)*1000 + ((uint64_t)now.tv_nsec)/1000000;
}

double PimocoMount::rangeDecNative(double r) {
    double res = r;
    while (res < -180.0)
        res += 360.0;
    while (res >= 180.0)
        res -= 360.0;
    return res;
}
