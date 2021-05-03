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


void PimocoMount::equatorialFromDevice(double *equRA, double *equDec, TelescopePierSide *equPS, double deviceHA, double deviceDec, double lst) {
    double equHA;
    if(abs(deviceDec)<=90) {
        *equPS =PIER_EAST; // normal pointing state, east pointing west
        equHA  =rangeHA(deviceHA);
        *equDec=rangeDec(deviceDec);
    } else {
        *equPS =PIER_WEST; // above the pole pointing state, west pointing east
        equHA  =rangeHA(deviceHA+12);
        *equDec=rangeDec(180.0-deviceDec);
    }

    if(isnan(lst))
        lst=getLocalSiderealTime();
    *equRA=range24(lst - equHA);
}


void PimocoMount::deviceFromEquatorial(double *deviceHA, double *deviceDec, double equRA, double equDec, TelescopePierSide equPS, double lst) {
    if(isnan(lst))
        lst=getLocalSiderealTime();
    double equHA=range24(lst - equRA);

    if(equPS==PIER_EAST || equPS==PIER_UNKNOWN) {
        // normal pointing state, east pointing west
        *deviceHA =rangeHA(equHA);
        *deviceDec=rangeDec(equDec);
    } else {
        // above the pole pointing state, west pointing east
        *deviceHA =rangeHA(equHA-12);
        *deviceDec=180.0-rangeDec(equDec);
    }
}


bool PimocoMount::checkLimitsPosAlt(double equRA, double equDec) {
    // convert equatorial position to horizon coordinates
   	struct ln_equ_posn equ_t0={equRA, equDec};
   	double jd=ln_get_julian_from_sys();
   	struct ln_hrz_posn hrz_t0;
   	ln_get_hrz_from_equ(&equ_t0, &lnobserver, jd, &hrz_t0);

    // check alt limits
   	bool inside_t0=(hrz_t0.alt>=AltLimitsN[0].value) && (hrz_t0.alt<=AltLimitsN[1].value);
	if(stepperHA.getDebugLevel()>=Stepper::TMC_DEBUG_DEBUG)
	   	LOGF_DEBUG("RA %f Dec %f Az %f Alt %f inside %d", equ_t0.ra, equ_t0.dec, hrz_t0.az, hrz_t0.alt, inside_t0);

	return inside_t0;
}


bool PimocoMount::checkLimitsPosHA(double deviceHA, double deviceDec) {
    return (deviceHA>=HALimitsN[0].value) && (deviceHA<=HALimitsN[1].value);
}


bool PimocoMount::checkLimitsPosSpeed(double equRA, double equDec, TelescopePierSide equPS, double haArcsecPerSec, double decArcsecPerSec, bool log) {
    // convert equatorial position to horizon coordinates
   	struct ln_equ_posn equ_t0={equRA, equDec};
   	double jd=ln_get_julian_from_sys();
   	struct ln_hrz_posn hrz_t0;
   	ln_get_hrz_from_equ(&equ_t0, &lnobserver, jd, &hrz_t0);

    // check alt limits
   	bool inside_t0=(hrz_t0.alt>=AltLimitsN[0].value) && (hrz_t0.alt<=AltLimitsN[1].value);
	if(inside_t0) {
		if(log)
		   	LOGF_INFO("RA %f Dec %f Az %f Alt %f inside %d", equ_t0.ra, equ_t0.dec, hrz_t0.az, hrz_t0.alt, inside_t0);
	} else {
    	// we're outside, check next pos in 1 second
    	struct ln_equ_posn equ_t1={equ_t0.ra-haArcsecPerSec/(60.0*60.0), equ_t0.dec+decArcsecPerSec/(60.0*60.0)}; // decSpeed=-haSpeed
       	struct ln_hrz_posn hrz_t1;
       	ln_get_hrz_from_equ(&equ_t1, &lnobserver, jd, &hrz_t1);

       	// will motion get us at least 0.1 arcsec closer to a compliant state?
       	bool right_dir_t1=((hrz_t0.alt < AltLimitsN[0].value) && (hrz_t1.alt > hrz_t0.alt + 0.1/(60.0*60.0))) || 
            	          ((hrz_t0.alt > AltLimitsN[1].value) && (hrz_t1.alt < hrz_t0.alt - 0.1/(60.0*60.0)))    ;

    	if(log)
    	   	LOGF_INFO("RA %f Dec %f Az %f Alt %f inside %d right_dir %d", equ_t0.ra, equ_t0.dec, hrz_t0.az, hrz_t0.alt, inside_t0, right_dir_t1);
       if(!right_dir_t1)
            return false;
    }

    // convert to device coordinates
    double deviceHA, deviceDec;
    deviceFromEquatorial(&deviceHA, &deviceDec, equRA, equDec, equPS);

    // check HA limits
    if(!checkLimitsPosHA(deviceHA, deviceDec)) {
        return false;
    }

   return true;
}


bool PimocoMount::applyLimitsPosSpeed(double haArcsecPerSec, double decArcsecPerSec, bool log) {
	if(!checkLimitsPosSpeed(EqN[0].value, EqN[1].value, getPierSide(), haArcsecPerSec, decArcsecPerSec, log)) {
		LOG_WARN("Mount limits reached");
		Abort();
		return false;
	}
	return true;
}