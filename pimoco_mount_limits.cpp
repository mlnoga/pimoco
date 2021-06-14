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


void PimocoMount::equatorialFromDevice(double *equRA, double *equDec, TelescopePierSide *equPS, double deviceHA, double deviceDec, double lst) {
    double equHA;

    // Conversion to equatorial coordinates per the ASCOM definition. However, note that
    // ASCOM encodes pointing state (normal/beyond the pole) in the side of pier field.
    // See https://ascom-standards.org/Help/Platform/html/P_ASCOM_DeviceInterface_ITelescopeV3_SideOfPier.htm
    // Indi needs physical sie of pier reporting (scope is east/west of pier), so we
    // override the ASCOM definition of the field.

    if(abs(deviceDec)<=90) {
        // normal pointing state, east pointing west
        //*equPS =PIER_EAST; // ignore ASCOM definition
        equHA  =rangeHA(deviceHA);
        *equDec=rangeDec(deviceDec);
    } else {
        // above the pole pointing state, west pointing east
        //*equPS =PIER_WEST; // ignore ASCOM definition
        equHA  =rangeHA(deviceHA+12);
        *equDec=rangeDec(180.0-deviceDec);
    }

    // set side of pier based on physical telescope position
    const auto dHA=rangeHA(deviceHA);
    *equPS = (dHA>-6 && dHA<6) ? PIER_WEST : PIER_EAST;

    if(lst<0)
        lst=getLocalSiderealTime();
    *equRA=range24(lst - equHA);

    if(stepperHA.getDebugLevel()>=Stepper::TMC_DEBUG_DEBUG)
        LOGF_DEBUG("eqFromDev: device HA %f Dec %f >> equ HA %f RA %f Dec %f pier %d %s @ lst %f", 
                   deviceHA, deviceDec, equHA, *equRA, *equDec, *equPS, getPierSideStr(*equPS), lst);
}


bool PimocoMount::deviceFromEquatorial(double *deviceHA, double *deviceDec, double equRA, double equDec, TelescopePierSide equPS, double lst) {
    if(lst<0)
        lst=getLocalSiderealTime();
    double equHA=rangeHA(lst - equRA);

    const auto impliedPS=(equHA>-6 && equHA<6) ? PIER_WEST : PIER_EAST;
    if(impliedPS==equPS) {
    //if(equPS==PIER_EAST || equPS==PIER_UNKNOWN) {
        // normal pointing state, east pointing west
        *deviceHA =rangeHA(equHA);
        *deviceDec=rangeDec(equDec);
    } else {
        // above the pole pointing state, west pointing east
        *deviceHA =rangeHA(equHA-12);
        *deviceDec=180.0-rangeDec(equDec);
    }

    // bring position into mount limits, if possible
    while(*deviceHA<HALimitsN[0].value)
        (*deviceHA)+=24;
    while(*deviceHA>HALimitsN[1].value)
        (*deviceHA)-=24;
    bool valid=((*deviceHA)>=HALimitsN[0].value) && ((*deviceHA)<=HALimitsN[1].value);
    if(!valid)

    if(stepperHA.getDebugLevel()>=Stepper::TMC_DEBUG_DEBUG) 
        LOGF_DEBUG("devFromEq: device HA %f Dec %f from equ HA %f RA %f Dec %f pier %d %s @ lst %f", 
                  *deviceHA, *deviceDec, equHA, equRA, equDec, equPS, getPierSideStr(equPS), lst);

    return valid;
}


bool PimocoMount::checkLimitsPosAlt(double equRA, double equDec) {
    // convert equatorial position to horizon coordinates
    // Note that unlike Indi, which uses hours for RA, libnova expects RA in degrees
   	struct ln_equ_posn equ_t0={equRA*(360.0/24.0), equDec};
   	double jd=ln_get_julian_from_sys();
   	struct ln_hrz_posn hrz_t0;
   	
    get_hrz_from_equ(&equ_t0, &lnobserver, jd, &hrz_t0);

    // check alt limits
   	bool inside_t0=(hrz_t0.alt>=AltLimitsN[0].value) && (hrz_t0.alt<=AltLimitsN[1].value);
	if(stepperHA.getDebugLevel()>=Stepper::TMC_DEBUG_DEBUG) {
        if(!inside_t0) {
            LOGF_INFO("RA %f Dec %f Az %f Alt %f inside %d", equ_t0.ra, equ_t0.dec, hrz_t0.az, hrz_t0.alt, inside_t0);
            LOGF_INFO("JD %f lat %f lon %f", jd, lnobserver.lat, lnobserver.lng);
        }
        else
            LOGF_DEBUG("RA %f Dec %f Az %f Alt %f inside %d", equ_t0.ra, equ_t0.dec, hrz_t0.az, hrz_t0.alt, inside_t0);
    }

	return inside_t0;
}


bool PimocoMount::checkLimitsPosSpeed(double equRA, double equDec, TelescopePierSide equPS, double haArcsecPerSec, double decArcsecPerSec, bool log) {
    // convert equatorial position to horizon coordinates
    // Note that unlike Indi, which uses hours for RA, libnova expects RA in degrees
    struct ln_equ_posn equ_t0={equRA*(360.0/24.0), equDec};
   	double jd=ln_get_julian_from_sys();
   	struct ln_hrz_posn hrz_t0;
   	get_hrz_from_equ(&equ_t0, &lnobserver, jd, &hrz_t0);

    // check alt limits
   	bool inside_t0=(hrz_t0.alt>=AltLimitsN[0].value) && (hrz_t0.alt<=AltLimitsN[1].value);
	if(inside_t0) {
        if(stepperHA.getDebugLevel()>=Stepper::TMC_DEBUG_DEBUG)
		   	LOGF_DEBUG("RA %f Dec %f Az %f Alt %f inside %d", equ_t0.ra, equ_t0.dec, hrz_t0.az, hrz_t0.alt, inside_t0);
	} else {
    	// we're outside, check next pos in 1 second
    	struct ln_equ_posn equ_t1={equ_t0.ra-haArcsecPerSec/(60.0*60.0), equ_t0.dec+decArcsecPerSec/(60.0*60.0)}; // raSpeed=-haSpeed
       	struct ln_hrz_posn hrz_t1;
       	get_hrz_from_equ(&equ_t1, &lnobserver, jd+1.0/(24.0*60.0*60.0), &hrz_t1);

       	// will motion get us at least 0.1 arcsec closer to a compliant state?
       	bool right_dir_t1=((hrz_t0.alt < AltLimitsN[0].value) && (hrz_t1.alt > hrz_t0.alt + 0.1/(60.0*60.0))) || 
            	          ((hrz_t0.alt > AltLimitsN[1].value) && (hrz_t1.alt < hrz_t0.alt - 0.1/(60.0*60.0)))    ;

        if(stepperHA.getDebugLevel()>=Stepper::TMC_DEBUG_DEBUG)
    	   	LOGF_DEBUG("RA %f Dec %f Az %f Alt %f inside %d right_dir %d", equ_t0.ra, equ_t0.dec, hrz_t0.az, hrz_t0.alt, inside_t0, right_dir_t1);
        if(!right_dir_t1)
            return false;
    }

    // convert to device coordinates
    double deviceHA, deviceDec;
    bool valid=deviceFromEquatorial(&deviceHA, &deviceDec, equRA, equDec, equPS);

    if(stepperHA.getDebugLevel()>=Stepper::TMC_DEBUG_DEBUG)
        LOGF_DEBUG("Device HA %f Dec %f HA limits [%f, %f] insideHA %d", deviceHA, deviceDec, HALimitsN[0].value, HALimitsN[1].value, valid);

    if(!valid)
        return false;

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