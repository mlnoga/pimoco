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


void PimocoMount::horizonFromEquatorial(double *horAlt, double *horAz, double equRA, double equDec, double jd) {
    struct ln_equ_posn equ_t0={equRA*(360.0/24.0), equDec};
    if(jd<0)
        jd=ln_get_julian_from_sys();
    struct ln_hrz_posn hor;

    get_hrz_from_equ(&equ_t0, &lnobserver, jd, &hor);
    *horAlt=hor.alt;
    *horAz =hor.az;
}


bool PimocoMount::checkLimitsHA(double deviceHA) {
    bool inside=(deviceHA>=HALimitsN[0].value) && (deviceHA<=HALimitsN[1].value);
    if(!inside)
        LOGF_ERROR("Device HA %f outside limits [%f, %f]", deviceHA, HALimitsN[0].value, HALimitsN[1].value);
    return inside;
}

bool PimocoMount::checkLimitsHA(double deviceHA, double arcsecPerSecHA) {
    bool inside=(deviceHA>=HALimitsN[0].value) && (deviceHA<=HALimitsN[1].value);
    if(!inside)
        inside= (deviceHA<HALimitsN[0].value && arcsecPerSecHA>0) ||
                (deviceHA>HALimitsN[1].value && arcsecPerSecHA<0)    ;
    if(!inside)
        LOGF_ERROR("Device HA %f speed %f outside limits [%f, %f]", deviceHA, arcsecPerSecHA, HALimitsN[0].value, HALimitsN[1].value);
    return inside;
}

bool PimocoMount::checkLimitsAlt(double horAlt) {
    bool inside=(horAlt>=AltLimitsN[0].value) && (horAlt<=AltLimitsN[1].value);
    if(!inside)
        LOGF_ERROR("Altitude %f outside limits [%f, %f]", horAlt, AltLimitsN[0].value, AltLimitsN[1].value);
    return inside;
}

bool PimocoMount::checkLimitsAlt(double horAlt, double deviceHA, double deviceDec, double arcsecPerSecHA, double arcsecPerSecDec, double jd, double lst) {
    bool inside=(horAlt>=AltLimitsN[0].value) && (horAlt<=AltLimitsN[1].value);
    if(!inside) {
        // check where we will be in one second
        double deviceHA2 =deviceHA  + arcsecPerSecHA  /(15*60);
        double deviceDec2=deviceDec + arcsecPerSecDec /(60*60);
        double jd2       =jd        + 1.0/(24.0*60.0*60.0);
        double lst2      =lst       + 1.0/(     60.0*60.0);

        double equRA2, equDec2; // RA in hours, declination in degrees
        TelescopePierSide equPS2;
        equatorialFromDevice(&equRA2, &equDec2, &equPS2, deviceHA2, deviceDec2, lst2);

        double horAlt2, horAz2;
        horizonFromEquatorial(&horAlt2, &horAz2, equRA2, equDec2, jd2);

        // will motion get us at least 0.1 arcsec closer to a compliant state?
        inside=((horAlt < AltLimitsN[0].value) && (horAlt2 > horAlt + 0.1/(60.0*60.0))) || 
               ((horAlt > AltLimitsN[1].value) && (horAlt2 < horAlt - 0.1/(60.0*60.0)))    ;
    }
    if(!inside)
        LOGF_ERROR("Altitude %f outside limits [%f, %f]", horAlt, AltLimitsN[0].value, AltLimitsN[1].value);
    return inside;
}

bool PimocoMount::applyLimits(double arcsecPerSecHA, double arcsecPerSecDec) {
    if(!checkLimitsAlt(AltAzN[0].value, DeviceCoordN[0].value, DeviceCoordN[1].value, arcsecPerSecHA, arcsecPerSecDec, TimeN[0].value, TimeN[1].value) ||
       !checkLimitsHA(DeviceCoordN[0].value, arcsecPerSecHA) ) {
        Abort();
        return false;
    }
    return true;
}



