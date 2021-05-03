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


bool PimocoMount::initProperties() {
	if(!INDI::Telescope::initProperties()) 
		return false;
	// Make pier side writable to allow triggering of meridian flips (INDI::Telescope has readonly)
    IUFillSwitchVector(&PierSideSP, PierSideS, 2, getDeviceName(), "TELESCOPE_PIER_SIDE", "Pier Side", MAIN_CONTROL_TAB,
                       IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

 	initGuiderProperties(getDeviceName(), GUIDE_TAB); // returns void, cannot fail

	// Create the four standard tracking modes
	AddTrackMode(trackRateNames[TRACK_SIDEREAL], trackRateLabels[TRACK_SIDEREAL], true);
	AddTrackMode(trackRateNames[TRACK_SOLAR],    trackRateLabels[TRACK_SOLAR],    false);
	AddTrackMode(trackRateNames[TRACK_LUNAR],    trackRateLabels[TRACK_LUNAR],    false);
	AddTrackMode(trackRateNames[TRACK_CUSTOM],   trackRateLabels[TRACK_CUSTOM],   false);

	// Initialize stepper properties
	stepperHA .initProperties( HAMotorN, & HAMotorNP,  HAMSwitchS, & HAMSwitchSP, HARampN, & HARampNP,  
							  "HA_MOTOR", "Motor", "HA_MSWITCH", "Switches", "HA_RAMP", "Ramp", HA_TAB);
	stepperDec.initProperties(DecMotorN, &DecMotorNP, DecMSwitchS, &DecMSwitchSP, DecRampN, &DecRampNP, 
							  "DEC_MOTOR", "Motor", "DEC_MSWITCH", "Switches", "DEC_RAMP", "Ramp", DEC_TAB);

	// Initialize mount properties
	IUFillNumber(&SlewRatesN[0], SlewRateS[0].name, SlewRateS[0].label, "%.1f", 0, 1600, 16, 0.5);
	IUFillNumber(&SlewRatesN[1], SlewRateS[1].name, SlewRateS[1].label, "%.1f", 0, 1600, 16, 16);
	IUFillNumber(&SlewRatesN[2], SlewRateS[2].name, SlewRateS[2].label, "%.1f", 0, 1600, 16, 250);
	IUFillNumber(&SlewRatesN[3], SlewRateS[3].name, SlewRateS[3].label, "%.1f", 0, 1600, 16, 1000);
	IUFillNumberVector(&SlewRatesNP, SlewRatesN, NUM_SLEW_RATES, getDeviceName(), "SLEW_RATES", "Slew rates [x sidereal]", MOTION_TAB, IP_RW, 0, IPS_IDLE);

	IUFillNumber(&HALimitsN[0], "MIN", "Min [hh:mm:ss]", "%010.6m", -12, 12, 0.25, -6.5);
	IUFillNumber(&HALimitsN[1], "MAX", "Max [hh:mm:ss]", "%010.6m", -12, 12, 0.25,  6.5);
	IUFillNumberVector(&HALimitsNP, HALimitsN, 2, getDeviceName(), "HA_LIMITS", "Hour angle limits", MOTION_TAB, IP_RW, 0, IPS_IDLE);

	IUFillNumber(&AltLimitsN[0], "MIN", "Min [dd:mm:ss]", "%010.6m", -5, 90, 1,  0);
	IUFillNumber(&AltLimitsN[1], "MAX", "Max [dd:mm:ss]", "%010.6m", -5, 90, 1, 90);
	IUFillNumberVector(&AltLimitsNP, AltLimitsN, 2, getDeviceName(), "ALT_LIMITS", "Altitude limits", MOTION_TAB, IP_RW, 0, IPS_IDLE);

	IUFillSwitch(&SyncToParkS[0], "SYNC_TO_PARK","Sync to park", ISS_OFF);
	IUFillSwitchVector(&SyncToParkSP, SyncToParkS, 1, getDeviceName(), "SYNC_TO_PARK", "Sync to park", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// Guider properties
	IUFillNumber(&GuiderSpeedN[0], "VALUE", "Rate [x sidereal]", "%.2f", 0, 1, 0.05, 0.75);
	IUFillNumberVector(&GuiderSpeedNP, GuiderSpeedN, 1, getDeviceName(), "GUIDER_SPEED", "Guider speed", GUIDE_TAB, IP_RW, 0, IPS_IDLE);

	IUFillNumber(&GuiderMaxPulseN[0], "VALUE", "Max [ms]", "%.f", 0, 10000, 100, 2500);
	IUFillNumberVector(&GuiderMaxPulseNP, GuiderMaxPulseN, 1, getDeviceName(), "GUIDER_MAX_PULSE", "Guider pulse", GUIDE_TAB, IP_RW, 0, IPS_IDLE);

	// load configuration data from file, as there is no device with own storage
	loadConfig(true, HAMotorNP.name);
	loadConfig(true, HAMSwitchSP.name);
	loadConfig(true, HARampNP.name);

	loadConfig(true, DecMotorNP.name);
	loadConfig(true, DecMSwitchSP.name);
	loadConfig(true, DecRampNP.name);

	loadConfig(true, SlewRatesNP.name);
	loadConfig(true, HALimitsNP.name);
	loadConfig(true, AltLimitsNP.name);

	loadConfig(true, GuiderSpeedNP.name);
	loadConfig(true, GuiderMaxPulseNP.name);

	// load park configuration data and status from file
	InitPark();

    addDebugControl();
 	setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);

    return true;
}

bool PimocoMount::updateProperties() {
	if(!INDI::Telescope::updateProperties())
		return false;

	if(!stepperHA .updateProperties(this,  HAMotorN, & HAMotorNP,  HAMSwitchS, & HAMSwitchSP,  HARampN, & HARampNP))
		return false;
	if(!stepperDec.updateProperties(this, DecMotorN, &DecMotorNP, DecMSwitchS, &DecMSwitchSP, DecRampN, &DecRampNP))
		return false;

	if(isConnected()) {
	    defineProperty(&SlewRatesNP);
	    defineProperty(&HALimitsNP);
	    defineProperty(&AltLimitsNP);
	    defineProperty(&SyncToParkSP);

	    defineProperty(&GuiderSpeedNP);
	    defineProperty(&GuiderMaxPulseNP);
        defineProperty(&GuideNSNP);
        defineProperty(&GuideWENP);

	} else {
	    deleteProperty(SlewRatesNP.name);
	    deleteProperty(HALimitsNP.name);
	    deleteProperty(AltLimitsNP.name);
	    deleteProperty(SyncToParkSP.name);

	    deleteProperty(GuiderSpeedNP.name);
	    deleteProperty(GuiderMaxPulseNP.name);
        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);
	}

	return true;
}

bool PimocoMount::ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
    	return INDI::Telescope::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);

    // if(strcmp(name, ...)) { } else

	return INDI::Telescope::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

bool PimocoMount::ISUpdateNumber(INumberVectorProperty *NP, double values[], char *names[], int n, bool res) 
{
	if(res) {
		IUUpdateNumber(NP, values, names, n);
		saveConfig(true, NP->name);
	}
	NP->s=res ? IPS_OK : IPS_ALERT;
	IDSetNumber(NP, NULL);
	return res;
}

bool PimocoMount::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
		return INDI::Telescope::ISNewNumber(dev, name, values, names, n);

	int res;
	if((res=stepperHA.ISNewNumber(&HAMotorNP, &HARampNP, name, values, names, n)) > 0) {
		saveConfig(true, HAMotorNP.name);
		saveConfig(true, HARampNP.name);
		return true;
	} else if(res==0)
		return false;

	if((res=stepperDec.ISNewNumber(&DecMotorNP, &DecRampNP, name, values, names, n)) > 0) {
		saveConfig(true, DecMotorNP.name);
		saveConfig(true, DecRampNP.name);
		return true;
	} else if(res==0)
		return false;
    
	if(!strcmp(name, SlewRatesNP.name)) {
        auto rc=ISUpdateNumber(&SlewRatesNP, values, names, n, true);
        if(rc)
	        saveConfig(true, SlewRatesNP.name);
        return rc;
	}

	if(!strcmp(name, HALimitsNP.name)) {
        auto rc=ISUpdateNumber(&HALimitsNP, values, names, n, true);
        if(rc)
	        saveConfig(true, HALimitsNP.name);
        return rc;
	}

	if(!strcmp(name, AltLimitsNP.name)) {
        auto rc=ISUpdateNumber(&AltLimitsNP, values, names, n, true);
        if(rc)
	        saveConfig(true, AltLimitsNP.name);
        return rc;
	}

	if(!strcmp(name, GuiderSpeedNP.name)) {
        auto rc=ISUpdateNumber(&GuiderSpeedNP, values, names, n, true);		
        if(rc)
	        saveConfig(true, GuiderSpeedNP.name);
        return rc;
	}

	if(!strcmp(name, GuiderMaxPulseNP.name)) {
        auto rc=ISUpdateNumber(&GuiderMaxPulseNP, values, names, n, true);		
        if(rc)
	        saveConfig(true, GuiderMaxPulseNP.name);
        return rc;
	}

	if(!strcmp(name, GuideNSNP.name) || !strcmp(name, GuideWENP.name)) {
		processGuiderProperties(name, values, names, n); // does not return a status or change persistent properties
		return true;
	}

	return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool PimocoMount::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
		return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);

	int res;
	if((res=stepperHA.ISNewSwitch(&HAMSwitchSP, name, states, names, n)) >0) {
		saveConfig(true, HAMSwitchSP.name);
		return true;
	} else if(res==0)
		return false;

	if((res=stepperDec.ISNewSwitch(&DecMSwitchSP, name, states, names, n)) >0) {
		saveConfig(true, DecMSwitchSP.name);
		return true;
	} else if(res==0)
		return false;

	if(!strcmp(name, SyncToParkSP.name))
		return SyncDeviceHADec(GetAxis1Park(), GetAxis2Park());

	if(!strcmp(name, PierSideSP.name)) {
		TelescopePierSide newPS= (PierSideS[PIER_WEST].s==ISS_ON) ? PIER_WEST : PIER_EAST;
		bool rc=Goto(EqN[0].value, EqN[1].value, newPS, true);
	    PierSideSP.s=rc ? IPS_OK : IPS_ALERT;
        IDSetSwitch(&PierSideSP, nullptr);
        return rc;
	}

	return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool PimocoMount::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) {
	if(dev==NULL || strcmp(dev,getDeviceName()))
		return INDI::Telescope::ISNewText(dev, name, texts, names, n);

    // if(strcmp(name, ...)) { } else

	return INDI::Telescope::ISNewText(dev, name, texts, names, n);
}

bool PimocoMount::ISSnoopDevice(XMLEle *root) {
	return INDI::Telescope::ISSnoopDevice(root);
}

// Protected class members
//

bool PimocoMount::saveConfigItems(FILE *fp){
    INDI::Telescope::saveConfigItems(fp);

    IUSaveConfigNumber(fp, &HAMotorNP);
    IUSaveConfigSwitch(fp, &HAMSwitchSP);
    IUSaveConfigNumber(fp, &HARampNP);

    IUSaveConfigNumber(fp, &DecMotorNP);
    IUSaveConfigSwitch(fp, &DecMSwitchSP);
    IUSaveConfigNumber(fp, &DecRampNP);

    IUSaveConfigNumber(fp, &SlewRatesNP);
    IUSaveConfigNumber(fp, &HALimitsNP);
    IUSaveConfigNumber(fp, &AltLimitsNP);

    IUSaveConfigNumber(fp, &GuiderSpeedNP);
    IUSaveConfigNumber(fp, &GuiderMaxPulseNP);

    return true;
}
