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

#ifndef PIMOCO_MOUNT_H
#define PIMOCO_MOUNT_H

#include <libindi/indicom.h>
#include <libindi/inditelescope.h>
#include <libindi/indiguiderinterface.h>
#include "pimoco_stepper.h"

// Indi class for pimoco mounts
class PimocoMount : public INDI::Telescope, public INDI::GuiderInterface {
public:
	// Creates a Pimoco mount
	PimocoMount();

	// Destroys this pimoco focuser. Stops device motion for safety's sake
	~PimocoMount();

	virtual const char *getDefaultName() override;
    virtual bool initProperties() override;
    virtual bool updateProperties() override;

    virtual bool ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n) override;
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
    virtual bool ISSnoopDevice(XMLEle *root) override;

    // Updates number vector property with the given values if res is true and display status IPS_OK, else display status IPS_ALERT. Returns res for convenience. 
    bool ISUpdateNumber(INumberVectorProperty *NP, double values[], char *names[], int n, bool res);

    enum {
        TRACK_KING = TRACK_CUSTOM+1,
    };

protected:
    virtual bool saveConfigItems(FILE *fp) override;

    virtual bool Connect() override;
    virtual bool Disconnect() override;
    virtual bool Handshake() override;
    virtual void TimerHit() override;

    // Updates HA/Dec rates if guiding timeouts are reached. Returns true on success, else false.
    bool guiderTimerHit();

    virtual bool ReadScopeStatus() override;

    // Returns next timer interval in milliseconds. Guiding has priority, then Goto tracking, else base polling rate. 
    uint32_t getNextTimerInterval();

    // Returns next guider timer interval in milliseconds. 
    uint32_t getGuiderTimerInterval();


    virtual bool Abort() override;

    virtual bool SetTrackEnabled(bool enabled) override;
    virtual bool SetTrackMode(uint8_t mode) override;
    virtual bool SetTrackRate(double raRate, double deRate) override;

    virtual bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
    virtual bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;
    virtual bool SetSlewRate(int index) override;

    virtual bool Sync(double ra, double dec) override;
    bool SyncHADec(double ha, double dec);
    virtual bool Goto(double ra, double dec) override;

    virtual bool SetParkPosition(double Axis1Value, double Axis2Value) override;
    virtual bool SetCurrentPark() override;
    virtual bool SetDefaultPark() override;
    virtual bool Park() override;
    virtual bool UnPark() override;

    virtual IPState GuideNorth(uint32_t ms) override;
    virtual IPState GuideSouth(uint32_t ms) override;
    virtual IPState GuideEast(uint32_t ms) override;
    virtual IPState GuideWest(uint32_t ms) override;

    // Returns local apparent sidereal time in hours for given Julian day number
    double getLocalSiderealTime(double julianDay);

    // Returns local apparent sidereal time in hours now
    double getLocalSiderealTime();

    // Returns current realtime clock in milliseconds
    static uint64_t getTimeMillis();

    // Converts given value into native dec range of [-180,180) degrees
    static double rangeDecNative(double r);

    // Internal tracking methods
    //

    // Gets tracking mode 
    uint8_t getTrackMode() const { return IUFindOnSwitchIndex(&TrackModeSP); }

    // Gets tracking rate for RA for current tracking mode
    double getTrackRateRA() const { 
        uint8_t mode=getTrackMode();
        return (mode==TRACK_CUSTOM) ? trackRateCustomRA : trackRates[mode];
    }

    // Gets tracking rate for Dec for current tracking mode
    double getTrackRateDec() const { 
        uint8_t mode=getTrackMode();
        return (mode==TRACK_CUSTOM) ? trackRateCustomDec : 0;
    }

    // Apply current tracking status, mode and rate to steppers. Checks mount limits and resets scope to idle if violated. Returns true on success, else failure 
    bool applyTracking(bool updateRA=true, bool updateDec=true);


    // Checks given position against mount limits. Returns true if within bounds, else false 
    bool checkLimitsPos(double ra, double dec, bool log=true);

    // Checks given position and direction of motion against mount limits. Returns true if motion OK, else false 
    bool checkLimitsPosSpeed(double ra, double dec, double arcsecPerSecHa, double arcsecPerSecDec, bool log=true);

    // Applies mount limits to current position. Stops all motion and updates scope status if out of bounds. Returns true if within bounds, else false 
    bool applyLimitsPos(bool log=true);

    // Applies mount limits to current position and direction of motion. Stops all motion and updates scope status if out of bounds and in wrong direction. Returns true if motion OK, else false 
    bool applyLimitsPosSpeed(double arcsecPerSecHa, double arcsecPerSecDec, bool log=true);


    // Physical pin numbers on Raspberry Pi connector for stepper DIAG0 lines 
    enum {
        HA_DIAG0_PIN = 13,
        DEC_DIAG0_PIN = 15,
    };

	Stepper stepperHA;
    Stepper stepperDec;

    const char *spiDeviceFilenameHA;
    const char *spiDeviceFilenameDec;

    // Tracking rates for sidereal, solar, lunar and custom (defaults to sidereal)
    static const double trackRates[];

    // Tracking rate variable names
    static const char *trackRateNames[];

    // Tracking rate UI labels
    static const char *trackRateLabels[];

    // Custom tracking rates for both axes. Active only in mode TRACK_CUSTOM
    double  trackRateCustomRA =trackRates[TRACK_CUSTOM], trackRateCustomDec=0;    

    // Stores if the scope was tracking before a goto slew was initiated. Required to work around INDI bug 
    bool    wasTrackingBeforeSlew=false;

    // Target position for gotos. For periodic refresh of the HA-based actual hardware gotos as time progresses
    double  gotoTargetRA=0, gotoTargetDec=0;

    // Manual slewing speed active on the given axis, or zero if inactive
    double manualSlewArcsecPerSecRA=0, manualSlewArcsecPerSecDec=0;

    // Flag: guider pulse currently active on the given axis
    bool guiderActiveRA=false, guiderActiveDec=false;

    // Timeouts for guider pulse
    uint64_t guiderTimeoutRA, guiderTimeoutDec;

    enum {
        NUM_SLEW_RATES = 4
    } SlewRatesType;

    // UI controls
    //

    INumber HAMotorN[Stepper::MOTORN_SIZE]={};
    INumberVectorProperty HAMotorNP;

    ISwitch HAMSwitchS[Stepper::MSWITCHS_SIZE]={};
    ISwitchVectorProperty HAMSwitchSP;

    INumber HARampN[Stepper::RAMPN_SIZE]={};
    INumberVectorProperty HARampNP;

    INumber DecMotorN[Stepper::MOTORN_SIZE]={};
    INumberVectorProperty DecMotorNP;

    ISwitch DecMSwitchS[Stepper::MSWITCHS_SIZE]={};
    ISwitchVectorProperty DecMSwitchSP;
    
    INumber DecRampN[Stepper::RAMPN_SIZE]={};
    INumberVectorProperty DecRampNP;

    INumber SlewRatesN[NUM_SLEW_RATES]={};
    INumberVectorProperty SlewRatesNP;

    ISwitch SyncToParkS[1]={};
    ISwitchVectorProperty SyncToParkSP;
    
    INumber GuiderSpeedN[1]={};
    INumberVectorProperty GuiderSpeedNP;

    INumber GuiderMaxPulseN[1]={};
    INumberVectorProperty GuiderMaxPulseNP;

    INumber AltLimitsN[2]={};
    INumberVectorProperty AltLimitsNP;

public:
    // Names of the mount configuration tabs
    static const char *HA_TAB;
    static const char *DEC_TAB;
};

#endif // PIMOCO_MOUNT_H
