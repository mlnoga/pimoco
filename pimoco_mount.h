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

    virtual bool Sync(double equRA, double equDec) override;

    // Syncs to given device HA/Dec coordinates
    bool SyncDeviceHADec(double deviceHA, double deviceDec);

    virtual bool Goto(double equRA, double equDec) override; 

    // Executes a goto command to given equatorial RA, Dec and pier side. If pier side is not forced, also tries the other side
    // to meet mount limits. Returns immediately with true on success, false on failure.
    bool Goto(double equRA, double equDec, TelescopePierSide equPS, bool forcePierSide);

    virtual bool SetParkPosition(double Axis1Value, double Axis2Value) override;
    virtual bool SetCurrentPark() override;
    virtual bool SetDefaultPark() override;
    virtual bool Park() override;
    virtual bool UnPark() override;

    virtual IPState GuideNorth(uint32_t ms) override;
    virtual IPState GuideSouth(uint32_t ms) override;
    virtual IPState GuideEast(uint32_t ms) override;
    virtual IPState GuideWest(uint32_t ms) override;

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


    // Converts device coordinates to equatorial coordinates. If local sidereal time below zero is given, uses current time 
    void equatorialFromDevice(double *equRA, double *equDec, TelescopePierSide *equPS, double deviceHA, double deviceDec, double lst=-1);

    // Converts equatorial coordinates into device coordinates. If local sidereal time below zero is given, uses current time 
    void deviceFromEquatorial(double *deviceHA, double *deviceDec, double equRA, double equDec, TelescopePierSide equPS, double lst=-1);

    // Checks given equatorial position against mount altitude limits. Returns true if within bounds, else false 
    bool checkLimitsPosAlt(double equRA, double equDec);

    // Checks given device position against mount hour angle limits. Returns true if within bounds, else false 
    bool checkLimitsPosHA(double deviceHA, double deviceDec);

    // Checks given position and direction of motion against mount limits. Returns true if motion OK, else false 
    bool checkLimitsPosSpeed(double equRA, double equDec, TelescopePierSide equPS, double arcsecPerSecHa, double arcsecPerSecDec, bool log=true);

    // Applies mount limits to current position and direction of motion. Stops all motion and updates scope status if out of bounds and in wrong direction. Returns true if motion OK, else false 
    bool applyLimitsPosSpeed(double arcsecPerSecHa, double arcsecPerSecDec, bool log=true);


    // Physical connector GPIO pin numbers for stepper DIAG0 lines 
    enum {
        HA_DIAG0_PIN  = 35,
        DEC_DIAG0_PIN = 36,
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

    // Target equatorial position for gotos. For periodic refresh of the HA-based actual hardware gotos as time progresses
    double  gotoTargetRA=0, gotoTargetDec=0;

    // Target side of pier for gotos
    TelescopePierSide gotoTargetPS=PIER_EAST;

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

    INumber HALimitsN[2]={};
    INumberVectorProperty HALimitsNP;

    INumber AltLimitsN[2]={};
    INumberVectorProperty AltLimitsNP;


public:
    // Names of the mount configuration tabs
    static const char *HA_TAB;
    static const char *DEC_TAB;
};

#endif // PIMOCO_MOUNT_H
