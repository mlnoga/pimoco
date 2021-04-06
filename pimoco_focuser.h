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

#ifndef PIMOCO_FOCUSER_H
#define PIMOCO_FOCUSER_H

#include <libindi/indifocuser.h>
#include "pimoco_stepper.h"

// Indi class for pimoco focusers
class PimocoFocuser : public INDI::Focuser {
public:
	// Creates a Pimoco focuser
	PimocoFocuser();

	// Destroys this pimoco focuser. Stops device motion for safety's sake
	~PimocoFocuser();

	virtual const char *getDefaultName() override;
    virtual bool initProperties() override;
    virtual bool updateProperties() override;

    virtual void ISGetProperties(const char *dev) override;
    virtual bool ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n) override;
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
    virtual bool ISSnoopDevice(XMLEle *root) override;

    // Updates number vector property with the given values if res is true and display status IPS_OK, else display status IPS_ALERT. Returns res for convenience. 
    bool ISUpdateNumber(INumberVectorProperty *NP, double values[], char *names[], int n, bool res);

protected:
    virtual bool saveConfigItems(FILE *fp) override;

    virtual bool Connect() override;
    virtual bool Disconnect() override;
    virtual bool Handshake() override;
    virtual void TimerHit() override;

    virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;
    virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
    virtual bool AbortFocuser() override;
    virtual bool ReverseFocuser(bool enabled) override;
    virtual bool SetFocuserSpeed(int speed) override;
    virtual bool SyncFocuser(uint32_t ticks) override;

	Stepper stepper;

    const char *spiDeviceFilename;

    // UI controls
    //

    INumber MotorN[4]={};
    INumberVectorProperty MotorNP;

    ISwitch MSwitchS[1]={};
    ISwitchVectorProperty MSwitchSP;

    INumber RampN[9]={};
    INumberVectorProperty RampNP;

};

#endif // PIMOCO_FOCUSER_H
