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

	virtual const char *getDefaultName();
    virtual bool initProperties();
    virtual bool updateProperties();

    virtual void ISGetProperties(const char *dev);
    virtual bool ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n);
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n);
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);
    virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n);
    virtual bool ISSnoopDevice(XMLEle *root);

protected:
    virtual bool Handshake();
    virtual void TimerHit();

    virtual IPState MoveAbsFocuser(uint32_t targetTicks);
    virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks);
    virtual bool AbortFocuser();
    virtual bool ReverseFocuser(bool enabled);
    virtual bool SetFocuserSpeed(int speed);
    virtual bool SyncFocuser(uint32_t ticks);

	Stepper stepper;

    const char *spiDeviceFilename;
};

#endif // PIMOCO_FOCUSER_H
