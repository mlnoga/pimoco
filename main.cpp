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


#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <locale.h> // for thousands separator
#include <unistd.h> // for sleep
#include <libindi/indifocuser.h>

#include "pimoco_stepper.h"

// Unused dummies for Indi functions to avoid linker errors
//
extern "C" {

void ISGetProperties(const char *dev) {}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n) {}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) {}

void ISSnoopDevice(XMLEle *root) {}

} // extern "C"


void panicf(const char *fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vfprintf(stdout, fmt, argp);
	exit(-1);
}

void getAndPrintState(Stepper *stepper) {
	int32_t pos, speed;
	if(!stepper->getPosition(&pos) || !stepper->getSpeed(&speed))
		panicf("Error getting position and speed\n");

	printf("Current position is %'+d; speed is %'+d and status is ", pos, speed);
	const int bufsize=1023;
	char buffer[bufsize+1]={0};
	TMC5160::printStatus(buffer, bufsize, stepper->getStatus());
	puts(buffer);
}

int main(int argc, char ** argv) {
	puts("Starting up...");

	setlocale(LC_ALL, ""); // for thousands separator
	Stepper stepper("Default stepper");

	if(!stepper.open(Stepper::defaultSPIDevice))
		panicf("Error opening device %s\n", Stepper::defaultSPIDevice);

	getAndPrintState(&stepper);

	if(!stepper.syncPosition(0))
		panicf("Error syncing position\n");

	getAndPrintState(&stepper);

	uint32_t fullRevolutionInUsteps=256ul*400*3*144;
	// move by 1h of RA with 256 microsteps, 400 steps/rev, belt ratio 1:3 and Vixen worm gear ratio 1:144  
	if(!stepper.setTargetPositionBlocking((fullRevolutionInUsteps*1ul)/24))  
		panicf("Error on goto");

	getAndPrintState(&stepper);

	if(!stepper.setTargetPositionBlocking(0))
		panicf("Error on goto");
	
	getAndPrintState(&stepper);

	// WiringPi uses crystal osciallator running at 19.2 MHz for void gpioClockSet (int pin, int freq). 
	// See https://github.com/WiringPi/WiringPi/blob/master/wiringPi/wiringPi.c
	// TMC5160 needs 8..10-16 MHz, see section 26.2.1 in https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC5160A_Datasheet_Rev1.14.pdf 
	// To have an exact frequency based on an integer divider of two, we choose 9.6 MHz.
	double stepperClockInHz=9600000;  
	double stepperChipTimeScaler=(double) (1ul<<24);
	double stepperTimeUnit=stepperChipTimeScaler/stepperClockInHz;

	double siderealDayInSeconds=86164.0905;

	double siderealRateInUstepsPerTimeUnit=(((double)fullRevolutionInUsteps)/siderealDayInSeconds)*stepperTimeUnit;
	uint32_t siderealRateInUstepsPerTimeUnitRounded=(uint32_t) (siderealRateInUstepsPerTimeUnit+0.5);
	if(!stepper.setTargetSpeed(siderealRateInUstepsPerTimeUnitRounded))
		panicf("Error on setSpeed");

	for(int i=0; i<60; i++) {
		sleep(1);
		getAndPrintState(&stepper);
	}

	if(!stepper.setTargetSpeed(0))
		panicf("Error on setSpeed");
	getAndPrintState(&stepper);
	sleep(1);
	getAndPrintState(&stepper);

	if(!stepper.setTargetPositionBlocking(0))
		panicf("Error on goto");
	
	getAndPrintState(&stepper);

	puts("Exiting...");
	return 0;
}
