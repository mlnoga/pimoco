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

#include "pimoco_stepper.h"

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
	TMC5160::printStatus(stdout, stepper->getStatus());
	puts("");
}

int main(int argc, char ** argv) {
	puts("Starting up...");

	setlocale(LC_ALL, ""); // for thousands separator
	Stepper stepper;
	stepper.setDebugLevel(Stepper::TMC_DEBUG_ACTIONS);

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

	double stepperClockInHz=12000000;
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
