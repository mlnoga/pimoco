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

#include "pimoco_tmc5160.h"

void panicf(const char *fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vfprintf(stdout, fmt, argp);
	exit(-1);
}

void getAndPrintState(TMC5160SPI *stepper) {
	int32_t pos, speed;
	if(!stepper->getPosition(&pos) || !stepper->getSpeed(&speed))
		panicf("Error getting position and speed\n");

	printf("Current position is %'+d; speed is %'+d and status is ", pos, speed);
	TMC5160SPI::printStatus(stdout, stepper->getStatus());
	puts("");
}

int main(int argc, char ** argv) {
	puts("Starting up...");

	setlocale(LC_ALL, ""); // for thousands separator
	TMC5160SPI stepper;
	stepper.setDebugLevel(TMC5160SPI::TMC_DEBUG_ACTIONS);

	if(!stepper.open(TMC5160SPI::defaultDevice))
		panicf("Error opening device %s\n", TMC5160SPI::defaultDevice);

	getAndPrintState(&stepper);

	if(!stepper.syncPosition(0))
		panicf("Error syncing position\n");

	getAndPrintState(&stepper);

	if(!stepper.setTargetPositionBlocking(10000))
		panicf("Error on goto");

	getAndPrintState(&stepper);

	if(!stepper.setTargetPositionBlocking(0))
		panicf("Error on goto");
	
	getAndPrintState(&stepper);

	puts("Exiting...");
	return 0;
}
