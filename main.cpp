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

#include "pimoco_tmc5160.h"

void panicf(const char *format, ...) {
	va_list vargs;
	printf(format, vargs);
	exit(-1);
}

int main(int argc, char ** argv) {
	puts("Starting up...");

	TMC5160SPI stepper;
	if(!stepper.open(TMC5160SPI::defaultDevice))
		panicf("error opening stepper device %s\n", TMC5160SPI::defaultDevice);

	int32_t pos, speed;
	if(!stepper.getPosition(&pos) || !stepper.getSpeed(&speed))
		panicf("error getting position and speed\n");
	printf("position %'6d\tspeed %'6d\tstatus 0x%2x\n", pos, speed, stepper.getStatus());

	puts("Exiting...");
	return 0;
}
