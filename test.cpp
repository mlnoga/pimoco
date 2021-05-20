/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 *
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

enum {
	TMCR_X_ENC = 0x39,
};

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 1000000;
static uint16_t delay=0;

static void transfer(int fd)
{
	uint8_t tx[]={
		TMCR_X_ENC | 0x80, 0x01, 0x02, 0x03, 0x04,  // set XENC to 0x01020304
		TMCR_X_ENC       , 0x11, 0x12, 0x13, 0x14,  // get XENC
		TMCR_X_ENC | 0x80, 0x21, 0x22, 0x23, 0x24,  // set XENC to 0x21222324
		TMCR_X_ENC       , 0x31, 0x32, 0x33, 0x34,  // get XENC
		TMCR_X_ENC       , 0x41, 0x42, 0x43, 0x44,  // get XENC
	};
	uint32_t len=ARRAY_SIZE(tx);
	printf("len %d\n", len);
	uint8_t rx[len];

	uint32_t numTransfers=len/5;
	struct spi_ioc_transfer tr[numTransfers];
	for(uint32_t t=0; t<numTransfers; t++)
		tr[t]={
			.tx_buf        = (unsigned long) &tx[5*t],
			.rx_buf        = (unsigned long) &rx[5*t],
			.len           = 5,
			.speed_hz      = speed,
			.delay_usecs   = delay,
			.bits_per_word = bits,
			.cs_change     = ((t==(numTransfers-1)) ? (uint8_t) 0 : (uint8_t) 0xff),
			.pad           = 0,
		};
	int res=ioctl(fd, SPI_IOC_MESSAGE(numTransfers), tr);
	if (res<0)
		pabort("can't send spi message");

	// dump transfer
	for (uint32_t t=0; t<numTransfers; t++) {
		printf("%2d: tx %02x %02x %02x %02x %02x  rx %02x %02x %02x %02x %02x\n",
			   t, tx[5*t+0], tx[5*t+1], tx[5*t+2], tx[5*t+3], tx[5*t+4], rx[5*t+0], rx[5*t+1], rx[5*t+2], rx[5*t+3], rx[5*t+4]);
	}

	// validate responses
	if(rx[5*1+1]!=tx[5*0+1] || rx[5*1+2]!=tx[5*0+2] || rx[5*1+3]!=tx[5*0+3] || rx[5*1+4]!=tx[5*0+4] ) {
		printf("Handshake failed: got %02x %02x %02x %02x after first set\n", rx[5*1+1], rx[5*1+2], rx[5*1+3], rx[5*1+4]);
		return;
	}
	if(rx[5*2+1]!=tx[5*0+1] || rx[5*2+2]!=tx[5*0+2] || rx[5*2+3]!=tx[5*0+3] || rx[5*2+4]!=tx[5*0+4] ) {
		printf("Handshake failed: got %02x %02x %02x %02x after first get\n", rx[5*2+1], rx[5*2+2], rx[5*2+3], rx[5*2+4]);
		return;
	}
	if(rx[5*3+1]!=tx[5*2+1] || rx[5*3+2]!=tx[5*2+2] || rx[5*3+3]!=tx[5*2+3] || rx[5*3+4]!=tx[5*2+4] ) {
		printf("Handshake failed: got %02x %02x %02x %02x after second set\n", rx[5*3+1], rx[5*3+2], rx[5*3+3], rx[5*3+4]);
		return;
	}
	if(rx[5*4+1]!=tx[5*2+1] || rx[5*4+2]!=tx[5*2+2] || rx[5*4+3]!=tx[5*2+3] || rx[5*4+4]!=tx[5*2+4] ) {
		printf("Handshake failed: got %02x %02x %02x %02x after second get\n", rx[5*4+1], rx[5*4+2], rx[5*4+3], rx[5*4+4]);
		return;
	}
	printf("OK\n");
}

void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}

void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;

	parse_opts(argc, argv);

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	printf("mode %d\n", mode);
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	transfer(fd);

	close(fd);

	return ret;
}
