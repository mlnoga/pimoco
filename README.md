# pimoco
Telescope mount and focuser control with a Raspberry Pi 

## Motivation

Why do we need a separate device to control stepper motors attached to a telescope? Often times, a Raspberry Pi 4 running Ekos/Kstars/Indi will be attached to the telescope anyway, providing plenty of compute power. Modern stepper motor controllers like the Trinamic TMC5160 have an on-board motion controller which accepts position read, set target speed and go-to commands via SPI. The Raspberry Pi has two such interfaces for a total of five devices on its GPIO port, and can generate the necessary clock signals via hardware PCM.

This is 
* robust (no Wifi, Bluetooth or USB connections to the mount as a source of error)
* beautiful (supports SkySafari on phones/tablets and game controllers via Ekos/Kstars/Indi) 
* inexpensive (no need for separate microcontrollers like OnStep or TeenAstro)
* manageable (no separate device firmware, only Indi drivers which Linux package managers can update)    

## References

* [Trinamic TMC5160 Datasheet](https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC5160A_Datasheet_Rev1.14.pdf)
* [Trinamic TMC5160-BOB Breakout Board](https://www.trinamic.com/fileadmin/assets/Products/Eval_Documents/TMC5160-BOB_datasheet_rev1.10.pdf)
* [Raspberry Pi 4 SOC Datasheet BCM2711](https://www.raspberrypi.org/documentation/hardware/raspberrypi/bcm2711/rpi_DATA_2711_1p0.pdf)
* [Raspberry Pi GPIO](https://www.raspberrypi.org/documentation/hardware/raspberrypi/gpio/README.md)
* [Raspberry Pi SPI](https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md)
* [Indi Github](https://github.com/indilib/indi)

## Digikey parts list

Apart from a Raspberry Pi 4 (get the 8 GB model just in case you want to run live stacking etc), here are the parts you will need:

| Vendor item | Digikey item | Name | Quantity |
|-------------|--------------|------|----------|
| TMC5160-BOB | 1460-1250-ND |  BREAKOUTBOARD WITH TMC5160 | 3 |
| EEU-FC1J151 | P10344-ND | CAP ALUM 150UF 20% 63V RADIAL | 3	|
|	DLS1XS5AK40X | 626-1561-ND | CONN D-SUB RCPT 9POS R/A SLDR | 3 |
|	PJ-037AH | CP-037AH-ND | CONN PWR JACK 2X5.5MM SOLDER | 1 |
|	PJ-002BH | CP-002BH-ND | CONN PWR JACK 2.5X5.5MM SOLDER | 1 |
|	4527 | 36-4527-ND | FUSE HLDR CARTRIDGE 250V 5A PCB | 1 |
|	0217005.HXP | F2395-ND | FUSE GLASS 5A 250VAC 5X20MM | 1 |
|	100SP1T2B4M6QE | EG2362-ND | SWITCH TOGGLE SPDT 5A 120V | 1 |
|	SB540 | SB540FSCT-ND | DIODE SCHOTTKY 40V 5A DO201AD | 1 |
|	PPTC202LFBN-RC | S6104-ND | CONN HDR 40POS 0.1 TIN PCB | 1 | 
