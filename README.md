# pimoco
Telescope mount and focuser control with a Raspberry Pi 


## Motivation

Why do we need a separate device to control stepper motors attached to a telescope? Often times, a Raspberry Pi 4 running Ekos/Kstars/Indi will be attached to the telescope anyway, providing plenty of compute power. Modern stepper motor controllers like the Trinamic TMC5160 have an on-board motion controller which accepts position read, set target speed and go-to commands via SPI. The Raspberry Pi has two such interfaces for a total of five devices on its GPIO port.

This is 
* capable (256x microsteps, up to 1000x sidereal slewing, silent tracking, load-adaptive speed w/o step loss)
* robust (no Wifi, Bluetooth or USB connections to the mount as a source of error)
* beautiful (supports SkySafari on phones/tablets and game controllers via Ekos/Kstars/Indi) 
* inexpensive (no need for separate microcontrollers like OnStep or TeenAstro)
* manageable (no separate device firmware, only Indi drivers which Linux package managers can update)     


## [Wiki](../../wiki)

* [Hardware](../../wiki/Hardware)
* [Software](../../wiki/Software)
* [Results](../../wiki/Results)
