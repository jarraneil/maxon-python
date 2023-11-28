# maxon-python
Python drivers for Maxon EPOS.
This is a pure Python implementation of drivers for the Maxon EPOS4 series of motion controllers.
It is intended for embedded systems running MicroPython.
It exposes the EPOS4 as a class with methods for 
- building and sending iand receiving the serial data frame including CRC calculation and byte stuffing/unstuffing
- an object dictionary
## state of development
It is not yet a full implementation but has the features added to achieve a specific task for a client develeopment that involves 
- setting and reading peak current 
- moving with a given velocity, following an acceleration profile
- moving to a given position, following a velocity and acceleration profile
It is known to work running on an RP2040 talking to an EPOS4 50/8 drive
## installation and use
1. download the drivers.py file
2. import it into your project using
> from drivers import EPOS4
4. instantiate an EPOS4 object by passing it a prevously created serial port
> serial = UART(0, baudrate=115200, bits=8, parity=None, stop=1)
> epos = EPOS4(serial)
