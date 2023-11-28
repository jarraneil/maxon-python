# maxon-python
Python drivers for Maxon EPOS.
This is a pure Python implementation of drivers for the Maxon EPOS4 series of motion controllers.
It is intended for embedded systems running MicroPython.
It exposes the EPOS4 as a class with methods for 
- building and sending the serial data frame including CRC calculation and byte stuffing
- an object dictionary
