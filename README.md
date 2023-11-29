# maxon-python
Python drivers for Maxon EPOS4.
This is a pure Python implementation of drivers for the Maxon EPOS4 series of motion controllers.
It is intended for embedded systems running MicroPython.
It exposes the EPOS4 as a class with methods for 
- building, sending and receiving the serial data frame including CRC calculation and byte stuffing/unstuffing
- an object dictionary
- higher level abstractions for interacting with the object dictionary
## state of development
It is not yet a full implementation but has the features required to achieve a specific function. This involves 
- setting and reading peak current 
- moving with a given velocity, following an acceleration profile
- moving to a given position, following a velocity and acceleration profile
- basic homing (two methods)
  
It is known to work running on an RP2040 talking to an EPOS4 50/8 drive
## installation and use
Note that for proper operation the EPOS4 needs to first be configured and tuned first using Maxon's EPOS Studio software. This is described [here](https://support.maxongroup.com/hc/en-us/articles/6719969220380-EPOS4-IDX-Important-steps-during-initial-commissioning)

1. download the drivers.py file
2. import it into your project using `from drivers import EPOS4`
3. instantiate an EPOS4 object by passing it a prevously created serial port `serial = UART(0, baudrate=115200, bits=8, parity=None, stop=1)`. Followed by `epos = EPOS4(serial)`

## Useful resources
[EPOS4 Communication Guide](https://www.maxongroup.com/medias/sys_master/root/8834324922398/EPOS4-Communication-Guide-En.pdf)

[EPOS4 Firmware Specification](https://www.maxongroup.com/medias/sys_master/8821690564638.pdf)

Chapter 7 in [EPOS4 Application Notes](https://www.maxongroup.com/medias/sys_master/8823917936670.pdf)

You can also contact the [Maxon support team]( https://support.maxongroup.com) directly; they are always very helpful and supportive.
