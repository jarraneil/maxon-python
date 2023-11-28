import struct
import time
from ubinascii import hexlify
from machine import Pin, ADC, PWM, UART, I2C

class EPOS4:
    #this class represents a Maxon EPOS4 drive
    #if we had a full OS we would use the Maxon linux drivers that would perform these functions
    #the serial communication is described in EPOS4 communications guide 
    #included CRC, byte stuffing, preamble

    def __init__(self, EPOS_SERIAL):
        #EPOS_SERIAL is the serial port we are using to talk to the drive
        self.serial = EPOS_SERIAL
        self.NODE_ID = 0 #CAN node id. Set to 0 for serial
        self.DLE = b'\x90' #data link escape character
        self.STX = b'\x02' #start of text character
        #define the object dictionary keyed by name and holding index, subindex and data format
        #data format is as per python struct
        #L = uint32
        #l = int32
        #H = uint16
        #h = int16
        #B = uint8
        #b = int8
        self.objects = {
                'error':                (0x1001, 0x00, 'B'),
                'actual_voltage':       (0x2200, 0x01, 'H'),
                'current_limit':        (0x3001, 0x02, 'L'),
                'homing_current_limit': (0x30b2, 0x00, 'H'),
                'average_current':      (0x30d1, 0x01, 'l'),
                'actual_current':       (0x30d1, 0x02, 'l'),
                'actual_temperature':   (0x3201, 0x01, 'h'),
                'control':              (0x6040, 0x00, 'H'),
                'status':               (0x6041, 0x00, 'H'),
                'mode':                 (0x6060, 0x00, 'b'),
                'mode_display':         (0x6061, 0x00, 'b'),
                'actual_position':      (0x6064, 0x00, 'l'),
                'velocity_demand':      (0x606b, 0x00, 'l'),
                'actual_velocity':      (0x606c, 0x00, 'l'),
                'target_position':      (0x607a, 0x00, 'l'),
                'profile_velocity':     (0x6081, 0x00, 'L'),
                'profile_acceleration': (0x6083, 0x00, 'L'),
                'profile_deceleration': (0x6084, 0x00, 'L'),
                'profile_type':         (0x6086, 0x00, 'h'),
                'homing_method':        (0x6098, 0x00, 'b'),
                'homing_speed_switch':  (0x6099, 0x01, 'L'),
                'target_velocity':      (0x60ff, 0x00, 'l'),
                }
        #define the allowed modes the drive can be in
        #we have the following mappings from Table 6-150 of Firmware Spec
        self.mode = {
                'PPM': 1,
                'PVM': 3,
                'HMM': 6,
                'CSP': 8,
                'CSV': 9,
                'CST': 10
                }
        #make a dictionary of control bit for control register
        #keyed on command text
        #entries in tuple are mask for bit, bit value and applibacle mode
        self.control = {
                'shutdown':             (0x0087, 0x0006, 'ANY'),
                'switch_on':            (0x0087, 0x0007, 'ANY'),
                'switch_on_and_enable': (0x008f, 0x000f, 'ANY'),
                'disable_voltage':      (0x0082, 0x0000, 'ANY'),
                'quick_stop':           (0x0086, 0x0002, 'ANY'),
                'disable_operation':    (0x008f, 0x0007, 'ANY'),
                'enable_operation':     (0x008f, 0x000f, 'ANY'),
                'start':                (0x0100, 0x0000, 'ANY'),
                'stop':                 (0x0100, 0x0100, 'ANY'),
                'homing_start':         (0x0010, 0x0010, 'HMM'),
                'clear_homing_start':   (0x0010, 0x0000, 'HMM'),
                'absolute':             (0x0040, 0x0000, 'PPM'),
                'relative':             (0x0040, 0x0040, 'PPM'),
                'new_setpoint':         (0x0010, 0x0010, 'PPM'),
                'clear_new_setpoint':   (0x0010, 0x0000, 'PPM'),
                'move_immediate':       (0x0020, 0x0020, 'PPM'),
                'move_after_current':   (0x0020, 0x0000, 'PPM'),
                'fault_low':            (0x0080, 0x0000, 'ANY'),
                'fault_high':           (0x0080, 0x0080, 'ANY'),
                }
        #clear out the serial buffer in case there is any garbage in it
        self.serial.read()

    def calc_crc(self, ba):
        #calculates the CRC for a given byte array
        #ba is a bytes object that is comprised of data followed by a 2 byte (0x0000) placeholder for the CRC
        #ba must be an even number of bytes in length
        assert len(ba)%2 == 0, f'expected even number of bytes but got in hex {ba}'
        #make sure the last two bytes are zero
        assert ba[-1] == 0, f'expected 0 but got {ba[-1]}'
        assert ba[-2] == 0, f'expected - but got {ba[-2]}'
        #find how many 16 bit words are passed in
        num_words = len(ba)//2
        #make the format string for the struct unpack by repeating the number of words
        formatString = '<'+ num_words*'H'
        #unpack the byte array as words
        wordArray = struct.unpack(formatString, ba)
        CRC = 0 #initialise the CRC
        for word in wordArray:
            shifter = 0x8000
            while True: #check if bit 15 of CRC is set
                if (CRC & 0x8000) >0:
                    carry=True
                else:
                    carry=False
                CRC <<= 1 #multiply CRC by 2
                if (word & shifter)>0: #check if bit X is set
                    CRC+=1 #if it is increment the CRC
                if carry:
                    CRC &= 0xffff #discard the carry
                    CRC ^= 0x1021
                shifter >>= 1 #move shifter along one bit
                if shifter == 0:
                    break
        #return CRC as bytes
        return CRC.to_bytes(2,'little')

    def add_crc(self, data, crc):
        #data is a bytes object that is comprised of data followed by a 2 byte (0x0000) placeholder for the CRC
        #data must be an even number of bytes in length
        #crc is a 16 bit crc
        #find how many 16 bit words are passed in
        #make sure we have an even number of bytes
        assert len(data)%2 == 0, f'expected even number but got {len(data)}'
        #make sure the last two bytes are zero
        assert data[-1] == 0
        assert data[-2] == 0
        #make sure the CRC is 16 bits i.e. 2 bytes
        assert len(crc) == 2, f'expected crc of 2 bytes but got {len(crc)}'
        #return the data with the last two empty bytes removed and the crc added in its place
        return data[0:-2] + crc

    def byte_stuff(self, ba):
        #ba is a byte array to send that may contain DLE byte as data
        #any DLE in the data need to be escaped via duplication
        #we also need to add preamble
        split = ba.split(self.DLE)
        #split is a list of byte arrays broken at the DLE bytes
        #we need to add two DLEs to every one (except the last)
        escaped = b''
        for s in split[0:-1]: #every one but the last
            escaped += s + 2*self.DLE #add two DLE bytes
        #finally we add the last section
        escaped += split[-1]
        #prepend a single DLE and STX
        return self.DLE + self.STX + escaped
    
    def byte_unstuff(self, ba):
        #ba is a byte array of the full frame received including preamble and stuffing
        #this method removes the preamble and stuffing from received data
        #first check that it is correctly formed with preamble
        well_formed = False #assume badly formed packet until proven otherwise
        if ba[0:2] == self.DLE + self.STX:
            #preamble is good
            well_formed = True
            #remove the preamble (first two bytes)
            remainder = ba[2:]
            #split the remainder at DLE
            split = remainder.split(self.DLE)
            #split should have an odd number of entries as single DLEs are not allowed
            if len(split)%2 != 1:
                well_formed = False
            #any even elements that exist must be of length zero to be well formed
            for s in split[1::2]: #just get the even elements
                if len(s) != 0: #check they are of length 0
                    well_formed = False
            if well_formed == True:
                #we now put single DLEs back into the data where doubles were before
                unescaped = b''
                #we take the odd elements only but not the last one
                for s in split[0:-1:2]:
                    #put a single DLE at the end
                    unescaped += s + self.DLE
                #finally add the last section without a DLE
                unescaped += split[-1]
                return unescaped
            else:
                print(f'badly formed packet {ba}')
                return None
        else:
            print(f'bad preamble in {ba}')
            return None

    def crc_match(self, ba):
        #check to see if the CRC in the last two bytes matches the data
        #ba is a byte array comprising data followed by a 16 bit CRC
        #extract just the data portion by making a copy and setting last two bytes to zero
        data = ba[0:-2] + b'\x00\x00'
        #extract just the last two bytes to hold the CRC received
        crc_received = ba[-2:]
        #compute the CRC over the data field
        crc_computed = self.calc_crc(data)
        #check if the computer and received CRCs match
        if crc_received == crc_computed:
            return True
        else:
            print(f'CRC computed is {crc_computed} on {ba}')
            return False

    def pack_and_write(self, ba):
        #make sure the serial buffer is empty before we start
        if self.serial.any():
            bogus_data = self.serial.read()
            print(f'found and removed {bogus_data} in serial buffer')
        #build the frame and transmit it to the EPOS4
        crc = self.calc_crc(ba)
        #put the computed CRC at the end
        f2 = self.add_crc(ba, crc)
        #escape any DLE in data and add framing bytes
        f3 = self.byte_stuff(f2)
        #write out to the Maxon drive
        num_bytes_sent = self.serial.write(f3)
        if num_bytes_sent == len(f3):
            return True
        else:
            print(f'expecting {len(f3)} bytes sent but only sent {num_bytes_sent}')
            return False

    def read_and_unpack(self):
        #read the data coming back from the EPOS4 and unpack it
        #sleep for a while to allow the Maxon to respond
        #if it has not responded with 1 second, we give up
        time_started = time.ticks_ms()
        while self.serial.any() == 0:
            #nothing received yet so wait
            time.sleep_ms(1)
            #check how long we have been waiting
            if time.ticks_diff(time.ticks_ms(), time_started) >=1000:
                print(f'EPOS read timeout')
                return None
        #there is something in the buffer
        #create a placeholder byte array variable for received data
        rxData = b''
        while self.serial.any() > 0:
            #there is still data in the buffer
            #read and append single byte
            rxData += self.serial.read(1)
            time.sleep_ms(1)
        #no more data in the buffer
        #remove the stuffing and preamble
        unstuffed = self.byte_unstuff(rxData)
        if unstuffed != None:
            #the unstuffing was successful
            if self.crc_match(unstuffed):
                #the CRC is good
                #return with the CRC bytes removed (last two)
                return unstuffed[0:-2]
            else:
                #the CRC was bad
                print('CRC mismatch')
                return None
        else:
            #the unstuffing failed
            print('bad framing received')
            return None

    def read_object(self, index, subindex, data_format):
        #make sure the data format passed in is valid
        assert data_format in [d[2] for d in self.objects.values()]
        OPCODE = 0x60 #this is used to read from object
        LENGTH = 2 #number of words to follow header
        format_string = '<BBBHBH' #8 bytes long, including 2 bytes for CRC
        #the last 0 word is used as place holder for 16 bit CRC
        #pack into a byte array as we need it to be mutable
        frame = struct.pack(format_string, OPCODE, LENGTH, self.NODE_ID, index, subindex,0)
        self.pack_and_write(frame)
        recvd = self.read_and_unpack()
        if recvd != None:
            #we had a good read
            #break the byte array (minus CRC) into sections as per p2-7 of communication guide
            format_string = '<BBI' + data_format
            try:
                unpacked = struct.unpack(format_string, recvd)
            except:
                print(f'expecting {format_string} but got {recvd}')
                return None
            if unpacked[0] == 0 and unpacked[1] == 4 and unpacked[2] == 0:
                #we have a proper response as we are expecting 4 words and error code is 0
                #return the remainder
                return unpacked[3]
            else:
                print('badly formed packet')
                return None
        else:
            return None

    def read_object_by_name(self, object_name):
        assert object_name in self.objects.keys(), f'{object_name} not in keys'
        index = self.objects[object_name][0]
        subindex = self.objects[object_name][1]
        data_format = self.objects[object_name][2]
        return self.read_object(index, subindex, data_format)

    def write_object(self, index, subindex, data, data_format):
        #ignore the data format and always send 4 bytes signed integer
        #TODO figure out if this is correct i.e. always send int32
        data_format = 'l'
        #data is 4 bytes
        #assert data >=0
        #assert data <= 2**32
        assert data_format in [d[2] for d in self.objects.values()], f'{data_format} not in dictionary'
        OPCODE = 0x68 #this is used to write object
        LENGTH = 4 #number of words to follow header
        #format_string = '<BBBHBiH' #12 bytes long, including 2 bytes for CRC
        format_string = '<BBBHB' + data_format + 'H' #12 bytes long, including 2 bytes for CRC
        #the last 0 word is used as place holder for 16 bit CRC
        #pack into a byte array as we need it to be mutable
        frame = struct.pack(format_string, OPCODE, LENGTH, self.NODE_ID, index, subindex, data, 0)
        #send and check it sent ok
        if self.pack_and_write(frame):
            #the write was a success
            recvd = self.read_and_unpack()
            if recvd != None:
                #we had a good read and unpack
                #break the byte array (minus CRC) into sections as per p2-9 of communication guide
                unpacked = struct.unpack('<BBH', recvd)
                if unpacked[0] == 0 and unpacked[1] == 2 and unpacked[2] == 0:
                    #we had a good write as error code is 0
                    return True
                elif unpacked[2] != 0:
                    print(f'error = {hex(unpacked[2])} from received {hexlify(recvd)}')
                    return False
                else:
                    #packet is not well formed
                    print(unpacked)
                    return False
            else:
                #we had a bad read_and_unpack
                return False
        else:
            return False

    def write_object_by_name(self, object_name, data):
        assert object_name in self.objects.keys(), f'{object_name} not in keys'
        index = self.objects[object_name][0]
        subindex = self.objects[object_name][1]
        data_format = self.objects[object_name][2]
        return self.write_object(index, subindex, data, data_format)

    def get_control(self):
        #read the control register
        return self.read_object_by_name('control')

    def set_control(self, command_text):
        #set or resets the appropriate bits in the control register
        #to achieve action defined in command_text
        #all other bits are untouched
        #read the existing control register
        existing = self.get_control()
        if existing != None:
            #the read was successful
            #set the mask and control according to Table 2-7 in Firmware Spec
            #mask defined which bits are touched (1) and which are left alone (0)
            #control defines how the bits are set
            #some of the bits are mode dependant so lets see what mode we are in
            current_mode = self.read_object_by_name('mode_display')
            #make sure the command is allowed
            assert command_text in self.control.keys()
            #read the mask, control and mode from the dictionary
            mask = self.control[command_text][0]
            control = self.control[command_text][1]
            applicable_mode = self.control[command_text][2]
            #if we are mode dependant we need to do some further checks
            if not (applicable_mode == 'ANY'):
                #make sure the mode is allowed
                assert applicable_mode in self.mode.keys()
                #make sure we are in the mode we are expecting for the command
                assert self.mode[applicable_mode] == current_mode
            #leave any non-masked bits untouched
            data_to_write = (existing & ~mask) | (control & mask)
            #write to the object and return status of write operation
            return self.write_object_by_name('control', data_to_write)
        else:
            #the read was no successful
            print('cannot read control register')
            return None

    def get_status(self):
        return self.read_object_by_name('status')

    def get_fault_status(self):
        error = self.read_object_by_name('error')
        if error != 0:
            return True
        else:
            return False

    def reset_fault(self):
        self.set_control('fault_low')
        self.set_control('fault_high')

    def set_peak_current(self, mA):
        #make sure mA is an integer
        mA_int = int(mA)
        self.write_object_by_name('current_limit', mA_int)
        #check it was accepted
        set_value = self.read_object_by_name('current_limit')
        assert set_value == mA_int, f'expected {mA_int} but got {set_value}'

    def get_peak_current(self):
        #returns peak current in mA
        mA = self.read_object_by_name('current_limit')
        return mA

    def get_actual_current(self):
        #returns actual current in mA
        mA = self.read_object_by_name('average_current')
        return mA

    def get_actual_velocity(self):
        #returns actual veolocity in rpm
        rpm = self.read_object_by_name('actual_velocity')
        return rpm

    def get_actual_voltage(self):
        dv = self.read_object_by_name('actual_voltage')
        return dv/10

    def get_actual_position(self):
        #returns actual position in counts
        counts = self.read_object_by_name('actual_position')
        return counts

    def get_temperature(self):
        #returns temperature in 0.1degree i.e. 322 represents 32.2C
        temp_deci = self.read_object_by_name('actual_temperature')
        return temp_deci

    def set_mode_of_operation(self, mode_text):
        #sets the mode of operation according to mode_text
        #make sure the mode text is a valid dictionary key
        assert mode_text in self.mode.keys(), f'bad key {mode_text}'
        #write to the modes register
        self.write_object_by_name('mode', self.mode[mode_text])
        #check mode is as we expect
        #read from the modes display register
        read_mode = self.read_object_by_name('mode_display')
        if read_mode == self.mode[mode_text]:
            return True
        else:
            return False

    def set_profile_velocity(self, rpm):
        #set the desired veolocity after acceleration ramp
        #make sure the rpm number is an integer
        #return the status of the write operation
        return self.write_object_by_name('profile_velocity', int(rpm))

    def set_profile_acceleration(self, rpmps):
        #set the desired veolocity after acceleration ramp
        #make sure the rpm number is an integer
        #return the status of the write operation
        return self.write_object_by_name('profile_acceleration', int(rpmps))

    def set_profile_deceleration(self, rpmps):
        #set the desired veolocity after acceleration ramp
        #make sure the rpm number is an integer
        #return the status of the write operation
        return self.write_object_by_name('profile_deceleration', int(rpmps))

    def set_target_velocity(self, rpm):
        #set the desired veolocity after acceleration ramp
        #make sure the rpm number is an integer
        #return the status of the write operation
        return self.write_object_by_name('target_velocity', int(rpm))

    def set_target_position(self, counts):
        return self.write_object_by_name('target_position', counts)

    def initialise(self):
        self.set_control('shutdown')
        self.set_control('switch_on')

    def enable(self):
        self.set_control('enable_operation')

    def disable(self):
        self.set_control('disable_operation')

    def home(self, method):
        #see table 6-153 in firmware guide
        #make an empty list of EPOS communication success
        comms = []
        self.set_mode_of_operation('HMM')
        if method == 'in_place':
            code = 37
            result = self.write_object_by_name('homing_method', code)
            comms.append(result)
            result = self.set_control('homing_start')
            comms.append(result)
            result = self.set_control('clear_homing_start')
            comms.append(result)
            if all(comms):
                pos = self.get_actual_position()
                if pos == 0:
                    return True
                else:
                    print(f'position not zero at {pos}')
                    return False
            else:
                print(f'homing object writes failed')
                return False
        elif method == 'crash':
            self.write_object_by_name('homing_method', -3)
            self.write_object_by_name('homing_current_limit', 1000)
            self.write_object_by_name('homing_speed_switch', 300)
            #self.set_control('start')
            self.set_control('start')
            self.set_control('clear_homing_start')
            self.set_control('homing_start')
        else:
            print(f'homing method {method} not supported')
            return False

    def homing_attained(self):
        mask = 0x8000
        if self.get_status() & mask > 0:
            return True
        else:
            return False

    def move_with_velocity(self, rpm):
        self.set_mode_of_operation('PVM')
        self.set_target_velocity(rpm)
        self.set_control('start')

    def move_to_position(self, counts, velocity=1000, accel=10000, immediate=False):
        self.set_mode_of_operation('PPM')
        self.set_profile_velocity(velocity)
        self.set_profile_acceleration(accel)
        self.set_profile_deceleration(accel)
        self.set_target_position(counts)
        self.set_control('absolute')
        if immediate:
            self.set_control('move_immediate')
        else:
            self.set_control('move_after_current')
        #the order of operations below is important
        self.set_control('start')
        self.set_control('new_setpoint')
        self.set_control('clear_new_setpoint')

    def stop(self):
        self.set_control('stop')

