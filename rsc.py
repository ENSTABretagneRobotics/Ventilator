#!/usr/bin/python

# Copyright (c) 2017 xDevs.com
# Author: Illya Tsemenko
#
# Based on the RSC datasheet
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from __future__ import print_function
from __future__ import division
import time
import struct
import spidev

# HRSC Commands
HRSC_SINGLE_READ_CMD = 0xAA
HRSC_AVG2_READ_CMD   = 0xAC
HRSC_AVG4_READ_CMD   = 0xAD
HRSC_AVG8_READ_CMD   = 0xAE
HRSC_AVG16_READ_CMD  = 0xAF

# HRSC Status bits
HRSC_STS_ZERO          = 0b10000000
HRSC_STS_PWRON         = 0b01000000
HRSC_STS_BUSY          = 0b00100000
HRSC_STS_MODE          = 0b00011000
HRSC_STS_EEPROM_CHKERR = 0b00000100
HRSC_STS_SNSCFG        = 0b00000010
HRSC_STS_ALUERR        = 0b00000001

HRSC_EAD_EEPROM_LSB    = 0x03
HRSC_EAD_EEPROM_MSB    = 0x0B
HRSC_ADC_WREG          = 0x40 
HRSC_ADC_RESET         = 0x06

#Prange   = 0.0                  # Pressure range from ROM 
#Pmin     = 0.0                  # Pressure offset from ROM
#EngUnit  = 0.0                  # Engineering units from ROM
#Praw     = 0.0                  # Uncompensated pressure from ADC
#Traw     = 0.0                  # Uncompensated temperature from ADC
#Pint1    = 0.0                  # Intermediate value 1
#Pint2    = 0.0                  # Intermediate value 2
#Pcomp_fs = 0.0                  # Compensated output pressure 
#Pcomp    = 0.0                  # Compensated output pressure, in units

class HRSC(object):

    def __init__(self, spi_bus = 0, spi_speed_hz = 1250000, spi_max_speed_hz = 1250000, force_spi_mode = True, EE_spi_mode = 0b00, alt_spi_cs0 = -1, alt_spi_cs1 = -1, **kwargs):
        # Create device.
        self.spi_bus = spi_bus
        self.spi_speed_hz = spi_speed_hz
        self.spi_max_speed_hz = spi_max_speed_hz
        self.force_spi_mode = force_spi_mode
        self.EE_spi_mode = EE_spi_mode
        self.alt_spi_cs0 = alt_spi_cs0
        self.alt_spi_cs1 = alt_spi_cs1
        self.spi = spidev.SpiDev()
        self.spi.open(self.spi_bus, 1)
        self.spi.max_speed_hz = self.spi_max_speed_hz
        if self.force_spi_mode: self.spi.mode = self.EE_spi_mode
        if ((self.alt_spi_cs0 >= 0) or (self.alt_spi_cs1 >= 0)): import RPi.GPIO as GPIO
        if (self.alt_spi_cs1 >= 0): GPIO.setup(self.alt_spi_cs1, GPIO.OUT, initial = GPIO.LOW)
        if (self.alt_spi_cs0 >= 0): GPIO.setup(self.alt_spi_cs0, GPIO.OUT, initial = GPIO.HIGH)
        self.spi.close()
        self.spi.open(self.spi_bus, 0)
        self.spi.max_speed_hz = self.spi_max_speed_hz
        if self.force_spi_mode: self.spi.mode = 0b01
        if (self.alt_spi_cs0 >= 0): GPIO.setup(self.alt_spi_cs0, GPIO.OUT, initial = GPIO.LOW)
        if (self.alt_spi_cs1 >= 0): GPIO.setup(self.alt_spi_cs1, GPIO.OUT, initial = GPIO.HIGH)
        self.spi.close()
        self.reg_dr = 0
        self.reg_mode = 0
        # HRSC ROM values
        self.sensor_rom = [0] * 512 # 512 bytes of EEPROM
        self.read_eeprom()
        self.PCompr = 0.0
        self.raw_auto_zero_pressure = 0.0
    
    def read_eeprom(self):
        #print "Loading EEPROM data from sensor",
        #print ".",
        # Assert EEPROM SS to L, Deassert ADC SS to H, Set mode 0 or mode 4
        self.spi.open(self.spi_bus, 1)
        self.spi.max_speed_hz = self.spi_max_speed_hz
        if self.force_spi_mode: self.spi.mode = self.EE_spi_mode
        if (self.alt_spi_cs1 >= 0): GPIO.setup(self.alt_spi_cs1, GPIO.OUT, initial = GPIO.LOW)
        if (self.alt_spi_cs0 >= 0): GPIO.setup(self.alt_spi_cs0, GPIO.OUT, initial = GPIO.HIGH)
        for i in range (0,256):
            self.sensor_rom[i] = self.spi.xfer([HRSC_EAD_EEPROM_LSB, i, 0x00], self.spi_speed_hz)[2] # Read low page
#            print "%c" % (self.sensor_rom[i]),
        for i in range (0,256):
            self.sensor_rom[i+256] = self.spi.xfer([HRSC_EAD_EEPROM_MSB, i, 0x00], self.spi_speed_hz)[2] # Read high page
        # Clear EEPROM SS , set mode 1 for ADC
        self.spi.close()
       
    def reset(self):
        # Clear EEPROM SS , assert ADC SS, set mode 1 for ADC
        self.spi.open(self.spi_bus, 0)
        self.spi.max_speed_hz = self.spi_max_speed_hz
        if self.force_spi_mode: self.spi.mode = 0b01
        if (self.alt_spi_cs0 >= 0): GPIO.setup(self.alt_spi_cs0, GPIO.OUT, initial = GPIO.LOW)
        if (self.alt_spi_cs1 >= 0): GPIO.setup(self.alt_spi_cs1, GPIO.OUT, initial = GPIO.HIGH)
        self.bytewr = 3
        self.regaddr = 0
        # Reset command
        test = self.spi.xfer([HRSC_ADC_RESET], self.spi_speed_hz)
        self.spi.close()
        
    def adc_configure(self):
        # Clear EEPROM SS , assert ADC SS, set mode 1 for ADC
        self.spi.open(self.spi_bus, 0)
        self.spi.max_speed_hz = self.spi_max_speed_hz
        if self.force_spi_mode: self.spi.mode = 0b01
        if (self.alt_spi_cs0 >= 0): GPIO.setup(self.alt_spi_cs0, GPIO.OUT, initial = GPIO.LOW)
        if (self.alt_spi_cs1 >= 0): GPIO.setup(self.alt_spi_cs1, GPIO.OUT, initial = GPIO.HIGH)
        self.bytewr = 3
        self.regaddr = 0
        # Write configuration registers from ROM
        #print ("%02X" % self.sensor_rom[61]),
        #print ("%02X" % self.sensor_rom[63]),
        #print ("%02X" % self.sensor_rom[65]),
        #print ("%02X" % self.sensor_rom[67]),
        test = self.spi.xfer2([HRSC_ADC_WREG|self.regaddr << 3|self.bytewr & 0x03, self.sensor_rom[61], self.sensor_rom[63], self.sensor_rom[65], self.sensor_rom[67] ], self.spi_speed_hz)        
        self.spi.close()

    def conv_to_float(self, byte1, byte2, byte3, byte4):
        temp = struct.pack("BBBB", byte1,byte2,byte3,byte4)
        output = struct.unpack("<f", temp)[0]
        return output

    def conv_to_short(self, byte1, byte2):
        temp = struct.pack("BB", byte1,byte2)
        output = struct.unpack("<H", temp)[0]
        return output
    
    def sensor_info(self):
        # Check for correct status
        print(('\033[0;32mCatalog listing : %s' % bytearray(self.sensor_rom[0:16]).decode("utf-8")))
        print(('Serial number   : %s' % bytearray(self.sensor_rom[16:27]).decode("utf-8")))
        print('Pressure range  :', end = ' ')
        b = self.conv_to_float(self.sensor_rom[27], self.sensor_rom[28], self.sensor_rom[29], self.sensor_rom[30])
        print((b, self.sensor_rom[27:31]))
        print('Pressure min    :', end = ' ')
        b = self.conv_to_float(self.sensor_rom[31], self.sensor_rom[32], self.sensor_rom[33], self.sensor_rom[34])
        print((b, self.sensor_rom[31:35]))                
        print(('Pressure units  : %s' % bytearray(self.sensor_rom[35:40]).decode("utf-8")))
        if (self.sensor_rom[40] == 68):
            print('Pressure ref    : Differential')
        else:
            print('Pressure ref    : Absolute')
        print('Checksum        :', end = ' ')
        b = self.conv_to_short(self.sensor_rom[450], self.sensor_rom[451])
        print((b, self.sensor_rom[450:452]))
        print('\033[0;39m')
   
    def set_speed(self, data_rate): # In SPS
        # Clear EEPROM SS , assert ADC SS, set mode 1 for ADC
        self.spi.open(self.spi_bus, 0)
        self.spi.max_speed_hz = self.spi_max_speed_hz
        if self.force_spi_mode: self.spi.mode = 0b01
        if (self.alt_spi_cs0 >= 0): GPIO.setup(self.alt_spi_cs0, GPIO.OUT, initial = GPIO.LOW)
        if (self.alt_spi_cs1 >= 0): GPIO.setup(self.alt_spi_cs1, GPIO.OUT, initial = GPIO.HIGH)
        self.bytewr = 0
        self.regaddr = 1
        if (data_rate == 20):
            self.reg_dr = 0
            self.reg_mode = 0
        elif (data_rate == 45):
            self.reg_dr = 1
            self.reg_mode = 0
        elif (data_rate == 90):
            self.reg_dr = 2
            self.reg_mode = 0
        elif (data_rate == 175):
            self.reg_dr = 3
            self.reg_mode = 0
        elif (data_rate == 330):
            self.reg_dr = 4
            self.reg_mode = 0
        elif (data_rate == 600):
            self.reg_dr = 5
            self.reg_mode = 0
        elif (data_rate == 1000):
            self.reg_dr = 6
            self.reg_mode = 0
        elif (data_rate == 40):
            self.reg_dr = 8
            self.reg_mode = 2
        elif (data_rate == 90):
            self.reg_dr = 9
            self.reg_mode = 2
        elif (data_rate == 180):
            self.reg_dr = 10
            self.reg_mode = 2
        elif (data_rate == 350):
            self.reg_dr = 11
            self.reg_mode = 2
        elif (data_rate == 660):
            self.reg_dr = 12
            self.reg_mode = 2
        elif (data_rate == 1200):
            self.reg_dr = 13
            self.reg_mode = 2
        elif (data_rate == 2000):
            self.reg_dr = 14
            self.reg_mode = 2
        #reg_sensor = 0 # pressure
        #self.reg_wr = (self.reg_dr << 5) | (self.reg_mode << 3) | (1 << 2) | (reg_sensor << 1) | 0b00
        ## Write configuration register
        #self.command = HRSC_ADC_WREG|(self.regaddr << 2)|(self.bytewr & 0x03)
        ##print(('\033[0;36mADC config %02X : %02X\033[0;39m') % (self.command, self.reg_wr))
        #test = self.spi.xfer([self.command, self.reg_wr], self.spi_speed_hz)
        #self.spi.close()

    def convert_temp(self, raw_temp):
        raw = (raw_temp & 0xFFFF00) >> 10
        #raw = raw_temp
        if (raw & 0x2000):
            #print('MSB is 1, negative temp')
            raw = (0x3fff - (raw - 1))
            temp = -(float(raw) * 0.03125)
        else:
            #print('MSB is 0, positive temp')
            temp = (float(raw) * 0.03125)
        #print(('RAW: %s %s , %4.3f') % (hex(raw_temp), hex(raw), temp))
        return temp
    
    def temperature_request(self):
        # Clear EEPROM SS , assert ADC SS, set mode 1 for ADC
        self.spi.open(self.spi_bus, 0)
        self.spi.max_speed_hz = self.spi_max_speed_hz
        if self.force_spi_mode: self.spi.mode = 0b01
        if (self.alt_spi_cs0 >= 0): GPIO.setup(self.alt_spi_cs0, GPIO.OUT, initial = GPIO.LOW)
        if (self.alt_spi_cs1 >= 0): GPIO.setup(self.alt_spi_cs1, GPIO.OUT, initial = GPIO.HIGH)
        self.bytewr = 0
        self.regaddr = 1
        reg_sensor = 1 # temperature
        self.reg_wr = (self.reg_dr << 5) | (self.reg_mode << 3) | (1 << 2) | (reg_sensor << 1) | 0b00
        # Write configuration register
        self.command = HRSC_ADC_WREG|(self.regaddr << 2)|(self.bytewr & 0x03)
        #print(('\033[0;36mADC config %02X : %02X\033[0;39m') % (self.command, self.reg_wr))
        test = self.spi.xfer([self.command, self.reg_wr], self.spi_speed_hz)
    
    def temperature_reply(self):
        adc_data = self.spi.xfer([0,0,self.command, self.reg_wr], self.spi_speed_hz)
        #print "RDATA = ",
        #print adc_data
        temperature_data = (adc_data[0]<<16|adc_data[1]<<8|adc_data[2])
        tempval = self.convert_temp(temperature_data)

        # first 14 bits represent temperature
        # following 10 bits are random thus discarded
        #_t_raw = ((adc_data[0] << 8) | adc_data[1]) >> 2;

        self.spi.close()
        return tempval
        #return _t_raw
    
    def read_temperature(self, delay = 0.050):
        self.temperature_request()
        time.sleep(delay)
        return self.temperature_reply()

    def twos_complement(self, byte_arr):
        a = byte_arr[0]; b = byte_arr[1]; c = byte_arr[2]
        out = ((a<<16)&0xff0000) | ((b<<8)&0xff00) | (c&0xff)
        neg = (a & (1<<7) != 0)  # first bit of a is the "signed bit." if it's a 1, then the value is negative
        if neg: out -= (1 << 24)
        #print(hex(a), hex(b), hex(c), neg, out)
        return out

    def pressure_request(self):
        # Clear EEPROM SS , assert ADC SS, set mode 1 for ADC
        self.spi.open(self.spi_bus, 0)
        self.spi.max_speed_hz = self.spi_max_speed_hz
        if self.force_spi_mode: self.spi.mode = 0b01
        if (self.alt_spi_cs0 >= 0): GPIO.setup(self.alt_spi_cs0, GPIO.OUT, initial = GPIO.LOW)
        if (self.alt_spi_cs1 >= 0): GPIO.setup(self.alt_spi_cs1, GPIO.OUT, initial = GPIO.HIGH)
        self.bytewr = 0
        self.regaddr = 1
        reg_sensor = 0 # Pressure readout
        self.reg_wr = (self.reg_dr << 5) | (self.reg_mode << 3) | (1 << 2) | (reg_sensor << 1) | 0b00
        # Write configuration register
        self.command = HRSC_ADC_WREG|(self.regaddr << 2)|(self.bytewr & 0x03)
        #print(('\033[0;36mADC config %02X : %02X\033[0;39m') % (self.command, self.reg_wr))
        test = self.spi.xfer([self.command, self.reg_wr], self.spi_speed_hz)

    def pressure_reply(self):
        adc_data = self.spi.xfer([0,0, self.command, self.reg_wr], self.spi_speed_hz)
        #print hex(adc_data[0]),hex(adc_data[1]),hex(adc_data[2]),
        press = (self.twos_complement(adc_data)) # 24-bit 2's complement math
        #print float(press) / pow(2,23)
        self.spi.close()
        if ((adc_data[0]<<16|adc_data[1]<<8|adc_data[2]) > 0x7FFFFF):
             #print('NEgative')
             pdatad = ((adc_data[0]<<16|adc_data[1]<<8|adc_data[2])) - 0x1000000
             return pdatad
        else: # ((adc_data[0]<<16|adc_data[1]<<8|adc_data[2]) < 0x1000000):
             #print('POSiTive')
             pdatad = (adc_data[0]<<16|adc_data[1]<<8|adc_data[2])
             return pdatad

    def read_pressure(self, delay = 0.050):
        self.pressure_request()
        time.sleep(delay)
        return self.pressure_reply()
        
    def comp_auto_zero(self, raw_pressure, temperature):
        OffsetCoefficient0 = self.conv_to_float(self.sensor_rom[130], self.sensor_rom[131], self.sensor_rom[132], self.sensor_rom[133])
        OffsetCoefficient1 = self.conv_to_float(self.sensor_rom[134], self.sensor_rom[135], self.sensor_rom[136], self.sensor_rom[137])
        OffsetCoefficient2 = self.conv_to_float(self.sensor_rom[138], self.sensor_rom[139], self.sensor_rom[140], self.sensor_rom[141])
        OffsetCoefficient3 = self.conv_to_float(self.sensor_rom[142], self.sensor_rom[143], self.sensor_rom[144], self.sensor_rom[145])
        raw_temp = temperature/0.03125
        raw_temp2 = raw_temp * raw_temp
        raw_temp3 = raw_temp2 * raw_temp
        self.raw_auto_zero_pressure = ( (OffsetCoefficient3 * raw_temp3) + (OffsetCoefficient2 * raw_temp2) + (OffsetCoefficient1 * raw_temp) + OffsetCoefficient0 ) - raw_pressure
        
    def comp_readings(self, raw_pressure, temperature):
        OffsetCoefficient0 = self.conv_to_float(self.sensor_rom[130], self.sensor_rom[131], self.sensor_rom[132], self.sensor_rom[133])
        OffsetCoefficient1 = self.conv_to_float(self.sensor_rom[134], self.sensor_rom[135], self.sensor_rom[136], self.sensor_rom[137])
        OffsetCoefficient2 = self.conv_to_float(self.sensor_rom[138], self.sensor_rom[139], self.sensor_rom[140], self.sensor_rom[141])
        OffsetCoefficient3 = self.conv_to_float(self.sensor_rom[142], self.sensor_rom[143], self.sensor_rom[144], self.sensor_rom[145])
        SpanCoefficient0   = self.conv_to_float(self.sensor_rom[210], self.sensor_rom[211], self.sensor_rom[212], self.sensor_rom[213])
        SpanCoefficient1   = self.conv_to_float(self.sensor_rom[214], self.sensor_rom[215], self.sensor_rom[216], self.sensor_rom[217])
        SpanCoefficient2   = self.conv_to_float(self.sensor_rom[218], self.sensor_rom[219], self.sensor_rom[220], self.sensor_rom[221])
        SpanCoefficient3   = self.conv_to_float(self.sensor_rom[222], self.sensor_rom[223], self.sensor_rom[224], self.sensor_rom[225])
        ShapeCoefficient0  = self.conv_to_float(self.sensor_rom[290], self.sensor_rom[291], self.sensor_rom[292], self.sensor_rom[293])
        ShapeCoefficient1  = self.conv_to_float(self.sensor_rom[294], self.sensor_rom[295], self.sensor_rom[296], self.sensor_rom[297]) 
        ShapeCoefficient2  = self.conv_to_float(self.sensor_rom[298], self.sensor_rom[299], self.sensor_rom[300], self.sensor_rom[301])
        ShapeCoefficient3  = self.conv_to_float(self.sensor_rom[302], self.sensor_rom[303], self.sensor_rom[304], self.sensor_rom[305])
        PRange = self.conv_to_float(self.sensor_rom[27], self.sensor_rom[28], self.sensor_rom[29], self.sensor_rom[30]) # Pressure range from ROM
        Pmin = self.conv_to_float(self.sensor_rom[31], self.sensor_rom[32], self.sensor_rom[33], self.sensor_rom[34]) # Pressure offset from ROM
        #print "\033[0;33mOffsets\033[0;39m",OffsetCoefficient0,OffsetCoefficient1,OffsetCoefficient2,OffsetCoefficient3
        #print "\033[0;33mSpan",SpanCoefficient0,SpanCoefficient1,SpanCoefficient2,SpanCoefficient3
        #print "\033[0;33mShape",ShapeCoefficient0,ShapeCoefficient1,ShapeCoefficient2,ShapeCoefficient3
        #print "\033[0;33mPrange",PRange
        #print "\033[0;33mPmin",Pmin

        raw_temp = temperature/0.03125
        raw_temp2 = raw_temp * raw_temp
        raw_temp3 = raw_temp2 * raw_temp

        #Pint1 = (self.read_pressure(delay) ) - ( (OffsetCoefficient3 * raw_temp3) + (OffsetCoefficient2 * raw_temp2) + (OffsetCoefficient1 * raw_temp) + OffsetCoefficient0 )
        Pint1 = raw_pressure + self.raw_auto_zero_pressure - ( (OffsetCoefficient3 * raw_temp3) + (OffsetCoefficient2 * raw_temp2) + (OffsetCoefficient1 * raw_temp) + OffsetCoefficient0 ) # Intermediate value 1
        Pint2 = Pint1 / ((SpanCoefficient3 * raw_temp3) + (SpanCoefficient2 * raw_temp2) + (SpanCoefficient1 * raw_temp) + SpanCoefficient0) # Intermediate value 2

        Ptemp2 = (Pint2 * Pint2)
        Ptemp3 = (Ptemp2 * Pint2)

        PComp_FS = (ShapeCoefficient3 * Ptemp3) + (ShapeCoefficient2 * Ptemp2) + (ShapeCoefficient1 * Pint2) + ShapeCoefficient0 # Compensated output pressure, in units
        self.PCompr = (PComp_FS * PRange) + Pmin #[Engineering Units]
        #print "\033[1;33mPComp out = %f" % self.PCompr
        return self.PCompr, temperature
 
    def conv_pressure_to_mbar(self, pressure_in_sensor_unit):
        sensor_unit = bytearray(self.sensor_rom[35:40]).decode("utf-8")
        output = 0
        if (sensor_unit.lower() == 'mbar'.lower()): output = pressure_in_sensor_unit
        elif (sensor_unit.lower() == 'bar'.lower()): output = pressure_in_sensor_unit*1000.0
        elif (sensor_unit.lower() == 'Pa'.lower()): output = pressure_in_sensor_unit*0.01
        elif (sensor_unit.lower() == 'kPa'.lower()): output = pressure_in_sensor_unit*10.0
        elif (sensor_unit.lower() == 'MPa'.lower()): output = pressure_in_sensor_unit*10000.0
        elif (sensor_unit.lower() == 'inH2O'.lower()): output = pressure_in_sensor_unit*2.4884
        elif (sensor_unit.lower() == 'psi'.lower()): output = pressure_in_sensor_unit*68.9476
        return output

    # DO NOT USE, TO TRY TO REPAIR ERASED DEVICES...
    def dump_eeprom(self):
        file = open('eeprom_rsc.txt', 'w')        
        for byte in self.sensor_rom:
            file.write('{:d} '.format(byte))
        file.close()

    # DO NOT USE, TO TRY TO REPAIR ERASED DEVICES...
    def erase_eeprom(self):
        file = open('eeprom_rsc.txt', 'r')       
        data = file.readlines()
        i = 0
        cols = data[0].split(' ')
        for val in cols:
            #print((int(val)))
            self.sensor_rom[i] = int(val)
            i = i+1
            if i >= len(self.sensor_rom): break
        file.close()

if __name__ == "__main__":
    flow_air_rsc = HRSC(spi_bus = 0)
    flow_air_rsc.sensor_info()
    flow_air_rsc.reset()
    time.sleep(0.005)
    flow_air_rsc.adc_configure()
    flow_air_rsc.set_speed(175)
    time.sleep(0.005)
    i = 0
    while True:        
        flow_air_rsc.pressure_request()
        time.sleep(0.05)
        raw_pressure_air = flow_air_rsc.pressure_reply()
        flow_air_rsc.temperature_request()
        time.sleep(0.05)
        raw_temperature_air = flow_air_rsc.temperature_reply()
        pressure_air, temperature_air = flow_air_rsc.comp_readings(raw_pressure_air, raw_temperature_air)
        print(('pressure_air = %0.5f mbar, temperature_air = %0.3f C') % (flow_air_rsc.conv_pressure_to_mbar(pressure_air), temperature_air))
        time.sleep(1)        
        i = i+1
