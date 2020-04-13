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

    def __init__(self, spi_bus=0, **kwargs):
        # Create device.
        self.spi_bus = spi_bus
        self.spi = spidev.SpiDev()
        self.spi.open(self.spi_bus, 1)
        self.spi.mode = 0b00
        self.spi.close()
        self.spi.open(self.spi_bus, 0)
        self.spi.mode = 0b01
        self.spi.close()
        self.reg_dr = 0
        self.reg_mode = 2
        # HRSC ROM values
        self.sensor_rom = [0] * 512 # 512 bytes of EEPROM
        self.read_eeprom()
        # Load calibration values.
        #self._load_calibration()
        #self.t_fine = 0.0
    
    def read_eeprom(self):
        #print "Loading EEPROM data from sensor",
        #print ".",
        # Assert EEPROM SS to L, Deassert ADC SS to H, Set mode 0 or mode 4
        self.spi.open(self.spi_bus, 1)
        self.spi.mode = 0b00
        for i in range (0,256):
            self.sensor_rom[i] = self.spi.xfer([HRSC_EAD_EEPROM_LSB, i, 0x00], 10000)[2] # Read low page
#            print "%c" % (self.sensor_rom[i]),
        for i in range (0,256):
            self.sensor_rom[i+256] = self.spi.xfer([HRSC_EAD_EEPROM_MSB, i, 0x00], 10000)[2] # Read high page
        # Clear EEPROM SS , set mode 1 for ADC
        self.spi.close()
        return 0
        
    def adc_configure(self):
        # Clear EEPROM SS , assert ADC SS, set mode 1 for ADC
        self.spi.open(self.spi_bus, 0)
        self.spi.mode = 1
        self.bytewr = 3
        self.regaddr = 0
        # Reset command
        test = self.spi.xfer([HRSC_ADC_RESET], 10000)
        time.sleep(1)
        # Write configuration registers from ROM
        #print ("%02X" % self.sensor_rom[61]),
        #print ("%02X" % self.sensor_rom[63]),
        #print ("%02X" % self.sensor_rom[65]),
        #print ("%02X" % self.sensor_rom[67]),
        test = self.spi.xfer2([HRSC_ADC_WREG|self.regaddr << 3|self.bytewr & 0x03, self.sensor_rom[61], self.sensor_rom[63], self.sensor_rom[65], self.sensor_rom[67] ], 10000)        
        self.spi.close()
        return 1

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
        print('\033[0;32mCatalog listing : %s' % str(bytearray(self.sensor_rom[0:16])))
        print('Serial number   : %s' % str(bytearray(self.sensor_rom[16:27])))
        print('Pressure range  :')
        b = self.conv_to_float(self.sensor_rom[27], self.sensor_rom[28], self.sensor_rom[29], self.sensor_rom[30])
        print(b, self.sensor_rom[27:31])
        print('Pressure min    :')
        b = self.conv_to_float(self.sensor_rom[31], self.sensor_rom[32], self.sensor_rom[33], self.sensor_rom[34])
        print(b, self.sensor_rom[31:35])                
        print('Pressure units  : %s' % str(bytearray(self.sensor_rom[35:40])))
        if (self.sensor_rom[40] == 68):
            print('Pressure ref    : Differential')
        else:
            print('Pressure ref    : Absolute')
        print('Checksum        :')
        b = self.conv_to_short(self.sensor_rom[450], self.sensor_rom[451])
        print(b, self.sensor_rom[450:452])
        print('\033[0;39m')
        return 1
    
    def set_speed(self, speed):
        # Clear EEPROM SS , assert ADC SS, set mode 1 for ADC
        self.spi.open(self.spi_bus, 0)
        self.spi.mode = 1
        self.bytewr = 0
        self.regaddr = 1
        if (speed == 20):
            self.reg_dr = 0
        elif (speed == 45):
            self.reg_dr = 1
        elif (speed == 90):
            self.reg_dr = 2
        elif (speed == 175):
            self.reg_dr = 3
        elif (speed == 330):
            self.reg_dr = 4
        elif (speed == 600):
            self.reg_dr = 5
        elif (speed == 1000):
            self.reg_dr = 6
        reg_sensor = 0 # pressure
        self.reg_wr = (self.reg_dr << 5) | (self.reg_mode << 3) | (1 << 2) | (reg_sensor << 1) | 0b00
        # Write configuration register
        self.command = HRSC_ADC_WREG|(self.regaddr << 2)|(self.bytewr & 0x03)
        #print ("\033[0;36mADC config %02X : %02X\033[0;39m" % (self.command, self.reg_wr))
        time.sleep(0.1)
        test = self.spi.xfer([self.command, self.reg_wr], 10000)
        self.spi.close()
        return 1

    def convert_temp(self, raw_temp):
        raw = (raw_temp & 0xFFFF00) >> 10
        if (raw & 0x2000):
            #print "MSB is 1, negative temp"
            raw = (0x3fff - (raw - 1))
            temp = -(float(raw) * 0.03125)
        else:
            #print "MSB is 0, positive temp"
            temp = (float(raw) * 0.03125)
        #print "RAW: %s %s , %4.3f" % (hex(raw_temp), hex(raw), temp )
#        with open('rsc_temp.dsv', 'ab') as o:
#            o.write (time.strftime("%d/%m/%Y-%H:%M:%S;")+('%4.5f;%3.3f;\r\n' % (0.0, temp)))
        return temp
    
    def temp_request(self):
        # Clear EEPROM SS , assert ADC SS, set mode 1 for ADC
        self.spi.open(self.spi_bus, 0)
        self.spi.mode = 1
        self.bytewr = 0
        self.regaddr = 1
        self.spi.max_speed_hz = 10000

        reg_sensor = 1 # temperature
        self.reg_wr = (self.reg_dr << 5) | (self.reg_mode << 3) | (1 << 2) | (reg_sensor << 1) | 0b00
        # Write configuration register
        self.command = HRSC_ADC_WREG|(self.regaddr << 2)|(self.bytewr & 0x03)
        #print ("\033[0;36mADC config %02X : %02X\033[0;39m" % (self.command, self.reg_wr))
        test = self.spi.xfer([self.command, self.reg_wr], 10000)
    
    def temp_reply(self):
        adc_data = self.spi.xfer([0,0,self.command, self.reg_wr], 10000)
        #print "RDATA = ",
        #print adc_data
        temp_data = (adc_data[0]<<16|adc_data[1]<<8|adc_data[2])
        tempval = self.convert_temp(temp_data)
        self.spi.close()
        return tempval
    
    def read_temp(self, delay):
        self.temp_request()
        time.sleep(delay)
        return self.temp_reply()

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
        self.spi.mode = 1
        self.bytewr = 0
        self.regaddr = 1

        reg_sensor = 0 # Pressure readout
        self.reg_wr = (self.reg_dr << 5) | (self.reg_mode << 3) | (1 << 2) | (reg_sensor << 1) | 0b00
        # Write configuration register
        self.command = HRSC_ADC_WREG|(self.regaddr << 2)|(self.bytewr & 0x03)
        #print ("\033[0;36mADC config %02X : %02X\033[0;39m" % (self.command, self.reg_wr))
        test = self.spi.xfer([self.command, self.reg_wr], 10000)

    def pressure_reply(self):
        adc_data = self.spi.xfer([0,0, self.command, self.reg_wr], 10000)
        #print hex(adc_data[0]),hex(adc_data[1]),hex(adc_data[2]),
        press = (self.twos_complement(adc_data)) # 24-bit 2's complement math
        #print float(press) / pow(2,23)
        self.spi.close()
        if ((adc_data[0]<<16|adc_data[1]<<8|adc_data[2]) > 0x7FFFFF):
        #    print "NEgative"
             pdatad = ((adc_data[0]<<16|adc_data[1]<<8|adc_data[2])) - 0x1000000
             return pdatad
        else: # ((adc_data[0]<<16|adc_data[1]<<8|adc_data[2]) < 0x1000000):
        #    print "POSiTive"
             pdatad = (adc_data[0]<<16|adc_data[1]<<8|adc_data[2])
             return pdatad

    def read_pressure(self, delay):
        self.pressure_request()
        time.sleep(delay)
        return self.pressure_reply()
        
    def comp_readings(self, raw_pressure, raw_temp):
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

        raw_temp2 = raw_temp * raw_temp
        raw_temp3 = raw_temp2 * raw_temp

        #print ("Temps ",raw_temp,raw_temp2,raw_temp3)

        #print "Pressure data = %X %d " % (self.read_pressure(), self.read_pressure())
        #Pint1 = (self.read_pressure(delay) ) - ( (OffsetCoefficient3 * raw_temp3) + (OffsetCoefficient2 * raw_temp2) + (OffsetCoefficient1 * raw_temp) + OffsetCoefficient0 )
        Pint1 = raw_pressure - ( (OffsetCoefficient3 * raw_temp3) + (OffsetCoefficient2 * raw_temp2) + (OffsetCoefficient1 * raw_temp) + OffsetCoefficient0 ) # Intermediate value 1
        #print "Pint1", Pint1
        Pint2 = Pint1 / ((SpanCoefficient3 * raw_temp3) + (SpanCoefficient2 * raw_temp2) + (SpanCoefficient1 * raw_temp) + SpanCoefficient0) # Intermediate value 2
        #print "Pint2", Pint2

        Ptemp2 = (Pint2 * Pint2)
        Ptemp3 = (Ptemp2 * Pint2)

        PComp_FS = (ShapeCoefficient3 * Ptemp3) + (ShapeCoefficient2 * Ptemp2) + (ShapeCoefficient1 * Pint2) + ShapeCoefficient0 # Compensated output pressure, in units
        #print "PComp_FS", PComp_FS
        self.PCompr = (PComp_FS * PRange) + Pmin #[Engineering Units]
        #print "\033[1;33mPComp out = %f" % self.PCompr
        tdata = float( (PComp_FS * PRange) + Pmin )
        return tdata, raw_temp
        