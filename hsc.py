#!/usr/bin/python

from __future__ import division, print_function
try:
    import smbus
except:
    print('Try sudo apt-get install python-smbus')
import time

class HHSC(object):

    # Check sensor reference in the Honeywell order guide to get its specific parameters (I2C address, min/max pressure, unit, transfer function)
    def __init__(self, bus = 1, addr = 0x48, min_pressure = 0.0, max_pressure = 5.0, unit = 'mbar', transfer = 'A'):
        self.bus = None
        self.addr = addr
        self.min_pressure = float(min_pressure)
        self.max_pressure = float(max_pressure)
        self.unit = unit
        self.transfer = transfer
        self.output_min = 0.00*0x4000
        self.output_max = 1.00*0x4000
        if (self.transfer == 'A'):
            self.output_min = 0.10*0x4000
            self.output_max = 0.90*0x4000
        elif (self.transfer == 'B'):
            self.output_min = 0.05*0x4000
            self.output_max = 0.95*0x4000
        elif (self.transfer == 'C'):
            self.output_min = 0.05*0x4000
            self.output_max = 0.85*0x4000
        elif (self.transfer == 'F'):
            self.output_min = 0.04*0x4000
            self.output_max = 0.94*0x4000
        else:
            print('Unknown transfer function')
            return
        try:
            self.bus = smbus.SMBus(bus)
        except:
            print(('Bus %d is not available') % bus)
            return

    # Pressure in sensor unit
    def read_pressure(self):
        ans = self.bus.read_word_data(self.addr, 0x01) # Send I2C address and read bit, return two 8 bit bytes
        #print((ans & 0x003F))
        #print(((ans & 0xFF00) >> 8))
        #ans = (((ans & 0x003F) << 8) + ((ans & 0xFF00) >> 8))
        ans = (((ans & 0x003F) << 8) + (ans >> 8)) # Byte swap them and ignore status bits
        ans = (ans-self.output_min)*(self.max_pressure-self.min_pressure)/(self.output_max-self.output_min)+self.min_pressure
        return ans
 
    def conv_pressure_to_mbar(self, pressure_in_sensor_unit):
        output = 0
        if (self.unit.lower() == 'mbar'.lower()): output = pressure_in_sensor_unit
        elif (self.unit.lower() == 'bar'.lower()): output = pressure_in_sensor_unit*1000.0
        elif (self.unit.lower() == 'Pa'.lower()): output = pressure_in_sensor_unit*0.01
        elif (self.unit.lower() == 'kPa'.lower()): output = pressure_in_sensor_unit*10.0
        elif (self.unit.lower() == 'MPa'.lower()): output = pressure_in_sensor_unit*10000.0
        elif (self.unit.lower() == 'inH2O'.lower()): output = pressure_in_sensor_unit*2.4884
        elif (self.unit.lower() == 'psi'.lower()): output = pressure_in_sensor_unit*68.9476
        return output

    # Return pressure in mbar and temperature in C
    def read(self):
        ans = self.bus.read_i2c_block_data(self.addr, 0x01, 4) # Send I2C address and read bit, return four 8 bit bytes
        #print(ans)
        pres = ((ans[0] & 0x3F) << 8) + ans[1] # Byte swap them and ignore status bits
        pres = (pres-self.output_min)*(self.max_pressure-self.min_pressure)/(self.output_max-self.output_min)+self.min_pressure
        pres = self.conv_pressure_to_mbar(pres)
        #temp = (ans[2] << 3) + ((ans[3] & 0xE0) >> 5)
        temp = (ans[2] << 3) + (ans[3] >> 5) # Byte swap them and ignore 5 bits
        temp = temp*200.0/2047.0-50
        return pres, temp

if __name__ == "__main__":
    p_inspi_hsc = HHSC(bus = 3, addr = 0x48, min_pressure = -160.0, max_pressure = 160.0, unit = 'mbar', transfer = 'A')
    i = 0
    while True:        
        start_time = time.time()

        #print((p_inspi_hsc.conv_pressure_to_mbar(p_inspi_hsc.read_pressure())))
        pressure, temperature = p_inspi_hsc.read()
        print(('pressure = %0.5f mbar, temperature = %0.3f C') % (pressure, temperature))
        #time.sleep(1)        
        i = i+1

        end_time = time.time()
        print('It has been %0.4f seconds since the loop started' %(end_time - start_time))
