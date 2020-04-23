from __future__ import division
try:
    import smbus
except:
    print('Try sudo apt-get install python-smbus')

class HHSC(object):

    # Check sensor reference in the Honeywell order guide to get its specific parameters (I2C address, min/max pressure, unit, transfer function)
    def __init__(self, bus = 1, addr = 0x48, min_pressure = 0.0, max_pressure = 5.0, unit = 'mbar', transfer = 'A'):
        self.bus = None
        self.addr = addr
        self.min_pressure = float(min_pressure)
        self.max_pressure = float(max_pressure)
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
            print('Bus %d is not available') % bus
            return

    # In C
    def read_temperature(self):
        # From https://www.raspberrypi.org/forums/viewtopic.php?t=33334
        ans = self.bus.read_word_data(self._addr, 0x03) # Send I2C address and read bit, return two 8 bit bytes
        ans = (((ans & 0x00FF) << 8) + ((ans & 0xFF00) >> 8)) # Byte swap them
        ans = ans*200.0/2047.0/32.0-50
        return ans

    # In sensor unit
    def read_pressure(self):
        ans = self.bus.read_word_data(self._addr, 0x01) # Send I2C address and read bit, return two 8 bit bytes
        ans = (((ans & 0x00FF) << 8) + ((ans & 0xFF00) >> 8)) # Byte swap them
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
        ans = [0] * 4
        ans = self.bus.read_block_data(self._addr, 0x01) # Send I2C address and read bit, return four 8 bit bytes
        pres = (ans[0] << 8) + ans[1]
        pres = (pres-self.output_min)*(self.max_pressure-self.min_pressure)/(self.output_max-self.output_min)+self.min_pressure
        pres = conv_pressure_to_mbar(self, pres)
        temp = (ans[2] << 8) + ans[3]
        temp = temp*200.0/2047.0/32.0-50
        return pres, temp
