import imp
import time
import math
import numpy as np
import sys
import os
import threading
from timeit import default_timer as timer

rho_air = 1.184
A1 = math.pi*(0.019/2.0)**2
A2 = math.pi*(0.015/2.0)**2

file = open('data.csv', 'a')
file.write('t (in s);t0;flow (in m3/s);\n')

sensorrs = imp.load_source('rsc', 'rsc.py') # From https://github.com/tin-/ascp
sensorrsc = sensorrs.HRSC()
print "Read the ADC settings and the compensation values from EEPROM."
sensorrsc.sensor_info()
print "Initialize the ADC converter using the settings provided in EEPROM."
sensorrsc.adc_configure()
print "Adjust the ADC sample rate if desired."
sensorrsc.set_speed(2000) #in SPS
# Delay (Example: if sample rate is 330SPS delay for 3.03 ms [1/330 s])
print "Apply the compensation formulae to the temperature and pressure readings in order to calculate a pressure value."

t = timer()
t_prev = t
t0 = t

pressure_rsc, temperature_rsc = sensorrsc.comp_readings(0.030)
offset = pressure_rsc
nb_count = 100

count = 0
while (True):
    pressure_rsc, temperature_rsc = sensorrsc.comp_readings(0.030)
    if (count < nb_count): offset = (1-1.0/nb_count)*pressure_rsc+(1.0/nb_count)*offset
    pressure_rsc = pressure_rsc-offset
    velocity = np.sign(pressure_rsc)*math.sqrt(2*(np.abs(pressure_rsc)*248.84)/(rho_air*((A1/A2)**2-1)))
    flow = A1*velocity
    print('Flow : %0.2f L/min, velocity : %0.3f m/s, Pressure : %0.6f inchH2O, offset : %0.6f inchH2O, Temperature : %0.6f C' % (flow*60000, velocity, pressure_rsc, offset, temperature_rsc))
    line = '{};{};{};\n'
    file.write(line.format(t, t0, flow))
    file.flush()
    count = count+1
    t_prev = t
    t = timer()
