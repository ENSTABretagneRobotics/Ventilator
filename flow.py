import imp
import time
import math
import numpy as np
import sys
import os
import threading
from timeit import default_timer as timer
import rsc # From https://github.com/tin-/ascp

R1 = 0.019/2.0
R2 = 0.015/2.0

A1 = math.pi*R1**2
A2 = math.pi*R2**2

delay = 0.030

file = open('data_flow.csv', 'a')
file.write('t (in s);t0;flow (in m3/s);\n')

sensorrsc = rsc.HRSC(spi_bus=0)
#sensorrsc = rsc.HRSC(spi_bus=3)
time.sleep(0.1)
print('Read the ADC settings and the compensation values from EEPROM.')
sensorrsc.sensor_info()
time.sleep(0.1)
print('Initialize the ADC converter using the settings provided in EEPROM.')
sensorrsc.adc_configure()
time.sleep(0.1)
print('Adjust the ADC sample rate if desired.')
sensorrsc.set_speed(2000) #in SPS
time.sleep(0.1)
# Delay (Example: if sample rate is 330SPS delay for 3.03 ms [1/330 s])
print('Apply the compensation formulae to the temperature and pressure readings in order to calculate a pressure value.')

t = timer()
t_prev = t
t0 = t

raw_pressure = sensorrsc.read_pressure(delay)
time.sleep(delay)
raw_temp = sensorrsc.read_temp(delay)
time.sleep(delay)
pressure_rsc, temperature_rsc = sensorrsc.comp_readings(raw_pressure, raw_temp)
offset = pressure_rsc
nb_count = 2000

count = 0
while (True):
    dt = t-t_prev
    raw_pressure = sensorrsc.read_pressure(delay)
    if (count % 100 == 0):         
        raw_temp = sensorrsc.read_temp(delay)
    pressure_rsc, temperature_rsc = sensorrsc.comp_readings(raw_pressure, raw_temp)
    if (count < nb_count): offset = (1-0.95)*pressure_rsc+(0.95)*offset
    pressure_rsc = pressure_rsc-offset
    rho_air = 1.292*(273.15/(273.15+temperature_rsc)) # In Kg/m3
    velocity = np.sign(pressure_rsc)*math.sqrt(2*(np.abs(pressure_rsc)*248.84)/(rho_air*((A1/A2)**2-1)))
    flow = A1*velocity
    print('Flow : %0.2f L/min, velocity : %0.3f m/s, Pressure : %0.6f inchH2O, offset : %0.6f inchH2O, Temperature : %0.6f C, dt : %0.3f s' % (flow*60000, velocity, pressure_rsc, offset, temperature_rsc, dt))
    line = '{};{};{};\n'
    file.write(line.format(t, t0, flow))
    file.flush()
    count = count+1
    t_prev = t
    t = timer()
