import imp
import time
import math
import numpy as np
import sys
import os
import threading
from timeit import default_timer as timer
import rsc # From https://github.com/tin-/ascp

R1 = 0.019100/2.0
R2 = 0.011651/2.0

A1 = math.pi*R1**2
A2 = math.pi*R2**2

delay = 0.030
filter_coef = 0.99

file = open('data_flow.csv', 'a')
file.write('t (in s);t0;flow_inspi (in m3/s);flow_expi (in m3/s);\n')

flow_inspi_rsc = rsc.HRSC(spi_bus=3)
flow_expi_rsc = rsc.HRSC(spi_bus=0)
time.sleep(0.1)
print('Read the ADC settings and the compensation values from EEPROM.')
flow_inspi_rsc.sensor_info()
flow_expi_rsc.sensor_info()
time.sleep(0.1)
print('Initialize the ADC converter using the settings provided in EEPROM.')
flow_inspi_rsc.adc_configure()
flow_expi_rsc.adc_configure()
time.sleep(0.1)
print('Adjust the ADC sample rate if desired.')
flow_inspi_rsc.set_speed(2000) #in SPS
flow_expi_rsc.set_speed(2000) #in SPS
time.sleep(0.1)
# Delay (Example: if sample rate is 330SPS delay for 3.03 ms [1/330 s])
print('Apply the compensation formulae to the temperature and pressure readings in order to calculate a pressure value.')

t = timer()
t_prev = t
t0 = t

raw_pres_inspi = flow_inspi_rsc.read_pressure(delay)
raw_pres_expi = flow_expi_rsc.read_pressure(delay)
time.sleep(delay)
raw_temp_inspi = flow_inspi_rsc.read_temp(delay)
raw_temp_expi = flow_expi_rsc.read_temp(delay)
time.sleep(delay)
pressure_inspi, temperature_inspi = flow_inspi_rsc.comp_readings(raw_pres_inspi, raw_temp_inspi)
pressure_expi, temperature_expi = flow_expi_rsc.comp_readings(raw_pres_expi, raw_temp_expi)
pressure_offset_inspi = pressure_inspi
pressure_offset_expi = pressure_expi
nb_count = 500

count = 0
while (True):
    dt = t-t_prev
    flow_inspi_rsc.pressure_request()
    flow_expi_rsc.pressure_request()
    time.sleep(delay)
    raw_pres_inspi = flow_inspi_rsc.pressure_reply()
    raw_pres_expi = flow_expi_rsc.pressure_reply()
    if (count % 100 == 0):         
        raw_temp_inspi = flow_inspi_rsc.read_temp(delay)
        raw_temp_expi = flow_expi_rsc.read_temp(delay)
    pressure_inspi, temperature_inspi = flow_inspi_rsc.comp_readings(raw_pres_inspi, raw_temp_inspi)
    pressure_expi, temperature_expi = flow_expi_rsc.comp_readings(raw_pres_expi, raw_temp_expi)
    if (count < nb_count): # Filter to estimate the offset in the beginning...
        pressure_offset_inspi = (1-filter_coef)*pressure_inspi+filter_coef*pressure_offset_inspi
        pressure_offset_expi = (1-filter_coef)*pressure_expi+filter_coef*pressure_offset_expi
    pressure_inspi = pressure_inspi-pressure_offset_inspi
    pressure_expi = pressure_expi-pressure_offset_expi
    rho_air_inspi = 1.292*(273.15/(273.15+temperature_inspi)) # In Kg/m3
    rho_air_expi = 1.292*(273.15/(273.15+temperature_expi)) # In Kg/m3
    vel_inspi = np.sign(pressure_inspi)*math.sqrt(2*(np.abs(pressure_inspi)*248.84)/(rho_air_inspi*((A1/A2)**2-1)))
    vel_expi = np.sign(pressure_expi)*math.sqrt(2*(np.abs(pressure_expi)*248.84)/(rho_air_expi*((A1/A2)**2-1)))
    flow_inspi = A1*vel_inspi
    flow_expi = A1*vel_expi
    print('Flow I : %0.2f L/min, Flow E : %0.2f L/min, vel I : %0.3f m/s, vel E : %0.3f m/s, Pres I : %0.6f inchH2O, Pres E : %0.6f inchH2O, offset I : %0.6f inchH2O, offset E : %0.6f inchH2O, Temp I : %0.6f C, Temp E : %0.6f C, dt : %0.3f s' % 
          (flow_inspi*60000, flow_expi*60000, vel_inspi, vel_expi, pressure_inspi, pressure_expi, pressure_offset_inspi, pressure_offset_expi, temperature_inspi, temperature_expi, dt))
    line = '{};{};{};{};\n'
    file.write(line.format(t, t0, flow_inspi, flow_expi))
    file.flush()
    count = count+1
    t_prev = t
    t = timer()
