#!/usr/bin/python
import RPi.GPIO as GPIO
import ms5837 # From https://github.com/bluerobotics/ms5837-python
import rsc # From https://github.com/tin-/ascp
import hsc
import numpy as np
import math
import sys
import os
import time
import threading
from timeit import default_timer as timer
#from utils import *

#sudo apt-get install python-smbus python-spidev
# or
#sudo pip install smbus
#sudo pip install spidev
# if it did not work.
# Raspberry Pi 4 configuration (GPIO that will be used : 0 for software PWM O2
# valve[, 1, 2, 3, 14, 15 for SPI3 Honeywell RSC for flow (O2)], 2, 3 for 
# I2C1 Bar02 (tube), 4 for digital input SELECT button, 5, 6 for digital 
# output, 7, 8, 9, 10, 11 for SPI0 Honeywell RSC for flow (air), valves, 12, 
# 13 for PWM servomotors, 16 for digital input UP button, 17 for digital 
# input DOWN button, 18, 19, 20, 21, 27 for SPI6 Honeywell RSC for flow 
# (expiration), 22, 23 for I2C6 Bar02 (room), 24 for software PWM air valve, 
# 25 for POWER button), 26 for software PWM buzzer :
#sudo nano /boot/config.txt
# Add in /boot/config.txt (SPI6 might appear as 3, check /dev...)
#dtparam=i2c_arm=on
#dtoverlay=i2c-gpio,bus=6,i2c_gpio_delay_us=1,i2c_gpio_sda=22,i2c_gpio_scl=23
##dtoverlay=i2c-gpio,bus=3,i2c_gpio_delay_us=1,i2c_gpio_sda=4,i2c_gpio_scl=5
#dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
#dtparam=spi=on
##dtoverlay=spi3-2cs,cs0_pin=14,cs1_pin=15
#dtoverlay=spi6-2cs
#dtoverlay=gpio-shutdown,gpio_pin=25,active_low=1,gpio_pull=up
# Then reboot and
#sudo -E python main.py

# Parameters
###############################################################################
Ppeak = 25 # In mbar (= approx. cmH2O)
PEEP = 5 # In mbar (= approx. cmH2O)
respi_rate = 15 # In breaths/min
inspi_ratio = 0.3
O2_air_ratio = -0.05 # < 0 means no ratio control...
flow_limit = 120 # In L/min, >= 120 means no limit...
assist = 0
#trim PWM value start, end depending on balloon size...
pwm0_ns_min = 1000000
pwm0_ns_max = 1500000
pwm1_ns_min = 1500000
pwm1_ns_max = 2000000
Ppeak_step = 1
Ppeak_min = 0
Ppeak_max = 100
PEEP_step = 1
PEEP_min = 0
PEEP_max = 100
respi_rate_step = 1
respi_rate_min = 0
respi_rate_max = 100
inspi_ratio_step = 0.05
inspi_ratio_min = 0.05
inspi_ratio_max = 0.95
O2_air_ratio_step = 0.05
O2_air_ratio_min = -0.05
O2_air_ratio_max = 1.00
flow_limit_step = 5
flow_limit_min = 0
flow_limit_max = 120
enable_p_ms5837 = True
enable_p0_ms5837 = True
enable_p_hsc = False
enable_air_rsc = True
enable_expi_rsc = False
enable_O2_rsc = True
R1 = 0.019100/2.0
R2 = 0.011651/2.0
#R2 = 0.006500/2.0
A1 = math.pi*R1**2
A2 = math.pi*R2**2
speed_rsc = 175 # In SPS
delay_rsc = 0.010
coef_filter_rsc = 0.95
nb_count_auto_zero_filter_rsc = 100
nb_count_offset_filter_rsc = 100
valves_delay = 0.2 # In s
coef_offset_filter_flow = 0.99
flow_thresh = 5 # In L/min
valve_flow_limit_coef = 50
valve_flow_mix_ratio_coef = 50
debug = True
###############################################################################

# Software PWM init (for buzzer and proportional valves)
GPIO.setwarnings(False)	
GPIO.setmode(GPIO.BCM)
buz_pin = 26
GPIO.setup(buz_pin, GPIO.OUT)
buz_pwm = GPIO.PWM(buz_pin, 4000)
buz_pwm.start(50) # Startup beep...
valve_air_pin = 24
GPIO.setup(valve_air_pin, GPIO.OUT)
valve_air_pwm = GPIO.PWM(valve_air_pin, 800)
valve_air_val_max = 100
valve_air_val = 0
valve_air_pwm.start(valve_air_val)
valve_O2_pin = 0
GPIO.setup(valve_O2_pin, GPIO.OUT)
valve_O2_pwm = GPIO.PWM(valve_O2_pin, 800)
valve_O2_val_max = 100
valve_O2_val = 0
valve_O2_pwm.start(valve_O2_val)

# Hardware PWM init (for servos)
os.system('echo 0 > /sys/class/pwm/pwmchip0/export')
os.system('echo 1 > /sys/class/pwm/pwmchip0/export')
os.system('echo 20000000 > /sys/class/pwm/pwmchip0/pwm0/period')
os.system('echo 20000000 > /sys/class/pwm/pwmchip0/pwm1/period')
os.system('echo 1500000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle')
os.system('echo 1500000 > /sys/class/pwm/pwmchip0/pwm1/duty_cycle')
os.system('echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable')
os.system('echo 1 > /sys/class/pwm/pwmchip0/pwm1/enable')

# Digital outputs (valves)
valve_inspi_pin = 6
valve_inspi_val = GPIO.LOW
GPIO.setup(valve_inspi_pin, GPIO.OUT, initial = valve_inspi_val)
valve_expi_pin = 5
valve_expi_val = GPIO.LOW
GPIO.setup(valve_expi_pin, GPIO.OUT, initial = valve_expi_val)

# Digital inputs (buttons)
select = -1 # Index of the selected parameter that should be changed by up/down buttons
parameters = ['Ppeak', 'PEEP', 'respi_rate', 'inspi_ratio', 'O2_air_ratio', 'flow_limit']
select_button_pin = 4
GPIO.setup(select_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
select_button_val = GPIO.input(select_button_pin)
select_button_val_prev = select_button_val
up_button_pin = 16
GPIO.setup(up_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
up_button_val = GPIO.input(up_button_pin)
up_button_val_prev = up_button_val
down_button_pin = 17
GPIO.setup(down_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
down_button_val = GPIO.input(down_button_pin)
down_button_val_prev = down_button_val

# Bar02, HSC
if enable_p_ms5837:
    p_ms5837 = ms5837.MS5837_02BA(bus = 1)
    try:
        if not p_ms5837.init():
            print('P sensor could not be initialized')
            time.sleep(0.1)
            if not p_ms5837.init():
                print('P sensor could not be initialized')
                exit(1)
    except:
        print('P sensor could not be initialized')
        time.sleep(0.1)
        if not p_ms5837.init():
            print('P sensor could not be initialized')
            exit(1)
if enable_p0_ms5837:
    p0_ms5837 = ms5837.MS5837_02BA(bus = 6)
    try:
        if not p0_ms5837.init():
            print('P0 sensor could not be initialized')
            time.sleep(0.1)
            if not p0_ms5837.init():
                print('P0 sensor could not be initialized')
                exit(1)
    except:
        print('P0 sensor could not be initialized')
        time.sleep(0.1)
        if not p0_ms5837.init():
            print('P0 sensor could not be initialized')
            exit(1)
if enable_p_hsc:
    try:
        p_hsc = hsc.HHSC(bus = 6, addr = 0x48, min_pressure = -160.0, max_pressure = 160.0, unit = 'mbar', transfer = 'A')
    except:
        print('HSC sensor could not be initialized')
        time.sleep(0.1)
        try:
            p_hsc = hsc.HHSC(bus = 6, addr = 0x48, min_pressure = -160.0, max_pressure = 160.0, unit = 'mbar', transfer = 'A')
        except:
            print('HSC sensor could not be initialized')
            exit(1)
# The very first pressure measurements might be wrong...
i = 0
while (i < 5):
    if enable_p_ms5837:
        if not p_ms5837.read(ms5837.OSR_256):
            print('P sensor read failed!')
            time.sleep(0.1)
    if enable_p0_ms5837:
        if not p0_ms5837.read(ms5837.OSR_256):
            print('P0 sensor read failed!')
            time.sleep(0.1)
    if enable_p_hsc:
        try:
            p_hsc.read_pressure()
        except:
            print('HSC sensor read failed!')
            time.sleep(0.1)
    i = i+1

# Honeywell RSC
pressure_air, temperature_air, flow_air, flow_filtered_air, vol_air, pressure_offset_air, flow_offset_air = 0, 25, 0, 0, 0, 0, 0
pressure_expi, temperature_expi, flow_expi, flow_filtered_expi, vol_expi, pressure_offset_expi, flow_offset_expi = 0, 25, 0, 0, 0, 0, 0
pressure_O2, temperature_O2, flow_O2, flow_filtered_O2, vol_O2, pressure_offset_O2, flow_offset_O2 = 0, 25, 0, 0, 0, 0, 0
if enable_air_rsc: 
    flow_air_rsc = rsc.HRSC(spi_bus = 3)
    flow_air_rsc.sensor_info()
    flow_air_rsc.reset()
if enable_expi_rsc: 
    flow_expi_rsc = rsc.HRSC(spi_bus = 4)
    flow_expi_rsc.sensor_info()
    flow_expi_rsc.reset()
if enable_O2_rsc: 
    flow_O2_rsc = rsc.HRSC(spi_bus = 0)
    flow_O2_rsc.sensor_info()
    flow_O2_rsc.reset()
time.sleep(0.005)
if enable_air_rsc: 
    flow_air_rsc.adc_configure()
    flow_air_rsc.set_speed(speed_rsc)
if enable_expi_rsc: 
    flow_expi_rsc.adc_configure()
    flow_expi_rsc.set_speed(speed_rsc)
if enable_O2_rsc: 
    flow_O2_rsc.adc_configure()
    flow_O2_rsc.set_speed(speed_rsc)
time.sleep(0.005)
if enable_air_rsc: 
    flow_air_rsc.pressure_request()
if enable_expi_rsc: 
    flow_expi_rsc.pressure_request()
if enable_O2_rsc: 
    flow_O2_rsc.pressure_request()
time.sleep(delay_rsc)
if enable_air_rsc: 
    raw_pressure_air = flow_air_rsc.pressure_reply()
    flow_air_rsc.temperature_request()
if enable_expi_rsc: 
    raw_pressure_expi = flow_expi_rsc.pressure_reply()
    flow_expi_rsc.temperature_request()
if enable_O2_rsc: 
    raw_pressure_O2 = flow_O2_rsc.pressure_reply()
    flow_O2_rsc.temperature_request()
time.sleep(delay_rsc)
if enable_air_rsc: 
    raw_temperature_air = flow_air_rsc.temperature_reply()
    raw_zero_pressure_air = raw_pressure_air
    raw_zero_temperature_air = raw_temperature_air
    pressure_air, temperature_air = flow_air_rsc.comp_readings(raw_pressure_air, raw_temperature_air)
    pressure_air = flow_air_rsc.conv_pressure_to_mbar(pressure_air)
    pressure_offset_air = pressure_air
if enable_expi_rsc: 
    raw_temperature_expi = flow_expi_rsc.temperature_reply()
    raw_zero_pressure_expi = raw_pressure_expi
    raw_zero_temperature_expi = raw_temperature_expi
    pressure_expi, temperature_expi = flow_expi_rsc.comp_readings(raw_pressure_expi, raw_temperature_expi)
    pressure_expi = flow_expi_rsc.conv_pressure_to_mbar(pressure_expi)
    pressure_offset_expi = pressure_expi
if enable_O2_rsc: 
    raw_temperature_O2 = flow_O2_rsc.temperature_reply()
    raw_zero_pressure_O2 = raw_pressure_O2
    raw_zero_temperature_O2 = raw_temperature_O2
    pressure_O2, temperature_O2 = flow_O2_rsc.comp_readings(raw_pressure_O2, raw_temperature_O2)
    pressure_O2 = flow_O2_rsc.conv_pressure_to_mbar(pressure_O2)
    pressure_offset_O2 = pressure_O2
# Auto-zero filter proposed by the documentation in the beginning (with valves closed)...
i = 0
while (i < nb_count_auto_zero_filter_rsc):
    if enable_air_rsc: 
        flow_air_rsc.pressure_request()
    if enable_expi_rsc: 
        flow_expi_rsc.pressure_request()
    if enable_O2_rsc: 
        flow_O2_rsc.pressure_request()
    time.sleep(delay_rsc)
    if enable_air_rsc: 
        raw_pressure_air = flow_air_rsc.pressure_reply()
        flow_air_rsc.temperature_request()
    if enable_expi_rsc: 
        raw_pressure_expi = flow_expi_rsc.pressure_reply()
        flow_expi_rsc.temperature_request()
    if enable_O2_rsc: 
        raw_pressure_O2 = flow_O2_rsc.pressure_reply()
        flow_O2_rsc.temperature_request()
    time.sleep(delay_rsc)
    if enable_air_rsc: 
        raw_temperature_air = flow_air_rsc.temperature_reply()
        raw_zero_pressure_air = (1-coef_filter_rsc)*raw_pressure_air+coef_filter_rsc*raw_zero_pressure_air
        raw_zero_temperature_air = (1-coef_filter_rsc)*raw_temperature_air+coef_filter_rsc*raw_zero_temperature_air
    if enable_expi_rsc: 
        raw_temperature_expi = flow_expi_rsc.temperature_reply()
        raw_zero_pressure_expi = (1-coef_filter_rsc)*raw_pressure_expi+coef_filter_rsc*raw_zero_pressure_expi
        raw_zero_temperature_expi = (1-coef_filter_rsc)*raw_temperature_expi+coef_filter_rsc*raw_zero_temperature_expi
    if enable_O2_rsc: 
        raw_temperature_O2 = flow_O2_rsc.temperature_reply()
        raw_zero_pressure_O2 = (1-coef_filter_rsc)*raw_pressure_O2+coef_filter_rsc*raw_zero_pressure_O2
        raw_zero_temperature_O2 = (1-coef_filter_rsc)*raw_temperature_O2+coef_filter_rsc*raw_zero_temperature_O2
    i = i+1
# Store auto-zero info...
if (nb_count_auto_zero_filter_rsc > 0):
    if enable_air_rsc: 
        flow_air_rsc.comp_auto_zero(raw_zero_pressure_air, raw_zero_temperature_air)
    if enable_expi_rsc: 
        flow_expi_rsc.comp_auto_zero(raw_zero_pressure_expi, raw_zero_temperature_expi)
    if enable_O2_rsc: 
        flow_O2_rsc.comp_auto_zero(raw_zero_pressure_O2, raw_zero_temperature_O2)
# Additional filter to estimate the offset in the beginning (with valves closed)...
i = 0
while (i < nb_count_offset_filter_rsc):
    if enable_air_rsc: 
        flow_air_rsc.pressure_request()
    if enable_expi_rsc: 
        flow_expi_rsc.pressure_request()
    if enable_O2_rsc: 
        flow_O2_rsc.pressure_request()
    time.sleep(delay_rsc)
    if enable_air_rsc: 
        raw_pressure_air = flow_air_rsc.pressure_reply()
        flow_air_rsc.temperature_request()
    if enable_expi_rsc: 
        raw_pressure_expi = flow_expi_rsc.pressure_reply()
        flow_expi_rsc.temperature_request()
    if enable_O2_rsc: 
        raw_pressure_O2 = flow_O2_rsc.pressure_reply()
        flow_O2_rsc.temperature_request()
    time.sleep(delay_rsc)
    if enable_air_rsc: 
        raw_temperature_air = flow_air_rsc.temperature_reply()
        pressure_air, temperature_air = flow_air_rsc.comp_readings(raw_pressure_air, raw_temperature_air)
        pressure_air = flow_air_rsc.conv_pressure_to_mbar(pressure_air)
        pressure_offset_air = (1-coef_filter_rsc)*pressure_air+coef_filter_rsc*pressure_offset_air
    if enable_expi_rsc: 
        raw_temperature_expi = flow_expi_rsc.temperature_reply()
        pressure_expi, temperature_expi = flow_expi_rsc.comp_readings(raw_pressure_expi, raw_temperature_expi)
        pressure_expi = flow_expi_rsc.conv_pressure_to_mbar(pressure_expi)
        pressure_offset_expi = (1-coef_filter_rsc)*pressure_expi+coef_filter_rsc*pressure_offset_expi
    if enable_O2_rsc: 
        raw_temperature_O2 = flow_O2_rsc.temperature_reply()
        pressure_O2, temperature_O2 = flow_O2_rsc.comp_readings(raw_pressure_O2, raw_temperature_O2)
        pressure_O2 = flow_O2_rsc.conv_pressure_to_mbar(pressure_O2)
        pressure_offset_O2 = (1-coef_filter_rsc)*pressure_O2+coef_filter_rsc*pressure_offset_O2
    i = i+1
if (nb_count_offset_filter_rsc <= 0):
    pressure_offset_air = 0
    pressure_offset_expi = 0
    pressure_offset_O2 = 0

respi_rate_converted = respi_rate*(1.0/60.0) # In breaths/s
cycle_duration = 1.0/respi_rate_converted
inspi_duration = inspi_ratio*cycle_duration
expi_duration = cycle_duration-inspi_duration

t = timer()
t_prev = t
dt = t-t_prev
t0 = t
t_cycle_start = t
p0 = 1000
p = 1000
if enable_p_ms5837: p = p_ms5837.pressure()
if enable_p_hsc: p = p0+p_hsc.conv_pressure_to_mbar(p_hsc.read_pressure())
p_prev = p
p_cycle_start = p
p0 = p
if enable_p0_ms5837: p0 = p0_ms5837.pressure() 
temperature = 25
if enable_p_ms5837: temperature = p_ms5837.temperature()
if enable_p_hsc: temperature = p_hsc.read_temperature()
temperature0 = temperature
if enable_p0_ms5837: temperature0 = p0_ms5837.temperature()
Ppeak_reached = False
PEEP_reached = False
inspi_end = False
inspi_duration_estim = inspi_duration
expi_duration_estim = expi_duration
cycle_duration_estim = cycle_duration

pwm0_ns = pwm0_ns_max
pwm1_ns = pwm1_ns_min

t_last_not_all_valves_closed = 0
t_all_valves_closed = 0

# File errors are not critical...
try:
    file = open('data.csv', 'a')
    file.write('t (in s);t0 (in s);p0 (in mbar);temperature0 (in C);p (in mbar);temperature (in C);select;Ppeak (in mbar);PEEP (in mbar);respi_rate (in breaths/min);inspi_ratio;O2_air_ratio;flow_limit (in L/min);assist;pwm0_ns;pwm1_ns;valve_air_val;valve_O2_val;valve_inspi_val;valve_expi_val;pressure_air (in mbar);pressure_expi (in mbar);pressure_O2 (in mbar);temperature_air (in C);temperature_expi (in C);temperature_O2 (in C);flow_air (in L/min);flow_expi (in L/min);flow_O2 (in L/min);flow_filtered_air (in L/min);flow_filtered_expi (in L/min);flow_filtered_O2 (in L/min);vol_air (in L);vol_expi (in L);vol_O2 (in L);\n')
except:
    pass

# Stop the startup beep
buz_pwm.ChangeDutyCycle(0)

# Divisions by 0, NAN, INF...?

count = 0
while True:
    if (t-t_cycle_start < inspi_duration_estim):
        #pwm0_ns = pwm0_ns_max-(pwm0_ns_max-pwm0_ns_min)*(t-t_cycle_start)/inspi_duration_estim
        #pwm1_ns = pwm1_ns_min+(pwm1_ns_max-pwm1_ns_min)*(t-t_cycle_start)/inspi_duration_estim
        pwm0_ns = pwm0_ns_min
        pwm1_ns = pwm1_ns_max
        PEEP_reached = False
        if ((p-p0 > Ppeak) or (Ppeak_reached == True)): # Should close both valves to maintain Ppeak...
            Ppeak_reached = True
            pwm0_ns = pwm0_ns_max
            pwm1_ns = pwm1_ns_min
            valve_air_val = 0
            valve_air_pwm.ChangeDutyCycle(valve_air_val) 
            valve_O2_val = 0
            valve_O2_pwm.ChangeDutyCycle(valve_O2_val) 
            valve_inspi_val = GPIO.LOW
            GPIO.output(valve_inspi_pin, valve_inspi_val)
        else:
            valve_air_val = valve_air_val_max
            valve_air_pwm.ChangeDutyCycle(valve_air_val) 
            valve_O2_val = valve_O2_val_max
            valve_O2_pwm.ChangeDutyCycle(valve_O2_val) 
            valve_inspi_val = GPIO.HIGH
            GPIO.output(valve_inspi_pin, valve_inspi_val)
        valve_expi_val = GPIO.LOW
        GPIO.output(valve_expi_pin, valve_expi_val)
    else:
        #pwm0_ns = pwm0_ns_min+(pwm0_ns_max-pwm0_ns_min)*(t-t_cycle_start-inspi_duration_estim)/expi_duration_estim
        #pwm1_ns = pwm1_ns_max-(pwm1_ns_max-pwm1_ns_min)*(t-t_cycle_start-inspi_duration_estim)/expi_duration_estim
        pwm0_ns = pwm0_ns_max
        pwm1_ns = pwm1_ns_min
        Ppeak_reached = False
        if ((p-p0 < PEEP) or (PEEP_reached == True)): # Should close valves to maintain PEEP...
        #if (p-p0 < PEEP): # Should close valves to maintain PEEP...
            PEEP_reached = True
            valve_air_val = 0
            valve_air_pwm.ChangeDutyCycle(valve_air_val) 
            valve_O2_val = 0
            valve_O2_pwm.ChangeDutyCycle(valve_O2_val) 
            valve_inspi_val = GPIO.LOW
            GPIO.output(valve_inspi_pin, valve_inspi_val)
            valve_expi_val = GPIO.LOW
            GPIO.output(valve_expi_pin, valve_expi_val)
        else:
            valve_air_val = 100 # Full to depress...
            valve_air_pwm.ChangeDutyCycle(valve_air_val) 
            valve_O2_val = 0 # Should not spend O2 to depress...
            valve_O2_pwm.ChangeDutyCycle(valve_O2_val) 
            valve_inspi_val = GPIO.LOW
            GPIO.output(valve_inspi_pin, valve_inspi_val)
            valve_expi_val = GPIO.HIGH
            GPIO.output(valve_expi_pin, valve_expi_val)
    pwm0_ns = min(pwm0_ns_max, max(pwm0_ns_min, pwm0_ns))
    pwm1_ns = min(pwm1_ns_max, max(pwm1_ns_min, pwm1_ns))

    pwm0_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm0/duty_cycle'
    pwm1_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm1/duty_cycle'
    os.system(pwm0_cmd.format(math.trunc(pwm0_ns)))
    os.system(pwm1_cmd.format(math.trunc(pwm1_ns)))

    # Buttons to set parameters
    select_button_val = GPIO.input(select_button_pin)
    if (select_button_val != select_button_val_prev):
        select_button_val_prev = select_button_val
        if (select_button_val == GPIO.HIGH):
            select = select + 1
            if (select >= len(parameters)): select = -1    
    if (select >= 0) and (select < len(parameters)):
        up_button_val = GPIO.input(up_button_pin)
        if (up_button_val != up_button_val_prev):
            up_button_val_prev = up_button_val
            if (up_button_val == GPIO.HIGH):
                if (select == 0):
                    Ppeak = Ppeak + Ppeak_step
                    if (Ppeak > Ppeak_max): Ppeak = Ppeak_max
                if (select == 1):
                    PEEP = PEEP + PEEP_step
                    if (PEEP > PEEP_max): PEEP = PEEP_max
                if (select == 2):
                    respi_rate = respi_rate + respi_rate_step
                    if (respi_rate > respi_rate_max): respi_rate = respi_rate_max
                if (select == 3):
                    inspi_ratio = inspi_ratio + inspi_ratio_step
                    if (inspi_ratio > inspi_ratio_max): inspi_ratio = inspi_ratio_max
                if (select == 4):
                    O2_air_ratio = O2_air_ratio + O2_air_ratio_step
                    if (O2_air_ratio > O2_air_ratio_max): O2_air_ratio = O2_air_ratio_max
                if (select == 5):
                    flow_limit = flow_limit + flow_limit_step
                    if (flow_limit > flow_limit_max): flow_limit = flow_limit_max
        down_button_val = GPIO.input(down_button_pin)
        if (down_button_val != down_button_val_prev):
            down_button_val_prev = down_button_val
            if (down_button_val == GPIO.HIGH):
                if (select == 0):
                    Ppeak = Ppeak - Ppeak_step
                    if (Ppeak < Ppeak_min): Ppeak = Ppeak_min
                if (select == 1):
                    PEEP = PEEP - PEEP_step
                    if (PEEP < PEEP_min): PEEP = PEEP_min
                if (select == 2):
                    respi_rate = respi_rate - respi_rate_step
                    if (respi_rate < respi_rate_min): respi_rate = respi_rate_min
                if (select == 3):
                    inspi_ratio = inspi_ratio - inspi_ratio_step
                    if (inspi_ratio < inspi_ratio_min): inspi_ratio = inspi_ratio_min
                if (select == 4):
                    O2_air_ratio = O2_air_ratio - O2_air_ratio_step
                    if (O2_air_ratio < O2_air_ratio_min): O2_air_ratio = O2_air_ratio_min
                if (select == 5):
                    flow_limit = flow_limit - flow_limit_step
                    if (flow_limit < flow_limit_min): flow_limit = flow_limit_min
 
    # Sensors
    if enable_air_rsc: 
        try:
            flow_air_rsc.pressure_request()
        except:
            print('RSC A sensor read failed!')
            time.sleep(0.1)
            flow_air_rsc.pressure_request()
    if enable_expi_rsc: 
        try:
            flow_expi_rsc.pressure_request()
        except:
            print('RSC E sensor read failed!')
            time.sleep(0.1)
            flow_expi_rsc.pressure_request()
    if enable_O2_rsc: 
        try:
            flow_O2_rsc.pressure_request()
        except:
            print('RSC O sensor read failed!')
            time.sleep(0.1)
            flow_O2_rsc.pressure_request()
    time.sleep(max(0, delay_rsc-0.005))
    if enable_p_ms5837:
        try:
            if not p_ms5837.read(ms5837.OSR_256):
                print('P sensor read failed!')
                time.sleep(0.1)
                if not p_ms5837.read(ms5837.OSR_256):
                    print('P sensor read failed!')
                    buz_pwm.ChangeDutyCycle(50)
                    exit(1)
        except:
            print('P sensor read failed!')
            time.sleep(0.1)
            if not p_ms5837.read(ms5837.OSR_256):
                print('P sensor read failed!')
                buz_pwm.ChangeDutyCycle(50)
                exit(1)
    elif enable_p_hsc:
        try:
            p_diff_hsc, temp_tmp_hsc = p_hsc.read()
        except:
            print('HSC sensor read failed!')
            time.sleep(0.1)
            try:
                p_diff_hsc, temp_tmp_hsc = p_hsc.read()
            except:
                print('HSC sensor read failed!')
                time.sleep(0.1)
                buz_pwm.ChangeDutyCycle(50)
                exit(1)
    else:
        time.sleep(0.005)
    if enable_air_rsc: 
        try:
            raw_pressure_air = flow_air_rsc.pressure_reply()
        except:
            print('RSC A sensor read failed!')
            time.sleep(0.1)
            raw_pressure_air = flow_air_rsc.pressure_reply()
    if enable_expi_rsc: 
        try:
            raw_pressure_expi = flow_expi_rsc.pressure_reply()
        except:
            print('RSC E sensor read failed!')
            time.sleep(0.1)
            raw_pressure_expi = flow_expi_rsc.pressure_reply()
    if enable_O2_rsc: 
        try:
            raw_pressure_O2 = flow_O2_rsc.pressure_reply()
        except:
            print('RSC O sensor read failed!')
            time.sleep(0.1)
            raw_pressure_O2 = flow_O2_rsc.pressure_reply()
    if enable_air_rsc: 
        try:
            flow_air_rsc.temperature_request()
        except:
            print('RSC A sensor read failed!')
            time.sleep(0.1)
            flow_air_rsc.temperature_request()
    if enable_expi_rsc: 
        try:
            flow_expi_rsc.temperature_request()
        except:
            print('RSC E sensor read failed!')
            time.sleep(0.1)
            flow_expi_rsc.temperature_request()
    if enable_O2_rsc: 
        try:
            flow_O2_rsc.temperature_request()
        except:
            print('RSC O sensor read failed!')
            time.sleep(0.1)
            flow_O2_rsc.temperature_request()
    time.sleep(delay_rsc)
    if enable_air_rsc: 
        try:
            raw_temperature_air = flow_air_rsc.temperature_reply()
        except:
            print('RSC A sensor read failed!')
            time.sleep(0.1)
            raw_temperature_air = flow_air_rsc.temperature_reply()
    if enable_expi_rsc: 
        try:
            raw_temperature_expi = flow_expi_rsc.temperature_reply()
        except:
            print('RSC E sensor read failed!')
            time.sleep(0.1)
            raw_temperature_expi = flow_expi_rsc.temperature_reply()
    if enable_O2_rsc: 
        try:
            raw_temperature_O2 = flow_O2_rsc.temperature_reply()
        except:
            print('RSC O sensor read failed!')
            time.sleep(0.1)
            raw_temperature_O2 = flow_O2_rsc.temperature_reply()
    if enable_air_rsc: 
        pressure_air, temperature_air = flow_air_rsc.comp_readings(raw_pressure_air, raw_temperature_air)
        pressure_air = flow_air_rsc.conv_pressure_to_mbar(pressure_air)
        pressure_air = pressure_air-pressure_offset_air
        rho_air = 1.292*(273.15/(273.15+temperature_air)) # In kg/m3
        vel_air = np.sign(pressure_air)*math.sqrt(2*(abs(pressure_air)*100.0)/(rho_air*((A1/A2)**2-1)))
        flow_air = A1*vel_air
    if enable_expi_rsc: 
        pressure_expi, temperature_expi = flow_expi_rsc.comp_readings(raw_pressure_expi, raw_temperature_expi)
        pressure_expi = flow_expi_rsc.conv_pressure_to_mbar(pressure_expi)
        pressure_expi = pressure_expi-pressure_offset_expi
        rho_expi = 1.292*(273.15/(273.15+temperature_expi)) # In kg/m3
        vel_expi = np.sign(pressure_expi)*math.sqrt(2*(abs(pressure_expi)*100.0)/(rho_expi*((A1/A2)**2-1)))
        flow_expi = A1*vel_expi
    if enable_O2_rsc: 
        pressure_O2, temperature_O2 = flow_O2_rsc.comp_readings(raw_pressure_O2, raw_temperature_O2)
        pressure_O2 = flow_O2_rsc.conv_pressure_to_mbar(pressure_O2)
        pressure_O2 = pressure_O2-pressure_offset_O2
        rho_O2 = 1.292*(273.15/(273.15+temperature_O2)) # In kg/m3
        vel_O2 = np.sign(pressure_O2)*math.sqrt(2*(abs(pressure_O2)*100.0)/(rho_O2*((A1/A2)**2-1)))
        flow_O2 = A1*vel_O2

    # Filters
    # Should run low-pass or median filter before...?
    if ((valve_air_val <= 0) and (valve_O2_val <= 0) and (valve_inspi_val <= 0) and (valve_expi_val <= 0)):
        t_all_valves_closed = t
        if (t_all_valves_closed-t_last_not_all_valves_closed > valves_delay):
            flow_offset_air = (1-coef_offset_filter_flow)*flow_air+coef_offset_filter_flow*flow_offset_air
            flow_offset_expi = (1-coef_offset_filter_flow)*flow_expi+coef_offset_filter_flow*flow_offset_expi
            flow_offset_O2 = (1-coef_offset_filter_flow)*flow_O2+coef_offset_filter_flow*flow_offset_O2
    else:
        t_last_not_all_valves_closed = t
    flow_filtered_air = flow_air-flow_offset_air
    flow_filtered_expi = flow_expi-flow_offset_expi
    flow_filtered_O2 = flow_O2-flow_offset_O2
    if (abs(flow_filtered_air) > flow_thresh/60000.0): vol_air = vol_air+dt*flow_filtered_air
    if (abs(flow_filtered_expi) > flow_thresh/60000.0): vol_expi = vol_expi+dt*flow_filtered_expi
    if (abs(flow_filtered_O2) > flow_thresh/60000.0): vol_O2 = vol_O2+dt*flow_filtered_O2

    # Proportional valves control
    if ((valve_air_val > 0) and (valve_O2_val > 0)):
        if (flow_limit < flow_limit_max):
            flow_mix = flow_filtered_air+flow_filtered_O2 # Should be controlled around flow_limit...
            err_flow_mix = (flow_limit-flow_mix)/flow_limit
            valve_air_val_max = min(1, max(100, valve_air_val_max+valve_flow_limit_coef*err_flow_mix)) # Min is 1 to not disable proportional valves control...
            valve_O2_val_max =  min(1, max(100, valve_O2_val_max+valve_flow_limit_coef*err_flow_mix)) # Min is 1 to not disable proportional valves control...
        else:
            valve_air_val_max = 100
            valve_O2_val_max = 100
        if (O2_air_ratio >= 0): # < 0 means no ratio control...
            flow_mix_ratio = flow_filtered_O2/flow_filtered_air # Should be controlled around O2_air_ratio...
            err_flow_mix_ratio = O2_air_ratio-flow_mix_ratio
            valve_air_val_max = min(1, max(100, valve_air_val_max+valve_flow_mix_ratio_coef*err_flow_mix_ratio)) # Min is 1 to not disable proportional valves control...
            valve_O2_val_max =  min(1, max(100, valve_O2_val_max-valve_flow_mix_ratio_coef*err_flow_mix_ratio)) # Min is 1 to not disable proportional valves control...

    # Log file
    # File errors are not critical...
    try:
        line = '{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};\n'
        file.write(line.format(t, t0, p0, temperature0, p, temperature, select, Ppeak, PEEP, respi_rate, inspi_ratio, O2_air_ratio, flow_limit, assist, 
                               pwm0_ns, pwm1_ns, valve_air_val, valve_O2_val, valve_inspi_val, valve_expi_val, 
                               pressure_air, pressure_expi, pressure_O2, temperature_air, temperature_expi, temperature_O2, 
                               flow_air*60000.0, flow_expi*60000.0, flow_O2*60000.0, flow_filtered_air*60000.0, flow_filtered_expi*60000.0, flow_filtered_O2*60000.0, vol_air*1000.0, vol_expi*1000.0, vol_O2*1000.0))
        file.flush()
    except:
        buz_pwm.ChangeDutyCycle(50)

    if (debug): print('t-t0: %0.2f s \tdt: %0.3f s \tP0: %0.1f mbar \tT0: %0.2f C \tP: %0.1f mbar \tT: %0.2f C \tP I: %0.5f mbar \tP off. I: %0.5f mbar \tP E: %0.5f mbar \tP off. E: %0.5f mbar \tP O: %0.5f mbar \tP off. O: %0.5f mbar') % (
        t-t0, dt, p0, temperature0, p, temperature, pressure_air, pressure_offset_air, pressure_expi, pressure_offset_expi, pressure_O2, pressure_offset_O2) 
    
    # Prepare next loop...
    t_prev = t
    t = timer()
    dt = t-t_prev
    p_prev = p
    if enable_p_ms5837:
        p = p_ms5837.pressure()
        temperature = p_ms5837.temperature()
    elif enable_p_hsc:
        p = p0+p_diff_hsc
        temperature = temp_tmp_hsc
    if (t-t_cycle_start > cycle_duration_estim):
        t_cycle_start = t
        p_cycle_start = p
        if enable_p0_ms5837:   
            try:
                if not p0_ms5837.read(ms5837.OSR_256):
                    print('P0 sensor read failed!')
                    time.sleep(0.1)
                    if not p0_ms5837.read(ms5837.OSR_256):
                        print('P0 sensor read failed!')
                        buz_pwm.ChangeDutyCycle(50)
                        exit(1)
            except:
                print('P0 sensor read failed!')
                time.sleep(0.1)
                if not p0_ms5837.read(ms5837.OSR_256):
                    print('P0 sensor read failed!')
                    buz_pwm.ChangeDutyCycle(50)
                    exit(1)
            p0 = p0_ms5837.pressure()
            temperature0 = p0_ms5837.temperature()
        vol_air = 0
        vol_expi = 0
        vol_O2 = 0
        inspi_end = False
    count = count+1
