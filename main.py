#!/usr/bin/python
from __future__ import division
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
# Raspberry Pi 4 configuration (GPIO that will be used : 0 for digital output
# (expi), 1, 2, 3, 14, 15 for SPI3 Honeywell RSC for flow (O2)/2, 3 for 
# software PWM air and O2 valves, 4, 5 for Honeywell HSC (inspi) and I2C3 Bar02
# (inspi), 6 for digital output (inspi), 7, 8, 9, 10, 11 for SPI0 Honeywell RSC
# for flow (air), 12, 13 for O2 and air valves/balloon PWM servomotors, 16 for 
# digital input UP button, 17 for digital input DOWN button, 18, 19, 20, 21, 27
# for SPI6 Honeywell RSC for flow (expiration), 22, 23 for Honeywell HSC 
# (expi), I2C6 Bar02 (room), touchscreen, RTC clock, 24 for digital input 
# SELECT button, 25 for POWER button), 26 for software PWM buzzer :
#sudo nano /boot/config.txt
# Add/modify in /boot/config.txt (SPI6 might appear as 4, check /dev...)
#enable_uart=0
#dtparam=i2c_arm=on
#dtoverlay=i2c1,pins_44_45
#dtoverlay=i2c-gpio,bus=3,i2c_gpio_delay_us=1,i2c_gpio_sda=4,i2c_gpio_scl=5
#dtoverlay=i2c-gpio,bus=6,i2c_gpio_delay_us=1,i2c_gpio_sda=22,i2c_gpio_scl=23
#dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
#dtparam=spi=on
#dtoverlay=spi3-2cs,cs0_pin=14,cs1_pin=15
#dtoverlay=spi6-2cs
#dtoverlay=gpio-shutdown,gpio_pin=25,active_low=1,gpio_pull=up
# Then reboot and
#sudo -E python main.py & python gui.py

# Parameters
###############################################################################
Ppeak = 25 # In mbar (= approx. cmH2O)
PEEP = 5 # In mbar (= approx. cmH2O)
respi_rate = 15 # In breaths/min
inspi_ratio = 0.3
flow_control_air = 30 # In L/min, >= flow_control_air_max means no limit...
flow_control_O2 = 30 # In L/min, >= flow_control_O2_max means no limit...
mode = 2 # 0 : ventilator, 1 : ventilator in assistance mode, 2 : only O2:Air mix
# Advanced parameters
# Trim PWM value start, end depending on balloon size...
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
inspi_ratio_min = 0.00
inspi_ratio_max = 1.00
flow_control_air_step = 2
flow_control_air_min = 0
flow_control_air_max = 80
flow_control_O2_step = 2
flow_control_O2_min = 0
flow_control_O2_max = 80
mode_step = 1
mode_min = 0
mode_max = 2
enable_buzzer = True
enable_pigpio_pwm = False
disable_hard_pwm = False
enable_hard_pwm_air_O2_valves = True
enable_pwm_expi_valve = True
enable_p_ms5837 = True
enable_p0_ms5837 = True
enable_p_inspi_hsc = False
enable_p_expi_hsc = False
enable_air_rsc = True
enable_expi_rsc = False
enable_O2_rsc = True
R1_air = 0.019100/2.0
#R2_air = 0.011651/2.0
R2_air = 0.006500/2.0
R1_expi = 0.019100/2.0
R2_expi = 0.011651/2.0
#R2_expi = 0.006500/2.0
R1_O2 = 0.019100/2.0
#R2_O2 = 0.011651/2.0
R2_O2 = 0.006500/2.0
A1_air = math.pi*R1_air**2
A2_air = math.pi*R2_air**2
A1_expi = math.pi*R1_expi**2
A2_expi = math.pi*R2_expi**2
A1_O2 = math.pi*R1_O2**2
A2_O2 = math.pi*R2_O2**2
speed_rsc = 175 # In SPS
delay_rsc = 0.010
coef_filter_rsc = 0.95
nb_count_auto_zero_filter_rsc = 0 # 100
nb_count_offset_filter_rsc = 0 # 100
valves_pwm_freq = 600 # In Hz
valves_max_init = 50
valves_delay = 0.2 # In s
coef_offset_filter_flow = 0.99
coef_filter_flow = 0.7
flow_thresh = 15 # In L/min
valve_flow_control_air_coef = 25.0
valve_flow_control_O2_coef = 25.0
valve_pressure_excess_control_air_coef = 100.0
valve_pressure_excess_control_O2_coef = 100.0
debug = True
###############################################################################

GPIO.setwarnings(False)	
GPIO.setmode(GPIO.BCM)

# Software PWM init for buzzer
if enable_buzzer:
    buz_pin = 26
    GPIO.setup(buz_pin, GPIO.OUT)
    buz_pwm = GPIO.PWM(buz_pin, 4000)
    buz_pwm.start(50) # Startup beep...

if enable_pigpio_pwm: 
    os.system('pigpiod')
    time.sleep(0.2)
    import pigpio
    pi = pigpio.pi()
    if not pi.connected:
       print('pigpio could not be initialized')
       exit(1)

# Other PWM init
if (flow_control_air < flow_control_air_max): valve_air_val_max = valves_max_init # Min > 0 to not always disable proportional valves control...
else: valve_air_val_max = 100
valve_air_val = 0
valve_air_val = min(100, max(0, valve_air_val))
if (flow_control_O2 < flow_control_O2_max): valve_O2_val_max = valves_max_init # Min > 0 to not always disable proportional valves control...
else: valve_O2_val_max = 100
valve_O2_val = 0
valve_O2_val = min(100, max(0, valve_O2_val))
if not disable_hard_pwm:
    os.system('echo 0 > /sys/class/pwm/pwmchip0/export')
    os.system('echo 1 > /sys/class/pwm/pwmchip0/export')
if not enable_hard_pwm_air_O2_valves:
    # Hardware PWM for balloon servos
    pwm_period = 20000000
    pwm0_period_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm0/period'
    pwm1_period_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm1/period'
    if not disable_hard_pwm:
        os.system(pwm0_period_cmd.format(math.trunc(pwm_period)))
        os.system(pwm1_period_cmd.format(math.trunc(pwm_period)))
    pwm0_ns = pwm0_ns_max
    pwm1_ns = pwm1_ns_min
    pwm0_ns = min(pwm0_ns_max, max(pwm0_ns_min, pwm0_ns))
    pwm1_ns = min(pwm1_ns_max, max(pwm1_ns_min, pwm1_ns))
    pwm0_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm0/duty_cycle'
    pwm1_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm1/duty_cycle'
    if not disable_hard_pwm:
        os.system(pwm0_cmd.format(math.trunc(pwm0_ns)))
        os.system(pwm1_cmd.format(math.trunc(pwm1_ns)))
    # Software PWM for air and O2 proportional valves
    valve_air_pin = 2
    GPIO.setup(valve_air_pin, GPIO.OUT)
    valve_air_pwm = GPIO.PWM(valve_air_pin, valves_pwm_freq)
    valve_air_pwm.start(valve_air_val)
    valve_O2_pin = 3
    GPIO.setup(valve_O2_pin, GPIO.OUT)
    valve_O2_pwm = GPIO.PWM(valve_O2_pin, valves_pwm_freq)
    valve_O2_pwm.start(valve_O2_val)
else:
    # Hardware PWM for air and O2 proportional valves
    pwm_period = max(1500000, min(0x7FFFFFFF, int(1000000000/valves_pwm_freq)))
    pwm0_period_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm0/period'
    pwm1_period_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm1/period'
    if not disable_hard_pwm:
        os.system(pwm0_period_cmd.format(math.trunc(pwm_period)))
        os.system(pwm1_period_cmd.format(math.trunc(pwm_period)))
    pwm0_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm0/duty_cycle'
    pwm1_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm1/duty_cycle'
    if not disable_hard_pwm:
        os.system(pwm0_cmd.format(math.trunc(min(pwm_period, max(0, pwm_period*valve_air_val/100)))))
        os.system(pwm1_cmd.format(math.trunc(min(pwm_period, max(0, pwm_period*valve_O2_val/100)))))
if not disable_hard_pwm:
    os.system('echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable')
    os.system('echo 1 > /sys/class/pwm/pwmchip0/pwm1/enable')

# Digital outputs (valves)
valve_inspi_pin = 6
valve_inspi_val = GPIO.LOW
GPIO.setup(valve_inspi_pin, GPIO.OUT, initial = valve_inspi_val)
valve_expi_pin = 0
if not enable_pwm_expi_valve:
    valve_expi_val = GPIO.LOW
    GPIO.setup(valve_expi_pin, GPIO.OUT, initial = valve_expi_val)
elif enable_pigpio_pwm:
    valve_expi_val = 0
    valve_expi_val = min(100, max(0, valve_expi_val))
    pi.set_mode(valve_expi_pin, pigpio.OUTPUT)
    pi.set_PWM_frequency(valve_expi_pin, valves_pwm_freq)
    pi.set_PWM_dutycycle(valve_expi_pin, 255*valve_expi_val/100) 
else:
    valve_expi_val = 0
    valve_expi_val = min(100, max(0, valve_expi_val))
    GPIO.setup(valve_expi_pin, GPIO.OUT)
    valve_expi_pwm = GPIO.PWM(valve_expi_pin, valves_pwm_freq)
    valve_expi_pwm.start(valve_expi_val)

# Digital inputs (buttons)
select = -1 # Index of the selected parameter that should be changed by up/down buttons
parameters = ['Ppeak', 'PEEP', 'respi_rate', 'inspi_ratio', 'flow_control_air', 'flow_control_O2', 'mode']
select_button_pin = 24
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
    p_ms5837 = ms5837.MS5837_02BA(bus = 3)
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
if enable_p_inspi_hsc:
    try:
        p_inspi_hsc = hsc.HHSC(bus = 3, addr = 0x48, min_pressure = -160.0, max_pressure = 160.0, unit = 'mbar', transfer = 'A')
    except:
        print('HSC I sensor could not be initialized')
        time.sleep(0.1)
        try:
            p_inspi_hsc = hsc.HHSC(bus = 6, addr = 0x48, min_pressure = -160.0, max_pressure = 160.0, unit = 'mbar', transfer = 'A')
        except:
            print('HSC I sensor could not be initialized')
            exit(1)
if enable_p_expi_hsc:
    try:
        p_expi_hsc = hsc.HHSC(bus = 6, addr = 0x48, min_pressure = -160.0, max_pressure = 160.0, unit = 'mbar', transfer = 'A')
    except:
        print('HSC E sensor could not be initialized')
        time.sleep(0.1)
        try:
            p_expi_hsc = hsc.HHSC(bus = 6, addr = 0x48, min_pressure = -160.0, max_pressure = 160.0, unit = 'mbar', transfer = 'A')
        except:
            print('HSC E sensor could not be initialized')
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
    if enable_p_inspi_hsc:
        try:
            p_inspi_hsc.read_pressure()
        except:
            print('HSC I sensor read failed!')
            time.sleep(0.1)
    if enable_p_expi_hsc:
        try:
            p_expi_hsc.read_pressure()
        except:
            print('HSC E sensor read failed!')
            time.sleep(0.1)
    i = i+1

# Honeywell RSC
pressure_air, temperature_air, flow_air, flow_filtered_air, vol_air, pressure_offset_air, flow_offset_air = 0, 25, 0, 0, 0, 0, 0
pressure_expi, temperature_expi, flow_expi, flow_filtered_expi, vol_expi, pressure_offset_expi, flow_offset_expi = 0, 25, 0, 0, 0, 0, 0
pressure_O2, temperature_O2, flow_O2, flow_filtered_O2, vol_O2, pressure_offset_O2, flow_offset_O2 = 0, 25, 0, 0, 0, 0, 0
if enable_air_rsc: 
    flow_air_rsc = rsc.HRSC(spi_bus = 4)
    flow_air_rsc.sensor_info()
    flow_air_rsc.reset()
if enable_expi_rsc: 
    flow_expi_rsc = rsc.HRSC(spi_bus = 3)
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
p_e = 1000
if enable_p_ms5837: p = p_ms5837.pressure()
if enable_p_inspi_hsc: p = p0+p_inspi_hsc.conv_pressure_to_mbar(p_inspi_hsc.read_pressure())
if enable_p_expi_hsc: p_e = p0+p_expi_hsc.conv_pressure_to_mbar(p_expi_hsc.read_pressure())
p_prev = p
p_cycle_start = p
p0 = p
if enable_p0_ms5837: p0 = p0_ms5837.pressure() 
temperature = 25
temperature_e = 25
if enable_p_ms5837: temperature = p_ms5837.temperature()
if enable_p_inspi_hsc: temperature = p_inspi_hsc.read_temperature()
if enable_p_expi_hsc: temperature_e = p_expi_hsc.read_temperature()
temperature0 = temperature
if enable_p0_ms5837: temperature0 = p0_ms5837.temperature()
Ppeak_reached = False
PEEP_reached = False
inspi_end = False
inspi_duration_estim = inspi_duration
expi_duration_estim = expi_duration
cycle_duration_estim = cycle_duration

t_last_not_all_valves_closed = 0
t_all_valves_closed = 0

# File errors are not critical...
try:
    file = open('data.csv', 'a')
    file.write('t (in s);t0 (in s);p0 (in mbar);temperature0 (in C);p (in mbar);temperature (in C);p_e (in mbar);temperature_e (in C);select;Ppeak (in mbar);PEEP (in mbar);respi_rate (in breaths/min);inspi_ratio;flow_control_air (in L/min);flow_control_O2 (in L/min);mode;pwm0_ns;pwm1_ns;valve_air_val;valve_O2_val;valve_inspi_val;valve_expi_val;pressure_air (in mbar);pressure_expi (in mbar);pressure_O2 (in mbar);temperature_air (in C);temperature_expi (in C);temperature_O2 (in C);flow_air (in L/min);flow_expi (in L/min);flow_O2 (in L/min);flow_filtered_air (in L/min);flow_filtered_expi (in L/min);flow_filtered_O2 (in L/min);vol_air (in L);vol_expi (in L);vol_O2 (in L);\n')
except:
    pass

# Stop the startup beep
if enable_buzzer: buz_pwm.ChangeDutyCycle(0)

# Divisions by 0, NAN, INF...?

count = 0
while True:
    if (t-t_cycle_start < inspi_duration_estim):
        #if (inspi_duration_estim != 0): pwm0_ns = pwm0_ns_max-(pwm0_ns_max-pwm0_ns_min)*(t-t_cycle_start)/inspi_duration_estim
        #else: pwm0_ns = pwm0_ns_max
        #if (inspi_duration_estim != 0): pwm1_ns = pwm1_ns_min+(pwm1_ns_max-pwm1_ns_min)*(t-t_cycle_start)/inspi_duration_estim
        #else: pwm1_ns = pwm1_ns_min
        pwm0_ns = pwm0_ns_min
        pwm1_ns = pwm1_ns_max
        PEEP_reached = False
        if ((p-p0 > Ppeak) or (Ppeak_reached == True)): # Should close both valves to maintain Ppeak...
            Ppeak_reached = True
            pwm0_ns = pwm0_ns_max
            pwm1_ns = pwm1_ns_min
            valve_air_val = 0
            valve_O2_val = 0
            valve_inspi_val = GPIO.LOW
        else:
            valve_air_val = valve_air_val_max
            valve_O2_val = valve_O2_val_max
            valve_inspi_val = GPIO.HIGH
        valve_expi_val = GPIO.LOW
    else:
        #if (expi_duration_estim != 0): pwm0_ns = pwm0_ns_min+(pwm0_ns_max-pwm0_ns_min)*(t-t_cycle_start-inspi_duration_estim)/expi_duration_estim
        #else: pwm0_ns = pwm0_ns_max
        #if (expi_duration_estim != 0): pwm1_ns = pwm1_ns_max-(pwm1_ns_max-pwm1_ns_min)*(t-t_cycle_start-inspi_duration_estim)/expi_duration_estim
        #else: pwm1_ns = pwm1_ns_min
        pwm0_ns = pwm0_ns_max
        pwm1_ns = pwm1_ns_min
        Ppeak_reached = False
        if ((p-p0 < PEEP) or (PEEP_reached == True)): # Should close valves to maintain PEEP...
        #if (p-p0 < PEEP): # Should close valves to maintain PEEP...
            PEEP_reached = True
            valve_air_val = 0
            valve_O2_val = 0
            valve_expi_val = GPIO.LOW
        else:
            valve_air_val = 100 # Full to depress...
            valve_O2_val = 0 # Should not spend O2 to depress...
            valve_expi_val = GPIO.HIGH
        valve_inspi_val = GPIO.LOW

    if (mode == 2):
        # Override to only make O2:Air mix...
        # Balloon not handled...
        if (p-p0 > Ppeak*1.25): # Allow 25 % more since a control should be made later to limit the flow to stay below Ppeak...
            valve_air_val = 0
            valve_air_val_max = valves_max_init
            valve_O2_val = 0
            valve_O2_val_max = valves_max_init
        else:
            valve_air_val = valve_air_val_max
            valve_O2_val = valve_O2_val_max
        valve_inspi_val = GPIO.HIGH
        valve_expi_val = GPIO.HIGH

    # Actuators
    valve_air_val = min(100, max(0, valve_air_val))
    valve_O2_val = min(100, max(0, valve_O2_val))
    if not enable_hard_pwm_air_O2_valves:
        # Hardware PWM for balloon servos
        pwm0_ns = min(pwm0_ns_max, max(pwm0_ns_min, pwm0_ns))
        pwm1_ns = min(pwm1_ns_max, max(pwm1_ns_min, pwm1_ns))
        pwm0_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm0/duty_cycle'
        pwm1_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm1/duty_cycle'
        if not disable_hard_pwm:
            os.system(pwm0_cmd.format(math.trunc(pwm0_ns)))
            os.system(pwm1_cmd.format(math.trunc(pwm1_ns)))
        # Software PWM for air and O2 proportional valves
        valve_air_pwm.ChangeDutyCycle(valve_air_val) 
        valve_O2_pwm.ChangeDutyCycle(valve_O2_val) 
    else:
        # Hardware PWM for air and O2 proportional valves
        pwm0_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm0/duty_cycle'
        pwm1_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm1/duty_cycle'
        if not disable_hard_pwm:
            os.system(pwm0_cmd.format(math.trunc(min(pwm_period, max(0, pwm_period*valve_air_val/100)))))
            os.system(pwm1_cmd.format(math.trunc(min(pwm_period, max(0, pwm_period*valve_O2_val/100)))))
    GPIO.output(valve_inspi_pin, valve_inspi_val)
    if not enable_pwm_expi_valve:
        GPIO.output(valve_expi_pin, valve_expi_val)
    elif enable_pigpio_pwm:
        valve_expi_val = min(100, max(0, valve_expi_val))
        pi.set_PWM_dutycycle(valve_expi_pin, 255*valve_expi_val/100)
    else:
        valve_expi_val = min(100, max(0, valve_expi_val))
        valve_expi_pwm.ChangeDutyCycle(valve_expi_val)

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
                    flow_control_air = flow_control_air + flow_control_air_step
                    if (flow_control_air > flow_control_air_max): flow_control_air = flow_control_air_max
                if (select == 5):
                    flow_control_O2 = flow_control_O2 + flow_control_O2_step
                    if (flow_control_O2 > flow_control_O2_max): flow_control_O2 = flow_control_O2_max
                if (select == 6):
                    mode = mode + mode_step
                    if (mode > mode_max): mode = mode_max
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
                    flow_control_air = flow_control_air - flow_control_air_step
                    if (flow_control_air < flow_control_air_min): flow_control_air = flow_control_air_min
                if (select == 5):
                    flow_control_O2 = flow_control_O2 - flow_control_O2_step
                    if (flow_control_O2 < flow_control_O2_min): flow_control_O2 = flow_control_O2_min
                if (select == 6):
                    mode = mode - mode_step
                    if (mode < mode_min): mode = mode_min
 
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
                    if enable_buzzer: buz_pwm.ChangeDutyCycle(50)
                    exit(1)
        except:
            print('P sensor read failed!')
            time.sleep(0.1)
            if not p_ms5837.read(ms5837.OSR_256):
                print('P sensor read failed!')
                if enable_buzzer: buz_pwm.ChangeDutyCycle(50)
                exit(1)
    elif enable_p_inspi_hsc:
        try:
            p_diff_inspi_hsc, temp_tmp_inspi_hsc = p_inspi_hsc.read()
        except:
            print('HSC I sensor read failed!')
            time.sleep(0.1)
            try:
                p_diff_inspi_hsc, temp_tmp_inspi_hsc = p_inspi_hsc.read()
            except:
                print('HSC I sensor read failed!')
                time.sleep(0.1)
                if enable_buzzer: buz_pwm.ChangeDutyCycle(50)
                exit(1)
    else:
        time.sleep(0.005)
    if enable_p_expi_hsc:
        try:
            p_diff_expi_hsc, temp_tmp_expi_hsc = p_expi_hsc.read()
        except:
            print('HSC E sensor read failed!')
            time.sleep(0.1)
            try:
                p_diff_expi_hsc, temp_tmp_expi_hsc = p_expi_hsc.read()
            except:
                print('HSC E sensor read failed!')
                time.sleep(0.1)
                if enable_buzzer: buz_pwm.ChangeDutyCycle(50)
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
        vel_air = np.sign(pressure_air)*math.sqrt(2*(abs(pressure_air)*100.0)/(rho_air*((A1_air/float(A2_air))**2-1)))
        flow_air = A1_air*vel_air
    if enable_expi_rsc: 
        pressure_expi, temperature_expi = flow_expi_rsc.comp_readings(raw_pressure_expi, raw_temperature_expi)
        pressure_expi = flow_expi_rsc.conv_pressure_to_mbar(pressure_expi)
        pressure_expi = pressure_expi-pressure_offset_expi
        rho_expi = 1.292*(273.15/(273.15+temperature_expi)) # In kg/m3
        vel_expi = np.sign(pressure_expi)*math.sqrt(2*(abs(pressure_expi)*100.0)/(rho_expi*((A1_expi/float(A2_expi))**2-1)))
        flow_expi = A1_expi*vel_expi
    if enable_O2_rsc: 
        pressure_O2, temperature_O2 = flow_O2_rsc.comp_readings(raw_pressure_O2, raw_temperature_O2)
        pressure_O2 = flow_O2_rsc.conv_pressure_to_mbar(pressure_O2)
        pressure_O2 = pressure_O2-pressure_offset_O2
        rho_O2 = 1.292*(273.15/(273.15+temperature_O2)) # In kg/m3
        vel_O2 = np.sign(pressure_O2)*math.sqrt(2*(abs(pressure_O2)*100.0)/(rho_O2*((A1_O2/float(A2_O2))**2-1)))
        flow_O2 = A1_O2*vel_O2

    # Filters
    # Offset...
    #if ((valve_air_val <= 0) and (valve_O2_val <= 0) and (valve_inspi_val <= 0) and (valve_expi_val <= 0)):
    #    t_all_valves_closed = t
    #    if (t_all_valves_closed-t_last_not_all_valves_closed > valves_delay):
    #        flow_offset_air = (1-coef_offset_filter_flow)*flow_air+coef_offset_filter_flow*flow_offset_air
    #        flow_offset_expi = (1-coef_offset_filter_flow)*flow_expi+coef_offset_filter_flow*flow_offset_expi
    #        flow_offset_O2 = (1-coef_offset_filter_flow)*flow_O2+coef_offset_filter_flow*flow_offset_O2
    #else:
    #    t_last_not_all_valves_closed = t
    #flow_filtered_air = flow_air-flow_offset_air
    #flow_filtered_expi = flow_expi-flow_offset_expi
    #flow_filtered_O2 = flow_O2-flow_offset_O2
    # Low-pass or median filter...
    flow_filtered_air = (1-coef_filter_flow)*flow_air+coef_filter_flow*flow_filtered_air
    flow_filtered_expi = (1-coef_filter_flow)*flow_expi+coef_filter_flow*flow_filtered_expi
    flow_filtered_O2 = (1-coef_filter_flow)*flow_O2+coef_filter_flow*flow_filtered_O2
    # Volume computation
    if (abs(flow_filtered_air) > flow_thresh/60000.0): vol_air = vol_air+dt*flow_filtered_air
    if (abs(flow_filtered_expi) > flow_thresh/60000.0): vol_expi = vol_expi+dt*flow_filtered_expi
    if (abs(flow_filtered_O2) > flow_thresh/60000.0): vol_O2 = vol_O2+dt*flow_filtered_O2
    if (mode == 2):
        # Override to only make O2:Air mix...
        # Volume is not much meaningful...
        vol_air = 0
        vol_expi = 0
        vol_O2 = 0

    # Proportional valves control
    if (flow_control_air <= 0):
        valve_air_val_max = 1 # Min > 0 to not always disable proportional valves control...
    elif (flow_control_air < flow_control_air_max):
        if (Ppeak <= 0):
            valve_air_val_max = 1 # Min > 0 to not always disable proportional valves control...
        else:
            pressure_excess_ratio = ((p-p0)-Ppeak)/float(Ppeak)
            if (pressure_excess_ratio > 0): # Control to limit the flow to stay below Ppeak
                valve_air_val_max =  max(1, min(100, valve_air_val_max-valve_pressure_excess_control_air_coef*(flow_control_air/float(flow_control_air_max))*dt*pressure_excess_ratio)) # Min > 0 to not always disable proportional valves control...
            else:
                if (valve_air_val > 0): # Proportional valve control
                    err_flow_air = (flow_control_air-flow_filtered_air*60000.0)/float(flow_control_air)
                    valve_air_val_max =  max(1, min(100, valve_air_val_max+valve_flow_control_air_coef*dt*err_flow_air)) # Min > 0 to not always disable proportional valves control...
    else:
        valve_air_val_max = 100
    if (flow_control_O2 <= 0):
        valve_O2_val_max = 1 # Min > 0 to not always disable proportional valves control...
    elif (flow_control_O2 < flow_control_O2_max):
        if (Ppeak <= 0):
            valve_O2_val_max = 1 # Min > 0 to not always disable proportional valves control...
        else:
            pressure_excess_ratio = ((p-p0)-Ppeak)/float(Ppeak)
            if (pressure_excess_ratio > 0): # Control to limit the flow to stay below Ppeak
                valve_O2_val_max =  max(1, min(100, valve_O2_val_max-valve_pressure_excess_control_O2_coef*(flow_control_O2/float(flow_control_O2_max))*dt*pressure_excess_ratio)) # Min > 0 to not always disable proportional valves control...
            else:
                if (valve_O2_val > 0): # Proportional valve control
                    err_flow_O2 = (flow_control_O2-flow_filtered_O2*60000.0)/float(flow_control_O2)
                    valve_O2_val_max =  max(1, min(100, valve_O2_val_max+valve_flow_control_O2_coef*dt*err_flow_O2)) # Min > 0 to not always disable proportional valves control...
    else:
        valve_O2_val_max = 100

    # Log file
    # File errors are not critical...
    try:
        line = '{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};\n'
        file.write(line.format(t, t0, p0, temperature0, p, temperature, p_e, temperature_e, select, Ppeak, PEEP, respi_rate, inspi_ratio, flow_control_air, flow_control_O2, mode, 
                               pwm0_ns, pwm1_ns, valve_air_val, valve_O2_val, valve_inspi_val, valve_expi_val, 
                               pressure_air, pressure_expi, pressure_O2, temperature_air, temperature_expi, temperature_O2, 
                               flow_air*60000.0, flow_expi*60000.0, flow_O2*60000.0, flow_filtered_air*60000.0, flow_filtered_expi*60000.0, flow_filtered_O2*60000.0, vol_air*1000.0, vol_expi*1000.0, vol_O2*1000.0))
        file.flush()
    except:
        if enable_buzzer: buz_pwm.ChangeDutyCycle(50)

    if (debug): print('t-t0: %0.2f s \tdt: %0.3f s \tP0: %0.1f mbar \tT0: %0.2f C \tP: %0.1f mbar \tT: %0.2f C \tP A: %0.5f mbar \tP off. A: %0.5f mbar \tP E: %0.5f mbar \tP off. E: %0.5f mbar \tP O: %0.5f mbar \tP off. O: %0.5f mbar') % (
        t-t0, dt, p0, temperature0, p, temperature, pressure_air, pressure_offset_air, pressure_expi, pressure_offset_expi, pressure_O2, pressure_offset_O2) 
    
    # Prepare next loop...
    t_prev = t
    t = timer()
    dt = t-t_prev
    p_prev = p
    if enable_p_ms5837:
        p = p_ms5837.pressure()
        temperature = p_ms5837.temperature()
    elif enable_p_inspi_hsc:
        p = p0+p_diff_inspi_hsc
        temperature = temp_tmp_inspi_hsc
    if enable_p_expi_hsc:
        p_e = p0+p_diff_expi_hsc
        temperature_e = temp_tmp_expi_hsc
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
                        if enable_buzzer: buz_pwm.ChangeDutyCycle(50)
                        exit(1)
            except:
                print('P0 sensor read failed!')
                time.sleep(0.1)
                if not p0_ms5837.read(ms5837.OSR_256):
                    print('P0 sensor read failed!')
                    if enable_buzzer: buz_pwm.ChangeDutyCycle(50)
                    exit(1)
            p0 = p0_ms5837.pressure()
            temperature0 = p0_ms5837.temperature()
        vol_air = 0
        vol_expi = 0
        vol_O2 = 0
        inspi_end = False
    count = count+1
