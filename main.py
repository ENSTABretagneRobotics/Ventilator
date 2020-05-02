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
import signal
from timeit import default_timer as timer
from common import *

#sudo apt-get install python-smbus python-spidev python-pyqtgraph
# or
#sudo pip install smbus
#sudo pip install spidev
#sudo pip install pyqtgraph
# if it did not work and/or try also with python3 and pip3.
# Raspberry Pi 4 configuration (GPIO that will be used : 0 for software PWM 
# expiration valve, 1, 2, 3, 14, 15 for SPI3 Honeywell RSC for flow (O2), 4, 5
# for Honeywell HSC (inspiration) and I2C3 Bar02 (inspiration), 6 for digital 
# output inspiration valve, 7, 8, 9, 10, 11 for SPI0 Honeywell RSC for flow 
# (air), 12, 13 for PWM O2 and air valves/balloon PWM servomotors, 16 for 
# digital input UP button, 17 for digital input DOWN button, 18, 19, 20, 21, 27
# for SPI6 Honeywell RSC for flow (expiration), 22, 23 for Honeywell HSC 
# (expiration), I2C6 Bar02 (room), touchscreen, RTC clock, 24 for digital input
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
# User parameters
Ppeak = 30 # In mbar (= approx. cmH2O)
PEEP = 5 # In mbar (= approx. cmH2O)
respi_rate = 20 # In breaths/min
inspi_ratio = 0.3
flow_control_air = 20 # In L/min, >= flow_control_air_max means no limit...
flow_control_O2 = 20 # In L/min, >= flow_control_O2_max means no limit...
flow_control_expi = 0 # In L/min, >= flow_control_expi_max means no limit, <= 0 means no depress...
mode = 1 # 0 : ventilator, 1 : ventilator in assistance mode, 2 : only O2:Air mix
# User advanced parameters
PEEP_dec_rate = 90 # In %, to limit the maximum expiration flow before reaching the PEEP
Fl_PEEP_air = 100 # In % of flow_control_air, to help detecting inspiration after reaching the PEEP
Fl_PEEP_O2 = 100 # In % of flow_control_O2, to help detecting inspiration after reaching the PEEP
PEEP_inspi_detection_delta = 2.5 # In mbar (= approx. cmH2O)
vol_inspi_detection_delta = 15 # In ml
inspi_detection_delta_duration = 250 # In ms
flow_thresh = 4 # To avoid noise in volume computation, in L/min
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
flow_control_air_max = 150
flow_control_O2_step = 2
flow_control_O2_min = 0
flow_control_O2_max = 150
flow_control_expi_step = 2
flow_control_expi_min = 0
flow_control_expi_max = 150
mode_step = 1
mode_min = 0
mode_max = 2
PEEP_dec_rate_step = 2
PEEP_dec_rate_min = 60
PEEP_dec_rate_max = 100
Fl_PEEP_air_step = 5
Fl_PEEP_air_min = 0
Fl_PEEP_air_max = 1000
Fl_PEEP_O2_step = 5
Fl_PEEP_O2_min = 0
Fl_PEEP_O2_max = 1000
PEEP_inspi_detection_delta_step = 0.1
PEEP_inspi_detection_delta_min = 0
PEEP_inspi_detection_delta_max = 50
vol_inspi_detection_delta_step = 1
vol_inspi_detection_delta_min = 0
vol_inspi_detection_delta_max = 100
inspi_detection_delta_duration_step = 10
inspi_detection_delta_duration_min = 0
inspi_detection_delta_duration_max = 1000
flow_thresh_step = 0.25
flow_thresh_min = 0
flow_thresh_max = 50
enable_alarms = True
enable_pigpio = True
enable_buzzer = True
disable_hard_pwm = False
enable_p_ms5837 = False
enable_p0_ms5837 = False
enable_p_inspi_hsc = True
enable_p_expi_hsc = True
enable_air_rsc = True
enable_expi_rsc = True
enable_O2_rsc = True
#R1_air = 0.019100/2.0 # In m
R1_air = 0.009000/2.0 # In m
#R2_air = 0.011651/2.0 # In m
R2_air = 0.006500/2.0 # In m
R1_expi = 0.019100/2.0 # In m
#R1_expi = 0.009000/2.0 # In m
R2_expi = 0.011651/2.0 # In m
#R2_expi = 0.006500/2.0 # In m
#R2_expi = 0.008000/2.0 # In m
#R1_O2 = 0.019100/2.0 # In m
R1_O2 = 0.009000/2.0 # In m
#R2_O2 = 0.011651/2.0 # In m
R2_O2 = 0.006500/2.0 # In m
A1_air = math.pi*R1_air**2 # In m2
A2_air = math.pi*R2_air**2 # In m2
A1_expi = math.pi*R1_expi**2 # In m2
A2_expi = math.pi*R2_expi**2 # In m2
A1_O2 = math.pi*R1_O2**2 # In m2
A2_O2 = math.pi*R2_O2**2 # In m2
speed_rsc = 175 # In SPS
delay_rsc = 0.010 # In s
coef_filter_rsc = 0.95
nb_count_auto_zero_filter_rsc = 100 # 100
nb_count_offset_filter_rsc = 100 # 100
valves_pwm_freq = 800 # In Hz
valves_init = 50
valves_closed = 10
valves_delay = 0.020 # In s
PEEP_err = 2.0 # In mbar (= approx. cmH2O)
P_err = 50 # 7.5 # In mbar (= approx. cmH2O)
P_absolute_min = -5 # In mbar (= approx. cmH2O)
P_absolute_max = 150 # In mbar (= approx. cmH2O)
coef_offset_filter_flow = 0.99
coef_filter_flow = 0.0
valve_pressure_PEEP_control_expi_coef = 1.0
valve_pressure_PEEP_excess_control_air_coef = 16.0
valve_pressure_PEEP_excess_control_O2_coef = 16.0
valve_flow_PEEP_control_air_coef = 2.0
valve_flow_PEEP_control_O2_coef = 2.0
valve_depress_flow_control_expi_coef = 4.0
valve_pressure_excess_control_air_coef = 16.0
valve_pressure_excess_control_O2_coef = 16.0
valve_flow_control_air_coef = 2.0
valve_flow_control_O2_coef = 2.0
debug = False
###############################################################################

alarms = 0

GPIO.setwarnings(False)	
GPIO.setmode(GPIO.BCM)

# Software PWM init for buzzer
if enable_buzzer:
    buz_pin = 26
    GPIO.setup(buz_pin, GPIO.OUT)
    buz_pwm = GPIO.PWM(buz_pin, 4000)
    buz_pwm.start(50) # Startup beep...

if enable_pigpio: 
    os.system('pigpiod -x 0x0FFFFFFF') # To be able to use GPIO 0 and 1 also...
    time.sleep(0.2)
    import pigpio
    pi = pigpio.pi()
    if not pi.connected:
       print('Unable to connect to pigpio')
       exit(1)
    print('pigpio connected')

# Other PWM init
valve_air_pin = 12
if (flow_control_air < flow_control_air_max): valve_air_val_max = valves_init # Min > 0 to not always disable proportional valves control...
else: valve_air_val_max = 100
if (flow_control_air*Fl_PEEP_air*0.01 > 0): valve_air_val_max_PEEP = valves_init
else: valve_air_val_max_PEEP = 100
if (flow_control_expi < flow_control_expi_max):valve_air_val_max_depress = valves_init
else: valve_air_val_max_depress = 100
valve_air_val = 0
valve_air_val = min(100, max(0, valve_air_val))
valve_O2_pin = 13
if (flow_control_O2 < flow_control_O2_max): valve_O2_val_max = valves_init # Min > 0 to not always disable proportional valves control...
else: valve_O2_val_max = 100
if (flow_control_O2*Fl_PEEP_O2*0.01 > 0): valve_O2_val_max_PEEP = valves_init
else: valve_O2_val_max_PEEP = 100
valve_O2_val = 0
valve_O2_val = min(100, max(0, valve_O2_val))
if enable_pigpio:
    pi.set_mode(valve_air_pin, pigpio.OUTPUT)
    pi.set_PWM_frequency(valve_air_pin, valves_pwm_freq)
    pi.set_PWM_dutycycle(valve_air_pin, int(min(255, max(0, 255*valve_air_val/100))))
    pi.set_mode(valve_O2_pin, pigpio.OUTPUT)
    pi.set_PWM_frequency(valve_O2_pin, valves_pwm_freq)
    pi.set_PWM_dutycycle(valve_O2_pin, int(min(255, max(0, 255*valve_O2_val/100))))
else:
    # Hardware PWM for air and O2 proportional valves
    if not disable_hard_pwm:
        os.system('echo 0 > /sys/class/pwm/pwmchip0/export')
        os.system('echo 1 > /sys/class/pwm/pwmchip0/export')
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
        os.system('echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable')
        os.system('echo 1 > /sys/class/pwm/pwmchip0/pwm1/enable')

# Digital outputs (valves)
valve_inspi_pin = 6
valve_inspi_val = GPIO.LOW
GPIO.setup(valve_inspi_pin, GPIO.OUT, initial = valve_inspi_val)
valve_expi_pin = 0
valve_expi_val_max_PEEP = PEEP_dec_rate
valve_expi_val = 0
valve_expi_val = min(100, max(0, valve_expi_val))
if enable_pigpio:
    pi.set_mode(valve_expi_pin, pigpio.OUTPUT)
    pi.set_PWM_frequency(valve_expi_pin, valves_pwm_freq)
    pi.set_PWM_dutycycle(valve_expi_pin, int(min(255, max(0, 255-255*valve_expi_val/100)))) 
else:
    GPIO.setup(valve_expi_pin, GPIO.OUT)
    valve_expi_pwm = GPIO.PWM(valve_expi_pin, valves_pwm_freq)
    valve_expi_pwm.start(int(100-valve_expi_val))

# Digital inputs (buttons)
select = -1 # Index of the selected parameter that should be changed by up/down buttons
parameters = ['Ppeak', 'PEEP', 'respi_rate', 'inspi_ratio', 'flow_control_air', 'flow_control_O2', 'flow_control_expi', 'mode', 
              'PEEP_dec_rate', 'Fl_PEEP_air', 'Fl_PEEP_O2', 'PEEP_inspi_detection_delta', 'vol_inspi_detection_delta', 'inspi_detection_delta_duration', 'flow_thresh']
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
            p_inspi_hsc = hsc.HHSC(bus = 3, addr = 0x48, min_pressure = -160.0, max_pressure = 160.0, unit = 'mbar', transfer = 'A')
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
            p_diff_inspi_hsc, temp_inspi_hsc = p_inspi_hsc.read()
        except:
            print('HSC I sensor read failed!')
            time.sleep(0.1)
    if enable_p_expi_hsc:
        try:
            p_diff_expi_hsc, temp_expi_hsc = p_expi_hsc.read()
        except:
            print('HSC E sensor read failed!')
            time.sleep(0.1)
    i = i+1

# Honeywell RSC
pressure_air, temperature_air, flow_air, flow_filtered_air, vol_air, pressure_offset_air, flow_offset_air = 0, 25, 0, 0, 0, 0, 0
pressure_expi, temperature_expi, flow_expi, flow_filtered_expi, vol_expi, pressure_offset_expi, flow_offset_expi = 0, 25, 0, 0, 0, 0, 0
pressure_O2, temperature_O2, flow_O2, flow_filtered_O2, vol_O2, pressure_offset_O2, flow_offset_O2 = 0, 25, 0, 0, 0, 0, 0
if enable_air_rsc: 
    flow_air_rsc = rsc.HRSC(spi_bus = 0)
    flow_air_rsc.sensor_info()
    flow_air_rsc.reset()
if enable_expi_rsc: 
    flow_expi_rsc = rsc.HRSC(spi_bus = 3)
    flow_expi_rsc.sensor_info()
    flow_expi_rsc.reset()
if enable_O2_rsc: 
    flow_O2_rsc = rsc.HRSC(spi_bus = 4)
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
if enable_p_inspi_hsc: p = p0+p_diff_inspi_hsc
if enable_p_expi_hsc: p_e = p0+p_diff_expi_hsc
p_prev = p
p_cycle_start = p
p0 = p
if enable_p0_ms5837: p0 = p0_ms5837.pressure() 
temperature = 25
temperature_e = 25
if enable_p_ms5837: temperature = p_ms5837.temperature()
if enable_p_inspi_hsc: temperature = temp_inspi_hsc
if enable_p_expi_hsc: temperature_e = temp_expi_hsc
temperature0 = temperature
if enable_p0_ms5837: temperature0 = p0_ms5837.temperature()
Ppeak_reached = False
PEEP_reached = False
inspi_end = False
force_new_cycle = False
t_hist = []
flow_hist = []
vol_hist = []

t_last_not_all_valves_closed = t0
t_all_valves_closed = t0
t_valve_air_closed = t0
t_valve_O2_closed = t0
t_valve_expi_closed = t0
t_alarms = t0

# Stop the startup beep
if enable_buzzer: buz_pwm.ChangeDutyCycle(0)

# File errors are not critical...
try:
    file = open('data.csv', 'a')
    file.write('t (in s);t0 (in s);p0 (in mbar);temperature0 (in C);p (in mbar);temperature (in C);p_e (in mbar);temperature_e (in C);alarms;select;'
                'Ppeak (in mbar);PEEP (in mbar);respi_rate (in breaths/min);inspi_ratio;flow_control_air (in L/min);flow_control_O2 (in L/min);flow_control_expi (in L/min);mode;'
                'PEEP_dec_rate (in %);Fl_PEEP_air (in %);Fl_PEEP_O2 (in %);PEEP_inspi_detection_delta (in mbar);vol_inspi_detection_delta (in ml);inspi_detection_delta_duration (in ms);flow_thresh (in L/min);'
                'pwm0_ns;pwm1_ns;valve_air_val;valve_O2_val;valve_inspi_val;valve_expi_val;'
                'pressure_air (in mbar);pressure_expi (in mbar);pressure_O2 (in mbar);temperature_air (in C);temperature_expi (in C);temperature_O2 (in C);'
                'flow_air (in L/min);flow_expi (in L/min);flow_O2 (in L/min);flow_filtered_air (in L/min);flow_filtered_expi (in L/min);flow_filtered_O2 (in L/min);vol_air (in L);vol_expi (in L);vol_O2 (in L);\n')
except:
    if enable_alarms:
        t_alarms = t
        alarms = alarms | OS_ALARM

def signal_handler(sig, frame):
    signal.signal(signal.SIGINT, signal.SIG_IGN) # To avoid interruption by other CTRL+C...
    print('Exiting...')
    valve_air_val = 0
    valve_O2_val = 0
    if enable_pigpio:
        pi.set_PWM_dutycycle(valve_air_pin, int(min(255, max(0, 255*valve_air_val/100))))
        pi.set_PWM_dutycycle(valve_O2_pin, int(min(255, max(0, 255*valve_O2_val/100))))
    else:
        # Hardware PWM for air and O2 proportional valves
        pwm0_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm0/duty_cycle'
        pwm1_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm1/duty_cycle'
        if not disable_hard_pwm:
            os.system(pwm0_cmd.format(math.trunc(min(pwm_period, max(0, pwm_period*valve_air_val/100)))))
            os.system(pwm1_cmd.format(math.trunc(min(pwm_period, max(0, pwm_period*valve_O2_val/100)))))
    valve_inspi_val = GPIO.HIGH
    GPIO.output(valve_inspi_pin, valve_inspi_val)
    valve_expi_val = 100
    if enable_pigpio:
        pi.set_PWM_dutycycle(valve_expi_pin, int(min(255, max(0, 255-255*valve_expi_val/100))))
    else:
        valve_expi_pwm.ChangeDutyCycle(int(100-valve_expi_val))
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler) # To be able to put actuators in a safe state when using CTRL+C

# Divisions by 0, NAN, INF, int divisions...?

count = 0
while True:
    if (t-t_cycle_start < inspi_duration):
        #if (inspi_duration != 0): pwm0_ns = pwm0_ns_max-(pwm0_ns_max-pwm0_ns_min)*(t-t_cycle_start)/inspi_duration
        #else: pwm0_ns = pwm0_ns_max
        #if (inspi_duration != 0): pwm1_ns = pwm1_ns_min+(pwm1_ns_max-pwm1_ns_min)*(t-t_cycle_start)/inspi_duration
        #else: pwm1_ns = pwm1_ns_min
        pwm0_ns = pwm0_ns_min
        pwm1_ns = pwm1_ns_max
        PEEP_reached = False
        if ((p-p0 > Ppeak) or (Ppeak_reached == True)): # Should close valves to maintain Ppeak...
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
        valve_expi_val = 0
    else:
        #if (expi_duration != 0): pwm0_ns = pwm0_ns_min+(pwm0_ns_max-pwm0_ns_min)*(t-t_cycle_start-inspi_duration)/expi_duration
        #else: pwm0_ns = pwm0_ns_max
        #if (expi_duration != 0): pwm1_ns = pwm1_ns_max-(pwm1_ns_max-pwm1_ns_min)*(t-t_cycle_start-inspi_duration)/expi_duration
        #else: pwm1_ns = pwm1_ns_min
        pwm0_ns = pwm0_ns_max
        pwm1_ns = pwm1_ns_min
        Ppeak_reached = False
        if ((p-p0 < PEEP) or (PEEP_reached == True)): # Should try to maintain PEEP...
            if not PEEP_reached:
                t_PEEP_reached = t
            PEEP_reached = True
            if ((flow_control_air*Fl_PEEP_air*0.01 <= 0) and (flow_control_O2*Fl_PEEP_O2*0.01 <= 0)):
                valve_air_val = 0
                valve_O2_val = 0
                valve_inspi_val = GPIO.LOW
                valve_expi_val = 0
                if ((mode == 1) and (t-t_PEEP_reached > inspi_detection_delta_duration*0.001) and (p-p0 < PEEP-PEEP_inspi_detection_delta)): # Attempt to detect inspiration in assist mode...
                    force_new_cycle = True
                    print('Inspiration detected at t=%f s' % (t-t0))
            else:
                if ((mode == 1) and (t-t_PEEP_reached > inspi_detection_delta_duration*0.001) and (len(t_hist) > 0) and (vol_hist[-1]-vol_hist[0] > float(vol_inspi_detection_delta)/1000000.0)): # Attempt to detect inspiration in assist mode...
                    force_new_cycle = True
                    print('Inspiration detected at t=%f s' % (t-t0))
                valve_inspi_val = GPIO.HIGH
                err_pressure_PEEP = (p-p0)-PEEP
                valve_expi_val_max_PEEP = max(PEEP_dec_rate_min, min(PEEP_dec_rate, valve_expi_val_max_PEEP+valve_pressure_PEEP_control_expi_coef*dt*err_pressure_PEEP))
                valve_expi_val = valve_expi_val_max_PEEP
            if (flow_control_air*Fl_PEEP_air*0.01 > 0):
                err_pressure_PEEP_excess = (p-p0)-PEEP
                if (err_pressure_PEEP_excess > 0): # Control to limit the flow to stay below PEEP
                    valve_air_val_max_PEEP = max(1, min(100, valve_air_val_max_PEEP-valve_pressure_PEEP_excess_control_air_coef*(flow_control_air*Fl_PEEP_air*0.01/float(flow_control_air_max))*dt*err_pressure_PEEP_excess)) # Min > 0 to not always disable proportional valves control...
                else:
                    err_flow_PEEP_air = flow_control_air*Fl_PEEP_air*0.01-flow_filtered_air*60000.0
                    if ((err_flow_PEEP_air < 0) or (t-t_valve_air_closed > valves_delay)): # Avoid accumulating error when flow is 0 just due to valve delay...
                        valve_air_val_max_PEEP = max(1, min(100, valve_air_val_max_PEEP+valve_flow_PEEP_control_air_coef*dt*err_flow_PEEP_air)) # Min > 0 to not always disable proportional valves control...
                valve_air_val = valve_air_val_max_PEEP
            else:
                valve_air_val = 0
            if (flow_control_O2*Fl_PEEP_O2*0.01 > 0):
                err_pressure_PEEP_excess = (p-p0)-PEEP
                if (err_pressure_PEEP_excess > 0): # Control to limit the flow to stay below PEEP
                    valve_O2_val_max_PEEP = max(1, min(100, valve_O2_val_max_PEEP-valve_pressure_PEEP_excess_control_O2_coef*(flow_control_O2*Fl_PEEP_O2*0.01/float(flow_control_O2_max))*dt*err_pressure_PEEP_excess)) # Min > 0 to not always disable proportional valves control...
                else:
                    err_flow_PEEP_O2 = flow_control_O2*Fl_PEEP_O2*0.01-flow_filtered_O2*60000.0
                    if ((err_flow_PEEP_O2 < 0) or (t-t_valve_O2_closed > valves_delay)): # Avoid accumulating error when flow is 0 just due to valve delay...
                        valve_O2_val_max_PEEP = max(1, min(100, valve_O2_val_max_PEEP+valve_flow_PEEP_control_O2_coef*dt*err_flow_PEEP_O2)) # Min > 0 to not always disable proportional valves control...
                valve_O2_val = valve_O2_val_max_PEEP
            else:
                valve_O2_val = 0
        else:
            if (p-p0 > PEEP+PEEP_err): 
                if (flow_control_expi <= 0):
                    valve_air_val = 0 # Do not depress...
                elif (flow_control_expi < flow_control_expi_max): # Ensure it does not depress too fast...
                    err_flow_expi = flow_control_expi-flow_filtered_expi*60000.0
                    if ((err_flow_expi < 0) or (t-t_valve_air_closed > valves_delay)): # Avoid accumulating error when flow is 0 just due to valve delay...
                        valve_air_val_max_depress = max(1, min(100, valve_air_val_max_depress+valve_depress_flow_control_expi_coef*dt*err_flow_expi)) # Min > 0 to not always disable proportional valves control...
                        valve_air_val = valve_air_val_max_depress
                else:
                    valve_air_val = 100 # Full to depress...
            else: 
                if (flow_control_expi <= 0): valve_air_val = 0 # Do not depress...
                else: valve_air_val = valves_init # We are close to PEEP so no need to depress much...
            valve_O2_val = 0 # Should not spend O2 to depress...
            valve_inspi_val = GPIO.LOW # To be able to use air to depress...
            valve_expi_val = PEEP_dec_rate

    if (mode == 2):
        # Override to only make O2:Air mix...
        # Balloon not handled...
        if (p-p0 > Ppeak+P_err): # Allow some error since a control should be made later to limit the flow to stay below Ppeak...
            valve_air_val = 0
            valve_air_val_max = valves_init
            valve_O2_val = 0
            valve_O2_val_max = valves_init
        else:
            valve_air_val = valve_air_val_max
            valve_O2_val = valve_O2_val_max
        valve_inspi_val = GPIO.HIGH
        valve_expi_val = 100

    # Pressure should never be too low or too high...
    if (((p-p0 > Ppeak+P_err) or (p-p0 < PEEP-P_err) or (p-p0 > P_absolute_max) or (p-p0 < P_absolute_min)) or \
        ((enable_p_expi_hsc) and ((p_e-p0 > Ppeak+P_err) or (p_e-p0 < PEEP-P_err) or (p_e-p0 > P_absolute_max) or (p_e-p0 < P_absolute_min)))): 
        if enable_alarms:
            t_alarms = t
            alarms = alarms | PRESSURE_ALARM
        valve_air_val = 0
        valve_O2_val = 0
        valve_inspi_val = GPIO.HIGH
        valve_expi_val = 100

    if (mode != 2):
        if ((t-t_cycle_start > 0.9*inspi_duration) and (not Ppeak_reached)):
            if enable_alarms:
                t_alarms = t
                alarms = alarms | PPEAK_ALARM
        if ((t-t_cycle_start > inspi_duration+0.9*expi_duration) and (not PEEP_reached)):
            if enable_alarms:
                t_alarms = t
                alarms = alarms | PEEP_ALARM

    if (alarms != 0):
        if (t-t_alarms > 5): # Clear alarms and stop buzzer after some time if no recent alarm
            alarms = 0
            if enable_buzzer: buz_pwm.ChangeDutyCycle(0)
        else:
            if enable_buzzer: buz_pwm.ChangeDutyCycle(50)

    # Actuators
    valve_air_val = min(100, max(0, valve_air_val))
    if (valve_air_val <= valves_closed): t_valve_air_closed = t
    valve_O2_val = min(100, max(0, valve_O2_val))
    if (valve_O2_val <= valves_closed): t_valve_O2_closed = t
    if enable_pigpio:
        pi.set_PWM_dutycycle(valve_air_pin, int(min(255, max(0, 255*valve_air_val/100))))
        pi.set_PWM_dutycycle(valve_O2_pin, int(min(255, max(0, 255*valve_O2_val/100))))
    else:
        # Hardware PWM for air and O2 proportional valves
        pwm0_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm0/duty_cycle'
        pwm1_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm1/duty_cycle'
        if not disable_hard_pwm:
            os.system(pwm0_cmd.format(math.trunc(min(pwm_period, max(0, pwm_period*valve_air_val/100)))))
            os.system(pwm1_cmd.format(math.trunc(min(pwm_period, max(0, pwm_period*valve_O2_val/100)))))
    GPIO.output(valve_inspi_pin, valve_inspi_val)
    valve_expi_val = min(100, max(0, valve_expi_val))
    if (valve_expi_val <= valves_closed): t_valve_expi_closed = t
    if enable_pigpio:
        pi.set_PWM_dutycycle(valve_expi_pin, int(min(255, max(0, 255-255*valve_expi_val/100))))
    else:
        valve_expi_pwm.ChangeDutyCycle(int(100-valve_expi_val))

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
                    flow_control_expi = flow_control_expi + flow_control_expi_step
                    if (flow_control_expi > flow_control_expi_max): flow_control_expi = flow_control_expi_max
                if (select == 7):
                    mode = mode + mode_step
                    if (mode > mode_max): mode = mode_max
                if (select == 8):
                    PEEP_dec_rate = PEEP_dec_rate + PEEP_dec_rate_step
                    if (PEEP_dec_rate > PEEP_dec_rate_max): PEEP_dec_rate = PEEP_dec_rate_max
                if (select == 9):
                    Fl_PEEP_air = Fl_PEEP_air + Fl_PEEP_air_step
                    if (Fl_PEEP_air > Fl_PEEP_air_max): Fl_PEEP_air = Fl_PEEP_air_max
                if (select == 10):
                    Fl_PEEP_O2 = Fl_PEEP_O2 + Fl_PEEP_O2_step
                    if (Fl_PEEP_O2 > Fl_PEEP_O2_max): Fl_PEEP_O2 = Fl_PEEP_O2_max
                if (select == 11):
                    PEEP_inspi_detection_delta = PEEP_inspi_detection_delta + PEEP_inspi_detection_delta_step
                    if (PEEP_inspi_detection_delta > PEEP_inspi_detection_delta_max): PEEP_inspi_detection_delta = PEEP_inspi_detection_delta_max
                if (select == 12):
                    vol_inspi_detection_delta = vol_inspi_detection_delta + vol_inspi_detection_delta_step
                    if (vol_inspi_detection_delta > vol_inspi_detection_delta_max): vol_inspi_detection_delta = vol_inspi_detection_delta_max
                if (select == 13):
                    inspi_detection_delta_duration = inspi_detection_delta_duration + inspi_detection_delta_duration_step
                    if (inspi_detection_delta_duration > inspi_detection_delta_duration_max): inspi_detection_delta_duration = inspi_detection_delta_duration_max
                if (select == 14):
                    flow_thresh = flow_thresh + flow_thresh_step
                    if (flow_thresh > flow_thresh_max): flow_thresh = flow_thresh_max
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
                    flow_control_expi = flow_control_expi - flow_control_expi_step
                    if (flow_control_expi < flow_control_expi_min): flow_control_expi = flow_control_expi_min
                if (select == 7):
                    mode = mode - mode_step
                    if (mode < mode_min): mode = mode_min
                if (select == 8):
                    PEEP_dec_rate = PEEP_dec_rate - PEEP_dec_rate_step
                    if (PEEP_dec_rate < PEEP_dec_rate_min): PEEP_dec_rate = PEEP_dec_rate_min
                if (select == 9):
                    Fl_PEEP_air = Fl_PEEP_air - Fl_PEEP_air_step
                    if (Fl_PEEP_air < Fl_PEEP_air_min): Fl_PEEP_air = Fl_PEEP_air_min
                if (select == 10):
                    Fl_PEEP_O2 = Fl_PEEP_O2 - Fl_PEEP_O2_step
                    if (Fl_PEEP_O2 < Fl_PEEP_O2_min): Fl_PEEP_O2 = Fl_PEEP_O2_min
                if (select == 11):
                    PEEP_inspi_detection_delta = PEEP_inspi_detection_delta - PEEP_inspi_detection_delta_step
                    if (PEEP_inspi_detection_delta < PEEP_inspi_detection_delta_min): PEEP_inspi_detection_delta = PEEP_inspi_detection_delta_min
                if (select == 12):
                    vol_inspi_detection_delta = vol_inspi_detection_delta - vol_inspi_detection_delta_step
                    if (vol_inspi_detection_delta < vol_inspi_detection_delta_min): vol_inspi_detection_delta = vol_inspi_detection_delta_min
                if (select == 13):
                    inspi_detection_delta_duration = inspi_detection_delta_duration - inspi_detection_delta_duration_step
                    if (inspi_detection_delta_duration < inspi_detection_delta_duration_min): inspi_detection_delta_duration = inspi_detection_delta_duration_min
                if (select == 14):
                    flow_thresh = flow_thresh - flow_thresh_step
                    if (flow_thresh < flow_thresh_min): flow_thresh = flow_thresh_min
 
    # Sensors
    if enable_air_rsc: 
        try:
            flow_air_rsc.pressure_request()
        except:
            print('RSC A sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            flow_air_rsc.pressure_request()
    if enable_expi_rsc: 
        try:
            flow_expi_rsc.pressure_request()
        except:
            print('RSC E sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            flow_expi_rsc.pressure_request()
    if enable_O2_rsc: 
        try:
            flow_O2_rsc.pressure_request()
        except:
            print('RSC O sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            flow_O2_rsc.pressure_request()
    time.sleep(max(0, delay_rsc-0.005))
    if enable_p_ms5837:
        try:
            if not p_ms5837.read(ms5837.OSR_256):
                print('P sensor read failed!')
                if enable_alarms:
                    t_alarms = t
                    alarms = alarms | HARDWARE_ALARM
                time.sleep(0.1)
                if not p_ms5837.read(ms5837.OSR_256):
                    print('P sensor read failed!')
                    exit(1)
        except:
            print('P sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            if not p_ms5837.read(ms5837.OSR_256):
                print('P sensor read failed!')
                exit(1)
    elif enable_p_inspi_hsc:
        try:
            p_diff_inspi_hsc, temp_inspi_hsc = p_inspi_hsc.read()
        except:
            print('HSC I sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            p_diff_inspi_hsc, temp_inspi_hsc = p_inspi_hsc.read()
        time.sleep(0.005)
    else:
        time.sleep(0.005)
    if enable_p_expi_hsc:
        try:
            p_diff_expi_hsc, temp_expi_hsc = p_expi_hsc.read()
        except:
            print('HSC E sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            p_diff_expi_hsc, temp_expi_hsc = p_expi_hsc.read()
    if enable_air_rsc: 
        try:
            raw_pressure_air = flow_air_rsc.pressure_reply()
        except:
            print('RSC A sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            raw_pressure_air = flow_air_rsc.pressure_reply()
    if enable_expi_rsc: 
        try:
            raw_pressure_expi = flow_expi_rsc.pressure_reply()
        except:
            print('RSC E sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            raw_pressure_expi = flow_expi_rsc.pressure_reply()
    if enable_O2_rsc: 
        try:
            raw_pressure_O2 = flow_O2_rsc.pressure_reply()
        except:
            print('RSC O sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            raw_pressure_O2 = flow_O2_rsc.pressure_reply()
    if enable_air_rsc: 
        try:
            flow_air_rsc.temperature_request()
        except:
            print('RSC A sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            flow_air_rsc.temperature_request()
    if enable_expi_rsc: 
        try:
            flow_expi_rsc.temperature_request()
        except:
            print('RSC E sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            flow_expi_rsc.temperature_request()
    if enable_O2_rsc: 
        try:
            flow_O2_rsc.temperature_request()
        except:
            print('RSC O sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            flow_O2_rsc.temperature_request()
    time.sleep(delay_rsc)
    if enable_air_rsc: 
        try:
            raw_temperature_air = flow_air_rsc.temperature_reply()
        except:
            print('RSC A sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            raw_temperature_air = flow_air_rsc.temperature_reply()
    if enable_expi_rsc: 
        try:
            raw_temperature_expi = flow_expi_rsc.temperature_reply()
        except:
            print('RSC E sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
            time.sleep(0.1)
            raw_temperature_expi = flow_expi_rsc.temperature_reply()
    if enable_O2_rsc: 
        try:
            raw_temperature_O2 = flow_O2_rsc.temperature_reply()
        except:
            print('RSC O sensor read failed!')
            if enable_alarms:
                t_alarms = t
                alarms = alarms | HARDWARE_ALARM
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
        err_pressure_excess = (p-p0)-Ppeak
        if (err_pressure_excess > 0): # Control to limit the flow to stay below Ppeak
            valve_air_val_max = max(1, min(100, valve_air_val_max-valve_pressure_excess_control_air_coef*(flow_control_air/float(flow_control_air_max))*dt*err_pressure_excess)) # Min > 0 to not always disable proportional valves control...
        else:
            if ((valve_air_val > 0) and ((mode == 2) or (t-t_cycle_start < inspi_duration))): # Proportional valve control
                err_flow_air = flow_control_air-flow_filtered_air*60000.0
                if ((err_flow_air < 0) or (t-t_valve_air_closed > valves_delay)): # Avoid accumulating error when flow is 0 just due to valve delay...
                    valve_air_val_max = max(1, min(100, valve_air_val_max+valve_flow_control_air_coef*dt*err_flow_air)) # Min > 0 to not always disable proportional valves control...
    else:
        valve_air_val_max = 100
    if (flow_control_O2 <= 0):
        valve_O2_val_max = 1 # Min > 0 to not always disable proportional valves control...
    elif (flow_control_O2 < flow_control_O2_max):
        err_pressure_excess = (p-p0)-Ppeak
        if (err_pressure_excess > 0): # Control to limit the flow to stay below Ppeak
            valve_O2_val_max =  max(1, min(100, valve_O2_val_max-valve_pressure_excess_control_O2_coef*(flow_control_O2/float(flow_control_O2_max))*dt*err_pressure_excess)) # Min > 0 to not always disable proportional valves control...
        else:
            if ((valve_O2_val > 0) and ((mode == 2) or (t-t_cycle_start < inspi_duration))): # Proportional valve control
                err_flow_O2 = flow_control_O2-flow_filtered_O2*60000.0
                if ((err_flow_O2 < 0) or (t-t_valve_O2_closed > valves_delay)): # Avoid accumulating error when flow is 0 just due to valve delay...
                    valve_O2_val_max = max(1, min(100, valve_O2_val_max+valve_flow_control_O2_coef*dt*err_flow_O2)) # Min > 0 to not always disable proportional valves control...
    else:
        valve_O2_val_max = 100

    # Log file
    # File errors are not critical...
    try:
        line = '{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};\n'
        file.write(line.format(t, t0, p0, temperature0, p, temperature, p_e, temperature_e, alarms, select, 
                               Ppeak, PEEP, respi_rate, inspi_ratio, flow_control_air, flow_control_O2, flow_control_expi, mode, 
                               PEEP_dec_rate, Fl_PEEP_air, Fl_PEEP_O2, PEEP_inspi_detection_delta, vol_inspi_detection_delta, inspi_detection_delta_duration, flow_thresh, 
                               pwm0_ns, pwm1_ns, valve_air_val, valve_O2_val, valve_inspi_val, valve_expi_val, 
                               pressure_air, pressure_expi, pressure_O2, temperature_air, temperature_expi, temperature_O2, 
                               flow_air*60000.0, flow_expi*60000.0, flow_O2*60000.0, flow_filtered_air*60000.0, flow_filtered_expi*60000.0, flow_filtered_O2*60000.0, vol_air*1000.0, vol_expi*1000.0, vol_O2*1000.0))
        file.flush()
    except:
        if enable_alarms:
            t_alarms = t
            alarms = alarms | OS_ALARM

    if (debug): print(('t-t0: %0.2f s \tdt: %0.3f s \tP0: %0.1f mbar \tT0: %0.2f C \tP: %0.1f mbar \tT: %0.2f C \tP A: %0.5f mbar \tP off. A: %0.5f mbar \tP E: %0.5f mbar \tP off. E: %0.5f mbar \tP O: %0.5f mbar \tP off. O: %0.5f mbar') % (
        t-t0, dt, p0, temperature0, p, temperature, pressure_air, pressure_offset_air, pressure_expi, pressure_offset_expi, pressure_O2, pressure_offset_O2)) 
    
    # Prepare next loop...
    respi_rate_converted = respi_rate*(1.0/60.0) # In breaths/s
    cycle_duration = 1.0/respi_rate_converted
    inspi_duration = inspi_ratio*cycle_duration
    expi_duration = cycle_duration-inspi_duration
    t_hist.append(t)
    flow_hist.append(flow_filtered_air+flow_filtered_O2+flow_filtered_expi)
    vol_hist.append(vol_air+vol_O2+vol_expi)
    while ((len(t_hist) > 0) and (t-t_hist[0] > inspi_detection_delta_duration*0.001)): 
        t_hist.pop(0)
        flow_hist.pop(0)
        vol_hist.pop(0)
    t_prev = t
    t = timer()
    dt = t-t_prev
    p_prev = p
    if enable_p_ms5837:
        p = p_ms5837.pressure()
        temperature = p_ms5837.temperature()
    elif enable_p_inspi_hsc:
        p = p0+p_diff_inspi_hsc
        temperature = temp_inspi_hsc
    if enable_p_expi_hsc:
        p_e = p0+p_diff_expi_hsc
        temperature_e = temp_expi_hsc
    if (t-t_cycle_start > cycle_duration) or force_new_cycle:
        force_new_cycle = False
        t_cycle_start = t
        p_cycle_start = p
        if enable_p0_ms5837:   
            try:
                if not p0_ms5837.read(ms5837.OSR_256):
                    print('P0 sensor read failed!')
                    if enable_alarms:
                        t_alarms = t
                        alarms = alarms | HARDWARE_ALARM
                    time.sleep(0.1)
                    if not p0_ms5837.read(ms5837.OSR_256):
                        print('P0 sensor read failed!')
                        exit(1)
            except:
                print('P0 sensor read failed!')
                if enable_alarms:
                    t_alarms = t
                    alarms = alarms | HARDWARE_ALARM
                time.sleep(0.1)
                if not p0_ms5837.read(ms5837.OSR_256):
                    print('P0 sensor read failed!')
                    exit(1)
            p0 = p0_ms5837.pressure()
            temperature0 = p0_ms5837.temperature()
        vol_air = 0
        vol_expi = 0
        vol_O2 = 0
        inspi_end = False
    count = count+1
