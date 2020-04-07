#!/usr/bin/python
import ms5837
import RPi.GPIO as GPIO
import math
import sys
import os
import time
import threading
from timeit import default_timer as timer

from matplotlib.pyplot import *

#sudo apt-get install python-smbus
# or
#sudo pip install smbus
# if it did not work.
# Raspberry Pi configuration :
#sudo nano /boot/config.txt
# Add in /boot/config.txt
#dtparam=i2c_arm=on
#dtoverlay=i2c-gpio,bus=6,i2c_gpio_delay_us=1,i2c_gpio_sda=22,i2c_gpio_scl=23
#dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
#dtparam=spi=on
# Then reboot and
#sudo python main.py

# Parameters
###############################################################################
Ppeak = 25 # In mbar (= approx. cmH2O)
PEEP = 5 # In mbar (= approx. cmH2O)
breath_freq = 15 # In cycles/min
inspi_ratio = 1.0/3.0
#trim PWM value start, end depending on balloon size...
pwm0_ns_min = 1000000
pwm0_ns_max = 1500000
pwm1_ns_min = 1500000
pwm1_ns_max = 2000000
enable_p0_sensor = False
enable_old_gui = False
scalex = 5
scaley = 50
offsety = 0
###############################################################################

# Software PWM init (for buzzer and status LED)
GPIO.setwarnings(False)	
GPIO.setmode(GPIO.BCM)
buz_pin = 26
GPIO.setup(buz_pin, GPIO.OUT)
buz_pwm = GPIO.PWM(buz_pin, 4000)
buz_pwm.start(50)
led_pin = 24
GPIO.setup(led_pin, GPIO.OUT)
led_pwm = GPIO.PWM(led_pin, 1000)
led_pwm.start(100)

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
GPIO.setup(valve_inspi_pin, GPIO.OUT, initial = GPIO.LOW)
valve_expi_pin = 5
GPIO.setup(valve_expi_pin, GPIO.OUT, initial = GPIO.LOW)

# Digital inputs (buttons)
select = 0
# TODO

#p_sensor = ms5837.MS5837(model=ms5837.MS5837_MODEL_30BA, bus=0)
p_sensor = ms5837.MS5837_02BA(bus=1)
if not p_sensor.init():
    print('P sensor could not be initialized')
    exit(1)
if not p_sensor.read():
    print('P sensor read failed!')
    exit(1)

if enable_p0_sensor:
    p0_sensor = ms5837.MS5837_02BA(bus=6)
    if not p0_sensor.init():
        print('P0 sensor could not be initialized')
        exit(1)
    if not p0_sensor.read():
        print('P0 sensor read failed!')
        exit(1)

breath_freq_converted = breath_freq*(1.0/60.0) # In cycles/s
cycle_duration = 1.0/breath_freq_converted
inspi_duration = inspi_ratio*cycle_duration
expi_duration = cycle_duration-inspi_duration

t = timer()
t_prev = t
t0 = t
t_cycle_start = t
p = p_sensor.pressure() # The very first pressure measurements might be wrong, but it does not seem to be the case...
p_prev = p
p_cycle_start = p
if enable_p0_sensor: p0 = p0_sensor.pressure() 
else: p0 = p
temperature = p_sensor.temperature()
Ppeak_reached = False
PEEP_reached = False
inspi_end = False
inspi_duration_estim = inspi_duration
expi_duration_estim = expi_duration
cycle_duration_estim = cycle_duration

pwm0_ns = pwm0_ns_max
pwm1_ns = pwm1_ns_min

file = open('data.csv', 'a')
file.write('t (in s);p0 (in mbar);p (in mbar);temperature (in C);select;Ppeak (in mbar);PEEP (in mbar);breath_freq (in cycles/min);inspi_ratio')

if enable_old_gui:
    fig = figure('Pressure')
    clf()
    axis('auto')
    offsetx = -scalex

# Divisions by 0...

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
            GPIO.output(valve_inspi_pin, GPIO.LOW)
        else:
            GPIO.output(valve_inspi_pin, GPIO.HIGH)
        GPIO.output(valve_expi_pin, GPIO.LOW)
    else:
        #pwm0_ns = pwm0_ns_min+(pwm0_ns_max-pwm0_ns_min)*(t-t_cycle_start-inspi_duration_estim)/expi_duration_estim
        #pwm1_ns = pwm1_ns_max-(pwm1_ns_max-pwm1_ns_min)*(t-t_cycle_start-inspi_duration_estim)/expi_duration_estim
        pwm0_ns = pwm0_ns_max
        pwm1_ns = pwm1_ns_min
        Ppeak_reached = False
        GPIO.output(valve_inspi_pin, GPIO.LOW)
        #if ((p-p0 < PEEP) or (PEEP_reached == True)): # Should close both valves to maintain PEEP...
        if (p-p0 < PEEP): # Should close both valves to maintain PEEP...
            PEEP_reached = True
            GPIO.output(valve_expi_pin, GPIO.LOW)
        else:
            GPIO.output(valve_expi_pin, GPIO.HIGH)
    pwm0_ns = min(pwm0_ns_max, max(pwm0_ns_min, pwm0_ns))
    pwm1_ns = min(pwm1_ns_max, max(pwm1_ns_min, pwm1_ns))

    pwm0_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm0/duty_cycle'
    pwm1_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm1/duty_cycle'
    os.system(pwm0_cmd.format(math.trunc(pwm0_ns)))
    os.system(pwm1_cmd.format(math.trunc(pwm1_ns)))

    # Test...
    buz_pwm.ChangeFrequency(math.trunc(pwm1_ns/1000.0))
    led_pwm.ChangeDutyCycle(min(100, max(0, math.trunc(100*min(1.0, (pwm1_ns-pwm1_ns_min)/(pwm1_ns_max-pwm1_ns_min))))))
    
    line = '{};{};{};{};{};{};{};{};{}\n'
    file.write(line.format(t, p0, p, temperature, select, Ppeak, PEEP, breath_freq, inspi_ratio))
    file.flush()

    if enable_old_gui:
        if ((t-t_cycle_start) != 0) and (count % 200 == 0): # Clear from time to time since the plots accumulate...
            clf()
        axis([-scalex+offsetx, scalex+offsetx, -scaley+offsety, scaley+offsety])
        #axis('auto')
        plot([t_prev-t0, t-t0], [p_prev-p0, p-p0], 'b')
        pause(0.000001)
        offsetx = offsetx+t-t_prev
 
    if not p_sensor.read(ms5837.OSR_256):
        print('P sensor read failed!')
        exit(1)

    print('t-t0: %0.2f s \tdt: %0.3f s \tP0: %0.1f mbar \tP: %0.1f mbar \tT: %0.2f C ') % (t-t0, t-t_prev, p0, p, temperature) 
    
    t_prev = t
    t = timer()
    p_prev = p
    p = p_sensor.pressure()
    temperature = p_sensor.temperature()
    if (t-t_cycle_start > cycle_duration_estim):
        t_cycle_start = t
        p_cycle_start = p
        if enable_p0_sensor:    
            if not p0_sensor.read(ms5837.OSR_8192):
                print('P0 sensor read failed!')
                exit(1)
            p0 = p0_sensor.pressure() 
        inspi_end = False
    count = count+1
