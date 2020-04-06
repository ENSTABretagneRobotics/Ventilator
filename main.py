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
#dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
# Then reboot and
#sudo python example.py

# Parameters
###############################################################################
p_delta_max = 25 # In mbar (= approx. cmH2O)
p_delta_min = 5 # In mbar (= approx. cmH2O)
breath_freq = 20 # In cycles/min
inspi_ratio = 1.0/3.0
#trim PWM value start, end depending on balloon size...
pwm0_ns_min = 1000000
pwm0_ns_max = 1500000
pwm1_ns_min = 1500000
pwm1_ns_max = 2000000
###############################################################################

# Software PWM init (for buzzer and status LED)
GPIO.setwarnings(False)	
GPIO.setmode(GPIO.BCM)
buz_pin = 26
GPIO.setup(buz_pin, GPIO.OUT)
buz_pwm = GPIO.PWM(buz_pin, 4000)
buz_pwm.start(50)
led_pin = 16
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

#sensor = ms5837.MS5837_30BA() # Default I2C bus is 1 (Raspberry Pi 3)
#sensor = ms5837.MS5837_30BA(0) # Specify I2C bus
sensor = ms5837.MS5837_02BA()
#sensor = ms5837.MS5837(model=ms5837.MS5837_MODEL_30BA, bus=0) # Specify model
#and bus
if not sensor.init():
    print('Sensor could not be initialized')
    exit(1)

if not sensor.read():
    print('Sensor read failed!')
    exit(1)

breath_freq_converted = breath_freq*(1.0/60.0) # In cycles/s
cycle_duration = 1.0/breath_freq_converted
inspi_duration = inspi_ratio*cycle_duration
expi_duration = cycle_duration-inspi_duration

t = timer()
t_prev = t
t0 = t
t_cycle_start = t
p = sensor.pressure() # The very first pressure measurements might be wrong, but it does not seem to be the case...
p_prev = p
p0 = p # External pressure should be monitored with e.g. another sensor...
p_cycle_start = p
temperature = sensor.temperature()
inspi_end = False
inspi_duration_estim = inspi_duration
expi_duration_estim = expi_duration
cycle_duration_estim = cycle_duration

pwm0_ns = pwm0_ns_max
pwm1_ns = pwm1_ns_min

file = open('data.csv', 'a')

fig = figure('Pressure')
clf()
#axis('square')
axis('auto')
scalex = 10
scaley = 50
offsetx = -scalex
offsety = p0

# Divisions by 0...

pp_reached = False

count = 0
while True:
    if (t-t_cycle_start < inspi_duration_estim):
        #pwm0_ns = pwm0_ns_max-(pwm0_ns_max-pwm0_ns_min)*(t-t_cycle_start)/inspi_duration_estim
        #pwm1_ns = pwm1_ns_min+(pwm1_ns_max-pwm1_ns_min)*(t-t_cycle_start)/inspi_duration_estim
        pwm0_ns = pwm0_ns_min
        pwm1_ns = pwm1_ns_max
        if ((p-p0 > p_delta_max) or (pp_reached == True)): # Should close both valves to maintain p_delta_max...
            pp_reached = True
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
        pp_reached = False
        GPIO.output(valve_inspi_pin, GPIO.LOW)
        if (p-p0 < p_delta_min): # Should close both valves to maintain p_delta_min...
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
    
    line = '{};{};{}\n'
    file.write(line.format(t, p, temperature))
    file.flush()

    if ((t-t_cycle_start) != 0) and (count % 200 == 0): # Clear from time to time since the plots accumulate...
        clf()
    axis([-scalex+offsetx, scalex+offsetx, -scaley+offsety, scaley+offsety])
    #axis('auto')
    plot([t_prev-t0, t-t0], [p_prev, p], 'b')
    pause(0.000001)
    offsetx = offsetx+t-t_prev
 
    if sensor.read(ms5837.OSR_8192):
        print('t-t0: %0.2f s \tdt: %0.2f s \tP: %0.1f mbar \tT: %0.2f C ') % (t-t0, t-t_prev, p, temperature) 
    else:
        print('Sensor read failed!')
        exit(1)
    
    t_prev = t
    t = timer()
    p_prev = p
    p = sensor.pressure()
    temperature = sensor.temperature()
    if (t-t_cycle_start > cycle_duration_estim):
        t_cycle_start = t
        p_cycle_start = p
        inspi_end = False
    count = count+1
