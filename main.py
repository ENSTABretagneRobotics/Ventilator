#!/usr/bin/python
import ms5837 # From https://github.com/bluerobotics/ms5837-python
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
# Raspberry Pi configuration (GPIO that will be used : 2, 3 for I2C1 Bar02 
# (tube), 4 for digital input SELECT button, 5, 6 for digital output, 7, 8, 9, 
# 10, 11 SPI0 for Honeywell RSC for flow, valves, 12, 13 for PWM servomotors, 
# 16 for digital input UP button, 17 for digital input DOWN button, 22, 23 for 
# I2C6 Bar02 (room), 24 for software PWM buzzer, 26 for software PWM LED) :
#sudo nano /boot/config.txt
# Add in /boot/config.txt
#dtparam=i2c_arm=on
#dtoverlay=i2c-gpio,bus=6,i2c_gpio_delay_us=1,i2c_gpio_sda=22,i2c_gpio_scl=23
#dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
#dtparam=spi=on
# Then reboot and
#sudo -E python main.py

# Parameters
###############################################################################
Ppeak = 25 # In mbar (= approx. cmH2O)
PEEP = 5 # In mbar (= approx. cmH2O)
breath_freq = 15 # In cycles/min
inspi_ratio = 0.3
assist = 0
#trim PWM value start, end depending on balloon size...
pwm0_ns_min = 1000000
pwm0_ns_max = 1500000
pwm1_ns_min = 1500000
pwm1_ns_max = 2000000
Ppeak_min = 0
Ppeak_max = 100
PEEP_min = 0
PEEP_max = 100
breath_freq_min = 0
breath_freq_max = 100
inspi_ratio_min = 0
inspi_ratio_max = 1
enable_p0_sensor = False
enable_old_gui = False
scalex = 5
scaley = 50
offsety = 0
###############################################################################

# Bar02
#p_sensor = ms5837.MS5837(model=ms5837.MS5837_MODEL_30BA, bus=0)
p_sensor = ms5837.MS5837_02BA(bus=1)
if not p_sensor.init():
    print('P sensor could not be initialized')
    exit(1)
if enable_p0_sensor:
    p0_sensor = ms5837.MS5837_02BA(bus=6)
    if not p0_sensor.init():
        print('P0 sensor could not be initialized')
        exit(1)
# The very first pressure measurements might be wrong...
i = 0
while (i < 5):
    if not p_sensor.read(ms5837.OSR_256):
        print('P sensor read failed!')
        exit(1)
    if enable_p0_sensor:
        if not p0_sensor.read(ms5837.OSR_256):
            print('P0 sensor read failed!')
            exit(1)
    i = i+1

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
valve_inspi_val = GPIO.LOW
GPIO.setup(valve_inspi_pin, GPIO.OUT, initial = valve_inspi_val)
valve_expi_pin = 5
valve_expi_val = GPIO.LOW
GPIO.setup(valve_expi_pin, GPIO.OUT, initial = valve_expi_val)

# Digital inputs (buttons)
select = -1 # Index of the selected parameter that should be changed by up/down buttons
parameters = ['Ppeak', 'PEEP', 'breath_freq', 'inspi_ratio']
select_button_pin = 4
GPIO.setup(select_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
select_button_val = GPIO.input(select_button_pin)
select_button_val_prev = select_button_val
up_button_pin = 16
GPIO.setup(up_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
up_button_val = GPIO.input(up_button_pin)
up_button_val_prev = up_button_val
down_button_pin = 17
GPIO.setup(down_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
down_button_val = GPIO.input(down_button_pin)
down_button_val_prev = down_button_val

breath_freq_converted = breath_freq*(1.0/60.0) # In cycles/s
cycle_duration = 1.0/breath_freq_converted
inspi_duration = inspi_ratio*cycle_duration
expi_duration = cycle_duration-inspi_duration

t = timer()
t_prev = t
t0 = t
t_cycle_start = t
p = p_sensor.pressure()
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
file.write('t (in s);t0 (in s);p0 (in mbar);p (in mbar);temperature (in C);select;Ppeak (in mbar);PEEP (in mbar);breath_freq (in cycles/min);inspi_ratio;assist;pwm0_ns;pwm1_ns;valve_inspi_val;valve_expi_val;\n')

if enable_old_gui:
    fig = figure('Pressure')
    clf()
    axis('auto')
    offsetx = -scalex

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
            valve_inspi_val = GPIO.LOW
            GPIO.output(valve_inspi_pin, valve_inspi_val)
        else:
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
        valve_inspi_val = GPIO.LOW
        GPIO.output(valve_inspi_pin, valve_inspi_val)
        if ((p-p0 < PEEP) or (PEEP_reached == True)): # Should close both valves to maintain PEEP...
        #if (p-p0 < PEEP): # Should close both valves to maintain PEEP...
            PEEP_reached = True
            valve_expi_val = GPIO.LOW
            GPIO.output(valve_expi_pin, valve_expi_val)
        else:
            valve_expi_val = GPIO.HIGH
            GPIO.output(valve_expi_pin, valve_expi_val)
    pwm0_ns = min(pwm0_ns_max, max(pwm0_ns_min, pwm0_ns))
    pwm1_ns = min(pwm1_ns_max, max(pwm1_ns_min, pwm1_ns))

    pwm0_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm0/duty_cycle'
    pwm1_cmd = 'echo {} > /sys/class/pwm/pwmchip0/pwm1/duty_cycle'
    os.system(pwm0_cmd.format(math.trunc(pwm0_ns)))
    os.system(pwm1_cmd.format(math.trunc(pwm1_ns)))

    # Test...
    buz_pwm.ChangeFrequency(math.trunc(pwm1_ns/1000.0))
    led_pwm.ChangeDutyCycle(min(100, max(0, math.trunc(100*min(1.0, (pwm1_ns-pwm1_ns_min)/(pwm1_ns_max-pwm1_ns_min))))))

    # Buttons to set parameters
    select_button_val = GPIO.input(select_button_pin)
    if (select_button_val != select_button_val_prev):
        select_button_val_prev = select_button_val
        if (select_button_val == GPIO.LOW):
            select = select + 1
            if (select >= len(parameters)): select = -1    
    if (select >= 0) and (select < len(parameters)):
        up_button_val = GPIO.input(up_button_pin)
        if (up_button_val != up_button_val_prev):
            up_button_val_prev = up_button_val
            if (up_button_val == GPIO.LOW):
                if (select == 0):
                    Ppeak = Ppeak + 1
                    if (Ppeak > Ppeak_max): Ppeak = Ppeak_max
                if (select == 1):
                    PEEP = PEEP + 1
                    if (PEEP > PEEP_max): PEEP = PEEP_max
                if (select == 2):
                    breath_freq = breath_freq + 1
                    if (breath_freq > breath_freq_max): breath_freq = breath_freq_max
                if (select == 3):
                    inspi_ratio = inspi_ratio + 0.1
                    if (inspi_ratio > inspi_ratio_max): inspi_ratio = inspi_ratio_max
        down_button_val = GPIO.input(down_button_pin)
        if (down_button_val != down_button_val_prev):
            down_button_val_prev = down_button_val
            if (down_button_val == GPIO.LOW):
                if (select == 0):
                    Ppeak = Ppeak - 1
                    if (Ppeak < Ppeak_min): Ppeak = Ppeak_min
                if (select == 1):
                    PEEP = PEEP - 1
                    if (PEEP < PEEP_min): PEEP = PEEP_min
                if (select == 2):
                    breath_freq = breath_freq - 1
                    if (breath_freq < breath_freq_min): breath_freq = breath_freq_min
                if (select == 3):
                    inspi_ratio = inspi_ratio - 0.1
                    if (inspi_ratio < inspi_ratio_min): inspi_ratio = inspi_ratio_min

    # Log file
    line = '{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};\n'
    file.write(line.format(t, t0, p0, p, temperature, select, Ppeak, PEEP, breath_freq, inspi_ratio, assist, pwm0_ns, pwm1_ns, valve_inspi_val, valve_expi_val))
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
        time.sleep(0.1)
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
            if not p0_sensor.read(ms5837.OSR_256):
                print('P0 sensor read failed!')
                time.sleep(0.1)
                if not p0_sensor.read(ms5837.OSR_256):
                    print('P0 sensor read failed!')
                    exit(1)
            p0 = p0_sensor.pressure() 
        inspi_end = False
    count = count+1
