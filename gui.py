#!/usr/bin/python
import math
import sys
import os
import time
import threading
from timeit import default_timer as timer
import numpy
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

# Parameters
###############################################################################
scalex = 5
scaley = 50
offsety = 30
###############################################################################

nb_cols = 15 # Not counting the final '\n'
delay = 0.025

t_plot = [0]
p_cmh2o_plot = [0]
valve_inspi_plot = [0]
valve_expi_plot = [0]

win = pg.GraphicsWindow()
#win.move(0, 0)
#win.resize(400, 400)
win.setWindowTitle('Pressure')
plt = win.addPlot()
plt.showGrid(x = True, y = True)
plt.setLabel('left', 'Pressure (in cmH2O)')
plt.setLabel('bottom', 'Time (in s)')
plt.addLegend()
c1 = plt.plot(t_plot, p_cmh2o_plot, pen = "y", name = 'Pressure')
c2 = plt.plot(t_plot, valve_inspi_plot, pen = "c", name = 'Inspiration valve')
c3 = plt.plot(t_plot, valve_expi_plot, pen = "m", name = 'Expiration valve')
if (scaley != 0): 
    plt.enableAutoRange("y", False)
    plt.setYRange(-scaley+offsety, scaley+offsety, 0)

# Waiting for the file to be created...
while True:
    try:
        file = open('data.csv', 'r')
    except IOError:
        time.sleep(1)
    else:
        break

file.seek(0, os.SEEK_END)

#count = 0
while True:
    start_time = time.time()

    try:
        data = file.readlines()
        if data:
            for line in data:
                cols = line.split(';')
                if (len(cols) > nb_cols): # Need the final '\n' so we are sure the numbers are fully written
                    #print(cols)
                    try:
                        t = float(cols[0])
                        t0 = float(cols[1])
                        p0 = float(cols[2])
                        p = float(cols[3])
                        temperature = float(cols[4])
                        select = float(cols[5])
                        Ppeak = float(cols[6])
                        PEEP = float(cols[7])
                        breath_freq = float(cols[8])
                        inspi_ratio = float(cols[9])
                        valve_inspi = float(cols[13])
                        valve_expi = float(cols[14])
                        dt = t-t0
                        p_cmh2o = (float(cols[3])-float(cols[2]))*1.01972
                        if (dt < t_plot[-1]): 
                            # Reset if time seems to decrease...
                            t_plot = [0]
                            p_cmh2o_plot = [0]
                            valve_inspi_plot = [0]
                            valve_expi_plot = [0]
                        if (select == 0): wintitle = '[Ppeak = {} cmH2O], PEEP = {} cmH2O, breath_freq = {} cycles/min, inspi_ratio = {}'
                        elif (select == 1): wintitle = 'Ppeak = {} cmH2O, [PEEP = {} cmH2O], breath_freq = {} cycles/min, inspi_ratio = {}'
                        elif (select == 2): wintitle = 'Ppeak = {} cmH2O, PEEP = {} cmH2O, [breath_freq = {} cycles/min], inspi_ratio = {}'
                        elif (select == 3): wintitle = 'Ppeak = {} cmH2O, PEEP = {} cmH2O], breath_freq = {} cycles/min, [inspi_ratio = {}]'
                        else: wintitle = 'Ppeak = {} cmH2O, PEEP = {} cmH2O, breath_freq = {} cycles/min, inspi_ratio = {}'
                        win.setWindowTitle(wintitle.format(Ppeak, PEEP, breath_freq, inspi_ratio))
                        plt.setTitle('Temperature : {} C'.format(temperature))
                        # Should ensure that no ValueError exception can happen here to avoid lists of different length, 
                        # so no float conversion should be done in the append()...
                        t_plot.append(dt)
                        p_cmh2o_plot.append(p_cmh2o)
                        valve_inspi_plot.append(10.0*valve_inspi)
                        valve_expi_plot.append(10.0*valve_expi)
                        if (t_plot[-1]-t_plot[0] > 2*scalex):
                            t_plot.pop(0)
                            p_cmh2o_plot.pop(0)
                            valve_inspi_plot.pop(0)
                            valve_expi_plot.pop(0)
                    except ValueError: 
                        time.sleep(delay)
                else:
                    time.sleep(delay)
        else:
            time.sleep(delay)
    except EOFError: 
        file.seek(0, os.SEEK_END) # Might be necessary on recent versions of Linux, see https://lists.gnu.org/archive/html/info-gnu/2018-08/msg00000.html...
   
    c1.setData(t_plot, p_cmh2o_plot)
    c2.setData(t_plot, valve_inspi_plot)
    c3.setData(t_plot, valve_expi_plot)

    pg.QtGui.QApplication.processEvents()

    #count += 1

    end_time = time.time()
    #print("It has been %0.3f seconds since the loop started" %(end_time - start_time))

file.close()

win.close()
