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

nb_cols = 10

t_plot = [0]
p_cmh2o_plot = [0]

win = pg.GraphicsWindow()
#win.move(0, 0)
#win.resize(400, 400)
win.setWindowTitle('Pressure')
plt = win.addPlot()
plt.showGrid(x = True, y = True)
plt.setLabel('left', 'Pressure (in cmH2O)')
plt.setLabel('bottom', 'Time (in s)')
plt.addLegend()
c1 = plt.plot(t_plot, p_cmh2o_plot, pen = "y")
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

count = 0
while True:
    start_time = time.time()

    try:
        data = file.readlines()
        if data:
            for line in data:
                cols = line.split(';')
                if (len(cols) >= nb_cols):
                    #print(cols)
                    try:
                        t = float(cols[0])
                        t0 = float(cols[1])
                        p0 = float(cols[2])
                        p = float(cols[3])
                        if (t-t0 < t_plot[-1]): 
                            # Reset if time seems to decrease...
                            t_plot = [0]
                            p_cmh2o_plot = [0]
                        t_plot.append(t-t0)
                        p_cmh2o_plot.append((float(cols[3])-float(cols[2]))*1.01972)
                        if (t_plot[-1]-t_plot[0] > 2*scalex):
                            t_plot.pop(0)
                            p_cmh2o_plot.pop(0)
                    except ValueError: 
                        time.sleep(0.05)
                else:
                    time.sleep(0.05)
        else:
            time.sleep(0.05)
    except EOFError: 
        file.seek(0, os.SEEK_END) # Might be necessary on recent versions of Linux, see https://lists.gnu.org/archive/html/info-gnu/2018-08/msg00000.html...
   
    c1.setData(t_plot, p_cmh2o_plot)

    pg.QtGui.QApplication.processEvents()

    count += 1

    end_time = time.time()
    #print("It has been %0.3f seconds since the loop started" %(end_time - start_time))

win.close()

file.close()
