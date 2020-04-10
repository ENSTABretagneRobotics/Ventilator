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
scaley = 100
offsety = 50
###############################################################################

nb_cols = 3 # Not counting the final '\n'
delay = 0.025

t_plot = [0]
flow_l_min_plot = [0]

win = pg.GraphicsWindow()
#win.move(0, 0)
#win.resize(400, 400)
win.setWindowTitle('Flow')
plt = win.addPlot()
plt.showGrid(x = True, y = True)
plt.setLabel('left', 'Flow (in L/min)')
plt.setLabel('bottom', 'Time (in s)')
plt.addLegend()
c1 = plt.plot(t_plot, flow_l_min_plot, pen = "y")
if (scaley != 0): 
    plt.enableAutoRange("y", False)
    plt.setYRange(-scaley+offsety, scaley+offsety, 0)

# Waiting for the file to be created...
while True:
    try:
        file = open('data_flow.csv', 'r')
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
                if (len(cols) > nb_cols): # Need the final '\n' so we are sure the numbers are fully written
                    #print(cols)
                    try:
                        t = float(cols[0])
                        t0 = float(cols[1])
                        flow = float(cols[2])
                        dt = t-t0
                        flow_l_min = flow*60000.0
                        if (dt < t_plot[-1]): 
                            # Reset if time seems to decrease...
                            t_plot = [0]
                            flow_l_min_plot = [0]
                        # Should ensure that no ValueError exception can happen here to avoid lists of different length, 
                        # so no float conversion should be done in the append()...
                        t_plot.append(dt)
                        flow_l_min_plot.append(flow_l_min)
                        if (t_plot[-1]-t_plot[0] > 2*scalex):
                            t_plot.pop(0)
                            flow_l_min_plot.pop(0)
                    except ValueError: 
                        time.sleep(delay)
                else:
                    time.sleep(delay)
        else:
            time.sleep(delay)
    except EOFError: 
        file.seek(0, os.SEEK_END) # Might be necessary on recent versions of Linux, see https://lists.gnu.org/archive/html/info-gnu/2018-08/msg00000.html...
   
    c1.setData(t_plot, flow_l_min_plot)

    pg.QtGui.QApplication.processEvents()

    count += 1

    end_time = time.time()
    #print("It has been %0.3f seconds since the loop started" %(end_time - start_time))

win.close()

file.close()
