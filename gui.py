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
scaley = 0
offsety = 0
###############################################################################

t_plot = [0]
p_cmh2o_plot = [0]

win = pg.GraphicsWindow()
#win.move(0, 0)
#win.resize(800, 800)
plt = win.addPlot()
c1 = plt.plot(t_plot, p_cmh2o_plot, pen = "y")
if (scaley != 0): 
    plt.enableAutoRange("y", False)
    plt.setYRange(-scaley+offsety, scaley+offsety, 0)

file = open('data.csv', 'r')
while True:
    line = file.readline()
    cols = line.split(';')
    if (len(cols) >= 9):
        try:
            t0 = float(cols[0])
        except ValueError: 
            pass
        else:
            break
    time.sleep(1)

file.seek(0, os.SEEK_END)

i = 0
while True:
    start_time = time.time()

    data = file.readlines()
    if data:
        for line in data:
            cols = line.split(';')
            if (len(cols) >= 9):
                #print(cols)
                try:
                    t_plot.append(float(cols[0])-t0)
                    p_cmh2o_plot.append((float(cols[2])-float(cols[1]))*1.01972)
                    if (t_plot[-1]-t_plot[0] > 2*scalex):
                        t_plot.pop(0)
                        p_cmh2o_plot.pop(0)
                except ValueError: 
                    time.sleep(0.1)
            else:
                time.sleep(0.1)
    else:
        time.sleep(0.1)
   
    c1.setData(t_plot, p_cmh2o_plot)

    pg.QtGui.QApplication.processEvents()

    i += 1

    end_time = time.time()
    #print("It has been %0.3f seconds since the loop started" %(end_time - start_time))

win.close()

file.close()
