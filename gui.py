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
scale2y = 100
offset2y = 20
###############################################################################

nb_cols = 24 # Not counting the final '\n'
delay = 0.025

t_plot = [0]
p_cmh2o_plot = [0]
valve_inspi_plot = [0]
valve_expi_plot = [0]
flow_inspi_l_min_plot = [0]
flow_expi_l_min_plot = [0]
vol_l_plot = [0]

win = pg.GraphicsWindow()
win.move(0, 0)
win.resize(800, 420)
win.setWindowTitle('Pressure, flow, volume')
plt = win.addPlot()
plt2 = win.addPlot()
plt.showGrid(x = True, y = True)
plt2.showGrid(x = True, y = True)
plt.setTitle('Temperature: 25.00 C', **{'color': '#FFF', 'size': '10pt'})
plt2.setTitle('Temp. I: 25.00 C, Temp. E: 25.00 C', **{'color': '#FFF', 'size': '10pt'})
plt.setLabel('left', 'Pressure (in cmH2O)', **{'color': '#FFF', 'font-size': '10pt'})
plt2.setLabel('left', 'Flow (in L/min)', **{'color': '#FFF', 'font-size': '10pt'})
plt2.setLabel('right', 'Volume (in cl)', **{'color': '#FFF', 'font-size': '10pt'})
plt.setLabel('bottom', 'Time (in s)', **{'color': '#FFF', 'font-size': '10pt'})
plt2.setLabel('bottom', 'Time (in s)', **{'color': '#FFF', 'font-size': '10pt'})
plt.addLegend(size = (0, 0), offset = (4, 1))
plt2.addLegend(size = (0, 0), offset = (4, 1))
font = QtGui.QFont()
font.setPixelSize(11)
plt.getAxis("left").tickFont = font
plt2.getAxis("left").tickFont = font
plt2.getAxis("right").tickFont = font
plt.getAxis("bottom").tickFont = font
plt2.getAxis("bottom").tickFont = font
c1 = plt.plot(t_plot, p_cmh2o_plot, pen = "y", name = 'Pressure')
c2 = plt.plot(t_plot, valve_inspi_plot, pen = "c", name = 'Inspiration valve')
c3 = plt.plot(t_plot, valve_expi_plot, pen = "m", name = 'Expiration valve')
c4 = plt2.plot(t_plot, flow_inspi_l_min_plot, pen = "g", name = 'Inspiration flow')
c5 = plt2.plot(t_plot, flow_expi_l_min_plot, pen = "r", name = 'Expiration flow')
c6 = plt2.plot(t_plot, vol_l_plot, pen = "m", name = 'Volume')
for item in plt.legend.items:
    for single_item in item:
        if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
            single_item.setText(single_item.text, **{'color': '#FFF', 'size': '10pt'})
for item in plt2.legend.items:
    for single_item in item:
        if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
            single_item.setText(single_item.text, **{'color': '#FFF', 'size': '10pt'})
if (scaley != 0): 
    plt.enableAutoRange("y", False)
    plt.setYRange(-scaley+offsety, scaley+offsety, 0)
if (scale2y != 0): 
    plt2.enableAutoRange("y", False)
    plt2.setYRange(-scale2y+offset2y, scale2y+offset2y, 0)

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
                        temperature0 = float(cols[3])
                        p = float(cols[4])
                        temperature = float(cols[5])
                        select = float(cols[6])
                        Ppeak = float(cols[7])
                        PEEP = float(cols[8])
                        respi_rate = float(cols[9])
                        inspi_ratio = float(cols[10])
                        valve_inspi = float(cols[14])
                        valve_expi = float(cols[15])
                        pressure_inspi = float(cols[16])
                        pressure_expi = float(cols[17])
                        temperature_inspi = float(cols[18])
                        temperature_expi = float(cols[19])
                        flow_inspi = float(cols[20])
                        flow_expi = float(cols[21])
                        vol_inspi = float(cols[22])
                        vol_expi = float(cols[23])
                        dt = t-t0
                        p_cmh2o = (float(cols[4])-float(cols[2]))*1.01972
                        flow_inspi_l_min = flow_inspi
                        flow_expi_l_min = flow_expi
                        vol_l = (vol_inspi+vol_expi)*1000.0
                        if (dt < t_plot[-1]): 
                            # Reset if time seems to decrease...
                            t_plot = [0]
                            p_cmh2o_plot = [0]
                            valve_inspi_plot = [0]
                            valve_expi_plot = [0]
                            flow_inspi_l_min_plot = [0]
                            flow_expi_l_min_plot = [0]
                            vol_l_plot = [0]
                        if (select == 0): 
                            wintitle = '[Ppeak: {:d}]'
                            win.setWindowTitle(wintitle.format(int(Ppeak)))
                        elif (select == 1): 
                            wintitle = '[PEEP: {:d}]'
                            win.setWindowTitle(wintitle.format(int(PEEP)))
                        elif (select == 2): 
                            wintitle = '[Respi. rate: {:d}/min]'
                            win.setWindowTitle(wintitle.format(int(respi_rate)))
                        elif (select == 3): 
                            wintitle = '[I:E ratio: {}]'
                            win.setWindowTitle(wintitle.format(inspi_ratio))
                        else: 
                            wintitle = 'Ppeak: {:d}, PEEP: {:d}, Respi. rate: {:d}/min, I:E ratio = {}'
                            win.setWindowTitle(wintitle.format(int(Ppeak), int(PEEP), int(respi_rate), inspi_ratio))
                        plt.setTitle('Temperature: {:.2f} C'.format(temperature))
                        plt2.setTitle('Temp. I: {:.2f} C, Temp. E: {:.2f} C'.format(temperature_inspi, temperature_expi))
                        # Should ensure that no ValueError exception can happen here to avoid lists of different length, 
                        # so no float conversion should be done in the append()...
                        t_plot.append(dt)
                        p_cmh2o_plot.append(p_cmh2o)
                        valve_inspi_plot.append(10.0*valve_inspi)
                        valve_expi_plot.append(10.0*valve_expi)
                        flow_inspi_l_min_plot.append(flow_inspi_l_min)
                        flow_expi_l_min_plot.append(flow_expi_l_min)
                        vol_l_plot.append(100.0*vol_l)
                        if (t_plot[-1]-t_plot[0] > 2*scalex):
                            t_plot.pop(0)
                            p_cmh2o_plot.pop(0)
                            valve_inspi_plot.pop(0)
                            valve_expi_plot.pop(0)
                            flow_inspi_l_min_plot.pop(0)
                            flow_expi_l_min_plot.pop(0)
                            vol_l_plot.pop(0)
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
    c4.setData(t_plot, flow_inspi_l_min_plot)
    c5.setData(t_plot, flow_expi_l_min_plot)
    c6.setData(t_plot, vol_l_plot)

    pg.QtGui.QApplication.processEvents()

    #count += 1

    end_time = time.time()
    #print("It has been %0.3f seconds since the loop started" %(end_time - start_time))

file.close()

win.close()
