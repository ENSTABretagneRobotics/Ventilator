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

nb_cols = 37 # Not counting the final '\n'
delay = 0.025

t_plot = [0]
p_cmh2o_plot = [0]
p_e_cmh2o_plot = [0]
Ppeak_plot = [0]
PEEP_plot = [0]
flow_control_air_plot = [0]
flow_control_O2_plot = [0]
valve_air_plot = [0]
valve_O2_plot = [0]
valve_inspi_plot = [0]
valve_expi_plot = [0]
flow_air_plot = [0]
flow_expi_plot = [0]
flow_O2_plot = [0]
flow_filtered_air_plot = [0]
flow_filtered_expi_plot = [0]
flow_filtered_O2_plot = [0]
vol_l_plot = [0]

win = pg.GraphicsWindow()
win.move(0, 0)
win.resize(800, 420)
win.setWindowTitle('Pressure, flow, volume')
plt = win.addPlot()
plt2 = win.addPlot()
plt.showGrid(x = True, y = True)
plt2.showGrid(x = True, y = True)
plt.setTitle('Temp. I: 25.00 C, Temp. E: 25.00 C', **{'color': '#FFFFFF', 'size': '10pt'})
plt2.setTitle('Temp. A: 25.00 C, Temp. E: 25.00 C, Temp. O: 25.00 C', **{'color': '#FFF', 'size': '10pt'})
plt.setLabel('left', 'Pressure (in cmH2O)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt2.setLabel('left', 'Flow (in L/min)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt2.setLabel('right', 'Volume (in cl)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt.setLabel('bottom', 'Time (in s)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt2.setLabel('bottom', 'Time (in s)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt.addLegend(size = (0, 0), offset = (4, 1))
plt2.addLegend(size = (0, 0), offset = (4, 1))
font = QtGui.QFont()
font.setPixelSize(11)
plt.getAxis('left').tickFont = font
plt2.getAxis('left').tickFont = font
plt2.getAxis('right').tickFont = font
plt.getAxis('bottom').tickFont = font
plt2.getAxis('bottom').tickFont = font
c_p_cmh2o = plt.plot(t_plot, p_cmh2o_plot, pen = '#FFFF00', name = 'Inspi. pres.')
c_p_e_cmh2o = plt.plot(t_plot, p_e_cmh2o_plot, pen = '#FFA500', name = 'Expi. pres.')
c_Ppeak = plt.plot(t_plot, Ppeak_plot, pen = '#FFFFFF')
c_PEEP = plt.plot(t_plot, PEEP_plot, pen = '#AAAAAA')
c_flow_control_air = plt2.plot(t_plot, flow_control_air_plot, pen = '#AAFF00')
c_flow_control_O2 = plt2.plot(t_plot, flow_control_O2_plot, pen = '#00AAFF')
c_valve_air = plt.plot(t_plot, valve_air_plot, pen = '#00FF00', name = 'Air valve')
c_valve_O2 = plt.plot(t_plot, valve_O2_plot, pen = '#0000FF', name = 'O2 valve')
c_valve_inspi = plt.plot(t_plot, valve_inspi_plot, pen = '#00FFFF', name = 'Inspi. valve')
c_valve_expi = plt.plot(t_plot, valve_expi_plot, pen = '#FF0000', name = 'Expi. valve')
c_flow_air = plt2.plot(t_plot, flow_air_plot, pen = '#003000')
c_flow_expi = plt2.plot(t_plot, flow_expi_plot, pen = '#300000')
c_flow_O2 = plt2.plot(t_plot, flow_O2_plot, pen = '#003030')
c_flow_filtered_air = plt2.plot(t_plot, flow_filtered_air_plot, pen = '#00FF00', name = 'Air flow')
c_flow_filtered_expi = plt2.plot(t_plot, flow_filtered_expi_plot, pen = '#FF0000', name = 'Expi. flow')
c_flow_filtered_O2 = plt2.plot(t_plot, flow_filtered_O2_plot, pen = '#0000FF', name = 'O2 flow')
c_vol_l = plt2.plot(t_plot, vol_l_plot, pen = '#FF00FF', name = 'Volume')
for item in plt.legend.items:
    for single_item in item:
        if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
            single_item.setText(single_item.text, **{'color': '#FFFFFF', 'size': '10pt'})
for item in plt2.legend.items:
    for single_item in item:
        if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
            single_item.setText(single_item.text, **{'color': '#FFFFFF', 'size': '10pt'})
if (scaley != 0): 
    plt.enableAutoRange('y', False)
    plt.setYRange(-scaley+offsety, scaley+offsety, 0)
if (scale2y != 0): 
    plt2.enableAutoRange('y', False)
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
                        p_e = float(cols[6])
                        temperature_e = float(cols[7])
                        select = float(cols[8])
                        Ppeak = float(cols[9])
                        PEEP = float(cols[10])
                        respi_rate = float(cols[11])
                        inspi_ratio = float(cols[12])
                        flow_control_air = float(cols[13])
                        flow_control_O2 = float(cols[14])
                        mode = float(cols[15])
                        valve_air = float(cols[18])
                        valve_O2 = float(cols[19])
                        valve_inspi = float(cols[20])
                        valve_expi = float(cols[21])
                        pressure_air = float(cols[22])
                        pressure_expi = float(cols[23])
                        pressure_O2 = float(cols[24])
                        temperature_air = float(cols[25])
                        temperature_expi = float(cols[26])
                        temperature_O2 = float(cols[27])
                        flow_air = float(cols[28])
                        flow_expi = float(cols[29])
                        flow_O2 = float(cols[30])
                        flow_filtered_air = float(cols[31])
                        flow_filtered_expi = float(cols[32])
                        flow_filtered_O2 = float(cols[33])
                        vol_air = float(cols[34])
                        vol_expi = float(cols[35])
                        vol_O2 = float(cols[36])

                        dt = t-t0
                        p_cmh2o = (p-p0)*1.01972
                        p_e_cmh2o = (p_e-p0)*1.01972                        
                        vol_l = (vol_air+vol_O2+vol_expi)

                        if (dt < t_plot[-1]): 
                            # Reset if time seems to decrease...
                            t_plot = [0]
                            p_cmh2o_plot = [0]
                            p_e_cmh2o_plot = [0]
                            Ppeak_plot = [0]
                            PEEP_plot = [0]
                            flow_control_air_plot = [0]
                            flow_control_O2_plot = [0]
                            valve_air_plot = [0]
                            valve_O2_plot = [0]
                            valve_inspi_plot = [0]
                            valve_expi_plot = [0]
                            flow_air_plot = [0]
                            flow_expi_plot = [0]
                            flow_O2_plot = [0]
                            flow_filtered_air_plot = [0]
                            flow_filtered_expi_plot = [0]
                            flow_filtered_O2_plot = [0]
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
                            wintitle = '[I:E: {:.2f}]'
                            win.setWindowTitle(wintitle.format(inspi_ratio))
                        elif (select == 4): 
                            wintitle = '[Flow A: {:d}]'
                            win.setWindowTitle(wintitle.format(int(flow_control_air)))
                        elif (select == 5): 
                            wintitle = '[Flow O: {:d}]'
                            win.setWindowTitle(wintitle.format(int(flow_control_O2)))
                        elif (select == 6): 
                            wintitle = '[Mode: {:d}]'
                            win.setWindowTitle(wintitle.format(int(mode)))
                        else: 
                            wintitle = 'Ppeak: {:d}, PEEP: {:d}, Respi. rate: {:d}/min, I:E: {:.2f}, Flow A: {:d}, Flow O: {:d}, Mode: {:d}'
                            win.setWindowTitle(wintitle.format(int(Ppeak), int(PEEP), int(respi_rate), inspi_ratio, int(flow_control_air), int(flow_control_O2), int(mode)))
                        plt.setTitle('Temp. I: {:.2f} C, Temp. E: {:.2f} C'.format(temperature, temperature_e))
                        plt2.setTitle('Temp. A: {:.2f}, E: {:.2f}, O: {:.2f}'.format(temperature_air, temperature_expi, temperature_O2))
                        # Should ensure that no ValueError exception can happen here to avoid lists of different length, 
                        # so no float conversion should be done in the append()...
                        t_plot.append(dt)
                        p_cmh2o_plot.append(p_cmh2o)
                        p_e_cmh2o_plot.append(p_e_cmh2o)
                        Ppeak_plot.append(Ppeak)
                        PEEP_plot.append(PEEP)
                        flow_control_air_plot.append(flow_control_air)
                        flow_control_O2_plot.append(flow_control_O2)
                        valve_air_plot.append(valve_air/10.0)
                        valve_O2_plot.append(valve_O2/10.0)
                        valve_inspi_plot.append(10.0*valve_inspi)
                        valve_expi_plot.append(10.0*valve_expi)
                        flow_air_plot.append(flow_air)
                        flow_expi_plot.append(flow_expi)
                        flow_O2_plot.append(flow_O2)
                        flow_filtered_air_plot.append(flow_filtered_air)
                        flow_filtered_expi_plot.append(flow_filtered_expi)
                        flow_filtered_O2_plot.append(flow_filtered_O2)
                        vol_l_plot.append(100.0*vol_l)
                        if (t_plot[-1]-t_plot[0] > 2*scalex):
                            t_plot.pop(0)
                            p_cmh2o_plot.pop(0)
                            p_e_cmh2o_plot.pop(0)
                            Ppeak_plot.pop(0)
                            PEEP_plot.pop(0)
                            flow_control_air_plot.pop(0)
                            flow_control_O2_plot.pop(0)
                            valve_air_plot.pop(0)
                            valve_O2_plot.pop(0)
                            valve_inspi_plot.pop(0)
                            valve_expi_plot.pop(0)
                            flow_air_plot.pop(0)
                            flow_expi_plot.pop(0)
                            flow_O2_plot.pop(0)
                            flow_filtered_air_plot.pop(0)
                            flow_filtered_expi_plot.pop(0)
                            flow_filtered_O2_plot.pop(0)
                            vol_l_plot.pop(0)
                    except ValueError: 
                        time.sleep(delay)
                else:
                    time.sleep(delay)
        else:
            time.sleep(delay)
    except EOFError: 
        file.seek(0, os.SEEK_END) # Might be necessary on recent versions of Linux, see https://lists.gnu.org/archive/html/info-gnu/2018-08/msg00000.html...
   
    c_p_cmh2o.setData(t_plot, p_cmh2o_plot)
    c_p_e_cmh2o.setData(t_plot, p_e_cmh2o_plot)
    c_Ppeak.setData(t_plot, Ppeak_plot)
    c_PEEP.setData(t_plot, PEEP_plot)
    c_flow_control_air.setData(t_plot, flow_control_air_plot)
    c_flow_control_O2.setData(t_plot, flow_control_O2_plot)
    c_valve_air.setData(t_plot, valve_air_plot)
    c_valve_O2.setData(t_plot, valve_O2_plot)
    c_valve_inspi.setData(t_plot, valve_inspi_plot)
    c_valve_expi.setData(t_plot, valve_expi_plot)
    c_flow_air.setData(t_plot, flow_air_plot)
    c_flow_expi.setData(t_plot, flow_expi_plot)
    c_flow_O2.setData(t_plot, flow_O2_plot)
    c_flow_filtered_air.setData(t_plot, flow_filtered_air_plot)
    c_flow_filtered_expi.setData(t_plot, flow_filtered_expi_plot)
    c_flow_filtered_O2.setData(t_plot, flow_filtered_O2_plot)
    c_vol_l.setData(t_plot, vol_l_plot)

    pg.QtGui.QApplication.processEvents()

    #count += 1

    end_time = time.time()
    #print('It has been %0.3f seconds since the loop started' %(end_time - start_time))

file.close()

win.close()
