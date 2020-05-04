#!/usr/bin/python

from __future__ import division, print_function
import math
import sys
import os
import time
import threading
from timeit import default_timer as timer
import numpy
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from common import *

# Parameters
###############################################################################
scalex = 5
scaley = 50
offsety = 30
scale2y = 100
offset2y = 20
debug = False
###############################################################################

nb_cols = 46 # Not counting the final '\n'
delay = 0.025

t_plot = [0]
p_cmh2o_plot = [0]
p_e_cmh2o_plot = [0]
Ppeak_plot = [0]
PEEP_plot = [0]
flow_control_air_plot = [0]
flow_control_O2_plot = [0]
flow_control_expi_plot = [0]
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
flow_filtered_plot = [0]
vol_l_plot = [0]

class GUIWindow(pg.GraphicsWindow):
    sigKeyPress = QtCore.pyqtSignal(object)

    def keyPressEvent(self, ev):
        self.scene().keyPressEvent(ev)
        self.sigKeyPress.emit(ev)

    def closeEvent(self, ev):
        # Global variables to share with other functions.
        global bExit
        #ev.ignore()
        ev.accept() # let the window close
        bExit = 1

def keyPressed(evt):
    # Global variables to share with other functions.
    global bExit, debug
    #print("Key pressed")
    #print(evt.key())
    if evt.key() == QtCore.Qt.Key_Escape:
        bExit = 1
    if evt.key() == QtCore.Qt.Key_D:
        debug = not debug

#win = pg.GraphicsWindow()
win = GUIWindow()
win.sigKeyPress.connect(keyPressed)
win.setWindowTitle('Pressure, flow, volume')
plt = win.addPlot(row = 0, col = 0)
plt2 = win.addPlot(row = 0, col = 1)
plt3 = win.addPlot(row = 0, col = 2)
plt4 = win.addPlot(row = 1, col = 2)
plt5 = win.addPlot(row = 2, col = 2)
plt6 = win.addPlot(row = 3, col = 2)
win.move(0, 0)
win.resize(800, 420)
if debug: 
    plt.show()
    plt2.show()
    plt3.hide()
    plt4.hide()
    plt5.hide()
else:
    plt.hide()
    plt2.hide()
    plt3.show()
    plt4.show()
    plt5.show()
plt6.hide()
plt.showGrid(x = True, y = True)
plt2.showGrid(x = True, y = True)
plt3.showGrid(x = True, y = True)
plt4.showGrid(x = True, y = True)
plt5.showGrid(x = True, y = True)
plt6.showGrid(x = True, y = True)
plt.setTitle('Temp. I: 25.00 C, Temp. E: 25.00 C', **{'color': '#FFFFFF', 'size': '10pt'})
plt2.setTitle('Temp. A: 25.00 C, Temp. E: 25.00 C, Temp. O: 25.00 C', **{'color': '#FFF', 'size': '10pt'})
plt.setLabel('left', 'Pressure (in cmH2O)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt2.setLabel('left', 'Flow (in L/min)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt2.setLabel('right', 'Volume (in cl)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt3.setLabel('left', 'Pressure (in cmH2O)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt4.setLabel('left', 'Flow (in L/min)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt5.setLabel('left', 'Volume (in cl)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt6.setLabel('left', 'Flow (in L/min)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt.setLabel('bottom', 'Time (in s)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt2.setLabel('bottom', 'Time (in s)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt.addLegend(size = (0, 0), offset = (4, 1))
plt2.addLegend(size = (0, 0), offset = (4, 1))
plt3.addLegend(size = (0, 0), offset = (4, 1))
plt4.addLegend(size = (0, 0), offset = (4, 1))
plt5.addLegend(size = (0, 0), offset = (4, 1))
plt6.addLegend(size = (0, 0), offset = (4, 1))
font = QtGui.QFont()
font.setPixelSize(11)
plt.getAxis('left').tickFont = font
plt2.getAxis('left').tickFont = font
plt2.getAxis('right').tickFont = font
plt3.getAxis('left').tickFont = font
plt4.getAxis('left').tickFont = font
plt5.getAxis('left').tickFont = font
plt6.getAxis('left').tickFont = font
plt.getAxis('bottom').tickFont = font
plt2.getAxis('bottom').tickFont = font
plt3.getAxis('bottom').tickFont = font
plt4.getAxis('bottom').tickFont = font
plt5.getAxis('bottom').tickFont = font
plt6.getAxis('bottom').tickFont = font
c_p_cmh2o = plt.plot(t_plot, p_cmh2o_plot, pen = '#FFFF00', name = 'Inspi. pres.')
c2_p_cmh2o = plt3.plot(t_plot, p_cmh2o_plot, pen = '#FFFF00', name = 'Inspi. pres.')
c_p_e_cmh2o = plt.plot(t_plot, p_e_cmh2o_plot, pen = '#FFA500', name = 'Expi. pres.')
c_Ppeak = plt.plot(t_plot, Ppeak_plot, pen = '#FFFFFF')
c2_Ppeak = plt3.plot(t_plot, Ppeak_plot, pen = '#FFFFFF')
c_PEEP = plt.plot(t_plot, PEEP_plot, pen = '#AAAAAA')
c2_PEEP = plt3.plot(t_plot, PEEP_plot, pen = '#AAAAAA')
c_flow_control_air = plt2.plot(t_plot, flow_control_air_plot, pen = '#AAFF00')
c_flow_control_O2 = plt2.plot(t_plot, flow_control_O2_plot, pen = '#00AAFF')
c_flow_control_expi = plt2.plot(t_plot, flow_control_expi_plot, pen = '#FF00AA')
c_valve_air = plt.plot(t_plot, valve_air_plot, pen = '#00FF00', name = 'Air valve')
c_valve_O2 = plt.plot(t_plot, valve_O2_plot, pen = '#0000FF', name = 'O2 valve')
c_valve_inspi = plt.plot(t_plot, valve_inspi_plot, pen = '#00FFFF', name = 'Inspi. valve')
c_valve_expi = plt.plot(t_plot, valve_expi_plot, pen = '#FF0000', name = 'Expi. valve')
c_flow_air = plt2.plot(t_plot, flow_air_plot, pen = '#003000')
c_flow_expi = plt2.plot(t_plot, flow_expi_plot, pen = '#300000')
c_flow_O2 = plt2.plot(t_plot, flow_O2_plot, pen = '#003030')
c_flow_filtered_air = plt2.plot(t_plot, flow_filtered_air_plot, pen = '#00FF00', name = 'Air flow')
c2_flow_filtered_air = plt6.plot(t_plot, flow_filtered_air_plot, pen = '#00FF00', name = 'Air flow')
c_flow_filtered_expi = plt2.plot(t_plot, flow_filtered_expi_plot, pen = '#FF0000', name = 'Expi. flow')
c_flow_filtered_O2 = plt2.plot(t_plot, flow_filtered_O2_plot, pen = '#0000FF', name = 'O2 flow')
c2_flow_filtered_O2 = plt6.plot(t_plot, flow_filtered_O2_plot, pen = '#0000FF', name = 'O2 flow')
c_flow_filtered = plt4.plot(t_plot, flow_filtered_plot, pen = '#FF0000', name = 'Flow')
c_vol_l = plt2.plot(t_plot, vol_l_plot, pen = '#FF00FF', name = 'Volume')
c2_vol_l = plt5.plot(t_plot, vol_l_plot, pen = '#FF00FF', name = 'Volume')
for item in plt.legend.items:
    for single_item in item:
        if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
            single_item.setText(single_item.text, **{'color': '#FFFFFF', 'size': '10pt'})
for item in plt2.legend.items:
    for single_item in item:
        if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
            single_item.setText(single_item.text, **{'color': '#FFFFFF', 'size': '10pt'})
for item in plt3.legend.items:
    for single_item in item:
        if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
            single_item.setText(single_item.text, **{'color': '#FFFFFF', 'size': '10pt'})
for item in plt4.legend.items:
    for single_item in item:
        if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
            single_item.setText(single_item.text, **{'color': '#FFFFFF', 'size': '10pt'})
for item in plt5.legend.items:
    for single_item in item:
        if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
            single_item.setText(single_item.text, **{'color': '#FFFFFF', 'size': '10pt'})
for item in plt6.legend.items:
    for single_item in item:
        if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
            single_item.setText(single_item.text, **{'color': '#FFFFFF', 'size': '10pt'})
if (scaley != 0): 
    plt.enableAutoRange('y', False)
    plt.setYRange(-scaley+offsety, scaley+offsety, 0)
    plt3.enableAutoRange('y', False)
    plt3.setYRange(-scaley+offsety, scaley+offsety, 0)
if (scale2y != 0): 
    plt2.enableAutoRange('y', False)
    plt2.setYRange(-scale2y+offset2y, scale2y+offset2y, 0)
    plt4.enableAutoRange('y', False)
    plt4.setYRange(-scale2y+offset2y, scale2y+offset2y, 0)
    plt5.enableAutoRange('y', False)
    plt5.setYRange(-scale2y+offset2y, scale2y+offset2y, 0)
    plt6.enableAutoRange('y', False)
    plt6.setYRange(-scale2y+offset2y, scale2y+offset2y, 0)

# Waiting for the file to be created...
while True:
    try:
        file = open('data.csv', 'r')
    except IOError:
        time.sleep(1)
    else:
        break

file.seek(0, os.SEEK_END)

debug_prev = -1
mode_prev = -1
bExit = 0
#count = 0
while (bExit != 1):
    start_time = time.time()

    try:
        data = file.readlines()
        if data:
            for line in data:
                cols = line.split(';')
                if (len(cols) > nb_cols): # Need the final '\n' so we are sure the numbers are fully written
                    #print(cols)
                    try:
                        index = 0
                        t = float(cols[index])
                        index = index+1
                        t0 = float(cols[index])
                        index = index+1
                        p0 = float(cols[index])
                        index = index+1
                        temperature0 = float(cols[index])
                        index = index+1
                        p = float(cols[index])
                        index = index+1
                        temperature = float(cols[index])
                        index = index+1
                        p_e = float(cols[index])
                        index = index+1
                        temperature_e = float(cols[index])
                        index = index+1
                        alarms = int(cols[index])
                        index = index+1
                        select = int(cols[index])
                        index = index+1
                        mode = int(cols[index])
                        index = index+1
                        flow_control_air = int(cols[index])
                        index = index+1
                        flow_control_O2 = int(cols[index])
                        index = index+1
                        flow_control_expi = int(cols[index])
                        index = index+1
                        Ppeak = int(cols[index])
                        index = index+1
                        PEEP = int(cols[index])
                        index = index+1
                        respi_rate = int(cols[index])
                        index = index+1
                        inspi_ratio = float(cols[index])
                        index = index+1
                        PEEP_dec_rate = int(cols[index])
                        index = index+1
                        Fl_PEEP_air = int(cols[index])
                        index = index+1
                        Fl_PEEP_O2 = int(cols[index])
                        index = index+1
                        PEEP_inspi_detection_delta = float(cols[index])
                        index = index+1
                        vol_inspi_detection_delta = int(cols[index])
                        index = index+1
                        inspi_detection_delta_duration = int(cols[index])
                        index = index+1
                        flow_thresh = float(cols[index])
                        index = index+1
                        index = index+1
                        index = index+1
                        valve_air = int(cols[index])
                        index = index+1
                        valve_O2 = int(cols[index])
                        index = index+1
                        valve_inspi = int(cols[index])
                        index = index+1
                        valve_expi = int(cols[index])
                        index = index+1
                        pressure_air = float(cols[index])
                        index = index+1
                        pressure_expi = float(cols[index])
                        index = index+1
                        pressure_O2 = float(cols[index])
                        index = index+1
                        temperature_air = float(cols[index])
                        index = index+1
                        temperature_expi = float(cols[index])
                        index = index+1
                        temperature_O2 = float(cols[index])
                        index = index+1
                        flow_air = float(cols[index])
                        index = index+1
                        flow_expi = float(cols[index])
                        index = index+1
                        flow_O2 = float(cols[index])
                        index = index+1
                        flow_filtered_air = float(cols[index])
                        index = index+1
                        flow_filtered_expi = float(cols[index])
                        index = index+1
                        flow_filtered_O2 = float(cols[index])
                        index = index+1
                        vol_air = float(cols[index])
                        index = index+1
                        vol_expi = float(cols[index])
                        index = index+1
                        vol_O2 = float(cols[index])
                        index = index+1

                        t_t0 = t-t0
                        p_cmh2o = (p-p0)*1.01972
                        p_e_cmh2o = (p_e-p0)*1.01972  
                        flow_filtered = flow_filtered_air+flow_filtered_O2+flow_filtered_expi
                        vol_l = (vol_air+vol_O2+vol_expi)

                        if (t_t0 < t_plot[-1]): 
                            # Reset if time seems to decrease...
                            t_plot = [0]
                            p_cmh2o_plot = [0]
                            p_e_cmh2o_plot = [0]
                            Ppeak_plot = [0]
                            PEEP_plot = [0]
                            flow_control_air_plot = [0]
                            flow_control_O2_plot = [0]
                            flow_control_expi_plot = [0]
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
                            flow_filtered_plot = [0]
                            vol_l_plot = [0]
                        if (select == 0): 
                            wintitle = '[Mode: {:d}]'
                            win.setWindowTitle(wintitle.format(int(mode)))
                        elif (select == 1): 
                            wintitle = '[Flow A: {:d}]'
                            win.setWindowTitle(wintitle.format(int(flow_control_air)))
                        elif (select == 2): 
                            wintitle = '[Flow O: {:d}]'
                            win.setWindowTitle(wintitle.format(int(flow_control_O2)))
                        elif (select == 3): 
                            wintitle = '[Flow E: {:d}]'
                            win.setWindowTitle(wintitle.format(int(flow_control_expi)))
                        elif (select == 4): 
                            wintitle = '[Ppeak: {:d}]'
                            win.setWindowTitle(wintitle.format(int(Ppeak*1.01972)))
                        elif (select == 5): 
                            wintitle = '[PEEP: {:d}]'
                            win.setWindowTitle(wintitle.format(int(PEEP*1.01972)))
                        elif (select == 6): 
                            wintitle = '[Respi. rate: {:d}/min]'
                            win.setWindowTitle(wintitle.format(int(respi_rate)))
                        elif (select == 7): 
                            wintitle = '[I:E: {:.2f}]'
                            win.setWindowTitle(wintitle.format(inspi_ratio))
                        elif (select == 8): 
                            wintitle = '[PE dec.: {:d}%]'
                            win.setWindowTitle(wintitle.format(int(PEEP_dec_rate)))
                        elif (select == 9): 
                            wintitle = '[Fl. E A: {:d}%]'
                            win.setWindowTitle(wintitle.format(int(Fl_PEEP_air)))
                        elif (select == 10): 
                            wintitle = '[Fl. E O: {:d}%]'
                            win.setWindowTitle(wintitle.format(int(Fl_PEEP_O2)))
                        elif (select == 11): 
                            wintitle = '[P I dlta: {:.1f}]'
                            win.setWindowTitle(wintitle.format(PEEP_inspi_detection_delta*1.01972))
                        elif (select == 12): 
                            wintitle = '[V I dlta: {:d}mL]'
                            win.setWindowTitle(wintitle.format(int(vol_inspi_detection_delta)))
                        elif (select == 13): 
                            wintitle = '[I dlta: {:d}ms]'
                            win.setWindowTitle(wintitle.format(int(inspi_detection_delta_duration)))
                        elif (select == 14): 
                            wintitle = '[Fl. th.: {:.2f}]'
                            win.setWindowTitle(wintitle.format(flow_thresh))
                        else: 
                            if ((int(t_t0) % 10) > 5): # Alternate text displayed
                                wintitle = 'Mode: {:d} Flow A: {:d} Flow O: {:d} Flow E: {:d} Ppeak: {:d} PEEP: {:d} Respi. rate: {:d}/min I:E: {:.2f}'
                                win.setWindowTitle(wintitle.format(int(mode), int(flow_control_air), int(flow_control_O2), int(flow_control_expi), int(Ppeak*1.01972), int(PEEP*1.01972), int(respi_rate), inspi_ratio))
                            else:
                                wintitle = 'PE dec.: {:d}% Fl. E A: {:d}% Fl. E O: {:d}% P I dlta: {:.1f} V I dlta: {:d}mL I dlta: {:d}ms Fl. th.: {:.2f}'
                                win.setWindowTitle(wintitle.format(int(PEEP_dec_rate), int(Fl_PEEP_air), int(Fl_PEEP_O2), PEEP_inspi_detection_delta*1.01972, int(vol_inspi_detection_delta), int(inspi_detection_delta_duration), flow_thresh))
                        if ((alarms != 0) and ((int(t_t0) % 2) == 0)):
                            wintitle = 'Alarm 0x{:04X} : '
                            if ((alarms & HARDWARE_ALARM) > 0):
                                wintitle = wintitle+'Check sensors, '
                            if ((alarms & OS_ALARM) > 0):
                                wintitle = wintitle+'Restart or check SD, '
                            if ((alarms & PRESSURE_ALARM) > 0):
                                wintitle = wintitle+'Pressure too low or high, '
                            if ((alarms & FLOW_ALARM) > 0):
                                wintitle = wintitle+'Flow too low or high, '
                            if ((alarms & VOL_ALARM) > 0):
                                wintitle = wintitle+'Volume too low or high, '
                            if ((alarms & PPEAK_ALARM) > 0):
                                wintitle = wintitle+'Ppeak not reached, '
                            if ((alarms & PEEP_ALARM) > 0):
                                wintitle = wintitle+'PEEP not reached, '
                            wintitle = wintitle[:-2]
                            win.setWindowTitle(wintitle.format(int(alarms)))
                        if debug:
                            if (debug_prev != debug):
                                win.move(-4, 4)
                                win.resize(800, 420)
                            plt.setTitle('Temp. I: {:.2f} C, Temp. E: {:.2f} C'.format(temperature, temperature_e))
                            plt2.setTitle('Temp. A: {:.2f}, E: {:.2f}, O: {:.2f}'.format(temperature_air, temperature_expi, temperature_O2))
                            plt.show()
                            plt2.show()
                            plt3.hide()
                            plt4.hide()
                            plt5.hide()
                            plt6.hide()
                        else:
                            if (debug_prev != debug):
                                win.move(-10, -32)
                                win.resize(808, 452)
                            plt.hide()
                            plt2.hide()
                            plt3.show()
                            if (mode == 2): 
                                plt4.hide()
                                plt5.hide()
                                plt6.show()
                            else:
                                plt4.show()
                                plt5.show()
                                plt6.hide()

                        debug_prev = debug
                        mode_prev = mode

                        # Should ensure that no ValueError exception can happen here to avoid lists of different length, 
                        # so no float conversion should be done in the append()...
                        t_plot.append(t_t0)
                        p_cmh2o_plot.append(p_cmh2o)
                        p_e_cmh2o_plot.append(p_e_cmh2o)
                        Ppeak_plot.append(Ppeak)
                        PEEP_plot.append(PEEP)
                        flow_control_air_plot.append(flow_control_air)
                        flow_control_O2_plot.append(flow_control_O2)
                        flow_control_expi_plot.append(flow_control_expi)
                        valve_air_plot.append(float(valve_air)/10.0)
                        valve_O2_plot.append(float(valve_O2)/10.0)
                        valve_inspi_plot.append(10.0*valve_inspi)
                        valve_expi_plot.append(float(valve_expi)/10.0)
                        flow_air_plot.append(flow_air)
                        flow_expi_plot.append(flow_expi)
                        flow_O2_plot.append(flow_O2)
                        flow_filtered_air_plot.append(flow_filtered_air)
                        flow_filtered_expi_plot.append(flow_filtered_expi)
                        flow_filtered_O2_plot.append(flow_filtered_O2)
                        flow_filtered_plot.append(flow_filtered)
                        vol_l_plot.append(100.0*vol_l)
                        if (t_plot[-1]-t_plot[0] > 2*scalex):
                            t_plot.pop(0)
                            p_cmh2o_plot.pop(0)
                            p_e_cmh2o_plot.pop(0)
                            Ppeak_plot.pop(0)
                            PEEP_plot.pop(0)
                            flow_control_air_plot.pop(0)
                            flow_control_O2_plot.pop(0)
                            flow_control_expi_plot.pop(0)
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
                            flow_filtered_plot.pop(0)
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
    c2_p_cmh2o.setData(t_plot, p_cmh2o_plot)
    c_p_e_cmh2o.setData(t_plot, p_e_cmh2o_plot)
    c_Ppeak.setData(t_plot, Ppeak_plot)
    c2_Ppeak.setData(t_plot, Ppeak_plot)
    c_PEEP.setData(t_plot, PEEP_plot)
    c2_PEEP.setData(t_plot, PEEP_plot)
    c_flow_control_air.setData(t_plot, flow_control_air_plot)
    c_flow_control_O2.setData(t_plot, flow_control_O2_plot)
    c_flow_control_expi.setData(t_plot, flow_control_expi_plot)
    c_valve_air.setData(t_plot, valve_air_plot)
    c_valve_O2.setData(t_plot, valve_O2_plot)
    c_valve_inspi.setData(t_plot, valve_inspi_plot)
    c_valve_expi.setData(t_plot, valve_expi_plot)
    c_flow_air.setData(t_plot, flow_air_plot)
    c_flow_expi.setData(t_plot, flow_expi_plot)
    c_flow_O2.setData(t_plot, flow_O2_plot)
    c_flow_filtered_air.setData(t_plot, flow_filtered_air_plot)
    c2_flow_filtered_air.setData(t_plot, flow_filtered_air_plot)
    c_flow_filtered_expi.setData(t_plot, flow_filtered_expi_plot)
    c_flow_filtered_O2.setData(t_plot, flow_filtered_O2_plot)
    c2_flow_filtered_O2.setData(t_plot, flow_filtered_O2_plot)
    c_flow_filtered.setData(t_plot, flow_filtered_plot)
    c_vol_l.setData(t_plot, vol_l_plot)
    c2_vol_l.setData(t_plot, vol_l_plot)

    pg.QtGui.QApplication.processEvents()

    #count += 1

    end_time = time.time()
    #print('It has been %0.4f seconds since the loop started' %(end_time - start_time))

file.close()

win.close()
