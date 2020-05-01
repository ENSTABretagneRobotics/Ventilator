#!/usr/bin/python

from __future__ import division
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
debug = False
###############################################################################

nb_cols = 44 # Not counting the final '\n'
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
    global bExit
    #print("Key pressed")
    #print(evt.key())
    if evt.key() == QtCore.Qt.Key_Escape:
        bExit = 1

#win = pg.GraphicsWindow()
win = GUIWindow()
win.sigKeyPress.connect(keyPressed)
if not debug:
    win.move(-10, -5)
    win.resize(808, 463)
else:
    win.move(0, 0)
    win.resize(800, 420)
win.setWindowTitle('Pressure, flow, volume')
if debug:
    plt = win.addPlot()
    plt2 = win.addPlot()
else: 
    plt = win.addPlot(row = 0, col = 0)
    plt2 = win.addPlot(row = 1, col = 0)
    plt3 = win.addPlot(row = 2, col = 0)
plt.showGrid(x = True, y = True)
plt2.showGrid(x = True, y = True)
if not debug: plt3.showGrid(x = True, y = True)
if debug: plt.setTitle('Temp. I: 25.00 C, Temp. E: 25.00 C', **{'color': '#FFFFFF', 'size': '10pt'})
if debug: plt2.setTitle('Temp. A: 25.00 C, Temp. E: 25.00 C, Temp. O: 25.00 C', **{'color': '#FFF', 'size': '10pt'})
plt.setLabel('left', 'Pressure (in cmH2O)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt2.setLabel('left', 'Flow (in L/min)', **{'color': '#FFFFFF', 'font-size': '10pt'})
if debug: plt2.setLabel('right', 'Volume (in cl)', **{'color': '#FFFFFF', 'font-size': '10pt'})
else: plt3.setLabel('left', 'Volume (in cl)', **{'color': '#FFFFFF', 'font-size': '10pt'})
if debug: plt.setLabel('bottom', 'Time (in s)', **{'color': '#FFFFFF', 'font-size': '10pt'})
if debug: plt2.setLabel('bottom', 'Time (in s)', **{'color': '#FFFFFF', 'font-size': '10pt'})
plt.addLegend(size = (0, 0), offset = (4, 1))
plt2.addLegend(size = (0, 0), offset = (4, 1))
if not debug: plt3.addLegend(size = (0, 0), offset = (4, 1))
font = QtGui.QFont()
font.setPixelSize(11)
plt.getAxis('left').tickFont = font
plt2.getAxis('left').tickFont = font
plt2.getAxis('right').tickFont = font
if not debug: plt3.getAxis('left').tickFont = font
plt.getAxis('bottom').tickFont = font
plt2.getAxis('bottom').tickFont = font
if not debug: plt3.getAxis('bottom').tickFont = font
c_p_cmh2o = plt.plot(t_plot, p_cmh2o_plot, pen = '#FFFF00', name = 'Inspi. pres.')
if debug: c_p_e_cmh2o = plt.plot(t_plot, p_e_cmh2o_plot, pen = '#FFA500', name = 'Expi. pres.')
c_Ppeak = plt.plot(t_plot, Ppeak_plot, pen = '#FFFFFF')
c_PEEP = plt.plot(t_plot, PEEP_plot, pen = '#AAAAAA')
if debug: c_flow_control_air = plt2.plot(t_plot, flow_control_air_plot, pen = '#AAFF00')
if debug: c_flow_control_O2 = plt2.plot(t_plot, flow_control_O2_plot, pen = '#00AAFF')
if debug: c_flow_control_expi = plt2.plot(t_plot, flow_control_expi_plot, pen = '#FF00AA')
if debug: c_valve_air = plt.plot(t_plot, valve_air_plot, pen = '#00FF00', name = 'Air valve')
if debug: c_valve_O2 = plt.plot(t_plot, valve_O2_plot, pen = '#0000FF', name = 'O2 valve')
if debug: c_valve_inspi = plt.plot(t_plot, valve_inspi_plot, pen = '#00FFFF', name = 'Inspi. valve')
if debug: c_valve_expi = plt.plot(t_plot, valve_expi_plot, pen = '#FF0000', name = 'Expi. valve')
if debug: c_flow_air = plt2.plot(t_plot, flow_air_plot, pen = '#003000')
if debug: c_flow_expi = plt2.plot(t_plot, flow_expi_plot, pen = '#300000')
if debug: c_flow_O2 = plt2.plot(t_plot, flow_O2_plot, pen = '#003030')
if debug: c_flow_filtered_air = plt2.plot(t_plot, flow_filtered_air_plot, pen = '#00FF00', name = 'Air flow')
if debug: c_flow_filtered_expi = plt2.plot(t_plot, flow_filtered_expi_plot, pen = '#FF0000', name = 'Expi. flow')
if debug: c_flow_filtered_O2 = plt2.plot(t_plot, flow_filtered_O2_plot, pen = '#0000FF', name = 'O2 flow')
if not debug: c_flow_filtered = plt2.plot(t_plot, flow_filtered_plot, pen = '#FF0000', name = 'Flow')
if debug: c_vol_l = plt2.plot(t_plot, vol_l_plot, pen = '#FF00FF', name = 'Volume')
else: c_vol_l = plt3.plot(t_plot, vol_l_plot, pen = '#00FF00', name = 'Volume')
for item in plt.legend.items:
    for single_item in item:
        if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
            single_item.setText(single_item.text, **{'color': '#FFFFFF', 'size': '10pt'})
for item in plt2.legend.items:
    for single_item in item:
        if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
            single_item.setText(single_item.text, **{'color': '#FFFFFF', 'size': '10pt'})
if not debug: 
    for item in plt3.legend.items:
        for single_item in item:
            if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
                single_item.setText(single_item.text, **{'color': '#FFFFFF', 'size': '10pt'})
if (scaley != 0): 
    plt.enableAutoRange('y', False)
    plt.setYRange(-scaley+offsety, scaley+offsety, 0)
if (scale2y != 0): 
    plt2.enableAutoRange('y', False)
    plt2.setYRange(-scale2y+offset2y, scale2y+offset2y, 0)
    if not debug: 
        plt3.enableAutoRange('y', False)
        plt3.setYRange(-scale2y+offset2y, scale2y+offset2y, 0)

# Waiting for the file to be created...
while True:
    try:
        file = open('data.csv', 'r')
    except IOError:
        time.sleep(1)
    else:
        break

file.seek(0, os.SEEK_END)

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
                        select = float(cols[index])
                        index = index+1
                        Ppeak = float(cols[index])
                        index = index+1
                        PEEP = float(cols[index])
                        index = index+1
                        respi_rate = float(cols[index])
                        index = index+1
                        inspi_ratio = float(cols[index])
                        index = index+1
                        flow_control_air = float(cols[index])
                        index = index+1
                        flow_control_O2 = float(cols[index])
                        index = index+1
                        flow_control_expi = float(cols[index])
                        index = index+1
                        mode = float(cols[index])
                        index = index+1
                        PEEP_dec_rate = float(cols[index])
                        index = index+1
                        Fl_PEEP = float(cols[index])
                        index = index+1
                        PEEP_inspi_detection_delta = float(cols[index])
                        index = index+1
                        vol_inspi_detection_delta = float(cols[index])
                        index = index+1
                        inspi_detection_delta_duration = float(cols[index])
                        index = index+1
                        flow_thresh = float(cols[index])
                        index = index+1
                        index = index+1
                        index = index+1
                        valve_air = float(cols[index])
                        index = index+1
                        valve_O2 = float(cols[index])
                        index = index+1
                        valve_inspi = float(cols[index])
                        index = index+1
                        valve_expi = float(cols[index])
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
                            wintitle = '[Ppeak: {:d}]'
                            win.setWindowTitle(wintitle.format(int(Ppeak*1.01972)))
                        elif (select == 1): 
                            wintitle = '[PEEP: {:d}]'
                            win.setWindowTitle(wintitle.format(int(PEEP*1.01972)))
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
                            wintitle = '[Flow E: {:d}]'
                            win.setWindowTitle(wintitle.format(int(flow_control_expi)))
                        elif (select == 7): 
                            wintitle = '[Mode: {:d}]'
                            win.setWindowTitle(wintitle.format(int(mode)))
                        elif (select == 8): 
                            wintitle = '[PEEP dec.: {:d}%]'
                            win.setWindowTitle(wintitle.format(int(PEEP_dec_rate)))
                        elif (select == 9): 
                            wintitle = '[Fl_PEEP: {:d}%]'
                            win.setWindowTitle(wintitle.format(int(Fl_PEEP)))
                        elif (select == 10): 
                            wintitle = '[PEEP I delta: {:.1f}]'
                            win.setWindowTitle(wintitle.format(PEEP_inspi_detection_delta*1.01972))
                        elif (select == 11): 
                            wintitle = '[Vol. I delta: {:d}mL]'
                            win.setWindowTitle(wintitle.format(int(vol_inspi_detection_delta)))
                        elif (select == 12): 
                            wintitle = '[I delta: {:d}ms]'
                            win.setWindowTitle(wintitle.format(int(inspi_detection_delta_duration)))
                        elif (select == 13): 
                            wintitle = '[Fl. th.: {:.2f}]'
                            win.setWindowTitle(wintitle.format(flow_thresh))
                        else: 
                            if ((t_t0) % 10 > 5): # Alternate text displayed
                                wintitle = 'Ppeak: {:d}, PEEP: {:d}, Respi. rate: {:d}/min, I:E: {:.2f}, Flow A: {:d}, Flow O: {:d}, Flow E: {:d}, Mode: {:d}'
                                win.setWindowTitle(wintitle.format(int(Ppeak*1.01972), int(PEEP*1.01972), int(respi_rate), inspi_ratio, int(flow_control_air), int(flow_control_O2), int(flow_control_expi), int(mode)))
                            else:
                                wintitle = 'PEEP dec.: {:d}%, Fl. PEEP: {:d}%, PEEP I delta: {:.1f}, Vol. I delta: {:d}mL, I delta: {:d}ms, Fl. th.: {:.2f}'
                                win.setWindowTitle(wintitle.format(int(PEEP_dec_rate), int(Fl_PEEP), PEEP_inspi_detection_delta*1.01972, int(vol_inspi_detection_delta), int(inspi_detection_delta_duration), flow_thresh))
                        if debug: plt.setTitle('Temp. I: {:.2f} C, Temp. E: {:.2f} C'.format(temperature, temperature_e))
                        if debug: plt2.setTitle('Temp. A: {:.2f}, E: {:.2f}, O: {:.2f}'.format(temperature_air, temperature_expi, temperature_O2))
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
                        valve_air_plot.append(valve_air/10.0)
                        valve_O2_plot.append(valve_O2/10.0)
                        valve_inspi_plot.append(10.0*valve_inspi)
                        valve_expi_plot.append(valve_expi/10.0)
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
    if debug: c_p_e_cmh2o.setData(t_plot, p_e_cmh2o_plot)
    c_Ppeak.setData(t_plot, Ppeak_plot)
    c_PEEP.setData(t_plot, PEEP_plot)
    if debug: c_flow_control_air.setData(t_plot, flow_control_air_plot)
    if debug: c_flow_control_O2.setData(t_plot, flow_control_O2_plot)
    if debug: c_flow_control_expi.setData(t_plot, flow_control_expi_plot)
    if debug: c_valve_air.setData(t_plot, valve_air_plot)
    if debug: c_valve_O2.setData(t_plot, valve_O2_plot)
    if debug: c_valve_inspi.setData(t_plot, valve_inspi_plot)
    if debug: c_valve_expi.setData(t_plot, valve_expi_plot)
    if debug: c_flow_air.setData(t_plot, flow_air_plot)
    if debug: c_flow_expi.setData(t_plot, flow_expi_plot)
    if debug: c_flow_O2.setData(t_plot, flow_O2_plot)
    if debug: c_flow_filtered_air.setData(t_plot, flow_filtered_air_plot)
    if debug: c_flow_filtered_expi.setData(t_plot, flow_filtered_expi_plot)
    if debug: c_flow_filtered_O2.setData(t_plot, flow_filtered_O2_plot)
    if not debug: c_flow_filtered.setData(t_plot, flow_filtered_plot)
    c_vol_l.setData(t_plot, vol_l_plot)

    pg.QtGui.QApplication.processEvents()

    #count += 1

    end_time = time.time()
    #print('It has been %0.4f seconds since the loop started' %(end_time - start_time))

file.close()

win.close()
