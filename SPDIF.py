# -*- coding: utf-8 -*-
"""
Created on Fri Aug 14 16:12:08 2015

@author: tobia
"""

from __future__ import print_function, division, absolute_import

from PyQt4 import QtGui, QtCore
#from PyQt4.QtGui import qApp
#import os
import PyQt4.Qwt5 as Qwt

import GUI_SPDIF

#==============================================================================
# PYTHON IMPORTS
#==============================================================================
import sys, pyaudio, numpy, threading, serial

#from recorder import *
#import matplotlib

#==============================================================================
# DEFINES
#==============================================================================
       
class SPDIF:
        
    def __init__(self):        
        
        self.RATE=44100
        self.BUFFERSIZE=1024
        self.secToRecord=.001
        self.threadDieNow=False
        self.newAudio=False
        self.bThreadStarted=False
        
    def setup(self):    #SPDIF        
        """
        initialisiere Soundkarte, Quelle: http://www.swharden.com/blog/ (Link vom 27.08.2015)
        """
        self.buffersToRecord=int(self.RATE*self.secToRecord/self.BUFFERSIZE)
        if self.buffersToRecord==0: self.buffersToRecord=1
        self.samplesToRecord=int(self.BUFFERSIZE*self.buffersToRecord)
        self.chunksToRecord=int(self.samplesToRecord/self.BUFFERSIZE)
        self.secPerPoint=1.0/self.RATE

        self.p = pyaudio.PyAudio()
        self.inStream = self.p.open(format=pyaudio.paInt16,channels=1,rate=self.RATE,input=True,input_device_index=uiplot.comboBox_Audio_In.currentIndex(),frames_per_buffer=self.BUFFERSIZE)
        self.xsBuffer=numpy.arange(self.BUFFERSIZE)*self.secPerPoint
        self.xs=numpy.arange(self.chunksToRecord*self.BUFFERSIZE)*self.secPerPoint
        self.audio=numpy.empty((self.chunksToRecord*self.BUFFERSIZE),dtype=numpy.int16)        
        
    def close(self):                                        #SPDIF
        """cleanly back out and release sound card."""
        self.p.close(self.inStream)
        self.threadDieNow = True
        self.tR._Thread__stop()
   
    def readSignal(self):                                   #SPDIF        
        """
        lese Audio Signal vom Audio Device ein
        """        
        AudioString=self.inStream.read(self.BUFFERSIZE)
        return numpy.fromstring(AudioString, dtype=numpy.int16)
        
    
    def record(self,forever=True):                          #SPDIF
        """
        Thread: zeichnet Zeitintervall des Audiosignals auf.
        """
        while True:
            self.lock.acquire()
            if self.threadDieNow: break
            for i in range(self.chunksToRecord):
                self.audio[i*self.BUFFERSIZE:(i+1)*self.BUFFERSIZE]=self.readSignal()
            self.newAudio=True 
            if forever==False: break
            self.lock.release()

    def continuousStart(self):                              #SPDIF
        """
        Vorbereitung Thread zur Darstellung des Audiosignals.
        """
        self.tR = threading.Thread(target=self.record)
        self.lock = threading.Lock()
                   
        QtGui.QDialog.connect(uiplot.pushButton_Start, QtCore.SIGNAL("clicked()"), self.countClick)    # Echtzeitdarstellung des Audiosignals wird gestartet           
        QtGui.QDialog.connect(uiplot.pushButton_Stop, QtCore.SIGNAL("clicked()"), self.suspend)        # Echtzeitdarstellung des Audiosignals wird angehalten     
    
    def suspend(self):                                      #SPDIF
        '''
        Thread wird angehalten, Auswahl der Audioquelle aktiviert, Stop_Button deaktiviert
        '''        
        self.lock.acquire()
        uiplot.comboBox_Audio_In.setEnabled(True)
        uiplot.pushButton_Stop.setDisabled(True)
        uiplot.pushButton_Start.setEnabled(True)
    
    def countClick(self):                                   #SPDIF
        """
        Startet den Thread "record" beim ersten Klick auf "Start"
        Setzt den Thread "record" fort bei jedem weiteren Klick auf "Start"
        """        
        if self.bThreadStarted == False:
            self.setup()
            self.tR.start()
            self.bThreadStarted = True
            uiplot.comboBox_Audio_In.setDisabled(True)
            uiplot.pushButton_Start.setDisabled(True)
            uiplot.pushButton_Stop.setEnabled(True)
            print(uiplot.comboBox_Audio_In.currentIndex())
        else:
            self.setup()
            self.lock.release()
            uiplot.comboBox_Audio_In.setDisabled(True)
            uiplot.pushButton_Start.setDisabled(True)
            uiplot.pushButton_Stop.setEnabled(True)
            print(uiplot.comboBox_Audio_In.currentIndex())
        
    def setupAudio(self):                               #SPDIF
        """
        Erstellt eine Auswahl verfuegbarer Audioquellen
        """
        deviceList = []
        uiplot.comboBox_Audio_In.clear()
        self.p = pyaudio.PyAudio() # instantiate PyAudio, start PortAudio system + list devices
        defaultInIdx = self.p.get_default_input_device_info()['index']
        #defaultOutIdx = self.p.get_default_output_device_info()['index']
        print("Defaultin", defaultInIdx)
        for i in range(self.p.get_device_count()):
             deviceList.append(self.p.get_device_info_by_index(i))    
             print (deviceList[i])
             if deviceList[i]['maxInputChannels'] > 0:
                 if i == defaultInIdx:
                     uiplot.comboBox_Audio_In.addItem('* '+deviceList[i]['name'], str(i))
                     defaultInBoxIdx = uiplot.comboBox_Audio_In.currentIndex()
                 else:
                     uiplot.comboBox_Audio_In.addItem(deviceList[i]['name'], str(i))
                     
#                 self.comboBoxAudioIn.setItemData(str(i))
#             else:
#                 if i == defaultOutIdx:
#                     self.comboBoxAudioOut.addItem('* '+deviceList[i]['name'], str(i))
#                     defaultOutBoxIdx = self.comboBoxAudioOut.currentIndex()
#                 else:
#                     self.comboBoxAudioOut.addItem(deviceList[i]['name'], str(i))   
        uiplot.comboBox_Audio_In.setCurrentIndex(defaultInBoxIdx)
        #self.comboBoxAudioOut.setCurrentIndex(defaultOutBoxIdx)
#        print("Default Output Device : %s" % self.p.get_default_output_device_info()['name'])
#        self.comboBoxAudioOut.addItems(deviceList)        
        
        
    def plotSignal(self):        
        """
        Gibt das vom Audio Device eingelsesene Signal als Plot in die GUI aus
        """
        if s.newAudio==False: 
            return
        c.setData(s.xs,s.audio)
        uiplot.qwtPlot_Zeitsignal.replot()
        s.newAudio=False
        
class I2C:

    def __init__(self):                                 #I2C
        """
        Initialisierung I2C für ELV USB/I2C Interface
        Scan nach erreichbaren COM Schnittstellen
        """
        self.BAUDRATE = 115200                          # Einstellungen des ELV USB/I2C Interface gemaess Bedienungsanleitung
        self.OpenPort = False
        self.threadDieNow=False
        uiplot.comboBox_Abfragerate.addItem(str(0.01))
        uiplot.comboBox_Abfragerate.addItem(str(0.1))
        uiplot.comboBox_Abfragerate.addItem(str(1.0))
        uiplot.comboBox_Abfragerate.addItem(str(2.0))
        uiplot.comboBox_Abfragerate.setCurrentIndex(3)
        uiplot.horizontalSlider_Volume.logicalDpiX()
        ValidComPorts = self.serialScan()
        if (len(ValidComPorts)) == 0:
            uiplot.comboBox_COM.addItem("KEIN COM!")
        else:
            for i in range(len(ValidComPorts)):
                uiplot.comboBox_COM.addItem(ValidComPorts[i][1])
        com = uiplot.comboBox_COM.currentText()
        if (com == "KEIN COM!"):
            print("Kein COM")
        else:
            self.serialPort()
            
        if self.OpenPort:
            self.ser.write('Y41')               # Konfigurationsbefehle für ELV gemaess Bedienungsanleitung S. 11 Tab. 3
            self.ser.write('Y01')
        else:
            print("Kein COM")
    
    def serialScan(self):                               #I2C
        """
        gibt alle erreichbaren COM Schnittstellen zurueck, hauptsaechlich aus "Verstaerker_v3_3_3.py" uebernommen
        """
        ports = []
        for i in range(256):
            try:
                self.ser = serial.Serial(i)
                ports.append([i, self.ser.portstr])
                self.ser.close()
            except serial.SerialException:
                pass
        if(len(ports) == 0):
            print("Es wurde kein freier COM-Port gefunden.")
        return ports
        
    def serialPort(self):                               #I2C
        """
        serialPort oeffnet den ausgewaehlten COM-Port, hauptsaechlich aus "Verstaerker_v3_3_3.py" uebernommen
        """
        com = uiplot.comboBox_COM.currentText()
        self.ser = serial.Serial(
                                 port=str(com),
                                 baudrate=self.BAUDRATE,
                                 parity=serial.PARITY_EVEN,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS
                                 )
        self.OpenPort = True
        if(self.ser.isOpen() == False):
            try:
                self.ser.open()
                self.OpenPort = True
            except serial.SerialException:
                self.OpenPort = False
        
    def requireData(self):                              #I2C
        """
        Thread zur zyklischen Abfrage von Daten auf I2C
        """
        while True:
            self.cond.acquire()
            self.cond.wait(float(uiplot.comboBox_Abfragerate.currentText()))
            if self.threadDieNow: break
            if self.OpenPort:
                self.ser.write('sbedapsbf09p')
                print("Anforderung Daten")
                print(uiplot.horizontalSlider_Volume.value())
                self.readI2C()
            else:
                print("COM Port nicht geoeffnet")
                self.testGUI()
            self.cond.release()
 

    def continuousStart(self):                          #I2C
        """
        Vorbereitung Thread zur zyklischen Abfrage
        Verarbeitung von Klick-Ereignissen zur MUX und Volume Einstellung
        """
        self.tI2C = threading.Thread(target=self.requireData)
        self.cond = threading.Condition()
        uiplot.comboBox_COM.currentIndexChanged.connect(self.serialPort)
        self.tI2C.start()
        QtGui.QDialog.connect(uiplot.radioButton_MUX1, QtCore.SIGNAL("clicked()"), self.sendMUX1)
        QtGui.QDialog.connect(uiplot.radioButton_MUX2, QtCore.SIGNAL("clicked()"), self.sendMUX2)
        QtGui.QDialog.connect(uiplot.horizontalSlider_Volume, QtCore.SIGNAL('valueChanged(int)'), self.sendVolume)
        
    def sendMUX1(self):                                 #I2C
        """
        schreibt die Auswahl MUX1 auf I2C
        """
        if self.OpenPort:
            self.ser.write('sbe1501p')
        else:
            print("COM nicht offen")
 
    def sendMUX2(self):                                 #I2C
        """
        schreibt die Auswahl MUX2 auf I2C
        """
        if self.OpenPort:       
            self.ser.write('sbe1502p')                   
        else:
            print("COM nicht offen")
            
    def sendVolume(self):                               #I2C
        """
        schreibt die Auswahl der Volume auf I2C
        """
        if self.OpenPort:
            data = str(uiplot.horizontalSlider_Volume.value())
            self.ser.write('sbe2a'+data+'p')           
        else:
            print("COM nicht offen")
            
    def readI2C(self):                                  #I2C
        """
        liest die empfangenen Daten von I2C
        """
        print("Lese I2C")
        while self.ser.inWaiting() > 0:
            self.wert=self.ser.read(2)
            print(self.wert)            # gibt die eingelesenen Werte in der Konsole aus, solange "writeGUI" noch nicht existiert
   
    def writeGUI(self, data):                           #I2C
        '''
        hier entsteht die Funktion, die die vom I2C-Bus gelesenen Werte in die GUI ausgibt,
        warte auf Anweisungen von Josef Klugbauer
        '''
        pass
        
    
    def testGUI(self):
        """
        Testfunktion, falls keine COM-Schnittstelle vorhanden
        """
        uiplot.lcdNumber_2_Ausgang.display(1860)
        uiplot.lcdNumber_Strom.display(1860)
        uiplot.lcdNumber_Temperatur.display(1860)
        uiplot.lcdNumber_Vcc.display(1860)
        
    def close(self):
        """
        Serielle Schnittstelle schliessen und Thread beenden
        """
        if self.OpenPort:    
            self.ser.close()
        self.threadDieNow = True
        self.tI2C._Thread__stop()
        
#==============================================================================
# MAIN
#==============================================================================
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    
    win_plot = GUI_SPDIF.QtGui.QMainWindow()
    uiplot = GUI_SPDIF.Ui_Dialog()
    uiplot.setupUi(win_plot)
    s = SPDIF()         # Instanz von SPDIF wird erzeugt
    bus = I2C()         # Instanz von I2C wird erzeugt
    
    c = Qwt.QwtPlotCurve()
    c.attach(uiplot.qwtPlot_Zeitsignal)
    
    uiplot.qwtPlot_Zeitsignal.setAxisScale(uiplot.qwtPlot_Zeitsignal.yLeft,-10000,10000)
    uiplot.Thermo_Ausgang.setRange(-100.0,100.0)
    uiplot.Thermo_Vcc.setRange(-100.0,100.0)
    
    uiplot.pushButton_Stop.setDisabled(True)
    uiplot.timer = QtCore.QTimer()
    uiplot.timer.start(1.0)
    
    win_plot.connect(uiplot.timer, QtCore.SIGNAL('timeout()'), s.plotSignal)
        
    s.setupAudio()
    s.setup()
    s.continuousStart()         # startet SPDIF
    bus.continuousStart()       # startet I2C
    
    win_plot.show()
    
    code = app.exec_()          # Beenden des Programms
    s.close()
    bus.close()
    sys.exit(code)