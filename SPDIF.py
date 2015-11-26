# -*- coding: utf-8 -*-
"""
Created on Fri Aug 14 16:12:08 2015

@author: tobia
"""

from __future__ import print_function, division, absolute_import
from PyQt4 import QtGui, QtCore, uic
#from PyQt4.QtGui import qApp
import PyQt4.Qwt5 as Qwt
import I2C_Daten

#==============================================================================
# PYTHON IMPORTS
#==============================================================================
import sys, pyaudio, numpy, threading, serial

#==============================================================================
# DEFINES
#==============================================================================
       
class SPDIF:
        
    def __init__(self):        
        
        #Initialisierung der Variablen:
        self.RATE = 44100
        self.BUFFERSIZE = 1024
        self.secToRecord = .001
        self.threadDieNow = False                   #beendet Thread, wenn True
        self.newAudio = False                       #gibt an, ob neues Audio-Zeitintervall vorliegt
        self.bThreadStarted = False                 #gibt an, ob Thread gestartet wurde (s. Methode countClick)
        self.channels = 0                           #Anzahl der Kanaele
        self.anzeige = 1                            #Auswahl der Darstellungsform (Einkanal, Differenz, XY)
        self.stream_open = False                    #gibt an, ob Audiostream geoeffnet werden konnte
        
        #Initialisierung der QWT-Objekte:
        uiplot.comboBox_channels.addItem(str(1))
        uiplot.comboBox_channels.addItem(str(2))
        uiplot.comboBox_channels.setCurrentIndex(1)
        uiplot.spinBox_Ymin.setRange(-10000,10000)
        uiplot.spinBox_Ymax.setRange(-10000,10000)
        uiplot.spinBox_Ymin.setValue(-10000)
        uiplot.spinBox_Ymax.setValue(10000)
        uiplot.radioButton_Einkanal.setChecked(1)
        
        #Programmaufrufe:
        self.setupAudio()                           #Erstellt eine Auswahl verfuegbarer Audioquellen
        self.continuousStart()                      #Vorbereitung Thread zur Darstellung des Audiosignals
        
        uiplot.connect(uiplot.timer, QtCore.SIGNAL('timeout()'), self.plotSignal)
        uiplot.show()
        
        
    def setup(self):    #SPDIF        
        """
        initialisiere Soundkarte, Quelle: http://www.swharden.com/blog/ (Link vom 27.08.2015)
        """
        self.channels = int(uiplot.comboBox_channels.currentText())
        self.buffersToRecord = int(self.RATE*self.secToRecord/self.BUFFERSIZE)        
        if 0 == self.buffersToRecord: 
            self.buffersToRecord = 1
        self.samplesToRecord = int(self.BUFFERSIZE*self.buffersToRecord)
        self.chunksToRecord = int(self.samplesToRecord/self.BUFFERSIZE)
        self.secPerPoint = 1.0/self.RATE
        
        self.p = pyaudio.PyAudio()        
        try:
            self.inStream = self.p.open(format=pyaudio.paInt16,channels=self.channels,rate=self.RATE,input=True,input_device_index=uiplot.comboBox_Audio_In.currentIndex(),frames_per_buffer=self.BUFFERSIZE)
            self.stream_open = True
            uiplot.statusBar().clear()
        except:
            print("Kein Audiodevice vorhanden")
            uiplot.statusMessage("Kein Audiodevice vorhanden.")
            self.stream_open = False
        self.xsBuffer = numpy.arange(self.BUFFERSIZE)*self.secPerPoint
        
        self.xs = numpy.arange(self.chunksToRecord*self.BUFFERSIZE)*self.secPerPoint
        self.audio = numpy.empty((self.chunksToRecord*self.BUFFERSIZE*self.channels),dtype=numpy.int16)        
        
    def close(self):                                        #SPDIF
        """cleanly back out and release sound card."""
        if self.stream_open:
            self.p.close(self.inStream)
            self.threadDieNow = True
            self.tR._Thread__stop()
   
    def readSignal(self):                                   #SPDIF        
        """
        lese Audio Signal vom Audio Device ein
        """        
        AudioString = self.inStream.read(self.BUFFERSIZE)
        
        return numpy.fromstring(AudioString, dtype=numpy.int16)        
    
    def record(self):                          #SPDIF
        """
        Thread: zeichnet Zeitintervall des Audiosignals auf.
        """
        while True:
            self.lock.acquire()             #wird bei Ereignis "Stop" unterbrochen
            
            if self.threadDieNow: break
            try:
                for i in range(self.chunksToRecord):                
                    self.audio[i*self.BUFFERSIZE:(i+1)*self.BUFFERSIZE*self.channels] = self.readSignal()
                uiplot.statusBar().clear()
            except:
                print("Kananlanzahl falsch")
                uiplot.statusMessage("Kananlanzahl falsch.")
            
            self.newAudio = True 
            self.audio_l = self.audio[0::2]               #Aufteilung in linker und rechter Kanal
            self.audio_r = self.audio[1::2]
            self.audio_diff = self.audio_l-self.audio_r   #Differenzbildung für Differenzdarstellung
            
            self.lock.release()             #wird bei Ereignis "Start" fortgesetzt

    def continuousStart(self):                              #SPDIF
        """
        Vorbereitung Thread zur Darstellung des Audiosignals.
        """
        self.tR = threading.Thread(target=self.record)
        self.lock = threading.Lock()
        self.tR.daemon = True
        
        #Klick-Ereignisse:           
        QtGui.QDialog.connect(uiplot.pushButton_Start, QtCore.SIGNAL("clicked()"), self.countClick)    # Echtzeitdarstellung des Audiosignals wird gestartet           
        QtGui.QDialog.connect(uiplot.pushButton_Stop, QtCore.SIGNAL("clicked()"), self.suspend)        # Echtzeitdarstellung des Audiosignals wird angehalten 
        uiplot.comboBox_channels.currentIndexChanged.connect(self.setup)
        QtGui.QDialog.connect(uiplot.radioButton_Einkanal, QtCore.SIGNAL("clicked()"), lambda: self.auswahlAnzeige(1))
        QtGui.QDialog.connect(uiplot.radioButton_Diff, QtCore.SIGNAL("clicked()"), lambda: self.auswahlAnzeige(2))
        QtGui.QDialog.connect(uiplot.radioButton_XY, QtCore.SIGNAL("clicked()"), lambda: self.auswahlAnzeige(3))
    
    def suspend(self):                                      #SPDIF
        '''
        Thread wird angehalten, Auswahl der Audioquelle aktiviert, Stop_Button deaktiviert
        '''        
        self.lock.acquire()
        
        uiplot.comboBox_Audio_In.setEnabled(True)
        uiplot.comboBox_channels.setEnabled(True)
        uiplot.pushButton_Stop.setDisabled(True)
        uiplot.pushButton_Start.setEnabled(True)
        
        print(self.audio_l)
        print(self.audio_r)
        print(self.audio_diff)
    
    def countClick(self):                                   #SPDIF
        """
        Startet den Thread "record" beim ersten Klick auf "Start"
        Setzt den Thread "record" fort bei jedem weiteren Klick auf "Start"
        """        
        if False == self.bThreadStarted:
            self.bThreadStarted = True
            
            self.setup()
            if self.stream_open:            
                self.tR.start()
                uiplot.statusBar().clear()
            else:
                print("Audio Thread konnte nicht gestartet werden")
                uiplot.statusMessage("Audio Thread konnte nicht gestartet werden.")
            
            uiplot.comboBox_Audio_In.setDisabled(True)
            uiplot.comboBox_channels.setDisabled(True)
            uiplot.pushButton_Start.setDisabled(True)
            uiplot.pushButton_Stop.setEnabled(True)
            #print(uiplot.comboBox_Audio_In.currentIndex())
        else:
            self.setup()
            self.lock.release()
            
            uiplot.comboBox_Audio_In.setDisabled(True)
            uiplot.comboBox_channels.setDisabled(True)
            uiplot.pushButton_Start.setDisabled(True)
            uiplot.pushButton_Stop.setEnabled(True)
            #print(uiplot.comboBox_Audio_In.currentIndex())
        
    def setupAudio(self):                               #SPDIF
        """
        Erstellt eine Auswahl verfuegbarer Audioquellen und stellt diese in einer Combo Box zur Auswahl bereit
        """
        deviceList = []
        uiplot.comboBox_Audio_In.clear()
        self.p = pyaudio.PyAudio() # instantiate PyAudio, start PortAudio system + list devices
        try:        
            defaultInIdx = self.p.get_default_input_device_info()['index']        
            #print("Defaultin", defaultInIdx)        
            for i in range(self.p.get_device_count()):
                 deviceList.append(self.p.get_device_info_by_index(i))    
                 #print (deviceList[i])
                 if deviceList[i]['maxInputChannels'] > 0:
                     if i == defaultInIdx:
                         uiplot.comboBox_Audio_In.addItem('* '+deviceList[i]['name'], str(i))
                         defaultInBoxIdx = uiplot.comboBox_Audio_In.currentIndex()
                     else:
                         uiplot.comboBox_Audio_In.addItem(deviceList[i]['name'], str(i))   
            uiplot.comboBox_Audio_In.setCurrentIndex(defaultInBoxIdx) 
        except:
            print("Kein Audio Eingang verfuegbar")
        
    def auswahlAnzeige(self, anz):
        """
        gibt an, welche Darstellungsform gewaehlt wurde (Einkanal, Differenz, XY)
        """
        
        if 1 == anz:
            self.anzeige = 1    #Einkanal
        elif 2 == anz:
            self.anzeige = 2    #Differenz
        elif 3 == anz:
            self.anzeige = 3    #XY
        else:
            print("Anzeige fehlerhaft")               
        
    def plotSignal(self):        
        """
        Gibt das vom Audio Device eingelsesene Signal als Plot in die GUI aus
        """
        if False == s.newAudio: 
            return        
        
        if 1 == self.anzeige:
            uiplot.qwtPlot_Zeitsignal.setAxisScale(uiplot.qwtPlot_Zeitsignal.xBottom,0,0.025)
            c.setData(self.xs,self.audio_l)            
        elif 2 == self.anzeige:
            uiplot.qwtPlot_Zeitsignal.setAxisScale(uiplot.qwtPlot_Zeitsignal.xBottom,0,0.025)
            c.setData(self.xs,self.audio_diff)
        elif 3 == self.anzeige:
            uiplot.qwtPlot_Zeitsignal.setAxisScale(uiplot.qwtPlot_Zeitsignal.xBottom,uiplot.spinBox_Ymin.value(),uiplot.spinBox_Ymax.value())
            c.setData(self.audio_l,self.audio_r)
        else:
            print("Anzeigeauswahl fehlerhaft")
        uiplot.qwtPlot_Zeitsignal.setAxisScale(uiplot.qwtPlot_Zeitsignal.yLeft,uiplot.spinBox_Ymin.value(),uiplot.spinBox_Ymax.value())
        uiplot.qwtPlot_Zeitsignal.replot()
        s.newAudio = False
        
class I2C:

    def __init__(self):                                 #I2C
        """
        Initialisierung I2C für ELV USB/I2C Interface
        Scan nach erreichbaren COM Schnittstellen
        """
        #Initialisierungen der Variablen:
        self.BAUDRATE_I2C = 115200                          # Einstellungen des ELV USB/I2C Interface gemaess Bedienungsanleitung
        self.BAUDRATE_UART = 9600        
        self.OpenPort = False                               #gibt an, ob COM Port geoeffnet ist
        self.threadDieNow=False                             #beendet Thread, wenn True
        self.Kanal = 1                                      #Kanalauswahl
        self.Uebertragung = 1                               #Art der Uebertragung (I2C, UART)
        
        #Initialisierungen der QWT Objekte:
        uiplot.comboBox_Abfragerate.addItem(str(0.01))
        uiplot.comboBox_Abfragerate.addItem(str(0.1))
        uiplot.comboBox_Abfragerate.addItem(str(1.0))
        uiplot.comboBox_Abfragerate.addItem(str(2.0))
        uiplot.comboBox_Abfragerate.setCurrentIndex(3)
        uiplot.radioButton_I2C.setChecked(1)        
        uiplot.radioButton_Kanal1.setChecked(1)
        uiplot.radioButton_MUX1.setChecked(1)        
        
        #Suche nach COM Ports:
        ValidComPorts = self.serialScan()
        if 0 == (len(ValidComPorts)):
            uiplot.comboBox_COM.addItem("KEIN COM!")
        else:
            for i in range(len(ValidComPorts)):
                uiplot.comboBox_COM.addItem(ValidComPorts[i][1])
        com = uiplot.comboBox_COM.currentText()
        if ("KEIN COM!" == com):
            #print("Kein COM")
            uiplot.statusMessage("Bitte COM Device anschliessen")
        else:
            self.serialPort()
            uiplot.statusBar().clear()
            
        if self.OpenPort:
            self.ser.write('Y41')               # Konfigurationsbefehle für ELV gemaess Bedienungsanleitung S. 11 Tab. 3
            self.ser.write('Y01')
            
        self.continuousStart()
    
    def serialScan(self):                               
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
            print("Es wurde kein COM-Port gefunden.")
            uiplot.statusMessage("Es wurde kein COM-Port gefunden.")
        else:
            uiplot.statusBar().clear()
        return ports
        
    def serialPort(self):                               
        """
        serialPort oeffnet den ausgewaehlten COM-Port, hauptsaechlich aus "Verstaerker_v3_3_3.py" uebernommen
        """        
        com = uiplot.comboBox_COM.currentText()
        if False == self.OpenPort:    
            try:
                if 1 == self.Uebertragung:                                  #I2C
#                    test = self.ser.isOpen()
#                    print(test)                    
                    self.ser = serial.Serial(
                                             port=str(com),
                                             baudrate=self.BAUDRATE_I2C,
                                             parity=serial.PARITY_EVEN,
                                             stopbits=serial.STOPBITS_ONE,
                                             bytesize=serial.EIGHTBITS
                                             )
                    self.OpenPort = True
#                    test = self.ser.isOpen()
#                    print(test)
#                    if False == test:
#                        try:
#                            self.ser.open()
#                            self.OpenPort = True
#                        except serial.SerialException:
#                            self.OpenPort = False
                    #print(self.OpenPort)
                elif 2 == self.Uebertragung:                                #UART
                    self.ser = serial.Serial(
                                             port=str(com),
                                             baudrate=self.BAUDRATE_UART,
                                             parity=serial.PARITY_NONE,
                                             stopbits=serial.STOPBITS_ONE,
                                             bytesize=serial.EIGHTBITS
                                             )
                    self.OpenPort = True
#                    if False == self.ser.isOpen():
#                        try:
#                            self.ser.open()
#                            self.OpenPort = True
#                        except serial.SerialException:
#                            self.OpenPort = False
                uiplot.statusBar().clear()                             
            except serial.SerialException:
                print("COM kann nicht geoeffnet werden")
                uiplot.statusMessage("COM kann nicht geoeffnet werden.")
            
        else:
            uiplot.statusMessage("COM ist bereits offen")
        
    def requireData(self):                              #I2C
        """
        Thread zur zyklischen Abfrage von Daten auf I2C
        """
        while True:
            self.cond.acquire()
            self.cond.wait(float(uiplot.comboBox_Abfragerate.currentText()))
            if self.threadDieNow: break
            if self.OpenPort:
                
                #print("Anforderung Daten")
                #print(uiplot.horizontalSlider_Volume.value())
                self.readI2C()
            else:
                #print("COM Port nicht geoeffnet")
                self.testGUI()
            self.cond.release()
 

    def continuousStart(self):                          #I2C
        """
        Vorbereitung Thread zur zyklischen Abfrage
        Verarbeitung von Klick-Ereignissen zur MUX und Volume Einstellung
        """
        self.tI2C = threading.Thread(target=self.requireData)
        self.cond = threading.Condition()
        self.tI2C.daemon = True        
        
        self.sendMUX1()
        self.sendVolume()        
        self.tI2C.start()
        
        QtGui.QDialog.connect(uiplot.pushButton_Open, QtCore.SIGNAL("clicked()"), self.serialPort)
        #QtGui.QDialog.connect(uiplot.radioButton_I2C, QtCore.SIGNAL("clicked()"), lambda: self.auswahl_uebertragung(1))
        #QtGui.QDialog.connect(uiplot.radioButton_seriell, QtCore.SIGNAL("clicked()"), lambda: self.auswahl_uebertragung(2))
        QtGui.QDialog.connect(uiplot.radioButton_MUX1, QtCore.SIGNAL("clicked()"), self.sendMUX1)
        QtGui.QDialog.connect(uiplot.radioButton_MUX2, QtCore.SIGNAL("clicked()"), self.sendMUX2)
        QtGui.QDialog.connect(uiplot.radioButton_Kanal1, QtCore.SIGNAL("clicked()"), lambda: self.auswahl_kanal(1))
        QtGui.QDialog.connect(uiplot.radioButton_Kanal2, QtCore.SIGNAL("clicked()"), lambda: self.auswahl_kanal(2))
        QtGui.QDialog.connect(uiplot.horizontalSlider_Volume, QtCore.SIGNAL('valueChanged(int)'), self.sendVolume)
        QtGui.QDialog.connect(uiplot.comboBox_COM, QtCore.SIGNAL('currentIndexChanged(int)'), self.serClose)
        
    def serClose(self):
        
        self.OpenPort=False
        self.ser.close()
        
    def sendMUX1(self):                                 #I2C
        """
        schreibt die Auswahl MUX1 auf I2C
        """
        if self.OpenPort:
            if 1 == self.Kanal:
                self.ser.write(I2C_Daten.MUX1_1)                  #TODO auslagern
            elif 2 == self.Kanal:
                self.ser.write(I2C_Daten.MUX1_2)
            else:
                print("Kanal fehlerhaft")
#        else:
#            print("COM nicht offen")
            #print(self.Kanal)
 
    def sendMUX2(self):                                 #I2C
        """
        schreibt die Auswahl MUX2 auf I2C
        """
        if self.OpenPort:       
            if 1 == self.Kanal:
                self.ser.write(I2C_Daten.MUX2_1)                  #TODO auslagern
            elif 2 == self.Kanal:
                self.ser.write(I2C_Daten.MUX2_2)
            else:
                print("Kanal fehlerhaft") 
#        else:
#            print("COM nicht offen")
            
    def auswahl_kanal(self, kanal):
        """
        Gibt an, welcher Kanal ausgewählt wurde
        """
        if 1 == kanal:
            self.Kanal = 1
        elif 2 == kanal:
            self.Kanal = 2
        else:
            print("Kanal fehlerhaft")
        
    def auswahl_uebertragung(self, uebertragung):
        """
        Gibt an, welche Uebertragungsart gewaehlt wurde
        """                
        if self.OpenPort:
            self.ser.close()
            self.OpenPort=False
            
        if 1 == uebertragung:
            self.Uebertragung = 1
        elif 2 == uebertragung:
            self.Uebertragung = 2
        else:
            print("Uebertragung fehlerhaft")
        
    def sendVolume(self):                               #I2C
        """
        schreibt die Auswahl der Volume auf I2C
        """
        if self.OpenPort:
            try:
                ivalue = uiplot.horizontalSlider_Volume.value()*5
                str_value_hex = str(hex(ivalue))
                if ivalue > 250:
                    print("Ungueltiger Wert")
                elif ivalue >= 20:                 
                    data = str_value_hex[2] + str_value_hex[3]
                else:
                    data = '0' + str_value_hex[2]                            
                if 1 == self.Kanal:
                    self.ser.write(I2C_Daten.VOLUME_1 + data + 'p')                
                elif 2 == self.Kanal:
                    self.ser.write(I2C_Daten.VOLUME_2 + data + 'p')
                else:
                    print("Kanal fehlerhaft")
                uiplot.statusBar().clear()
            except:
                print("Schreiben auf COM fehlgeschlagen")
                uiplot.statusMessage("Schreiben auf COM fehlgeschlagen.")
#        else:
#            print("COM nicht offen")
            
    def readI2C(self):                                  #I2C
        """
        liest die empfangenen Daten von I2C
        """
        i = 0;
        myList=[0 for j in range(9)]
        if 1 == self.Kanal:
            self.ser.write(I2C_Daten.CYCLIC_1)  
        elif 2 == self.Kanal:
            self.ser.write(I2C_Daten.CYCLIC_2)
        while self.ser.inWaiting() > 0:
            try:            
                myList[i] = self.ser.read(2)           
                i = i + 1                                 
            except:
                print("Index out of range")
        
        self.writeGUI(myList)                                      
   
    def writeGUI(self, data):                           #I2C
        '''
        I2C-Bus lesen, Werte in die GUI ausgeben     
        '''
        strStatus = str(data[0])
        
        try:
            iStatus = int(strStatus, 16)
        except:
            iStatus = "Err"
        uiplot.lcdNumber_Status.display(iStatus)
        
        strV_in = str(data[1] + data[2])
        #print(strV_in)
        try:
            iV_in = int(strV_in, 16)
        except:
            iV_in = "Err"
        #print(iV_in)
        uiplot.lcdNumber_Vcc.display(iV_in)
        
        strV_out = str(data[3] + data[4])
        try:        
            iV_out = int(strV_out, 16)
        except:
            iV_out = "Err"
        uiplot.lcdNumber_2_Ausgang.display(iV_out)
        
        strI = str(data[5])
        try:        
            iI = int(strI, 16)
        except:
            iI = "Err"
        uiplot.lcdNumber_Strom.display(iI)
        
        strTemp = str(data[6])
        try:
            iTemp = int(strTemp, 16)
        except:
            iTemp = "Err"
        uiplot.lcdNumber_Temperatur.display(iTemp)
        
        strVol = str(data[7])
        try:
            iVol = int(strVol, 16)
        except:
            iVol = "Err"
        uiplot.lcdNumber_Volume.display(iVol)
        
        strMUX = str(data[8])
        try:
            iMUX = int(strMUX, 16)
        except:
            iMUX = "Err"
        uiplot.lcdNumber_MUX.display(iMUX)
        
    
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

class MyMainWindow(QtGui.QMainWindow):
    
    def __init__(self, parent=None):
        
        QtGui.QMainWindow.__init__(self, parent)        
        self.ui = uic.loadUi('GUI_AudioControl.ui',self)
        
        self.qwtPlot_Zeitsignal.setAxisScale(self.qwtPlot_Zeitsignal.yLeft,-10000,10000)
        self.Thermo_Ausgang.setRange(-100.0,100.0)
        self.Thermo_Vcc.setRange(-100.0,100.0)        
        self.pushButton_Stop.setDisabled(True)
        self.timer = QtCore.QTimer()
        self.timer.start(1.0)
        
    def statusMessage(self, message):    
        """
        Display a message in the statusbar.
        """        
        self.statusBar().showMessage(message)
      
#==============================================================================
# MAIN
#==============================================================================
if __name__ == '__main__':
    
    app = QtGui.QApplication(sys.argv)    
    
    uiplot = MyMainWindow()
    s = SPDIF()             # Instanz von SPDIF wird erzeugt
    bus = I2C()             # Instanz von I2C wird erzeugt    
    c = Qwt.QwtPlotCurve()
    
    c.attach(uiplot.qwtPlot_Zeitsignal)
    
    code = app.exec_()          # Beenden des Programms
    s.close()
    bus.close()
    sys.exit(code)