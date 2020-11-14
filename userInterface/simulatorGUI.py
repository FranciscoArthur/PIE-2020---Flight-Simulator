import os
import sys

from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtWidgets import QMainWindow, QApplication, QFileDialog

from FG_commands import FG
from userInterface.GUI import *


class SimulatorGUI(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        super().setupUi(self)

        scriptDir = os.path.dirname(os.path.realpath(__file__))
        self.setWindowIcon(QIcon(scriptDir + os.path.sep + 'logoTest5.jpg'))

        # self.btnConfig.clicked.connect(self.getFGdir)
        self.btnStartFG.clicked.connect(self.startFG)

        self.airplaneImage = None
        self.aircraftFile = 'c172p'
        self.aircraft = 'Cessna 172p'
        self.aircraftList = None
        self.airport = None  # TODO atualize with airport complete names
        self.airportFile = 'PNHL'
        self.logText = 'Software name V0.1\n' \
                       'Created by __ \n' \
                       'Last update: __\n' \
                       '!!!Always remeber to close any FlightGear instances before starting the simulation!!!\n'
        self.FGdir = 'C:\Program Files\FlightGear 2018.3.4'  # None
        self.FGstarted = False
        self.FGconnected = False

        self.setListAircraft()
        self.setListAirport()
        self.selectAircraft(self.aircraft)
        self.selectAirport(self.airportFile)
        self.listAirport.currentTextChanged.connect(self.selectAirport)
        self.listAircraft.currentTextChanged.connect(self.selectAircraft)

    def getFGdir(self):
        directory, _ = QFileDialog.getOpenFileName(self.centralwidget, 'Select FlightGear directory')

    def startFG(self):  # TODO add chose the dir and print log
        fgSimulator = FG(self.FGdir)
        fgSimulator.start(self.aircraftFile, self.airportFile)
        self.FGstarted = True

        self.logAdd('|--------------------------------------------|\n')
        self.logAdd('FlightGear started. Wait until it is fully started to establish a connection.\n')

    def connectFG(self):  # TODO must change. when the server connect, it blocks the code
        fgSimulator = FG(self.FGdir)
        fgSimulator.connect()
        self.FGconnected = True

        self.logAdd('|--------------------------------------------|\n')
        self.logAdd('Connected to FlightGear. You can now start your simulation.\n')

    def setListAirport(self):
        self.listAirport.addItem('PNHL')
        self.listAirport.addItem('PNHL')

    def setListAircraft(self):
        self.listAircraft.addItem('Cessna 172p')
        self.listAircraft.addItem('Mirage F1')

        self.aircraftList = {'Cessna 172p': 'c172p', 'Mirage F1': 'Mirage_F1'}

    def selectAirport(self, airport):
        self.airportFile = airport
        self.logAdd('Airport set to ' + self.airportFile)

    def selectAircraft(self, aircraft):
        self.aircraft = aircraft
        self.aircraftFile = self.aircraftList.get(self.aircraft)

        self.logAdd('Aircraft set to ' + self.aircraft)

        self.airplaneImage = QPixmap(self.FGdir + '\\data\\Aircraft\\' + self.aircraftFile + '\\thumbnail.jpg')
        self.airplaneImage = self.airplaneImage.scaledToWidth(210)
        self.label.setPixmap(self.airplaneImage)

    def log(self):
        self.textBoxLog.setText(self.logText)
        self.textBoxLog.verticalScrollBar().setValue(self.textBoxLog.verticalScrollBar().maximum())
        self.textBoxLog.show()

    def logAdd(self, text):
        self.logText = self.logText + text + '\n'
        self.log()

    def logClear(self):
        self.logText = ''
        self.log()


if __name__ == '__main__':
    qt = QApplication(sys.argv)
    qt.setStyle('Fusion')
    simGUI = SimulatorGUI()
    simGUI.show()
    qt.exec_()
