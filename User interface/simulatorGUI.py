import os
import sys

from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtWidgets import QMainWindow, QApplication, QFileDialog

from FG_commands import FG
from userInterface.GUI import *


# TODO add more airports, aircrafts, get params, start simulation, send data to FG

class SimulatorGUI(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        super().setupUi(self)

        scriptDir = os.path.dirname(os.path.realpath(__file__))
        self.setWindowIcon(QIcon(scriptDir + os.path.sep + 'logoTest5.jpg'))

        self.btnFGDir.clicked.connect(self.getFGdir)
        self.btnStartFG.clicked.connect(self.startFG)
        self.btnChoseParamDir.clicked.connect(self.getParamsFile)

        self.tabWidget.setCurrentIndex(0)

        self.logText = 'Software name V0.1\n' \
                       'Created by __ \n' \
                       'Last update: __\n' \
                       '!!!Always remember to close any FlightGear instances before starting the simulation!!!\n'

        self.FGdirFile = open('FGdir.txt', 'r')
        self.inputFGDir.setText(self.FGdirFile.read())
        self.FGdir = self.inputFGDir.text()
        self.FGdirFile.close()

        self.airplaneImage = None
        self.aircraftFile = 'c172p'
        self.aircraft = 'Cessna 172p'
        self.aircraftList = None
        self.airport = None  # TODO atualize with airport complete names
        self.airportFile = 'PNHL'
        self.FGstarted = False
        self.FGconnected = False
        self.paramsFile = None

        self.simTime = 50  # seconds
        self.simTimeStep = 1e-6  # seconds

        self.inputSimTime.setText(str(self.simTime))
        self.inputSimTimeStep.setText(str(self.simTimeStep))

        self.inputSimTime.returnPressed.connect(self.simTimeChange)
        self.inputSimTimeStep.returnPressed.connect(self.simTimeStepChange)

        self.setListAircraft()
        self.setListAirport()
        self.selectAircraft(self.aircraft)
        self.selectAirport(self.airportFile)
        self.listAirport.currentTextChanged.connect(self.selectAirport)
        self.listAircraft.currentTextChanged.connect(self.selectAircraft)

    def getFGdir(self):
        dialog = QFileDialog()
        folder_path = dialog.getExistingDirectory(None, "Select FlightGear Folder")

        self.FGdirFile = open('FGdir.txt', 'w')
        self.FGdirFile.write(folder_path)
        self.inputFGDir.setText(folder_path)
        self.FGdir = self.inputFGDir.text()
        self.FGdirFile.close()

        self.logAdd('FlightGear path set to: ' + folder_path + '\n')
        return folder_path

    def getParamsFile(self):
        dialog = QFileDialog()
        folder_path, _ = dialog.getOpenFileName(self.centralwidget, "Select Parameters File")
        self.inputParams.setText(folder_path)
        self.paramsFile = folder_path

        self.logAdd('Parameters file selected: ' + folder_path + '\n')

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

    def simTimeChange(self):
        self.simTime = float(self.inputSimTime.text())

    def simTimeStepChange(self):
        self.simTimeStep = float(self.inputSimTimeStep.text())

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
