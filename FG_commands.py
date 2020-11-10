from telnetlib import Telnet
import time
import re
import subprocess


class FG:
    def __init__(self, FG_path, FG_port=5555):
        self.path = FG_path
        self.port = FG_port
        self.tnGET = None
        self.tnSET = None

    def connect(self):
        # Two different telnet object must be
        # created. Otherwise it won't work.
        self.tnGET = Telnet("localhost", 5555)
        self.tnSET = Telnet("localhost", 5555)

    def start(self, aircraft='c172p', airport='PNHL', startAlt=2000):
        subprocess.Popen([self.path + '\\bin\\fgfs.exe',
                          '--fg-root=' + self.path + '\\data',
                          '--fg-scenery=' + self.path + '\\data\\Scenery',
                          '--aircraft=' + aircraft,
                          '--fdm=null',
                          '--enable-auto-coordination',
                          '--native-fdm=socket,in,30,localhost,5502,udp',
                          '--fog-disable',
                          '--enable-clouds3d',
                          '--start-date-lat=2004:06:01:09:00:00',
                          '--enable-sound',
                          '--visibility=15000',
                          '--timeofday=noon',
                          '--in-air',
                          '--prop:/engines/engine0/running=true',
                          '--disable-freeze',
                          '--airport=' + airport,
                          '--altitude=' + str(startAlt),
                          '--heading=0',
                          '--offset-distance=0',
                          '--offset-azimuth=0',
                          '--enable-rembrandt',
                          '--telnet=socket,out,60,localhost,5555,udp'
                          ])

    # Manually get and set double or boolean
    # value by using regular expression
    def getManual(self, command, varType="double"):
        commandStr = "get " + command + "\r\n"
        self.tnGET.write(commandStr.encode())
        varTypeStr = "(" + varType + ")"
        data = self.tnGET.read_until(varTypeStr.encode())
        decodedData = data.decode('utf-8')

        lastSlashIndex = command.rindex("/")
        element = command[lastSlashIndex + 1:]

        try:
            if decodedData.count(element):
                if varType == "double":
                    try:
                        extractedData = float((re.findall("-?\d+\.\d+", decodedData))[0])
                        return extractedData
                    except:
                        return 0
                elif varType == "bool":
                    extractedData = (re.findall("(?:^|\s)'([^']*?)'(?:$|\s)", decodedData))[0]
                    return extractedData
        except:
            print("Error Occured")
            return -9999

    def setManual(self, command, value):
        commandStr = "set " + command + " " + str(value) + "\r\n"
        self.tnSET.write(bytes(commandStr, 'utf-8'))

    def getAltitude(self):
        return FG.getManual(self, "position/altitude-ft")

    def getLatitude(self):
        return FG.getManual(self, "position/latitude-deg")

    def getLongitude(self):
        return FG.getManual(self, "position/longitude-deg")

    def getTrack(self):
        return FG.getManual(self, "orientation/track-deg")

    def setAltitude(self, value):
        FG.setManual(self, "position/altitude-ft", value)

    def setLatitude(self, value):
        FG.setManual(self, "position/latitude-deg", value)

    def setLongitude(self, value):
        FG.setManual(self, "position/longitude-deg", value)

    def setPitch(self, value):
        FG.setManual(self, "orientation/pitch-deg", value)

    def setYaw(self, value):  # TODO: check
        FG.setManual(self, "orientation/yaw-deg", value)

    def setRoll(self, value):
        FG.setManual(self, "orientation/roll-deg", value)

    def setHeading(self, value):
        FG.setManual(self, "orientation/heading-deg", value)

    def setAileron(self, value):
        FG.setManual(self, "controls[0]/flight/aileron", value)

    def setElevator(self, value):
        FG.setManual(self, "controls[0]/flight/elevator", value)

    def setRudder(self, value):
        FG.setManual(self, "controls[0]/flight/rudder", value)

    def setFlaps(self, value):
        FG.setManual(self, "controls[0]/flight/flaps", value)

    def setWingSweep(self, value):
        FG.setManual(self, "controls[0]/flight/wing-sweep", value)
