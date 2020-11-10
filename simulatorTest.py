from FG_commands import FG
import time

if __name__ == '__main__':

    fgSimulator = FG('C:\Program Files\FlightGear 2018.3.4')

    start = False

    if start:
        fgSimulator.start()

    fgSimulator.connect()

    i = 1

    while True:
        print("altitude: " + str(fgSimulator.getAltitude()))
        print('track= ' + str(fgSimulator.getTrack()))

        # set altitude
        fgSimulator.setAltitude(2000 + i)
        fgSimulator.setLatitude(37.607 + i / 100)
        fgSimulator.setLongitude(-122.381 - i / 100)

        # set pitch
        fgSimulator.setPitch(0 + i * 20)
        # set heading
        fgSimulator.setHeading(0 + i * 20)
        # set roll
        fgSimulator.setRoll(0 + i * 20)
        # set yaw*
        # setManual(tnSET, "orientation/yaw-deg", 20)

        # set aileron
        fgSimulator.setAileron(0)
        # set elevator*
        fgSimulator.setElevator(0)
        # set rudder
        fgSimulator.setRudder(0)
        # set flaps*
        fgSimulator.setFlaps(0)
        # set wing-sweep*
        fgSimulator.setWingSweep(0)

        # print(getAltitude(tnGET))
        time.sleep(0.0000001)
        i = i + 0.01
