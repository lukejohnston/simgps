#!/usr/bin/python
"""
        simgps: A simple GPS simulator

        simgps outputs NMEA sentences to a serial port that simulate the
        movement of a GPS from one point to another.

        Copyright 2011 Luke Johnston

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see <http://www.gnu.org/licenses/>.


"""

import sys
import math
import datetime
import serial
import sys
from Tkinter import *
import platform
import glob
import tkFileDialog

class Coord:
        """
        Class that represents a LatLon coordinate. Arguments to the constructor
        should be in decimal degrees.
        """
        
        def __init__(self, lat, lon):
                self.latdeg = lat
                self.londeg = lon

        def __repr__(self):
                return "%f, %f" % (self.latdeg, self.londeg)
        
        def __str__(self):
                return repr(self)

        def __getattr__(self, name):
                if name == "lon":
                        return math.radians(self.londeg)
                elif name == "lat":
                        return math.radians(self.latdeg)
                else:
                        raise AttributeError

class SegmentIter:

        def __init__(self, start, end, speed):
                self.start = start
                self.end = end
                self.speed = speed

                self.d = haversine(start, end)

                self.distance = self.d * 6371000
                if tachoDirection >= 0:
                    self.count = 0
                else :
                    self.count = 1
                
                time = self.distance / (self.speed / 3.6)
                self.step = 1 / time    
        

        def next(self):
                """
                This is an implementation of a formula that calculates
                an intermediate lat and lon between two given points a 
                certain fraction of the distance between the point.

                It was inspired by the description given on this page:

                http://williams.best.vwh.net/avform.htm#Intermediate

                """
                A = math.sin((1 - self.count)*self.d) / math.sin(self.d)
                B = math.sin(self.count * self.d) / math.sin(self.d)
                x = A * math.cos(self.start.lat) * math.cos(self.start.lon) +\
                        B*math.cos(self.end.lat) * math.cos(self.end.lon)
                y = A* math.cos(self.start.lat) * math.sin(self.start.lon) +\
                        B * math.cos(self.end.lat) * math.sin(self.end.lon)
                z = A * math.sin(self.start.lat) + B * math.sin(self.end.lat)
                lat = math.atan2(z, math.sqrt(x**2 + y**2))
                lon = math.atan2(y, x)
                
                self.count += self.step * tachoDirection

                if self.count < 0 or self.count > 1:
                        raise StopIteration
                
                return Coord(math.degrees(lat), math.degrees(lon))
        
        def __iter__(self):
                return self
class SimGPSApp:

        def __init__(self, master = None, pathfilename = ""):
                master.title("SimGPS")

                frame = Frame(master)

                self.master = master

                topFrame = Frame(frame, bd=2, relief=RIDGE)
                gpsSerialFrame = Frame(frame, bd=2, relief=RIDGE)
                tachSerialFrame = Frame(frame, bd=2, relief=RIDGE)
                botFrame = Frame(frame, bd=2, relief=RIDGE)
                tachoControlFrame = Frame(frame, bd=2, relief=RIDGE)
                labelFrame = Frame(frame)
                buttonFrame = Frame(frame, bd=1)

                #Contents of topFrame, open file button and label
                self.fileButton = Button(topFrame, text = "Open Path File",\
                                         command = self.openFile)
                self.fileButton.pack(side = LEFT, anchor = W)
                if pathfilename:
                    self.filename = cmdlineFilename
                    self.fileLabel = Label(topFrame, text = self.filename)
                else :
                    self.fileLabel = Label(topFrame, text = "No file loaded")
                self.fileLabel.pack(side = LEFT, anchor = W)

                #Content of midFrame, serial options

                ports = findSerialPorts()
                if not ports:
                        ports = [""]

                self.baudVar = StringVar()
                self.baudVar.set("57600")
                self.baudBox = Entry(gpsSerialFrame, textvariable = self.baudVar)
                self.baudBox.pack(side = RIGHT, padx = 10)
                self.serialVar = StringVar(master)
                self.serialVar.set(ports[0])
                self.serialMenu = apply(OptionMenu, (gpsSerialFrame, self.serialVar)\
                                        + tuple(ports))
                self.serialMenu.pack(side = RIGHT)
                self.gpsLabel = Label(gpsSerialFrame, text = "GPS output:")
                self.gpsLabel.pack(side = RIGHT, anchor = W)

                #Contents of tachSerialFrame, tacho serial options
                self.tachBaudVar = StringVar()
                self.tachBaudVar.set("57600")
                self.tachBaudBox = Entry(tachSerialFrame, textvariable = self.baudVar)
                self.tachBaudBox.pack(side = RIGHT, padx = 10)
                self.tachSerialVar = StringVar(master)
                self.tachSerialVar.set(ports[0])
                self.tachSerialMenu = apply(OptionMenu, (tachSerialFrame, self.tachSerialVar)\
                                        + tuple(ports))
                self.tachSerialMenu.pack(side = RIGHT)
                self.tachLabel = Label(tachSerialFrame, text = "Tacho control:")
                self.tachLabel.pack(side = RIGHT, anchor = W)

                #Contents of botFrame, speed and fix type
                self.fixVar = StringVar(master)
                self.fixVar.set("3D Fix")
                self.fixVar.trace('w', self.changeFix)
                self.fixType = OptionMenu(botFrame, self.fixVar, "No Fix",\
                                          "2D Fix", "3D Fix")
                self.fixType.pack(side = LEFT)
                self.speedVar = StringVar()
                self.speedVar.set("100")
                self.speedBox = Entry(botFrame, textvariable = self.speedVar, 
                                      justify = RIGHT)
                self.speedBox.pack(side = LEFT)
                Label(botFrame, text = "km/h").pack(side = LEFT)

                #Contents of tachoControlFrame, direction controls and
                #calibration
                self.fButton = Button(tachoControlFrame, text = "F", state=DISABLED,\
                                         command = self.forward)
                self.fButton.pack(side = LEFT, anchor = W, padx = 2)
                self.nButton = Button(tachoControlFrame, text = "N",state=DISABLED, relief=SUNKEN,\
                                         command = self.neutral)
                self.nButton.pack(side = LEFT, anchor = W, padx = 2)
                self.rButton = Button(tachoControlFrame, text = "R", state=DISABLED,\
                                         command = self.backward)
                self.rButton.pack(side = LEFT, anchor = W, padx = 2)

                self.calibVar = StringVar()
                self.calibVar.set("12.00")
                self.calibBox = Entry(tachoControlFrame, textvariable =
                                      self.calibVar, justify = RIGHT)
                self.calibBox.pack(side = LEFT)
                Label(tachoControlFrame, text = "ticks/m").pack(side = LEFT)

                #Frame exclusively for the current location label
                self.currentLocationLabel = Label(labelFrame)
                self.currentLocationLabel.pack()

                #Frame for the go and stop buttons
                self.goButton = Button(buttonFrame, command = self.go,
                                       text = "Go")
                self.goButton.pack(side = LEFT, padx = 10)
                self.pauseButton = Button(buttonFrame, command = self.pause,
                                          state = DISABLED, text = "Pause")
                self.pauseButton.pack(side = LEFT, padx = 10)
                self.restartButton = Button(buttonFrame,
                                            command = self.restart,
                                            text = "Restart")
                self.restartButton.pack(side = LEFT, padx = 10)

                #Pack all the frames
                topFrame.pack(side = TOP, fill=X)
                gpsSerialFrame.pack(side = TOP, fill=X)
                tachSerialFrame.pack(side = TOP, fill=X)
                botFrame.pack(side = TOP, fill=X)
                tachoControlFrame.pack(side = TOP, fill = X)
                labelFrame.pack(side = TOP)
                buttonFrame.pack(side = TOP)
                frame.pack()

                self.filename = ''

                self.path = None

        def changeFix(self, _, __, ___):
                if self.path:
                        self.path.setFix(self.fixVar.get())

        def openFile(self):
                self.filename = tkFileDialog.askopenfilename()
                self.fileLabel.config(text = self.filename)

        def forward(self):
                global tachoDirection
                tachoDirection = 1
                self.fButton.config(state = ACTIVE, relief=SUNKEN)
                self.nButton.config(state = NORMAL, relief=RAISED)
                self.rButton.config(state = NORMAL, relief=RAISED)
                if self.path:
                    self.path.forward(float(self.calibVar.get()))
        def neutral(self):
                self.nButton.config(state = ACTIVE, relief=SUNKEN)
                self.fButton.config(state = NORMAL, relief=RAISED)
                self.rButton.config(state = NORMAL, relief=RAISED)
                if self.path:
                    self.path.neutral(float(self.calibVar.get()))
        def backward(self):
                global tachoDirection
                tachoDirection = -1
                self.rButton.config(state = ACTIVE, relief=SUNKEN)
                self.nButton.config(state = NORMAL, relief=RAISED)
                self.fButton.config(state = NORMAL, relief=RAISED)
                if self.path:
                    self.path.backward(float(self.calibVar.get()))
        
        def go(self):
                if self.path:
                        self.nextSentence()
                        self.pauseButton.config(state = NORMAL)
                        self.goButton.config(state = DISABLED)
                elif self.filename:
                        self.path = PathSim(self.filename, self.serialVar.get(),
                                            int(self.baudVar.get()),
                                            self.tachSerialVar.get(),
                                            int(self.tachBaudVar.get()),
                                            int(self.speedVar.get()),
                                            self.fixVar.get())
                        self.speedBox.config(state = DISABLED)
                        self.baudBox.config(state = DISABLED)
                        self.tachBaudBox.config(state = DISABLED)
                        self.fileButton.config(state = DISABLED)
                        self.goButton.config(state = DISABLED)
                        self.serialMenu.config(state = DISABLED)
                        self.tachSerialMenu.config(state = DISABLED)
                        self.pauseButton.config(state = NORMAL)
                        self.fButton.config(state = NORMAL, relief=RAISED)
                        self.nButton.config(state = NORMAL, relief=SUNKEN)
                        self.rButton.config(state = NORMAL, relief=RAISED)
                        self.nextSentence()
                        
        def nextSentence(self):
                self.path.nextSentence()
                self.currentLocationLabel.config(text = str(self.path.current))
                self.afterHandle = self.master.after(1000, self.nextSentence)

        def pause(self):
                self.goButton.config(state = NORMAL)
                self.master.after_cancel(self.afterHandle)
                self.pauseButton.config(state = DISABLED)

        def restart(self):
                if self.path:
                        self.path.restart()
                        self.currentLocationLabel.config(text = "")
                        
class PathSim:

        def __init__(self, filename, serialPort, baud, tachoPort, tachoBaud,
                     speed, fix):
                if filename.endswith(".kml"):
                        self.open_kml(filename)
                else :
                        self.open_file(filename)
                        
                self.ser = serial.Serial(serialPort, baud, timeout=0)
                self.tacho = serial.Serial(tachoPort, tachoBaud, timeout=0)
                self.speed = speed #km/h
                self.heading = 0

                self.count = 1
                self.segment = SegmentIter(self.path[self.count-1],\
                                           self.path[self.count], self.speed)
                self.fix = fix

        def forward(self, cal = 12):
               speedms = self.speed / 3.6
               tps = speedms * cal #ticks per second
               tps = int(round(tps))
               self.tacho.write("%d\rF\r" % (tps))
 
        def neutral(self, cal = 12):
               speedms = self.speed / 3.6
               tps = speedms * cal #ticks per second
               tps = int(round(tps))
               self.tacho.write("%d\rN\r" % (tps))
                
        def backward(self, cal = 12):
               speedms = self.speed / 3.6
               tps = speedms * cal #ticks per second
               tps = int(round(tps))
               self.tacho.write("%d\rB\r" % (tps))
        
        def nextSentence(self):
                try:
                        self.current = self.segment.next()
                        sentence = self.toNMEA(self.current)
                        self.ser.write(sentence)
                        print self.ser.read(1000),
                except StopIteration:
                        self.count += tachoDirection
                        if self.count >= len(self.path):
                                self.count = 1
                        if self.count < 1:
                                self.count = len(self.path)-1
                        self.segment = SegmentIter(self.path[self.count-1],\
                                           self.path[self.count], self.speed)

        def setFix(self, fix):
                self.fix = fix

        def toNMEA(self, coord):
                """
                Returns the NMEA sentence the represents the coordinate
                with the current time in the time field.
                """
                timestr = datetime.datetime.now().strftime("%H%M%S.00")
                datestr = datetime.datetime.now().strftime("%d%m%y")
                
                lat = abs(coord.latdeg)
                latdegrees = int(lat)
                latminutes = (lat - latdegrees) * 60
                latminutes += latdegrees * 100

                lon = abs(coord.londeg)
                londegrees = int(lon)
                lonminutes = (lon - londegrees) * 60
                lonminutes += londegrees * 100

                if(coord.latdeg > 0):
                        latchar = "N"
                else :
                        latchar = "S"
                if(coord.londeg > 0):
                        lonchar = "E"
                else:
                        lonchar = "W"

                gga = "GPGGA,%s,%f,%s,%f,%s,2,08,6.8,10.0,M,1.0,M,,0000" %\
                       (timestr, latminutes, latchar, lonminutes, lonchar)
                rmc = "GPRMC,%s,A,%f,%s,%f,%s,%f,%d,%s,,,," %\
                      (timestr, latminutes, latchar, lonminutes, lonchar, 
                       self.speed/1.85, self.heading, datestr)
                if self.fix == "No Fix":
                        gsa = "GPGSA,A,1,1,2,3,4,5,6,7,8,9,10,11,12,1.0,1.0,1.0"
                elif self.fix == "2D Fix":
                        gsa = "GPGSA,A,2,1,2,3,4,5,6,7,8,9,10,11,12,1.0,1.0,1.0"
                else :
                        gsa = "GPGSA,A,3,1,2,3,4,5,6,7,8,9,10,11,12,1.0,1.0,1.0"
                
                ggachecksum = 0
                for i in gga:
                        ggachecksum ^= ord(i)

                rmcchecksum = 0
                for i in rmc:
                        rmcchecksum ^= ord(i)

                gsachecksum = 0
                for i in gsa:
                        gsachecksum ^= ord(i)

                return "$%s*%x\r\n$%s*%x\r\n$%s*%x\r\n" %\
                       (gga, ggachecksum & 0xff, rmc, rmcchecksum & 0xff, gsa,\
                        gsachecksum & 0xff)

        def open_file(self, filename):
                """
                Process a file containing points on lines. One point per line with a
                comma separator.
                """
                f = open(filename)
                data = f.readlines()
                self.path = []
                for i in data:
                        numbers = i.split(',')
                        if len(numbers) != 2:
                                print "Bad line in file"
                                f.close()
                                sys.exit(1)
                        self.path.append(Coord(float(numbers[0]), float(numbers[1])))
                f.close()

        def open_kml(self, filename):
                """
                Process a KML file that contains a path.
                """
                
                f= open(filename)
                data = f.read()
                coordTag = data.split("<coordinates>")[1]
                coordTag = coordTag.split("</coordinates>")[0]
                coordTag = coordTag.strip()
                coordLines = coordTag.split(" ")
                self.path = []
                for i in coordLines:
                        numbers = i.split(',')
                        self.path.append(Coord(float(numbers[1]), float(numbers[0])))
                f.close()

        def restart(self):
                self.count = 1
                self.segment = SegmentIter(self.path[self.count-1],\
                                           self.path[self.count], self.speed)


def haversine(start, end):
        """
        Implementation of the haversine formula that calculates the 
        angle between two point on a sphere. This returns the value in radians
        which then needs to be multiplied by the radius of the earth to get 
        the actual distance.

        It was inspired by the implementation that should be found here:

        http://gorny.edu.pl/haversine.py
        """

        start_lat = start.lat
        start_lon = start.lon
        end_lat = end.lat
        end_lon = end.lon

        d_lat = end_lat - start_lat
        d_lon = end_lon - start_lon

        a = (math.sin(d_lat/2)**2) + math.cos(start_lat) * math.cos(end_lat)\
                * (math.sin(d_lon/2)**2)
        c = 2 * math.asin(math.sqrt(a))

        return c

def findSerialPorts():
    """ Based mostly on the pySerial example. Attempts to open each serial port
    and returns a list of the name of the successful ones. 
    Globs the filesystem on Mac. """

    if(platform.system() == 'Darwin'):
        return glob.glob('/dev/tty.usb*')
    available = []
    for i in range(256):
        try:
            s = serial.Serial(i)
            available.append(s.portstr)
            s.close()
        except serial.SerialException:
            pass
    return available

if __name__ == "__main__":
        global tachoDirection
        tachoDirection=0
        if len(sys.argv) > 1:
            cmdlineFilename=sys.argv[1]
        else :
            cmdlineFilename=""
        root = Tk()
        SimGPSApp(root, cmdlineFilename)
        root.mainloop()
