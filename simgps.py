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



import math
import binascii
import datetime
import serial
import sys
from Tkinter import *

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
                self.count = 0
                
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
                
                self.count += self.step

                if self.count > 1:
                        raise StopIteration
                
                return Coord(math.degrees(lat), math.degrees(lon))
        
        def __iter__(self):
                return self
class SimGPSApp:

        def __init__(self, master = None):
                master.title("SimGPS")

                frame = Frame(master)

                self.master = master

                self.currentLocationLabel = Label(frame)
                self.currentLocationLabel.pack(side = TOP)

                self.pauseButton = Button(frame, text = "Pause",\
                                          command = self.pause)
                self.pauseButton.pack(side = TOP)

                self.paused = False
                print "simgps: A simple GPS simulator"
                print
                
                filename = raw_input("File: ")
                if filename.endswith(".kml"):
                        self.path = process_kml(filename)
                else :
                        self.path = process_file(filename)
                        
                self.ser = serial.Serial(raw_input("Serial port: "), 57600, timeout=0)
                self.speed = input("Speed (km/h): ")
                self.heading = 0



                self.count = 1
                self.segment = SegmentIter(self.path[self.count-1],\
                                           self.path[self.count], self.speed)
                self.master.after(1000, self.nextSentence)

                frame.pack()

        def nextSentence(self):
                try:
                        coord = self.segment.next()
                        self.currentLocationLabel.configure(text = str(coord))
                        sentence = self.toNMEA(coord)
                        #print sentence
                        self.ser.write(sentence)
                        print self.ser.read(1000)
                except StopIteration:
                        self.count += 1
                        if self.count >= len(self.path):
                                self.count = 1
                        self.segment = SegmentIter(self.path[self.count-1],\
                                           self.path[self.count], self.speed)
                finally:
                        self.master.after(1000, self.nextSentence)
                        

        def pause(self):
                self.paused = not self.paused
                if self.paused:
                        self.pauseButton.configure(text = "Resume")
                else :
                        self.pauseButton.configure(text = "Pause")

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
                      (timestr, latminutes, latchar, lonminutes, lonchar, self.speed/1.85,\
                       self.heading, datestr)
                if self.paused:
                        gsa = "GPGSA,A,1,1,2,3,4,5,6,7,8,9,10,11,12,1.0,1.0,1.0"
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

def process_file(filename):
        """
        Process a file containing points on lines. One point per line with a
        comma separator.
        """
        f = open(filename)
        data = f.readlines()
        coords = []
        for i in data:
                numbers = i.split(',')
                if len(numbers) != 2:
                        print "Bad line in file"
                        f.close()
                        sys.exit(1)
                coords.append(Coord(float(numbers[0]), float(numbers[1])))
        f.close()
        return coords

def process_kml(filename):
        """
        Process a KML file that contains a path.
        """
        
        f= open(filename)
        data = f.read()
        coordTag = data.split("<coordinates>")[1]
        coordTag = coordTag.split("</coordinates>")[0]
        coordTag = coordTag.strip()
        coordLines = coordTag.split(" ")
        coords = []
        for i in coordLines:
                numbers = i.split(',')
                coords.append(Coord(float(numbers[1]), float(numbers[0])))
        f.close()
        return coords
                
if __name__ == "__main__":
        root = Tk()
        SimGPSApp(root)
        root.mainloop()
