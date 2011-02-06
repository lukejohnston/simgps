import math
import binascii
import datetime
import serial

class Coord:
	
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
	def toNMEA(self):
		timestr = datetime.datetime.now().strftime("%H%M%S.00")
		lat = abs(self.latdeg)
		lon = abs(self.londeg)

		if(self.latdeg > 0):
			latchar = "N"
		else :
			latchar = "S"
		if(self.londeg > 0):
			lonchar = "E"
		else:
			lonchar = "W"

		data = "GPGGA,%s,%f,%s,%f,%s,2,08,6.8,10.0,M,1.0,M,,0000" % (timestr, lat, latchar, lon, lonchar)
		checksum = binascii.crc32(data) & 0xFF

		return "$%s*%x\r\n" % (data, checksum)

class SegmentIter:

	def __init__(self, start, end, speed):
		self.start = start
		self.end = end
		self.speed = speed

		self.d = haversine(start, end)
		print start
		print end
		self.distance = self.d * 6371000
		self.count = 0
		
		time = self.distance / (self.speed / 3.6)
		self.step = 1 / time	
	
	def next(self):
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
		

def haversine(start, end):

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

if __name__ == "__main__":
	import time
	import sys

	print "simgps: A simple GPS simulator"
	print
	start = Coord(input("Start latitude: "), input("Start longitude: "))
	end = Coord(input("End latitude: "), input("End longitude: "))
	
	segment = SegmentIter(start, end, input("Speed (km/h): "))

	ser = serial.Serial(raw_input("Serial port: "), 4800, timeout=0)

	for i in segment:
		print i.toNMEA()
		print i
		ser.write(i.toNMEA())
		time.sleep(1)
	
	print "Done!"

	sys.exit(0)
