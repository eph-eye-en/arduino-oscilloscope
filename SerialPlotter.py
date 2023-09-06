#!/usr/bin/python3

# Modified from https://thepoorengineer.com/en/arduino-python-plot/#python

from threading import Thread, Lock
import serial
import time
from datetime import datetime
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import pandas as pd


class serialPlot:
	def __init__(self, serialPort = '/dev/ttyUSB0', serialBaud = 38400,
	maxSamples = 500, maxTime = 100000, startMargin = 10, highThreshold = (0, 0),
	numReadings = 1, bufferSize = 1):
		self.port = serialPort
		self.baud = serialBaud
		self.maxSamples = maxSamples
		self.startMargin = startMargin
		self.highThreshold = highThreshold
		self.maxTime = maxTime
		self.numReadings = numReadings
		self.toPlot = [True] * self.numReadings
		self.bufferSize = bufferSize
		self.rawData = bytearray(bufferSize)
		self.rawDataLen = 0
		self.timeOffset = 0
		self.data = collections.deque([(0, [0] * self.numReadings)] * maxSamples,
			maxlen=maxSamples)
		self.isRun = True
		self.isReceiving = False
		self.thread = None
		self.rawDataMutex = Lock()
		self.plotTimer = 0
		self.previousTimer = 0

		# I = 4 byte unsigned int; H = 2 byte unsigned short
		self.dataFormat = {
			"timestamp": "H",
			"reading": "H",
		}

		print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
		try:
			self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
			print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
		except IOError as err:
			print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
			print(err)

	def readSerialStart(self):
		if self.thread == None:
			self.thread = Thread(target=self.backgroundThread)
			self.thread.start()
			# Block till we start receiving values
			while self.isReceiving != True:
				time.sleep(0.1)

	def getSerialData(self, frame, ax, lines, lineValueText, lineLabel, timeText):
		currentTimer = time.perf_counter()
		self.plotTimer = int((currentTimer - self.previousTimer) * 1000)     # the first reading will be erroneous
		self.previousTimer = currentTimer
		timeText.set_text('Plot Interval = ' + str(self.plotTimer) + 'ms')
		
		self.readRawData()
		d, minVal, maxVal = self.preprocessData()

		#lines[0].set_data(zip(*d))
		for i, l in enumerate(lines):
			t = [(r[0], r[1][i]) for r in d]
			l.set_data(zip(*t))
		lineValueText.set_text(f"Min = {str(minVal)}; Max = {str(maxVal)}")
		#ax.set_xlim(0, d[-1][0] * 1.2)
	
	def readRawData(self):
		f = self.dataFormat["timestamp"] \
			+ self.dataFormat["reading"] * self.numReadings
		b = struct.calcsize(f)
		self.rawDataMutex.acquire()
		if self.rawDataLen < b:
			self.rawDataMutex.release()
			return

		v = memoryview(self.rawData)
		tSize = struct.calcsize(self.dataFormat["timestamp"])
		maxT = 2 ** (tSize * 8) - 1
		for i in range(0, self.rawDataLen - b, b):
			t, *values = struct.unpack(f, v[i:i + b])
			if t + self.timeOffset < self.data[-1][0]:
				self.timeOffset += maxT
			t += self.timeOffset
			self.data.append((t, values))
		unusedLen = self.rawDataLen % b
		usedLen = self.rawDataLen - unusedLen
		self.rawData[:unusedLen] = self.rawData[usedLen:self.rawDataLen]
		self.rawDataLen = unusedLen
		
		#if self.data[-1][1] > 1500 or self.data[-1][1] < 0:
		#	self.rawDataLen += 1
		self.rawDataMutex.release()

		while self.data[-1][0] - self.data[0][0] > self.maxTime:
			self.data.popleft()

	def preprocessData(self):
		thresIdx, thresVal = self.highThreshold
		minVal = min(self.data, key=lambda x : x[1][thresIdx])[1][thresIdx]
		maxVal = max(self.data, key=lambda x : x[1][thresIdx])[1][thresIdx]
		firstTime = self.data[0][0]
		d = list(self.data)
		if thresVal > 0:
			threshold = lerp(minVal, maxVal, thresVal)
			try:
				startIdx, _ = getFirstWith(d,
					lambda x : x[0] >= firstTime + self.startMargin)
				minIdx, _ = getFirstWith(d, lambda x : x[1][thresIdx] < threshold,
					start=startIdx)
				_, (riseTime, *_) = getFirstWith(d, lambda x : x[1][thresIdx] >= threshold,
					start=minIdx)
				firstTime = riseTime - self.startMargin
			except StopIteration:
				pass
		d = list(map(lambda x : (x[0] - firstTime, x[1]), d))
		return d, minVal, maxVal

	def backgroundThread(self):    # retrieve data
		time.sleep(1.0)  # give some buffer time for retrieving data
		self.serialConnection.reset_input_buffer()
		buf = bytearray(self.bufferSize)
		v = memoryview(buf)
		while (self.isRun):
			w = max(self.serialConnection.in_waiting, 2)
			read = self.serialConnection.readinto(v[:w])

			self.rawDataMutex.acquire()
			self.rawData[self.rawDataLen:self.rawDataLen + read] = buf[:read]
			self.rawDataLen += read
			self.rawDataMutex.release()

			self.isReceiving = True

	def close(self):
		self.isRun = False
		self.thread.join()
		self.serialConnection.close()
		print('Disconnected...')

def getFirstWith(l, p, start=0):
	return next((i + start, x) for i, x in enumerate(l[start:]) if p(x))

def currentMicroseconds():
	now = datetime.now() - datetime(2000, 1, 1)
	return round(now.total_seconds() * 1000000)

def lerp(a, b, x):
	return a + (b - a) * x

def main():
	portName = '/dev/ttyACM0'
	baudRate = 2000000
	#maxSamples = 1000
	#maxTime = 120000
	maxSamples = 20000
	maxTime = 1500000
	startMargin = 10000
	highThreshold = (1, 0.5)
	numReadings = 2
	bufferSize = 500
	s = serialPlot(portName, baudRate, maxSamples, maxTime, startMargin,
		highThreshold, numReadings, bufferSize)
	s.readSerialStart()     # starts background thread

	# plotting starts below
	pltInterval = 50    # Period at which the plot animation updates [ms]
	xmin = 0
	#xmax = 70000
	xmax = 1200000
	ymin = 0
	ymax = 1000
	fig = plt.figure()
	ax = plt.axes(xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
	ax.set_title('Arduino Analog Read')
	ax.set_xlabel("time / us")
	ax.set_ylabel("AnalogRead Value")

	lineLabel = 'Analog Value '
	timeText = ax.text(0.50, 0.95, '', transform=ax.transAxes)
	lines = [ax.plot([], [], label=lineLabel + str(i))[0]
		for i in range(numReadings)]
	lineValueText = ax.text(0.50, 0.90, '', transform=ax.transAxes)
	anim = animation.FuncAnimation(fig, s.getSerialData, fargs=(ax, lines, lineValueText, lineLabel, timeText), interval=pltInterval)    # fargs has to be a tuple

	plt.legend(loc="upper left")
	plt.show()

	s.close()


if __name__ == '__main__':
	main()
