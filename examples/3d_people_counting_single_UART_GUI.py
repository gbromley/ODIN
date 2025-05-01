import serial
import struct
import numpy as np
from time import time, sleep
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

reset = True
savePointCloud = True
saveTracks = True
saveTracksID = True
Port_name = '/dev/serial0'

configFile_name = './chirp_config/chirp_config.cfg'
pointCloud_fileName = './output_files/PointCloud.txt'
targetsObject_fileName = './output_files/Targets.txt'
targetsIndex_FileName = './output_files/TargetsIndex.txt'

syncPattern = 0x708050603040102

try:

	if (reset):
		from gpiozero import OutputDevice
		PinReset = OutputDevice(6)
		PinReset.off()
		sleep(10e-3)
		PinReset.on()
		sleep(1)

	sleep(1)

	pg.setConfigOption('background', 'w')
	MyWindow = pg.GraphicsWindow()
	Plot_2D = MyWindow.addPlot(title="2D Data")
	Plot_2D.setMouseEnabled(x=False, y=False)
	Plot_2D.setMenuEnabled(False)
	Plot_2D.hideButtons()

	boundaryBox = np.zeros(6)
	staticBoundaryBox = np.zeros(6)

	#print('Reading configuration file')
	with open(configFile_name, 'r') as fp:
		cnt = 0
		commands = []
		for line in fp:
			if (len(line) > 1):
				if (line[0] != '%'):
					commands.append(line)
					cnt += 1
					parameters = line.split(' ')
					if (parameters[0] == 'boundaryBox'):
						for i in range(6):
							boundaryBox[i] = eval(parameters[i+1])
					elif (parameters[0] == 'staticBoundaryBox'):
						for i in range(6):
							staticBoundaryBox[i] = eval(parameters[i+1])
	fp.close()

	#print('Opening ports')

	configPort = serial.Serial(Port_name, 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.3)
	
	for index in range(cnt):
		configPort.write(bytearray(commands[index].encode()))
		sleep(20e-3)

	configPort.close()

	dataPort = serial.Serial(Port_name, 921600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.3)
	dataPort.reset_output_buffer()

	packetHeader = bytearray([])

	tlvHeaderLen = 8
	headerLen = 48

	iterations = 0
	iterations_plot = 3

	while (True):

		timePacket = time()

		packetHeader += dataPort.read(headerLen-len(packetHeader))

		sync, version, totalPacketLen, platform, frameNumber, subFrameNumber, chirpProcessingMargin, frameProcessingMargin, trackProcessTime, uartSentTime, numTLVs, checksum =  struct.unpack('Q9I2H', packetHeader[:headerLen])

		if (sync == syncPattern):

			packetHeader = bytearray([])
			numDetectedObj = 0
			numOfTargets = 0
			packetPayload = dataPort.read(totalPacketLen-headerLen)
			iterations += 1

			for i in range(numTLVs):

				tlvType, tlvLength = struct.unpack('2I', packetPayload[:tlvHeaderLen])

				if (tlvType > 20 or tlvLength > 10000):
					packetHeader = bytearray([])
					break

				packetPayload = packetPayload[tlvHeaderLen:]

				if (tlvType == 6):

					pointUnit = struct.unpack('5f', packetPayload[:20])
					packetPayload = packetPayload[20:]
					numDetectedObj = int((tlvLength-tlvHeaderLen-20)/8)
					detectedObjects = np.zeros((numDetectedObj, 5))
					results_string = ''

					print('numDetectedObj: %d' % numDetectedObj)

					for j in range(numDetectedObj):

						elevation_j, azimuth_j, doppler_j, range_j, snr_j = struct.unpack('2bh2H', packetPayload[:8])

						detectedObjects[j, 0] = range_j * pointUnit[3]
						detectedObjects[j, 1] = azimuth_j * pointUnit[1] * 180/np.pi
						detectedObjects[j, 2] = elevation_j * pointUnit[0] * 180/np.pi
						detectedObjects[j, 3] = doppler_j * pointUnit[2]
						detectedObjects[j, 4] = snr_j * pointUnit[4]

						packetPayload = packetPayload[8:]

						results_string += '%1.3f %1.3f %1.3f %1.3f %1.3f ' % (detectedObjects[j, 0], detectedObjects[j, 1], detectedObjects[j, 2], detectedObjects[j, 3], detectedObjects[j, 4])

					if (savePointCloud):
						filePoints = open(pointCloud_fileName, 'a')
						filePoints.write(results_string + '%1.3f\n' % timePacket)
						filePoints.close()

				elif (tlvType == 7):

					numOfTargets = int((tlvLength-tlvHeaderLen)/112)
					DetectedTargets = np.zeros((numOfTargets, 10))
					results_string = ''

					print('numOfTargets: %d' % numOfTargets)

					for j in range(numOfTargets):

						tid_j, posX_j, posY_j, posZ_j, velX_j, velY_j, velZ_j, accX_j, accY_j, accZ_j = struct.unpack('I9f', packetPayload[:40])
						ec = struct.unpack('16f', packetPayload[40:40+64])
						g, confidenceLevel = struct.unpack('2f', packetPayload[40+64:112])

						DetectedTargets[j, 0] = tid_j
						DetectedTargets[j, 1] = posX_j
						DetectedTargets[j, 2] = posY_j
						DetectedTargets[j, 3] = posZ_j
						DetectedTargets[j, 4] = velX_j
						DetectedTargets[j, 5] = velY_j
						DetectedTargets[j, 6] = velZ_j
						DetectedTargets[j, 7] = accX_j
						DetectedTargets[j, 8] = accY_j
						DetectedTargets[j, 9] = accZ_j

						results_string += '%d %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f ' % (DetectedTargets[j, 0], DetectedTargets[j, 1], DetectedTargets[j, 2], DetectedTargets[j, 3], DetectedTargets[j, 4], DetectedTargets[j, 5], DetectedTargets[j, 6], DetectedTargets[j, 7], DetectedTargets[j, 8], DetectedTargets[j, 9])

						packetPayload = packetPayload[112:]

					if (saveTracks):
						fileTracks = open(targetsObject_fileName, 'a')
						fileTracks.write(results_string + '%1.3f\n' % timePacket)
						fileTracks.close()

				elif (tlvType == 8):

					numDetectedObj_previous = tlvLength-tlvHeaderLen
					targetIndex = [0]*numDetectedObj_previous
					results_string = ''

					for j in range(numDetectedObj_previous):

						targetIndex[j] = struct.unpack('B', packetPayload[:1])
						results_string += '%d ' % targetIndex[j]
						packetPayload = packetPayload[1:]

					if (saveTracksID):
						fileTracksID = open(targetsIndex_FileName, 'a')
						fileTracksID.write(results_string + '%1.3f\n' % timePacket)
						fileTracksID.close()

			if (numTLVs > 0):
				print(' ')
			
			if (iterations % iterations_plot == 0):
				# Plot section
				# Clear the plot
				Plot_2D.clear()
				r1 = pg.QtGui.QGraphicsRectItem(boundaryBox[0], boundaryBox[2], -boundaryBox[0]+boundaryBox[1], -boundaryBox[2]+boundaryBox[3])
				r2 = pg.QtGui.QGraphicsRectItem(staticBoundaryBox[0], staticBoundaryBox[2], -staticBoundaryBox[0]+staticBoundaryBox[1], -staticBoundaryBox[2]+staticBoundaryBox[3])
				r1.setPen(pg.mkPen('r', width=3))
				r2.setPen(pg.mkPen('g', width=3))
				Plot_2D.addItem(r1)
				Plot_2D.addItem(r2)

				if (numDetectedObj > 0):

					x = np.zeros(numDetectedObj)
					y = np.zeros(numDetectedObj)
					z = np.zeros(numDetectedObj)

					for i in range(numDetectedObj):

						range_i = detectedObjects[i, 0]
						azimuth_i = detectedObjects[i, 1] * np.pi/180
						elevation_i = detectedObjects[i, 2] * np.pi/180
						snr_i = detectedObjects[i, 4]

						x[i] = range_i * np.cos(elevation_i)*np.sin(azimuth_i)
						y[i] = range_i * np.cos(elevation_i)*np.cos(azimuth_i)
						z[i] = range_i * np.sin(elevation_i)

					Plot_2D.plot(x, y, pen=(0,0,0,0), symbolBrush=(0,0,0,255), symbolSize=5)
				
				if (numOfTargets > 0):

					x = np.zeros(numOfTargets)
					y = np.zeros(numOfTargets)
					z = np.zeros(numOfTargets)

					for i in range(numOfTargets):

						x[i] = DetectedTargets[i, 1]
						y[i] = DetectedTargets[i, 2]
						z[i] = DetectedTargets[i, 3]
					
					Plot_2D.plot(x, y, pen=(0,0,0,0), symbolBrush=(255,0,0,50), symbolSize=20, symbolPen='r')

				Plot_2D.setXRange(boundaryBox[0]-2, boundaryBox[1]+2, padding=0)
				Plot_2D.setYRange(boundaryBox[2]-2, boundaryBox[3]+2, padding=0)
				Plot_2D.setLabel('bottom', 'X (m)')
				Plot_2D.setLabel('left', 'Y (m)')
				Plot_2D.setAspectLocked()
				Plot_2D.showGrid(x=True, y=True)


				pg.QtGui.QApplication.processEvents()

		else:
			packetHeader = packetHeader[1:]

except:

	print('Closing ports')
	dataPort.close()
	raise
