import serial
import struct
import numpy as np
from time import time, sleep

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
		PinReset = OutputDevice(21)
		PinReset.off()
		sleep(10e-3)
		PinReset.on()
		sleep(1)

	sleep(1)

	#print('Reading configuration file')
	with open(configFile_name, 'r') as fp:
		cnt = 0
		commands = []
		for line in fp:
			if (len(line) > 1):
				if (line[0] != '%'):
					commands.append(line)
					cnt += 1
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

	while (True):

		timePacket = time()

		packetHeader += dataPort.read(headerLen-len(packetHeader))

		sync, version, totalPacketLen, platform, frameNumber, subFrameNumber, chirpProcessingMargin, frameProcessingMargin, trackProcessTime, uartSentTime, numTLVs, checksum =  struct.unpack('Q9I2H', packetHeader[:headerLen])

		if (sync == syncPattern):

			packetHeader = bytearray([])
			packetPayload = dataPort.read(totalPacketLen-headerLen)

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

		else:
			packetHeader = packetHeader[1:]

except:

	print('Closing ports')
	dataPort.close()
