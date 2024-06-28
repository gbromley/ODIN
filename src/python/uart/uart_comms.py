import time
import serial
import queue
import threading
import struct


### Loads the 

def readConfigFile(configFileName):
    """
    Reads the chirp configuration file.

    Parameters
    ----------
    file_path : string
        Path to the chrip config file.

    Returns
    -------
    list[string]
        Every command from the chirp config loaded into an array.
    """
    
    
    try:
        with open(configFileName, 'r') as fp:
            
            # Keeping count for writing line by line.
            cnt = 0
            commands = []
            for line in fp:
                if (len(line) > 1):
                    # '%' is the comment character in chirp file
                    if (line[0] != '%'):
                        commands.append(line)
                        cnt += 1
        return (cnt,commands)
    
    except FileNotFoundError as err:
        print("Chirp config file not found.")
    finally:
        fp.close()
        
def writeConfig(configPort, configFilePath):
    """
    Writes the configuration out to a serial port.

    Parameters
    ----------
    configPort : Serial object
        pyserial object
        
    configFileName : String
        Radar chirp configuration file

    Returns
    -------
    list[string]
        Every command from the chirp config loaded into an array.
    """
    cnt,commands = readConfigFile(configFilePath)
    
    for i in range(cnt):
        configPort.write(bytearray(commands[i].encode()))
        time.sleep(20e-3)
        response = bytearray([])
        while(configPort.in_waiting > 0):
            response += configPort.read(1)
        print(response.decode())
        
def readPacketHeader(dataPort):
    
    
    dataPort.reset_input_buffer()
    syncPattern = 0x708050603040102
    ### Packet header is 40 total bytes, 8 bytes for magic/sync word and 32 bytes for 8 other variables. 
    ### The format for struct.unpack is Q8I because the magic word is 8 bytes and Q stands for unsigned long long. 8I means that 8 unsigned ints follow
    packetHeader = bytearray([])
    tlvHeaderLen = 8
    headerLen = 40
    timePacket = time.time()
    run = True
    
    while(True):
        
        try:
            
            
            ### These lines of code basically grab the header data and shifts it by one until the sync value is found. 
            packetHeader += dataPort.read(headerLen-len(packetHeader))
            
            #print(len(packetHeader))
            sync, version, totalPacketLen, platform, frameNumber, timeCpuCycles, numDetectedObj, numTLVs, subFrameNumber =  struct.unpack('Q8I', packetHeader[:headerLen])
            if (sync == syncPattern):
                
                print('success')
                print(time.perf_counter())
                print(f'MagicWord: {sync}, Version: {version}, Total Packet Length: {totalPacketLen}, Platform: {platform}, Frame Number: {frameNumber}, Time CPU Cycles {timeCpuCycles}, Number of Objects: {numDetectedObj}, Number of TLVs: {numTLVs}, Sub Frame Number: {subFrameNumber}')
                    
                return sync, version, totalPacketLen, platform, frameNumber, timeCpuCycles, numDetectedObj, numTLVs, subFrameNumber
                #break
            else:
                packetHeader = packetHeader[1:]
        except serial.SerialException as serialError:
            print('Something is not correct with the serial port')
            break
            
        except KeyboardInterrupt:
            break


class RadarSerialCommunication:
    def __init__(self, data_port, data_baud, config_port, config_baud):
        self.config_port = config_port
        self.data_port = data_port
        self.config_baud = config_baud
        self.data_baud = data_baud
        self.data_connection = None
        self.config_connection = None
        ### Do we need two seperate connected flags?
        self.is_connected = False
        self.data_queue = queue.Queue()
        self.stop_event = threading.Event()

    def connect(self):
        try:
            
            self.config_connection = serial.Serial(self.config_port, 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.3)
            self.data_connection = serial.Serial(self.data_port, 921600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.3)
        
            
            self.is_connected = True
            print(f"Connected to {self.data_port} at {self.data_baud} baud")
            print(f"Connected to {self.config_port} at {self.config_baud} baud")
        except serial.SerialException as e:
            print(f"Failed to connect: {e}")

    def disconnect(self):
        if self.is_connected:
            self.data_connection.close()
            self.config_connection.close()
            self.is_connected = False
            print("Disconnected from serial port")

    def read_data(self, sync_pattern, header_len):
        while not self.stop_event.is_set():
            if self.is_connected:
                try:
                    
                    
                    
                    timePacket = time()
                    

                    packetHeader += self.data_connection.read(header_len-len(packetHeader))
                    
                    sync, version, total_packet_len, platform, frameNumber, subFrameNumber, chirpProcessingMargin, frameProcessingMargin, trackProcessTime, uartSentTime, numTLVs, checksum =  struct.unpack('Q9I2H', packetHeader[:header_len])
                    
                    if (sync == sync_pattern):

                        packetHeader = bytearray([])
                        # Want to read and send the header and data so removing totalPacketLen-header_len
                        packet_payload = self.data_connection.read(total_packet_len)

                        self.data_queue.put(packet_payload)
                    else:
                        packetHeader = packetHeader[1:]

                except serial.SerialException as e:
                    print(f"Error reading data: {e}")
                    self.disconnect()
                    break

    def start_streaming(self):
        self.stop_event.clear()
        self.stream_thread = threading.Thread(target=self.read_data)
        self.stream_thread.start()

    def stop_streaming(self):
        self.stop_event.set()
        if hasattr(self, 'stream_thread'):
            self.stream_thread.join()

    def get_data(self):
        if not self.data_queue.empty():
            return self.data_queue.get()
        return None
    
class RadarDataProcessor:
    def __init__(self, radar_serial_com):
        self.radar_communication = radar_serial_com

    def process_data(self):
        while True:
            data = self.radar_communication.get_data()
            if data:
                # Process the radar data here
                processed_data = self.parse_radar_data(data)
                yield processed_data
            else:
                break

    def parse_radar_data(self, data):
        # Implement your radar data parsing logic here
        # This is just a placeholder
        return {'raw': data, 'processed': 'Some processed data'}
    
        """
        
        
        
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
                                    print(results_string)
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

        """