import time
import serial
import queue
import threading
import struct
import numpy as np

import sys
import os



        
""" def readPacketHeader(dataPort):
    
    
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
"""

class RadarCom:
    def __init__(self, data_port, data_baud, config_port, config_baud, sync_pattern):
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
        self.tlv_header_len = 8
        self.header_len = 48
        self.sync_pattern = sync_pattern

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
        self.stop_streaming()
        if self.is_connected:
            self.data_connection.close()
            self.config_connection.close()
            self.is_connected = False
            print("Disconnected from serial port")

    def read_data(self, sync_pattern, header_len):
        packet_header = bytearray([])
        while not self.stop_event.is_set():
            if self.is_connected:
                try:

                    time_packet = time.time()

                    packet_header += self.data_connection.read(header_len-len(packet_header))
                    
                    sync, version, total_packet_len, platform, frameNumber, subFrameNumber, chirpProcessingMargin, frameProcessingMargin, trackProcessTime, uartSentTime, numTLVs, checksum =  struct.unpack('Q9I2H', packet_header[:header_len])
                    
                    if (sync == sync_pattern):

                        packet_header = bytearray([])
                        # Want to read and send the header and data so removing totalPacketLen-header_len
                        packet_payload = self.data_connection.read(total_packet_len)

                        self.data_queue.put(packet_payload)
                    else:
                        # basically, move along the array until we find sync
                        packet_header = packet_header[1:]

                except serial.SerialException as e:
                    print(f"Error reading data: {e}")
                    self.disconnect()
                    break

    def start_streaming(self):
        self.stop_event.clear()
        self.stream_thread = threading.Thread(target=self.read_data, kwargs={'sync_pattern':0x708050603040102, 'header_len':self.header_len})
        self.stream_thread.start()

    def stop_streaming(self):
        self.stop_event.set()
        if hasattr(self, 'stream_thread'):
            self.stream_thread.join()

    def get_data(self):
        if not self.data_queue.empty():
            return self.data_queue.get()
        return None
    
    # This is getting to be radar specific. Once we have more sensors, we might need to rethink
    # class structure
    def read_config_file(self, config_file):
        try:
            with open(config_file, 'r') as fp:
                
                self.cmd_count = 0
                self.commands = []
                for line in fp:
                    if (len(line) > 1):
                        if (line[0] != '%'):
                            self.commands.append(line)
                            self.cmd_count += 1
            
        except FileNotFoundError as err:
            print("Chirp config file not found.")
        finally:
            fp.close()
    
    
    def write_config(self):
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
        
        if self.is_connected:
            try:
                for i in range(self.cmd_count):
                    self.config_connection.write(bytearray(self.commands[i].encode()))
                    time.sleep(20e-3)
                    response = bytearray([])
                    while(self.config_connection.in_waiting > 0):
                        response += self.config_connection.read(1)
                    print(response.decode())
            except serial.SerialException as e:
                    print(f"Error reading data: {e}")
                    self.disconnect()
                    
        
class RadarDataProcessor:
    def __init__(self, radar_serial_com):
        self.radar_communication = radar_serial_com
        # Need a better way of defining these
        self.header_len = 48
        self.tlv_header_len = 8

    def process_data(self):
        while True:
            data = self.radar_communication.get_data()
            if data:
                # Process the radar data here
                header_data =  struct.unpack('Q9I2H', data[:self.header_len])
                num_TLV = header_data[10]
                uart_sent_time = header_data[9]
                
                processed_data = self.parse_radar_data(num_TLV,uart_sent_time, data[self.header_len:])
                
                yield processed_data
            else:
                break

    def parse_radar_data(self, num_TLV, uart_sent_time, tlv_packet):
        # Implement your radar data parsing logic here
        # This is just a placeholder
        for i in range(num_TLV):

            tlv_type, tlv_len = struct.unpack('2I', tlv_packet[:self.tlv_header_len])
            #What doe these tlv numbers mean?
            if (tlv_type > 20 or tlv_len > 10000):
                break

            tlv_data = tlv_packet[self.tlv_header_len:]

            if (tlv_type == 6):
                #Need to figure out out to build out the data structure
                self.process_tlv_6(tlv_len, tlv_data)
                
            elif (tlv_type == 7):
                self.process_tlv_7(self, tlv_len, tlv_data)

            elif (tlv_type == 8):
                self.process_tlv_8(self, tlv_len, tlv_data)
        if (num_TLV > 0):
            print(' ')

    def process_tlv_6(self, tlv_len, tlv_data):
        #Need to define what the ints mean here for getting data
        point_unit = struct.unpack('5f', tlv_data[:20])
        # slowly removing data from the tlv_data 
        tlv_data = tlv_data[20:]
        num_detected_obj = int((tlv_len-self.tlv_header_len-20)/8)
        detected_objects = np.zeros((num_detected_obj, 5))
        #this is the final output variable for now
        results_string = ''

        print('numDetectedObj: %d' % num_detected_obj)

        for j in range(num_detected_obj):

            elevation_j, azimuth_j, doppler_j, range_j, snr_j = struct.unpack('2bh2H', tlv_data[:8])

            detected_objects[j, 0] = range_j * point_unit[3]
            detected_objects[j, 1] = azimuth_j * point_unit[1] * 180/np.pi
            detected_objects[j, 2] = elevation_j * point_unit[0] * 180/np.pi
            detected_objects[j, 3] = doppler_j * point_unit[2]
            detected_objects[j, 4] = snr_j * point_unit[4]

            tlv_data = tlv_data[8:]

            results_string += '%1.3f %1.3f %1.3f %1.3f %1.3f ' % (detected_objects[j, 0], detected_objects[j, 1], detected_objects[j, 2], detected_objects[j, 3], detected_objects[j, 4])
        #return detected_objects
        print(results_string)
    
    def process_tlv_7(self, tlv_len, tlv_data):
        num_targets = int((tlv_len-self.tlv_header_len)/112)
        detected_targets = np.zeros((num_targets, 10))
        results_string = ''

        print('numOfTargets: %d' % num_targets)

        for j in range(num_targets):

            tid_j, posX_j, posY_j, posZ_j, velX_j, velY_j, velZ_j, accX_j, accY_j, accZ_j = struct.unpack('I9f', tlv_data[:40])
            ec = struct.unpack('16f', tlv_data[40:40+64])
            g, confidenceLevel = struct.unpack('2f', tlv_data[40+64:112])

            detected_targets[j, 0] = tid_j
            detected_targets[j, 1] = posX_j
            detected_targets[j, 2] = posY_j
            detected_targets[j, 3] = posZ_j
            detected_targets[j, 4] = velX_j
            detected_targets[j, 5] = velY_j
            detected_targets[j, 6] = velZ_j
            detected_targets[j, 7] = accX_j
            detected_targets[j, 8] = accY_j
            detected_targets[j, 9] = accZ_j

            results_string += '%d %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f ' % (detected_targets[j, 0], detected_targets[j, 1], detected_targets[j, 2], detected_targets[j, 3], detected_targets[j, 4], detected_targets[j, 5], detected_targets[j, 6], detected_targets[j, 7], detected_targets[j, 8], detected_targets[j, 9])

            tlv_data = tlv_data[112:]
            print(results_string)
            #return detected_targets

    def process_tlv_8(self, tlv_len, tlv_data):
        pass
        numDetectedObj_previous = tlv_len - self.tlv_header_len
        targetIndex = [0] * numDetectedObj_previous
        results_string = ''

        for j in range(numDetectedObj_previous):

            targetIndex[j] = struct.unpack('B', packetPayload[:1])
            results_string += '%d ' % targetIndex[j]
            packetPayload = packetPayload[1:]

