"""
Radar module for interfacing with radar sensors, managing serial connections,
and processing radar data.
"""

from .sensors import SerialSensor, RadarSensor
from .serial_manager import SerialDeviceManger
from .uart_comms import RadarCom, RadarDataProcessor