import time
from abc import ABC, abstractmethod

import serial
import serial.tools.list_ports


class SerialSensor(ABC):

    SERIAL_DEFAULTS = {
        "parity": serial.PARITY_NONE,
        "stopbits": serial.STOPBITS_ONE,
        "timeout": 1,
    }

    def __init__(self):
        self._connected = False

    @property
    def is_connected(self):
        return self._connected

    @abstractmethod
    def connect(self, port1=None, port2=None):
        pass

    @abstractmethod
    def disconnect(self):
        pass

    def __enter__(self):
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def __del__(self):
        """Destructor to ensure connections are closed."""
        self.disconnect()


class RadarSensor(SerialSensor):

    RADAR_CONFIG = "odin/radar/config/chirp_3DPeople.cfg"

    CONFIG_PORT_KWARGS = {
        **SerialSensor.SERIAL_DEFAULTS,
        "baudrate": 115200,
    }

    DATA_PORT_KWARGS = {
        **SerialSensor.SERIAL_DEFAULTS,
        "baudrate": 921600,
    }

    def __init__(self):

        super().__init__()
        self.config_conn = None
        self.data_conn = None

    @property
    def is_connected(self):
        """check if connections are open"""
        return (
            self.config_conn is not None
            and self.data_conn is not None
            and self._connected
        )

    def connect(self, config_port=None, data_port=None):
        """Connect to radar device with automatic port detection and verification."""
        # Prevent multiple connections
        if self.is_connected:
            print("Already connected to radar device")
            return True

        try:
            # Auto-detect ports if not provided
            if all(x is None for x in [config_port, data_port]):
                config_port, data_port = self._find_radar_ports()

            # Check if ports were found/provided
            if config_port is None or data_port is None:
                raise ConnectionError("Could not find or identify radar ports")

            # Connect to identified ports
            self.config_conn = serial.Serial(config_port, **self.CONFIG_PORT_KWARGS)
            self.data_conn = serial.Serial(data_port, **self.DATA_PORT_KWARGS)

            self._connected = True
            print(
                f"Connected to radar: config={self.config_conn.port}, data={self.data_conn.port}"
            )
            return True

        except Exception as e:
            self.disconnect()
            raise ConnectionError(f"Connection failed: {e}")

    def disconnect(self):
        """Safely disconnect from all radar ports."""
        try:
            if self.config_conn is not None:
                self.config_conn.close()
            if self.data_conn is not None:
                self.data_conn.close()
        except Exception as e:
            print(f"Error closing ports: {e}")
        finally:
            self.config_conn = None
            self.data_conn = None
            self._connected = False

    def _find_radar_ports(self):
        """Find and identify radar ports for CP2105 dual UART."""
        try:
            # Find candidate ports
            candidate_ports = []
            for port in serial.tools.list_ports.comports():
                if "CP2105" in port.description and "SLAB_USBtoUART" in port.device:
                    candidate_ports.append(port.device)

            if len(candidate_ports) != 2:
                print(f"Warning: Found {len(candidate_ports)} radar ports, expected 2")
                return None, None

            # Sort ports to get consistent ordering
            candidate_ports.sort()
            port1, port2 = candidate_ports

            # Test which port is config
            config_port = self._identify_config_port(port1, port2)
            if config_port is None:
                return None, None

            # Return ports in correct order
            data_port = port2 if config_port == port1 else port1
            return config_port, data_port

        except Exception as e:
            print(f"Error finding radar ports: {e}")
            return None, None

    def _identify_config_port(self, port1, port2):
        """Test which port responds to config commands."""
        for port in [port1, port2]:
            try:
                with serial.Serial(port, **self.CONFIG_PORT_KWARGS) as ser:
                    # Try sending a version command
                    ser.write(b"version\n")
                    ser.flush()
                    time.sleep(0.3)

                    # Check for response
                    if ser.in_waiting > 0:
                        response = ser.read(ser.in_waiting)
                        if (
                            "mmwave"
                            in response.decode("utf-8", errors="ignore").lower()
                        ):
                            print(f"Found config port: {port}")
                            return port
            except Exception as e:
                print(f"Error testing {port} as config: {e}")

        print("Could not identify config port")
        return None

    def parse_radar_config(self, config_file=None):
        """Read radar configuration file.

        Args:
            config_file: Path to config file. If None, uses default.
        """
        if config_file is None:
            config_file = self.RADAR_CONFIG

        try:
            with open(config_file, "r") as fp:
                self.cmd_count = 0
                self.commands = []
                for line in fp:
                    if len(line) > 1:
                        if line[0] != "%":
                            self.commands.append(line)
                            self.cmd_count += 1
            return True
        except FileNotFoundError:
            print(f"Config file not found: {config_file}")
            return False
        except Exception as e:
            print(f"Error reading config file: {e}")
            return False
