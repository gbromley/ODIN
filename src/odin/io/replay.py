"""Tools for replaying captured sensor data."""
from pathlib import Path
from typing import Iterator, Tuple
import numpy as np


class DataReplay:
    """Replay captured sensor data from files."""
    
    def __init__(self, capture_path: str):
        """Initialize replay with captured data file."""
        self.capture_path = Path(capture_path)
        
    def read_frames(self) -> Iterator[Tuple[float, int, np.ndarray]]:
        """Iterate through frames in the capture file."""
        # TODO: Implement frame reading
        pass
    
    def get_metadata(self) -> dict:
        """Read metadata from capture file."""
        # TODO: Implement metadata reading
        pass