#!/usr/bin/env python3
"""Script to capture radar data and save to CSV."""
import argparse
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent))

from python.sensors.radar.urad import URadSensor
from python.capture.logger import CSVLogger


def main():
    parser = argparse.ArgumentParser(description="Capture radar data to CSV")
    parser.add_argument("--config-port", required=True, help="Config port (e.g., /dev/tty.SLAB_USBtoUART)")
    parser.add_argument("--data-port", required=True, help="Data port (e.g., /dev/tty.SLAB_USBtoUART3)")
    parser.add_argument("--output-dir", default="./captures", help="Output directory for CSV files")
    parser.add_argument("--duration", type=int, default=60, help="Capture duration in seconds")
    
    args = parser.parse_args()
    
    # TODO: Implement capture logic
    print(f"Capturing radar data for {args.duration} seconds...")
    print(f"Output directory: {args.output_dir}")


if __name__ == "__main__":
    main()