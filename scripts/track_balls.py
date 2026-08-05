#!/usr/bin/env python3
"""Script to track balls in real-time from radar data."""
import argparse
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from python.sensors.radar.urad import URadSensor
from python.tracking.detector import BallisticDetector
from python.tracking.tracker import MultiObjectTracker


def main():
    parser = argparse.ArgumentParser(description="Track balls using radar")
    parser.add_argument("--config-port", required=True, help="Config port")
    parser.add_argument("--data-port", required=True, help="Data port")
    parser.add_argument("--visualize", action="store_true", help="Enable visualization")
    
    args = parser.parse_args()
    
    # TODO: Implement tracking logic
    print("Starting ball tracking...")


if __name__ == "__main__":
    main()