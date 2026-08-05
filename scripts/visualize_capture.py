#!/usr/bin/env python3
"""Script to visualize captured radar data."""
import argparse
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from python.capture.replay import DataReplay


def main():
    parser = argparse.ArgumentParser(description="Visualize captured radar data")
    parser.add_argument("capture_file", help="Path to capture CSV file")
    parser.add_argument("--fps", type=int, default=10, help="Playback FPS")
    
    args = parser.parse_args()
    
    # TODO: Implement visualization
    print(f"Visualizing: {args.capture_file}")


if __name__ == "__main__":
    main()