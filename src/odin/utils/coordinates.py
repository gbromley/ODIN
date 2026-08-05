"""Coordinate system transformations and utilities."""
import numpy as np
from typing import Tuple


def cartesian_to_spherical(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert Cartesian coordinates to spherical (range, azimuth, elevation)."""
    range_m = np.sqrt(x**2 + y**2 + z**2)
    azimuth_rad = np.arctan2(y, x)
    elevation_rad = np.arctan2(z, np.sqrt(x**2 + y**2))
    return range_m, azimuth_rad, elevation_rad


def spherical_to_cartesian(range_m: float, azimuth_rad: float, elevation_rad: float) -> Tuple[float, float, float]:
    """Convert spherical coordinates to Cartesian."""
    x = range_m * np.cos(elevation_rad) * np.cos(azimuth_rad)
    y = range_m * np.cos(elevation_rad) * np.sin(azimuth_rad)
    z = range_m * np.sin(elevation_rad)
    return x, y, z