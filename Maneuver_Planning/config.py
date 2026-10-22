#  Orbit, maneuver limits, and settings
import numpy as np

# Physical constants (SI units)
MU_Earth = 3.986004418e14  # Earth's gravitational parameter, m^3/s^2
EARTH_RADIUS = 6378137  # Earth's radius, m
ALTITUDE = 500000  # Altitude above Earth's surface, m

ORBIT_RADIUS = EARTH_RADIUS + ALTITUDE  # Orbit radius from Earth's center, m
MEAN_MOTION = np.sqrt(MU_Earth / ORBIT_RADIUS**3)  # Mean motion, rad/s


# 500 km example scenario (for demonstration purposes)
T_CA = 1200
T_BURN = 300
T_END = 1800

REQUIRED_SEPARATION = 100  # Required separation distance, m
PLANNING_SEPARATION = 105  # Planned separation distance (small numerical margin), m
MAX_DELTA_V = 0.5  # Maximum allowable delta-v, m/s
