import laspy
import numpy as np
from pyproj import Transformer

_transformer_cache = {}

def get_epsg(laz_path):
    """
    Reads the EPSG code from the LAZ file header.
    
    LiDAR data is always stored in a specific coordinate system (projection).
    The EPSG code is a standard number that identifies which coordinate system
    is being used. For example:
        EPSG:4326  -> standard GPS lat/lon (WGS84)
        EPSG:32612 -> UTM Zone 12N (likely what your Utah LiDAR uses)
    
    We need this code so we can convert GPS coordinates into the exact same
    coordinate system as the LiDAR points, making them directly comparable.
    
    Args:
        laz_path: path to the LAZ/LAS file
    Returns:
        epsg: integer EPSG code of the LAZ file's coordinate system
    """

    # Open the LAZ file in streaming mode to read the header only —
    # laspy.read() loads all point data into memory which is wasteful
    # here since we only need the CRS metadata from the header.
    with laspy.open(laz_path) as f:
        crs = f.header.parse_crs()

    # Convert the CRS object to a simple integer EPSG code
    # e.g. 32612 for UTM Zone 12N
    epsg = crs.to_epsg()

    print(f"LAZ file EPSG: {epsg}")
    return epsg


def gps_to_xy(lat, lon, epsg):
    """
    Converts a GPS coordinate (lat, lon) into the same horizontal coordinate
    system as the LiDAR points in the LAZ file.

    Why only x/y and not z:
        Competition waypoints are given as lat/lon only — no altitude.
        Even if altitude were available, GPS altitude (WGS84 ellipsoidal) does
        not match LiDAR z values which are in a different vertical datum.
        The correct z for any position comes from the LiDAR terrain data itself,
        so snapping to the nearest node is done in 2D (x/y only).

    Args:
        lat:  latitude in degrees (WGS84)
        lon:  longitude in degrees (WGS84)
        epsg: EPSG code of the LAZ file's coordinate system (from get_epsg)
    Returns:
        x, y: horizontal coordinates in the same projection as the LiDAR points
    """

    # Create a transformer that converts FROM WGS84 GPS (EPSG:4326)
    # TO the LAZ file's coordinate system (e.g. EPSG:32612 for UTM Zone 12N)
    # always_xy=True ensures we always pass (longitude, latitude) in that order
    # regardless of what the projection expects — avoids a common axis-swap bug
    if epsg not in _transformer_cache:
        _transformer_cache[epsg] = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
    transformer = _transformer_cache[epsg]

    # Transform longitude and latitude into projected x and y coordinates
    # Note: transformer.transform takes (longitude, latitude) not (latitude, longitude)
    # because always_xy=True enforces x (lon) before y (lat)
    x, y = transformer.transform(lon, lat)

    return x, y