# Enter the python3 execution path here for making it an executable script
import sys
import argparse
from geopy.point import Point
from geopy.distance import distance

def main():
  parser = argparse.ArgumentParser(description="Calculate base station coordinates from a reference pole (point).")
  parser.add_argument("--pole-lat", type=float, required=True, help="Latitude of the competition pole")
  parser.add_argument("--pole-lon", type=float, required=True, help="Longitude of the competition pole")
  parser.add_argument("--pole-alt", type=float, required=True, help="Altitude/Height of the pole in meters - height to base")
  parser.add_argument("--dist", type=float, required=True, help="Distance from pole to setup place in metres")
  parser.add_argument("--bearing", type=float, required=True, help="Compass bearing from pole to setup place")

  args = parser.parse_args()
  # Calculate a 2D position
  pole_point = Point(args.pole_lat, args.pole_lon)
  destination = distance(kilometers=args.dist/1000).destination(pole_point, args.bearing)

  # Alt to cm for the shell script
  alt_cm = int(args.pole-alt * 100)

  print(f"{destination.latitude:.7f} {destination.longitude:.7f} {alt_cm}")

if __name__ == "__main__":
  main()
