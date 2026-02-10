import math

class HeadingVerifier:
    def __init__(self, min_move_dist, alpha=0.1):
        """
        Args:
            min_move_dist (float): Meters robot must move before updating correction.
            alpha (float): Smoothing factor (0.0 to 1.0). Lower = smoother, slower updates.
        """
        self.prev_gps = None
        self.min_move_dist = min_move_dist 
        self.heading_offset = 0.0 
        self.alpha = alpha 

    def get_corrected_heading(self, current_imu, current_gps):
        """
        Returns the IMU heading corrected by the GPS track.
        """
        # 1. Handle initialization or invalid GPS
        if not self._is_valid_gps(current_gps):
            return (current_imu + self.heading_offset) % 360

        if self.prev_gps is None:
            self.prev_gps = current_gps
            return (current_imu + self.heading_offset) % 360

        # 2. Check distance moved
        dist = self.haversine_distance(self.prev_gps, current_gps)

        # 3. If we moved enough, calculate the REAL GPS bearing and update offset
        if dist > self.min_move_dist:
            # Calculate GPS bearing (0-360 clockwise from North)
            gps_bearing = self.compute_bearing(self.prev_gps, current_gps)
            
            # Calculate the error between GPS and Raw IMU
            # We use math logic to handle the 0/360 wrap around (e.g. 359 vs 1 degree)
            raw_diff = gps_bearing - current_imu
            
            # Normalize diff to [-180, 180] to find shortest rotation
            diff = (raw_diff + 180) % 360 - 180
            
            # Update the offset (using a filter to smooth it out)
            self.heading_offset = (1 - self.alpha) * self.heading_offset + self.alpha * diff
            
            print(f"[GPS FIX] Moved {dist:.2f}m | GPS Bearing: {gps_bearing:.1f} | IMU Raw: {current_imu:.1f} | New Offset: {self.heading_offset:.1f}")

            # Update previous GPS to current for the next segment
            self.prev_gps = current_gps

        # 4. Apply the offset to the current IMU reading
        corrected_heading = (current_imu + self.heading_offset) % 360
        return corrected_heading

    def _is_valid_gps(self, gps):
        return gps is not None and gps != (0,0) and gps != (0.0, 0.0)

    @staticmethod
    def haversine_distance(coord1, coord2):
        R = 6371000  # Radius of Earth in meters
        lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
        lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    @staticmethod
    def compute_bearing(p1, p2):
        lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
        lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing_rad = math.atan2(x, y)
        return (math.degrees(bearing_rad) + 360) % 360