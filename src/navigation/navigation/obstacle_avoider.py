import depthai as dai
import cv2
import numpy as np
import time
import math
import zmq
import base64
from collections import deque
#100m 25 images

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from ublox_ubx_msgs.msg import UBXNavPVT
from std_msgs.msg import Float32

class SwervePublisher(Node):
    def __init__(self):
        super().__init__("swerve_publisher")
        self.pub = self.create_publisher(Float32MultiArray, "swerve", 10)

        self.lat = 0
        self.lon = 0

        self.origin_lat = None
        self.origin_lon = None

        self.heading = None

        self.curr_waypoint = (None , None)

        self.create_subscription(UBXNavPVT, '/rover1/ubx_nav_pvt', self.gps_callback, 10)
        self.heading_sub = self.create_subscription(Float32, '/heading', self.heading_cb, 10)
        self.waypoint_sub = self.create_subscription(Float64MultiArray, '/waypoint', self.waypoint_cb, 10)

    def send(self, swrv):
        msg = Float32MultiArray()
        msg.data = swrv
        self.pub.publish(msg)

    def gps_callback(self, msg):
        current_lat = msg.lat * 1e-7
        current_lon = msg.lon * 1e-7
        
        if self.origin_lat is None:
            # First fix
            self.origin_lat = current_lat
            self.origin_lon = current_lon
            self.lat = current_lat
            self.lon = current_lon
        else:
            # Smooth out minor RTK jitter (Alpha = 0.6 means trust new reading 60%, old reading 40%)
            alpha = 0.25 
            self.lat = (alpha * current_lat) + ((1.0 - alpha) * self.lat)
            self.lon = (alpha * current_lon) + ((1.0 - alpha) * self.lon)
    
    def waypoint_cb(self, msg):
        self.curr_waypoint = (msg.data[0], msg.data[1]) # lat, lon

    # def current_pos_cb(self, msg):
    #     self.curr_pos = (msg.lat * 1e-7, msg.lon * 1e-7) # lat, lon

    def heading_cb(self, msg):
        self.heading = msg.data
    
    


class SectorDepthClassifier():
    def __init__(self):
        self.debug   = False
        self.onRover = False

        # ── ZMQ video stream ───────────────────────────────────────────────
        self.context = zmq.Context()
        self.socket  = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.bind("tcp://*:5555")
        print("Video streamer ready on port 5555")

        # ── Camera intrinsics ──────────────────────────────────────────────
        self.X_PIXEL_OFFSET = np.float32(643.2372)
        self.Y_PIXEL_OFFSET = np.float32(367.1311)
        self.FOCAL_LENGTH = np.float32(568.15)
        self.W = 1280
        self.H = 720

        # ── Obstacle avoidance params ──────────────────────────────────────
        self.DEPTH_THRESH = np.float32(4.5)   # metres — beyond this = ignore
        self.MAX_FORWARD  = 1.0
        self.kP           = 0.02

        # ── VFH sector setup ───────────────────────────────────────────────
        self.DEG_PER_SECT = 4.0
        self.FOV_DEG      = 2 * np.degrees(np.arctan(self.W / (2 * self.FOCAL_LENGTH)))
        self.NUM_SECTORS  = int(360 / self.DEG_PER_SECT)

        # ── VFH histogram state ────────────────────────────────────────────
        self.sector_state    = np.zeros(self.NUM_SECTORS, dtype=int)
        self.hist_persistent = np.zeros(self.NUM_SECTORS)
        self.T_HIGH          = 38
        self.T_LOW           = 26
        self.MIN_VALLEY_SECTS = 2
        self.SMAX             = 6

        # ── Steering smoothing ─────────────────────────────────────────────
        self.previous_best_theta = None
        self.EWA_ALPHA           = 0.27

        # ── Recovery state ─────────────────────────────────────────────────
        self.recovery_frame_count = 0

        # ── RANSAC ground plane cache ──────────────────────────────────────
        self.cached_n        = None
        self.cached_d        = None
        self.ransac_counter  = 0
        self.RANSAC_INTERVAL = 3

        self.lat = 0
        self.lon = 0
        self.inflated = None
        self.origin_lat = 1
        self.origin_lon = 2

        self.R = 6378137.0

        self.prev_rx = 0.0
        self.prev_ry = 0.0

        self.ground_thresh = -0.25 

        self.map_res = 0.05
        
        self.MAP_SIZE = 180   
        self.MAP_HALF = self.MAP_SIZE // 2 
        self.map = np.zeros((self.MAP_SIZE, self.MAP_SIZE), dtype=np.float32)
        self.elevation_map = np.full((self.MAP_SIZE, self.MAP_SIZE), -np.inf, dtype=np.float32)
        self.map_center_rx = 0.0
        self.map_center_ry = 0.0

        self.heading = None
        self.waypoint = (None, None)

        ROBOT_RADIUS_M = 0.55
        r = int(ROBOT_RADIUS_M / self.map_res)
        kernel_size = 2 * r + 1
        self.dilation_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

        self.is_aligning = False
        self.debug_removed_slopes = None
    
    def _build_elev_bg(self):
        CELL_PX = 4
        PAD     = 24
        SZ      = self.MAP_SIZE * CELL_PX
        
        # 1. Allocate full canvas and paint static background color
        canvas  = np.zeros((SZ + 2 * PAD, SZ + 2 * PAD + 50, 3), dtype=np.uint8)
        canvas[:] = (18, 18, 31)

        # 2. Pre-draw static text: Compass Labels
        MID = PAD + self.MAP_HALF * CELL_PX
        for txt, pos in [("N", (MID - 4, 13)), ("S", (MID - 4, PAD + SZ + 18)),
                        ("W", (4, MID + 4)),  ("E", (PAD + SZ + 4, MID + 4))]:
            cv2.putText(canvas, txt, pos, cv2.FONT_HERSHEY_PLAIN, 0.85, (100, 100, 140), 1)

        # 3. Pre-draw static text: Grid tick distances
        for i in range(0, self.MAP_SIZE + 1, 20):
            px = PAD + i * CELL_PX
            if 0 < i < self.MAP_SIZE:
                cv2.putText(canvas, f"{(i - self.MAP_HALF) * self.map_res:.0f}m",
                            (px - 10, PAD - 5), cv2.FONT_HERSHEY_PLAIN, 0.7, (80, 80, 100), 1)

        # 4. Pre-build and draw static color scale bar image
        BAR_X = PAD + SZ + 6
        BAR_W = 12
        BAR_H = SZ

        gradient_1d  = np.arange(255, -1, -1, dtype=np.uint8).reshape(256, 1)
        colored_bar  = cv2.applyColorMap(gradient_1d, cv2.COLORMAP_TURBO)
        bar_img      = cv2.resize(colored_bar, (BAR_W, BAR_H), interpolation=cv2.INTER_LINEAR)

        canvas[PAD : PAD + BAR_H, BAR_X : BAR_X + BAR_W] = bar_img
        cv2.putText(canvas, "elev", (BAR_X, PAD + BAR_H // 2),
                    cv2.FONT_HERSHEY_PLAIN, 0.7, (160, 160, 180), 1)

        # Cache variables
        self._elev_bg_canvas = canvas
        self._elev_CELL_PX   = CELL_PX
        self._elev_PAD       = PAD
        self._elev_SZ        = SZ
        self._elev_BAR_X     = BAR_X
        self._elev_BAR_W     = BAR_W
        self._elev_BAR_H     = BAR_H

    def _build_map_background(self):
        CELL_PX = 4
        PAD     = 24
        SZ      = self.MAP_SIZE * CELL_PX
        canvas  = np.zeros((SZ + 2 * PAD, SZ + 2 * PAD, 3), dtype=np.uint8)
        canvas[:] = (18, 18, 31)
        GRID_COLOR = (40, 40, 60)
        MID = PAD + self.MAP_HALF * CELL_PX

        for i in range(0, self.MAP_SIZE + 1, 20):
            px = PAD + i * CELL_PX
            cv2.line(canvas, (px, PAD), (px, PAD + SZ), GRID_COLOR, 1)
            cv2.line(canvas, (PAD, px), (PAD + SZ, px), GRID_COLOR, 1)
            if 0 < i < self.MAP_SIZE:
                cv2.putText(canvas, f"{(i - self.MAP_HALF) * self.map_res:.0f}m",
                            (px - 10, PAD - 5), cv2.FONT_HERSHEY_PLAIN, 0.7, (80, 80, 100), 1)

        for txt, pos in [("N", (MID-4, 13)), ("S", (MID-4, PAD+SZ+18)),
                        ("W", (4, MID+4)),  ("E", (PAD+SZ+4, MID+4))]:
            cv2.putText(canvas, txt, pos, cv2.FONT_HERSHEY_PLAIN, 0.85, (100, 100, 140), 1)

        # Scale bar
        SB_X, SB_Y = PAD + 8, PAD + SZ - 12
        SB_W = 20 * CELL_PX
        cv2.line(canvas,  (SB_X, SB_Y),     (SB_X + SB_W, SB_Y),    (160,160,180), 1)
        cv2.line(canvas,  (SB_X, SB_Y-3),   (SB_X, SB_Y+3),          (160,160,180), 1)
        cv2.line(canvas,  (SB_X+SB_W,SB_Y-3),(SB_X+SB_W,SB_Y+3),    (160,160,180), 1)
        cv2.putText(canvas, "1 m", (SB_X + SB_W//2 - 8, SB_Y - 5),
                    cv2.FONT_HERSHEY_PLAIN, 0.75, (160,160,180), 1)

        # Legend
        legend = [((40,30,100),"inflated"),((60,30,226),"obstacle"),
                ((30,70,70),"sub-thresh"),((55,138,221),"robot"),((29,158,117),"heading")]
        lx, ly = PAD + SZ - 110, PAD + 8
        for color, label in legend:
            cv2.rectangle(canvas, (lx, ly), (lx+10, ly+9), color, -1)
            cv2.putText(canvas, label, (lx+14, ly+9), cv2.FONT_HERSHEY_PLAIN, 0.75, (160,160,180), 1)
            ly += 14

        self._map_bg = canvas      # cache
        self._map_CELL_PX = CELL_PX
        self._map_PAD = PAD
        self._map_SZ  = SZ

        # Precompute per-pixel angle/distance for sector overlay (only done ONCE)
        ys, xs = np.mgrid[0:self.MAP_SIZE, 0:self.MAP_SIZE]
        dy_m =  (self.MAP_HALF - ys) * self.map_res   # north
        dx_m =  (xs - self.MAP_HALF) * self.map_res   # east
        self._sector_dist_m    = np.sqrt(dx_m**2 + dy_m**2)
        # Angle in compass degrees (0=N, clockwise), matching visualize_map's convention
        self._sector_angle_deg = (np.degrees(np.arctan2(dx_m, dy_m)) + 360) % 360

    def angular_dist(self, s1, s2, max_val):
        """Calculates shortest distance between two sectors in a circular array."""
        diff = abs(s1 - s2) % max_val
        return min(diff, max_val - diff)
    

    def remove_upslopes(self, box_size=3, smooth_size=5, slope_thresh_deg=40.0):
        """
        Slope-based obstacle removal pass. 
        Gentle slopes (< 40°) are traversable ramps -> removed from obstacle map.
        Steep slopes  (> 40°) are walls             -> kept in obstacle map.
        """
        valid_mask = self.elevation_map > -np.inf
        elev_input = np.where(valid_mask, self.elevation_map, 0.0).astype(np.float32)
        smoothed = cv2.blur(elev_input, (smooth_size, smooth_size))
        elev_for_max = np.where(valid_mask, smoothed, -1e9).astype(np.float32)
        elev_for_min = np.where(valid_mask, smoothed,  1e9).astype(np.float32)
        kernel = np.ones((box_size, box_size), dtype=np.uint8)
        local_max = cv2.dilate(elev_for_max, kernel)
        local_min = cv2.erode(elev_for_min, kernel)
        dz = local_max - local_min
        dx = (box_size - 1) * self.map_res
        slope_angle = np.degrees(np.arctan(dz / dx)) 
        remove_mask = (slope_angle < slope_thresh_deg) & valid_mask
        self.map[remove_mask] = 0.0

        self.debug_removed_slopes = remove_mask


    def map_to_histogram(self, compass_angle):
        yaw   = math.radians(90.0 - compass_angle)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        hist = np.zeros(self.NUM_SECTORS)

        self.remove_upslopes(box_size=3, smooth_size=3, slope_thresh_deg=40.0)

        # Cells with fewer than BUSH_THRESH hits 
        # zero them out before inflating.
        MAX_CELL_WEIGHT = (self.map_res * self.FOCAL_LENGTH) ** 2  # 793 
        OBJ_DENSITY  = 0.5          # ignores anything thats less than 50% solid
        BUSH_THRESH     = MAX_CELL_WEIGHT * OBJ_DENSITY
        filtered_map = np.where(self.map >= BUSH_THRESH, self.map, 0.0).astype(np.float32)
        self.inflated = cv2.dilate(filtered_map, self.dilation_kernel)
        inflated = self.inflated

        cell_i, cell_j = np.where(inflated > 0)
        if len(cell_i) == 0:
            return hist

        # Robot-centric offsets in metres (ENU)
        d_east  = (cell_j - self.MAP_HALF).astype(np.float64) * self.map_res
        d_north = (self.MAP_HALF - cell_i).astype(np.float64) * self.map_res

        # Rotate ENU robot frame  (forward = robot heading)
        robot_fwd  =  d_east * cos_y + d_north * sin_y
        robot_left = -d_east * sin_y + d_north * cos_y

        # m = c² · (a − b·d)
        #   c  = certainty value (hit count stored in map cell)
        #   d  = Euclidean distance from robot to cell (metres)
        #   a  = 1.0  (weight at d = 0)
        #   b  = a / d_max  → weight decays linearly to ~0 at sensor horizon
        dist_m = np.sqrt(d_east**2 + d_north**2)
        dist_m = np.clip(dist_m, 0.01, self.DEPTH_THRESH)

        a = 1.0
        b = a / self.DEPTH_THRESH

        certainty = np.clip(inflated[cell_i, cell_j] / MAX_CELL_WEIGHT, 0.0, 1.0)  #normalized to [0,1]
        magnitude = (certainty ** 2) * (a - b * dist_m)
        magnitude = np.clip(magnitude, 0.0, None)   # never negative

        angle_deg  = np.degrees(np.arctan2(robot_left, robot_fwd)) % 360.0
        
        # Safely bin into sectors
        sector_idx = (angle_deg / self.DEG_PER_SECT).astype(int) % self.NUM_SECTORS

        np.add.at(hist, sector_idx, magnitude)

        return hist

    # ──────────────────────────────────────────────────────────────────────────
    # Main callback
    # ──────────────────────────────────────────────────────────────────────────
    def cb(self, depth_full, compass_angle):
        #compass_angle = self.heading
        if self.origin_lat is None:
            return [0.0, 0.0, -1.0, -1.0]
        start_time = time.time()
        H, W = depth_full.shape

        # Invalid pixels NaN
        invalid_mask = (depth_full == 0) | np.isnan(depth_full)
        depth_full[invalid_mask] = np.nan

        #  Ground remova
        # ground_mask = self.get_ground_mask(depth_full)
        # if ground_mask is None:
            # fallback row-angle heuristic
        rows = (self.Y_PIXEL_OFFSET - np.arange(H, dtype=np.float32)) / self.FOCAL_LENGTH
        ground_mask = np.nan_to_num(depth_full) * rows[:, None] < self.ground_thresh

        obstacle_mask = (
            ~ground_mask         &
            ~np.isnan(depth_full) &
            (depth_full > 0.001)   &
            (depth_full < self.DEPTH_THRESH)
        )

        free_mask = (
            ~obstacle_mask         &
            ~np.isnan(depth_full)  &
            (depth_full > 0.001)   &
            (depth_full < self.DEPTH_THRESH)
        )

# memory for vfh ---------------------------------------------------------

        ry = math.radians(self.lat - self.origin_lat) * self.R
        rx = math.radians(self.lon - self.origin_lon) * self.R * math.cos(math.radians(self.origin_lat))
        
        dx = rx - self.map_center_rx
        dy = ry - self.map_center_ry

        shiftx = int(round(dx / self.map_res))
        shifty = int(round(dy / self.map_res))

        if shiftx != 0 or shifty != 0:
            new_map = np.zeros_like(self.map)
            
            # Slice shifting logic prevents wrapping obstacles to the other side
            if abs(shiftx) < self.MAP_SIZE and abs(shifty) < self.MAP_SIZE:
                if shiftx > 0:    
                    src_x, dst_x = slice(shiftx, self.MAP_SIZE), slice(0, self.MAP_SIZE - shiftx)
                elif shiftx < 0:  
                    src_x, dst_x = slice(0, self.MAP_SIZE + shiftx), slice(-shiftx, self.MAP_SIZE)
                else:             
                    src_x, dst_x = slice(None), slice(None)


                if shifty > 0:    
                    src_y, dst_y = slice(0, self.MAP_SIZE - shifty), slice(shifty, self.MAP_SIZE)
                elif shifty < 0:  
                    src_y, dst_y = slice(-shifty, self.MAP_SIZE), slice(0, self.MAP_SIZE + shifty)
                else:             
                    src_y, dst_y = slice(None), slice(None)

                new_map[dst_y, dst_x] = self.map[src_y, src_x]
            
            self.map = new_map
            
            self.map_center_rx += shiftx * self.map_res
            self.map_center_ry += shifty * self.map_res

        heading_rad = math.radians(90.0 - compass_angle)
        # Rotate vectors into world coordinates
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
        cols = (np.arange(W, dtype=np.float32) - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH
        
        # Process Obstacles
        y_obs, x_obs = np.nonzero(obstacle_mask)
        if len(x_obs) > 0:
            fwd_valid = depth_full[obstacle_mask]
            left_valid = -fwd_valid * cols[x_obs]
            
            d_east = fwd_valid * cos_h - left_valid * sin_h
            d_north = fwd_valid * sin_h + left_valid * cos_h

            # Map coordinates to grid (Rover is at 45, 45)
            cell_x_obs = (d_east / self.map_res).astype(int) + self.MAP_HALF
            cell_y_obs = (-d_north / self.map_res).astype(int) + self.MAP_HALF
          
            # Filter coordinates that fall outside the 90x90 array
            valid_cells = (cell_x_obs >= 0) & (cell_x_obs < self.MAP_SIZE) & (cell_y_obs >= 0) & (cell_y_obs < self.MAP_SIZE)
            cell_x_obs = cell_x_obs[valid_cells]
            cell_y_obs = cell_y_obs[valid_cells]
            y_obs      = y_obs[valid_cells]
            fwd_valid  = fwd_valid[valid_cells] 
            
        else:
            cell_x_obs = np.array([], dtype=int)
            cell_y_obs = np.array([], dtype=int)
        
        # Process Free Space
        y_free, x_free = np.nonzero(free_mask)
        if len(x_free) > 0:
            fwd_free  = depth_full[free_mask]
            left_free = -fwd_free * cols[x_free]

            d_east_free  = fwd_free * cos_h - left_free * sin_h
            d_north_free = fwd_free * sin_h + left_free * cos_h

            cell_x_free = (d_east_free / self.map_res).astype(int) + self.MAP_HALF
            cell_y_free = (-d_north_free / self.map_res).astype(int) + self.MAP_HALF

            valid_free = (
                (cell_x_free >= 0) & (cell_x_free < self.MAP_SIZE) &
                (cell_y_free >= 0) & (cell_y_free < self.MAP_SIZE)
            )
            cell_x_free = cell_x_free[valid_free]
            cell_y_free = cell_y_free[valid_free]
        else:
            cell_x_free = np.array([], dtype=int)
            cell_y_free = np.array([], dtype=int)

        # Collect all cells being touched this frame
        touched_y = np.concatenate([cell_y_obs, cell_y_free])
        touched_x = np.concatenate([cell_x_obs, cell_x_free])
        
        # zero them all first wipes old value before this frame writes
        if len(touched_x) > 0:
            self.map[touched_y, touched_x] = 0.0
            self.elevation_map[touched_y, touched_x] = self.ground_thresh
        

        
        # Now accumulate obstacle hits into the clean cells
        if len(cell_x_obs) > 0:
            # Passing 1.0 means every pixel hit adds +1 to that cell's counter
            pixel_weights = fwd_valid ** 2               # Z² weighting to cancel out distance difference in hits
            np.add.at(self.map, (cell_y_obs, cell_x_obs), pixel_weights)

            row_angles = (self.Y_PIXEL_OFFSET - y_obs.astype(np.float32)) / self.FOCAL_LENGTH
            cam_y = fwd_valid * row_angles   # height in camera frame, upward = positive

            # Store max height per cell
            np.maximum.at(self.elevation_map, (cell_y_obs, cell_x_obs), cam_y)
        # free cells stay 0 

        # the map is generated and also shifts depending on where we moved
        # generate histogram based on the map instead of doing via image now
        raw_hist = self.map_to_histogram(compass_angle=compass_angle)


        window = np.array([1.0, 2.0, 3.0, 2.0, 1.0])
        window /= window.sum()
        
        pad_size = len(window) // 2
        padded_hist = np.pad(raw_hist, pad_size, mode='wrap')
        hist_smooth = np.convolve(padded_hist, window, mode='valid')

        # Hysteresis thresholding
        for i in range(self.NUM_SECTORS):
            if hist_smooth[i] > self.T_HIGH:
                self.sector_state[i] = 1   # blocked
            elif hist_smooth[i] < self.T_LOW:
                self.sector_state[i] = 0   # open

        print("HIST_MAX:", hist_smooth.max(), "HIST_MEAN_NONZERO:", hist_smooth[hist_smooth > 0].mean())
        # Valley detection
        valleys   = []
        in_valley = False
        v_start   = 0

        # Extend array to detect valleys crossing the 0/360 boundary
        extended_state = np.concatenate([self.sector_state, self.sector_state])
        
        for i in range(2 * self.NUM_SECTORS):
            if extended_state[i] == 0 and not in_valley:
                in_valley = True
                v_start   = i
            elif extended_state[i] == 1 and in_valley:
                in_valley = False
                v_end = i - 1
                if (v_end - v_start) >= self.MIN_VALLEY_SECTS:
                    # Only append if the valley started in the original 360 degrees
                    if v_start < self.NUM_SECTORS: 
                        valleys.append((v_start, v_end))

        # Edge case: The entire 360 space is open
        if in_valley and v_start == 0 and np.sum(self.sector_state) == 0:
            valleys = [(0, self.NUM_SECTORS - 1)]

        #Target bearing
        target_gps        = (43.0724831900561, -89.41245993537561) 
        rover_gps         = (self.lat, self.lon)
        bearing_to_target = self.compute_bearing(rover_gps, target_gps)
        
        # Calculate target vector in ENU, then rotate to robot frame
        yaw_rad = math.radians(90.0 - compass_angle)
        tx = math.cos(math.radians(90.0 - bearing_to_target))
        ty = math.sin(math.radians(90.0 - bearing_to_target))
        
        r_fwd  = tx * math.cos(yaw_rad) + ty * math.sin(yaw_rad)
        r_left = -tx * math.sin(yaw_rad) + ty * math.cos(yaw_rad)
        
        # Target angle mapping perfectly matches the histogram (0 = Front)
        target_angle_deg = np.degrees(math.atan2(r_left, r_fwd)) % 360.0
        target_sector    = int(target_angle_deg / self.DEG_PER_SECT) % self.NUM_SECTORS

        # Cost-based Valley Selection
        best_steering_sector = None
        best_cost            = float('inf')

        for v_start, v_end in valleys:
            size = v_end - v_start + 1
            candidates = []
            
            if size > self.SMAX:
                margin = self.SMAX // 2
                candidates.extend([v_start + margin, v_end - margin])
                if (target_sector - v_start) % self.NUM_SECTORS <= size:
                    candidates.append(target_sector)
            else:
                candidates.append((v_start + v_end) // 2)

            for c in candidates:
                c_norm = c % self.NUM_SECTORS
                cost = 5.0 * self.angular_dist(c_norm, target_sector, self.NUM_SECTORS) + \
                       1.0 * self.angular_dist(c_norm, 0, self.NUM_SECTORS)
                       
                if cost < best_cost:
                    best_cost = cost
                    best_steering_sector = c_norm


        is_recovery = False

        if best_steering_sector is None:
            # ENTIRE 360 map is blocked. We are truly stuck.
            is_recovery = True
            self.recovery_frame_count += 1
            error_deg = 45.0 # Force a spin to scan
            
        else:
            self.recovery_frame_count = 0
            # Convert chosen sector back to an angle in [-180, 180] for steering
            best_angle_deg = best_steering_sector * self.DEG_PER_SECT
            if best_angle_deg > 180.0:
                best_angle_deg -= 360.0
                
            best_theta = math.radians(best_angle_deg)

            if self.previous_best_theta is None:
                smoothed_theta = best_theta
            else:
                # Find the shortest angular difference (handles the -180/180 wrap-around)
                diff = (best_theta - self.previous_best_theta + math.pi) % (2 * math.pi) - math.pi
                smoothed_theta = self.previous_best_theta + (self.EWA_ALPHA * diff)
                
                # Re-normalize to [-pi, pi]
                smoothed_theta = (smoothed_theta + math.pi) % (2 * math.pi) - math.pi
                
            self.previous_best_theta = smoothed_theta
            
            error_deg = math.degrees(smoothed_theta)

        if self.is_aligning:
            # If we are currently turning, keep turning until we are highly accurate 
            if abs(error_deg) <= 3.0:
                self.is_aligning = False
        else:
            # If we are driving, allow some slop. Only stop to fix it if we drift past 8°
            if abs(error_deg) > 8.0:
                self.is_aligning = True

        if self.is_aligning:
            forward_speed = 0.0
            turn_speed = max(min(abs(error_deg) * self.kP, 1.0), 0.2)
        else:
            # Pure Forward Drive
            forward_speed = self.MAX_FORWARD
            turn_speed = -1.0

        #print(f"Aligning: {self.is_aligning} | error: {error_deg:.1f}°  fwd: {forward_speed:.2f}  turn: {turn_speed:.2f}")


        # ── Visualization ──────────────────────────────────────────────
        if self.debug:
            depth_vis = cv2.normalize(
                np.nan_to_num(depth_full), None, 0, 255, cv2.NORM_MINMAX
            ).astype(np.uint8)
            depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

            # ground — blue
            depth_vis[ground_mask] = [255, 80, 0]
            prev_s = int((-np.degrees(np.arctan((0 - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH)) % 360) / self.DEG_PER_SECT) % self.NUM_SECTORS
            # Draw sector borders cleanly without looping over every pixel
            half_sectors = int((self.FOV_DEG / 2) / self.DEG_PER_SECT)
            for s_offset in range(-half_sectors, half_sectors + 1):
                s = (-s_offset) % self.NUM_SECTORS
                
                # Project the sector boundary angle into a pixel column
                x_col = int(math.tan(math.radians((s_offset - 0.5) * self.DEG_PER_SECT)) * self.FOCAL_LENGTH + self.X_PIXEL_OFFSET)
                
                if 0 <= x_col < W:
                    color = (0, 0, 200) if self.sector_state[s] == 1 else (0, 200, 0)
                    cv2.line(depth_vis, (x_col, 0), (x_col, H - 1), color, 1)
            # steering line — purple
            if not is_recovery and abs(smoothed_theta) < math.radians(self.FOV_DEG / 2):
                chosen_pixel = int(math.tan(-smoothed_theta) * self.FOCAL_LENGTH + self.X_PIXEL_OFFSET)
                chosen_pixel = max(0, min(W - 1, chosen_pixel))
                cv2.line(depth_vis, (chosen_pixel, 0), (chosen_pixel, H - 1), (203, 192, 255), 3)

            # target bearing line — yellow
            target_angle_rad_viz = math.radians(target_angle_deg)
            if target_angle_rad_viz > math.pi: 
                target_angle_rad_viz -= 2 * math.pi
            
            if abs(target_angle_rad_viz) < math.radians(self.FOV_DEG / 2):
                target_pixel = int(math.tan(-target_angle_rad_viz) * self.FOCAL_LENGTH + self.X_PIXEL_OFFSET)
                target_pixel = max(0, min(W - 1, target_pixel))
                cv2.line(depth_vis, (target_pixel, 0), (target_pixel, H - 1), (0, 255, 255), 2)

            if is_recovery:
                cv2.putText(depth_vis, f"RECOVER",
                            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)


            # magnitude print
            #print("hist:", " ".join(f"[{s:02d}]{'X' if self.sector_state[s] else 'O'}{hist_smooth[s]:.2f}"for s in range(self.NUM_SECTORS)))
            # ── In-view histogram bar chart ───────────────────────────────────
            BAR_MAX_H = 80
            BAR_BASE  = H - 4
            scale     = BAR_MAX_H / max(hist_smooth.max(), self.T_HIGH * 1.5, 0.001)

            cv2.rectangle(depth_vis, (0, H - BAR_MAX_H - 10), (W, H), (15, 15, 15), -1)

            t_high_y = BAR_BASE - int(self.T_HIGH * scale)
            t_low_y  = BAR_BASE - int(self.T_LOW  * scale)
            cv2.line(depth_vis, (0, t_high_y), (W, t_high_y), (0, 50, 220), 1)
            cv2.line(depth_vis, (0, t_low_y),  (W, t_low_y),  (0, 200, 255), 1)
            cv2.putText(depth_vis, f"T_H={self.T_HIGH:.2f}", (4, t_high_y - 3),
                        cv2.FONT_HERSHEY_PLAIN, 0.8, (0, 50, 220), 1)
            cv2.putText(depth_vis, f"T_L={self.T_LOW:.2f}",  (4, t_low_y  - 3),
                        cv2.FONT_HERSHEY_PLAIN, 0.8, (0, 200, 255), 1)

            half_sectors = int((self.FOV_DEG / 2) / self.DEG_PER_SECT)
            for s_offset in range(-half_sectors, half_sectors + 1):
                s = (-s_offset) % self.NUM_SECTORS  # <--- CHANGED to -s_offset

                # Project sector edges into pixel columns using the same math as the camera
                x_left  = int(math.tan(math.radians((s_offset - 0.5) * self.DEG_PER_SECT))
                            * self.FOCAL_LENGTH + self.X_PIXEL_OFFSET)
                # ... rest of the code remains exactly the same ...
                x_right = int(math.tan(math.radians((s_offset + 0.5) * self.DEG_PER_SECT))
                            * self.FOCAL_LENGTH + self.X_PIXEL_OFFSET)
                x_left  = max(0, x_left)
                x_right = min(W - 1, x_right)

                bar_h = int(hist_smooth[s] * scale)

                # Three zones based on raw value — not sector_state which lags due to hysteresis
                if hist_smooth[s] > self.T_HIGH:
                    color = (0, 50, 220)    # red   — would be blocked
                elif hist_smooth[s] > self.T_LOW:
                    color = (0, 165, 255)   # orange — hysteresis zone
                else:
                    color = (30, 180, 60)   # green  — open

                cv2.rectangle(depth_vis,
                            (x_left + 1, BAR_BASE - bar_h),
                            (x_right - 1, BAR_BASE),
                            color, -1)

                if hist_smooth[s] > self.T_LOW * 0.3:
                    cv2.putText(depth_vis, f"{hist_smooth[s]:.2f}",
                                (x_left + 2, BAR_BASE - bar_h - 3),
                                cv2.FONT_HERSHEY_PLAIN, 0.65, (200, 200, 200), 1)
            if not self.onRover:
                cv2.imshow("VFH Obstacle Avoidance", depth_vis)
                self.visualize_map(compass_angle)
                #self.visualize_elevation_map()
                cv2.waitKey(1) 
            else:
                try:
                    ret, buffer = cv2.imencode(
                        '.jpg', depth_vis, [int(cv2.IMWRITE_JPEG_QUALITY), 30]
                    )
                    if ret:
                        self.socket.send(base64.b64encode(buffer.tobytes()))
                except Exception:
                    pass
        
        elapsed = time.time() - start_time
        print("time ===================", elapsed)

        if is_recovery:
            # Blind spin  to gather map data
            return [0.0, 0.0, 0.7, -1.0]

        # Positive error_deg means the target is to the LEFT.
        if error_deg > 0:
            return [forward_speed, 0.0, turn_speed, -1.0]  # Turn Left
        else:
            return [forward_speed, 0.0, -1.0, turn_speed]  # Turn Right

    @staticmethod
    def compute_bearing(p1, p2):
        lat1 = math.radians(p1[0]);  lon1 = math.radians(p1[1])
        lat2 = math.radians(p2[0]);  lon2 = math.radians(p2[1])
        dlon = lon2 - lon1
        x    = math.sin(dlon) * math.cos(lat2)
        y    = (math.cos(lat1) * math.sin(lat2)
                - math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        return (math.degrees(math.atan2(x, y)) + 360) % 360
    
    def visualize_map(self, compass_angle: float) -> None:
        # Build background cache on first call
        if not hasattr(self, '_map_bg'):
            self._build_map_background()

        CELL_PX = self._map_CELL_PX
        PAD     = self._map_PAD
        SZ      = self._map_SZ
        MID     = PAD + self.MAP_HALF * CELL_PX

        canvas = self._map_bg.copy()   # fast — just a memcpy

        # ── Cell image (already vectorized, unchanged) ────────────────────────
        MAX_CELL_WEIGHT = (self.map_res * self.FOCAL_LENGTH) ** 2
        OBJ_DENSITY     = 0.5
        BUSH_THRESH     = MAX_CELL_WEIGHT * OBJ_DENSITY

        filtered  = np.where(self.map >= BUSH_THRESH, self.map, 0.0).astype(np.float32)
        inflated  = getattr(self, 'inflated', filtered)   # ← now actually populated

        raw_norm  = np.clip(self.map / (MAX_CELL_WEIGHT * 1.5), 0.0, 1.0)
        inf_mask  = (inflated >= BUSH_THRESH) & (filtered < BUSH_THRESH)
        obs_mask  = filtered >= BUSH_THRESH
        bush_mask = (self.map > 0.01) & (self.map < BUSH_THRESH) & ~inf_mask

        cell_img       = np.full((self.MAP_SIZE, self.MAP_SIZE, 3), (25, 25, 35), dtype=np.uint8)
        cell_img[bush_mask] = (30, 70, 70)
        cell_img[inf_mask]  = (40, 30, 100)

        r_ch = (40  + 186 * raw_norm).astype(np.uint8)
        g_ch = (30  +  45 * raw_norm).astype(np.uint8)
        b_ch = (60  +  14 * raw_norm).astype(np.uint8)
        cell_img[obs_mask, 0] = b_ch[obs_mask]
        cell_img[obs_mask, 1] = g_ch[obs_mask]
        cell_img[obs_mask, 2] = r_ch[obs_mask]

        if hasattr(self, 'debug_removed_slopes'):
            # Paint removed gentle slopes bright Magenta (BGR format)   
            cell_img[self.debug_removed_slopes] = (255, 0, 255)

        scaled = cv2.resize(cell_img, (SZ, SZ), interpolation=cv2.INTER_NEAREST)
        canvas[PAD:PAD+SZ, PAD:PAD+SZ] = scaled

        # ── Sector overlay — VECTORIZED, replaces 90x fillPoly + line loops ──
        # Rotate precomputed angle grid so 0° aligns with current compass heading
        rotated_angle = (self._sector_angle_deg - compass_angle + 360) % 360
        sector_idx    = (rotated_angle / self.DEG_PER_SECT).astype(int) % self.NUM_SECTORS
        in_range      = self._sector_dist_m <= self.DEPTH_THRESH
        blocked       = self.sector_state[sector_idx].astype(bool)

        # Build a sector colour image at grid resolution, then resize
        sector_img = np.zeros((self.MAP_SIZE, self.MAP_SIZE, 3), dtype=np.uint8)
        sector_img[in_range &  blocked] = (40, 40, 180)    # red-ish blocked
        sector_img[in_range & ~blocked] = (30, 140, 55)    # green open

        sector_scaled = cv2.resize(sector_img, (SZ, SZ), interpolation=cv2.INTER_NEAREST)

        # Blend only the in-range pixels (avoids blending the whole canvas)
        in_range_scaled = cv2.resize(
            in_range.astype(np.uint8) * 255, (SZ, SZ), interpolation=cv2.INTER_NEAREST
        ).astype(bool)
        roi = canvas[PAD:PAD+SZ, PAD:PAD+SZ]
        roi[in_range_scaled] = (
            roi[in_range_scaled].astype(np.float32) * 0.70
            + sector_scaled[in_range_scaled].astype(np.float32) * 0.30
        ).astype(np.uint8)

        # Sector boundary lines — still individual calls but only ~1° borders matter;
        # draw just the RAY_LEN_PX line per boundary using precomputed sin/cos
        RAY_LEN_PX = int(self.DEPTH_THRESH / self.map_res) * CELL_PX
        sector_angles_rad = np.radians(
            compass_angle + (np.arange(self.NUM_SECTORS) - 0.5) * self.DEG_PER_SECT
        )
        tips = np.stack([
            (MID + (np.sin(sector_angles_rad) * RAY_LEN_PX)).astype(int),
            (MID - (np.cos(sector_angles_rad) * RAY_LEN_PX)).astype(int),
        ], axis=1)
        for tip in tips:
            cv2.line(canvas, (MID, MID), tuple(tip), (55, 55, 75), 1, cv2.LINE_AA)

        cv2.circle(canvas, (MID, MID), RAY_LEN_PX, (70, 70, 90), 1, cv2.LINE_AA)

        # ── Robot + heading arrow (cheap, unchanged) ──────────────────────────
        h_rad = math.radians(compass_angle)
        tip   = (int(MID + math.sin(h_rad)*50), int(MID - math.cos(h_rad)*50))
        r_px  = int(0.5 / self.map_res) * CELL_PX
        cv2.circle(canvas, (MID, MID), r_px, (55, 100, 180), 1, cv2.LINE_AA)
        cv2.circle(canvas, (MID, MID), 5,    (55, 138, 221), -1, cv2.LINE_AA)
        cv2.arrowedLine(canvas, (MID, MID), tip, (29, 158, 117), 2,
                        cv2.LINE_AA, tipLength=0.25)

        cv2.imshow("Occupancy Map", canvas)

    def visualize_elevation_map(self) -> None:
        # Build background cache on first call to prevent recreating large static elements
        if not hasattr(self, '_elev_bg_canvas'):
            self._build_elev_bg()

        # Ultra-fast memory copy of the pre-rendered UI
        canvas = self._elev_bg_canvas.copy()

        CELL_PX = self._elev_CELL_PX
        PAD     = self._elev_PAD
        SZ      = self._elev_SZ
        BAR_X   = self._elev_BAR_X
        BAR_W   = self._elev_BAR_W
        BAR_H   = self._elev_BAR_H

        elev         = self.elevation_map
        populated    = elev > -np.inf
        is_populated = populated.any() # Cache the boolean result so it's not run multiple times

        if is_populated:
            elev_pop = elev[populated]
            e_min = float(elev_pop.min())
            e_max = float(elev_pop.max())
        else:
            e_min, e_max = 0.0, 1.0
            
        e_range = max(e_max - e_min, 0.001)

        # ── Build cell image ───────────────────────────────────────────────
        cell_img = np.full((self.MAP_SIZE, self.MAP_SIZE, 3), (25, 25, 35), dtype=np.uint8)

        if is_populated:
            norm     = np.clip((elev - e_min) / e_range, 0.0, 1.0)
            norm_u8  = (norm * 255).astype(np.uint8)
            colored  = cv2.applyColorMap(norm_u8, cv2.COLORMAP_TURBO)
            cell_img[populated] = colored[populated]

        scaled = cv2.resize(cell_img, (SZ, SZ), interpolation=cv2.INTER_NEAREST)
        canvas[PAD : PAD + SZ, PAD : PAD + SZ] = scaled

        # ── Grid lines (Cheap enough to draw dynamically over the map) ─────────
        for i in range(0, self.MAP_SIZE + 1, 20):
            px = PAD + i * CELL_PX
            cv2.line(canvas, (px, PAD), (px, PAD + SZ), (40, 40, 60), 1)
            cv2.line(canvas, (PAD, px), (PAD + SZ, px), (40, 40, 60), 1)

        # ── Overlay center marker ─────────────────────────────────────────
        MID = PAD + self.MAP_HALF * CELL_PX
        cv2.circle(canvas, (MID, MID), 5, (55, 138, 221), -1, cv2.LINE_AA)

        # ── Dynamic Text and Markers ──────────────────────────────────────
        cv2.putText(canvas, f"{e_max:.2f}m", (BAR_X, PAD - 4),
                    cv2.FONT_HERSHEY_PLAIN, 0.7, (200, 200, 200), 1)
        cv2.putText(canvas, f"{e_min:.2f}m", (BAR_X, PAD + BAR_H + 10),
                    cv2.FONT_HERSHEY_PLAIN, 0.7, (200, 200, 200), 1)

        CHASSIS_LIMIT_M = 0.35
        limit_norm = np.clip((CHASSIS_LIMIT_M - e_min) / e_range, 0.0, 1.0)
        limit_y    = PAD + int((1.0 - limit_norm) * BAR_H)
        cv2.line(canvas, (BAR_X - 2, limit_y), (BAR_X + BAR_W + 2, limit_y), (255, 255, 255), 1)
        cv2.putText(canvas, "35cm", (BAR_X + BAR_W + 3, limit_y + 4),
                    cv2.FONT_HERSHEY_PLAIN, 0.65, (200, 200, 200), 1)

        cv2.imshow("Elevation Map", canvas)

def quaternion_to_yaw(rv_x, rv_y, rv_z, rv_w):
    siny_cosp = 2 * (rv_w * rv_z + rv_x * rv_y)
    cosy_cosp = 1 - 2 * (rv_y * rv_y + rv_z * rv_z)
    return (math.degrees(math.atan2(siny_cosp, cosy_cosp)) + 360) % 360



with dai.Pipeline() as pipeline:
    monoLeft  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo    = pipeline.create(dai.node.StereoDepth)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
    stereo.setOutputSize(1280, 720)

    config = stereo.initialConfig
    config.postProcessing.median                   = dai.MedianFilter.KERNEL_5x5
    config.postProcessing.thresholdFilter.maxRange = 7500
    config.setConfidenceThreshold(0)
    #config.setSubpixel(True)
    config.setExtendedDisparity(False)
    config.setLeftRightCheck(True)

    # device = pipeline.getDefaultDevice()
    # cali_data = device.readCalibration()
    # intrinsics = cali_data.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 1280, 720)
    # print(intrinsics)

    monoLeftOut  = monoLeft.requestOutput((1280, 720))
    monoRightOut = monoRight.requestOutput((1280, 720))
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    stereoOut = stereo.depth.createOutputQueue(maxSize=1, blocking=False)

    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ARVR_STABILIZED_ROTATION_VECTOR, 100)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)
    imuQueue = imu.out.createOutputQueue(maxSize=1, blocking=False)

    obj         = SectorDepthClassifier()
    rclpy.init()
    swerve_node = SwervePublisher()

    pipeline.start()
    current_heading = 0.0

    while pipeline.isRunning():
        rclpy.spin_once(swerve_node, timeout_sec=0.0)

        # obj.lat        = swerve_node.lat
        # obj.lon        = swerve_node.lon
        # obj.origin_lat = swerve_node.origin_lat
        # obj.origin_lon = swerve_node.origin_lon
        # obj.waypoint = swerve_node.curr_waypoint
        # obj.heading = swerve_node.heading

        imuData = imuQueue.tryGet()
        if imuData:
            rv              = imuData.packets[-1].rotationVector
            current_heading = (-quaternion_to_yaw(rv.i, rv.j, rv.k, rv.real)) % 360

        stereoFrame = stereoOut.get()

        depth      = stereoFrame.getCvFrame().astype(np.float32) / 1000.0
        swerve_cmd = obj.cb(depth, current_heading)
        swerve_node.send(swerve_cmd)

    pipeline.stop()

cv2.destroyAllWindows()