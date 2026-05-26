import cv2
import numpy as np
import time
import math
import zmq
import base64
from collections import deque


class SectorDepthClassifier():
    def __init__(self):
        self.debug   = True
        self.onRover = False

        # ── ZMQ video stream ───────────────────────────────────────────────
        self.context = zmq.Context()
        self.socket  = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.bind("tcp://*:5555")
        print("Video streamer ready on port 5555")
        
        SCALE = 2.0

        # ── Camera intrinsics ──────────────────────────────────────────────
        self.X_PIXEL_OFFSET = np.float32(643.2372 / SCALE)
        self.Y_PIXEL_OFFSET = np.float32(367.1311 / SCALE)
        self.FOCAL_LENGTH   = np.float32(568.15 / SCALE)
        self.W = int(1280 / SCALE)
        self.H = int(720 / SCALE)

        # ── Obstacle avoidance params ──────────────────────────────────────
        self.DEPTH_THRESH = np.float32(4.5)   # metres — beyond this = ignore
        self.MAX_FORWARD  = 1.0
        self.kP           = 0.015 # Bump up angular speed scale factor

        # ── VFH sector setup ───────────────────────────────────────────────
        self.DEG_PER_SECT = 4.0
        self.FOV_DEG      = 2 * np.degrees(np.arctan(self.W / (2 * self.FOCAL_LENGTH)))
        self.NUM_SECTORS  = int(360 / self.DEG_PER_SECT)

        # ── VFH histogram state ────────────────────────────────────────────
        self.sector_state    = np.zeros(self.NUM_SECTORS, dtype=int)
        self.hist_persistent = np.zeros(self.NUM_SECTORS)
        self.T_HIGH          = 12
        self.T_LOW           = 8
        self.MIN_VALLEY_SECTS = 1
        self.SMAX             = 4

        # ── Steering smoothing ─────────────────────────────────────────────
        self.previous_best_theta = None
        self.EWA_ALPHA           = 0.85

        # ── Recovery state ─────────────────────────────────────────────────
        self.recovery_frame_count = 0

        # GPS antenna offset from rover center, in robot body frame (meters)
        # Positive = forward of center, positive left_m = left of center
        self.GPS_FWD_M  =  0.05
        self.GPS_LEFT_M =  0.4518  

        # Camera position offset from rover center, in robot body frame (meters)
        self.CAM_FWD_M  =  0.2794  
        self.CAM_LEFT_M =  0.00  

        self.lat = 0
        self.lon = 0
        self.inflated = None
        self.origin_lat = None
        self.origin_lon = None

        self.R = 6378137.0

        self.prev_rx = 0.0
        self.prev_ry = 0.0

        self.ground_thresh = -0.40 # Maybe -0.3 for going over small obstacles? TUNE : 0.55 away from ground - 0.15 for 3/4 of a wheel

        self.map_res = 0.1
        
        self.MAP_SIZE = 90  
        self.MAP_HALF = self.MAP_SIZE // 2 
        self.map = np.zeros((self.MAP_SIZE, self.MAP_SIZE), dtype=np.float32)
        self.elevation_map = np.full((self.MAP_SIZE, self.MAP_SIZE), self.ground_thresh, dtype=np.float32)
        self.map_center_rx = 0.0
        self.map_center_ry = 0.0

        self.heading = None
        self.waypoint = (None, None)

        self.ROBOT_RADIUS_M = 0.5
        r = int(self.ROBOT_RADIUS_M / self.map_res)
        kernel_size = 2 * r + 1
        self.dilation_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

        self.is_aligning = False
        self.debug_removed_slopes = None


        self.rows = (self.Y_PIXEL_OFFSET - np.arange(self.H, dtype=np.float32)) / self.FOCAL_LENGTH
        self.cols = (np.arange(self.W, dtype=np.float32) - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH

        self.frame_times = deque(maxlen=30)

        _r = int(self.ROBOT_RADIUS_M / self.map_res)
        _yy, _xx = np.ogrid[0:self.MAP_SIZE, 0:self.MAP_SIZE]
        self.footprint_mask = (_xx - self.MAP_HALF)**2 + (_yy - self.MAP_HALF)**2 <= _r**2

        self.rgb_frame = None
        self.EXGR_THRESHOLD = -0.13
    
    def _vegetation_mask(self, bgr_frame: np.ndarray) -> np.ndarray:
        f = bgr_frame.astype(np.float32)
        B, G, R = f[:,:,0], f[:,:,1], f[:,:,2]

        total = R + G + B + 1e-6
        r, g, b = R / total, G / total, B / total

        ExG  = 2.0 * g - r - b
        ExR  = 1.4 * r - g
        ExGR = ExG - ExR

        return ExGR > self.EXGR_THRESHOLD

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

    # ──────────────────────────────────────────────────────────────────────────
    # Main callback
    # ──────────────────────────────────────────────────────────────────────────
    def cb(self, depth_full, compass_angle):
        compass_angle = self.heading
        if compass_angle is None or self.origin_lat is None or depth_full is None:
            return [0.0, 0.0, -1.0, -1.0]
        
        self.frame_count = getattr(self, "frame_count", 0) + 1
        if self.frame_count < 20:
            return [0.0, 0.0, -1.0, -1.0]

        start_time = time.perf_counter()

        if depth_full.shape != (self.H, self.W):
            depth_full = cv2.resize(depth_full, (self.W, self.H))
        # Update map position based on GPS
        self._shift_local_map()

        # Process point cloud and update occupancy map for objects
        ground_mask, bush_mask = self._process_depth_to_map(depth_full, compass_angle)
        
        # Generate VFH Histogram and find safe valleys
        hist_smooth, valleys = self._find_valleys(compass_angle)
        
        # Calculate best steering sector and commands
        swerve_cmd, steering_context = self._calculate_steering(valleys, compass_angle)
        # ── Visualization ──────────────────────────────────────────────
        t0 = time.perf_counter()
        if self.debug:
            self._render_visualization(depth_full, ground_mask, compass_angle, hist_smooth, steering_context, bush_mask)
        print("viz time = ", (time.perf_counter() - t0) * 1000)
        elapsed = time.perf_counter() - start_time
        self.frame_times.append(elapsed)
        avg_time = sum(self.frame_times) / len(self.frame_times)
        print(f"Current Frame: {elapsed * 1000:.1f} ms | 30-Frame Avg: {avg_time * 1000:.1f} ms")

        return swerve_cmd

    def _shift_local_map(self):
        # RTK Jitter can potentially vibrate back and forth, smearing obstacles and obstructing future stuff
        ry = math.radians(self.lat - self.origin_lat) * self.R
        rx = math.radians(self.lon - self.origin_lon) * self.R * math.cos(math.radians(self.origin_lat))

        dx = rx - self.map_center_rx
        dy = ry - self.map_center_ry

        shiftx = int(round(dx / self.map_res))
        shifty = int(round(dy / self.map_res))

        if shiftx != 0 or shifty != 0:
            new_map = np.zeros_like(self.map)
            new_elev = np.full_like(self.elevation_map, self.ground_thresh)
            
            # Slice shifting logic prevents wrapping obstacles to the other side
            # Can be problematic if we lose RTK fix, cuz its like rover teleported.
            if abs(shiftx) < self.MAP_SIZE and abs(shifty) < self.MAP_SIZE:
                
                src_x = slice(shiftx, self.MAP_SIZE) if shiftx > 0 else slice(0, self.MAP_SIZE + shiftx) if shiftx < 0 else slice(None)
                dst_x = slice(0, self.MAP_SIZE - shiftx) if shiftx > 0 else slice(-shiftx, self.MAP_SIZE) if shiftx < 0 else slice(None)
                src_y = slice(0, self.MAP_SIZE - shifty) if shifty > 0 else slice(-shifty, self.MAP_SIZE) if shifty < 0 else slice(None)
                dst_y = slice(shifty, self.MAP_SIZE) if shifty > 0 else slice(0, self.MAP_SIZE + shifty) if shifty < 0 else slice (None)

                new_map[dst_y, dst_x] = self.map[src_y, src_x]
                new_elev[dst_y, dst_x] = self.elevation_map[src_y, src_x]
            
            self.map = new_map
            self.elevation_map = new_elev
            self.map_center_rx += shiftx * self.map_res
            self.map_center_ry += shifty * self.map_res

    def _process_depth_to_map(self, depth_full, compass_angle):
        
        # Invalid pixels NaN
        invalid_mask = (depth_full == 0) | np.isnan(depth_full)
        depth_full[invalid_mask] = np.nan

        #  Ground removal
        with np.errstate(invalid='ignore'):
            Z = depth_full
            Y = depth_full * self.rows[:, None]
            
            # Create a 2D vertical kernel (7 rows, 1 column)
            # We divide by 3 to average the 3 active pixels on each side
            kernel = np.array([[-1], [-1], [-1], [0], [1], [1], [1]], dtype=np.float32) / 3.0

            # Apply the filter. cv2.CV_32F ensures the output is 32-bit float to handle negatives
            dZ = cv2.filter2D(Z.astype(np.float32), cv2.CV_32F, kernel)
            dY = cv2.filter2D(Y.astype(np.float32), cv2.CV_32F, kernel)

            # Compute slope
            slope_deg = np.degrees(np.arctan2(np.abs(dY), np.abs(dZ)))

            # OpenCV automatically pads the edges, so slope_deg is the exact same shape as Z and Y.
            # No complicated slicing needed for your masks.
            slope_check_mask = (Z > 0.5) & (Z < 4.5)
            ground_slope_mask = (slope_deg < 30.0) & slope_check_mask

            # ground_mask = (depth_full * self.rows[:, None]) < self.ground_thresh
            abs_ground_mask = (Y < self.ground_thresh)

            ground_mask = abs_ground_mask | ground_slope_mask
            # valid_depth implicitly excludes NaNs because NaN > 0.001 is False
            valid_depth = (depth_full > 0.001) & (depth_full < self.DEPTH_THRESH)
        
        bush_mask = self._vegetation_mask(self.rgb_frame)
        obstacle_mask = valid_depth & ~ground_mask & ~bush_mask
        free_mask = valid_depth & (ground_mask | bush_mask)
    
        heading_rad = math.radians(90.0 - compass_angle)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)

        eff_cam_fwd  = self.CAM_FWD_M  - self.GPS_FWD_M    #  0.229 m
        eff_cam_left = self.CAM_LEFT_M - self.GPS_LEFT_M    # -0.4518 m (May need changes based on GPS loc)

        cam_east  =  eff_cam_fwd * cos_h - eff_cam_left * sin_h
        cam_north =  eff_cam_fwd * sin_h + eff_cam_left * cos_h

#----------------------------------------------------------------------------------

        # # Project Obstacles
        # y_obs, x_obs = np.nonzero(obstacle_mask)
        # if len(x_obs) > 0:
        #     fwd_valid = depth_full[obstacle_mask]
        #     left_valid = -fwd_valid * self.cols[x_obs]
            
        #     d_east = fwd_valid * cos_h - left_valid * sin_h
        #     d_north = fwd_valid * sin_h + left_valid * cos_h

        #     # Map coordinates to grid (Rover is at 45, 45)
        #     cell_x_obs = ((d_east  + cam_east)  / self.map_res).astype(int) + self.MAP_HALF
        #     cell_y_obs = ((-d_north - cam_north) / self.map_res).astype(int) + self.MAP_HALF
          
        #     # Filter coordinates that fall outside the map
        #     valid_cells = (cell_x_obs >= 0) & (cell_x_obs < self.MAP_SIZE) & (cell_y_obs >= 0) & (cell_y_obs < self.MAP_SIZE)

        #     if valid_cells.any():
        #         pixel_weights = fwd_valid[valid_cells] ** 2               # Z² weighting to cancel out distance difference in hits
        #         flat_obs_idx = cell_y_obs[valid_cells] * self.MAP_SIZE + cell_x_obs[valid_cells]
        #         updates = np.bincount(flat_obs_idx, weights=pixel_weights, minlength=self.MAP_SIZE * self.MAP_SIZE)
        #         self.map += updates.reshape((self.MAP_SIZE, self.MAP_SIZE))

        #         row_angles = (self.Y_PIXEL_OFFSET - y_obs[valid_cells].astype(np.float32)) / self.FOCAL_LENGTH
        #         cam_y = fwd_valid[valid_cells] * row_angles   # height in camera frame, upward = positive
        #         cam_y = np.clip(cam_y, self.ground_thresh, 1.0)
        #         # Store max height per cell
        #         np.maximum.at(self.elevation_map, (cell_y_obs[valid_cells], cell_x_obs[valid_cells]), cam_y)
        # # free cells stay 0 
        # # Clear proven free-space directly, rather than wiping the whole cone here
        # y_free, x_free = np.nonzero(free_mask)
        # if len(x_free) > 0:
        #     fwd_free = depth_full[free_mask]
        #     left_free = -fwd_free * self.cols[x_free]
        #     d_east_f = fwd_free * cos_h - left_free * sin_h
        #     d_north_f = fwd_free * sin_h + left_free * cos_h

        #     cell_x_f = ((d_east_f + cam_east) / self.map_res).astype(int) + self.MAP_HALF
        #     cell_y_f = ((-d_north_f - cam_north / self.map_res)).astype(int) + self.MAP_HALF

        #     valid_f = (cell_x_f >= 0) & (cell_x_f < self.MAP_SIZE) & (cell_y_f >= 0) & (cell_y_f < self.MAP_SIZE)

        #     if valid_f.any():
        #         # Empty space decremented smoothly, so noise is not a concern, but strong obstacles will decay as we drive over them
        #         flat_free_idx = cell_y_f[valid_f] * self.MAP_SIZE + cell_x_f[valid_f]
        #         free_weights = fwd_free[valid_f] ** 2
        #         free_updates = np.bincount(flat_free_idx, weights=free_weights, minlength=self.MAP_SIZE * self.MAP_SIZE)

        #         # Reduce confidence of mapped obstacles when we see clear ground
        #         self.map -= (free_updates.reshape((self.MAP_SIZE, self.MAP_SIZE)) * 0.75)
            
        # MAX_CELL_WEIGHT = ((self.map_res * self.FOCAL_LENGTH)) ** 2
        # self.map = np.clip(self.map * 0.997, 0.0, MAX_CELL_WEIGHT) # Slow decay so ghost noise dissappears over time
        # self.map[self.footprint_mask] = 0.0
        # self.elevation_map[self.footprint_mask] = self.ground_thresh

         # CALCULATE OBSTACLE UPDATES
        y_obs, x_obs = np.nonzero(obstacle_mask)
        if len(x_obs) > 0:
            fwd_valid = depth_full[obstacle_mask]
            left_valid = -fwd_valid * self.cols[x_obs]
            
            d_east = fwd_valid * cos_h - left_valid * sin_h
            d_north = fwd_valid * sin_h + left_valid * cos_h

            cell_x_obs = ((d_east  + cam_east)  / self.map_res).astype(int) + self.MAP_HALF
            cell_y_obs = ((-d_north - cam_north) / self.map_res).astype(int) + self.MAP_HALF
          
            valid_cells = (cell_x_obs >= 0) & (cell_x_obs < self.MAP_SIZE) & (cell_y_obs >= 0) & (cell_y_obs < self.MAP_SIZE)
            
            obs_pixel_weights = fwd_valid[valid_cells] ** 2               
            flat_obs_idx = cell_y_obs[valid_cells] * self.MAP_SIZE + cell_x_obs[valid_cells]
            
            # Generate the 2D array of obstacle additions
            obs_updates = np.bincount(flat_obs_idx, weights=obs_pixel_weights, minlength=self.MAP_SIZE * self.MAP_SIZE)
            obs_updates_2d = obs_updates.reshape((self.MAP_SIZE, self.MAP_SIZE))
            
            # Elevation mapping
            row_angles = (self.Y_PIXEL_OFFSET - y_obs[valid_cells].astype(np.float32)) / self.FOCAL_LENGTH
            cam_y = fwd_valid[valid_cells] * row_angles   
            np.maximum.at(self.elevation_map, (cell_y_obs[valid_cells], cell_x_obs[valid_cells]), cam_y)
        else:
            obs_updates_2d = np.zeros((self.MAP_SIZE, self.MAP_SIZE), dtype=np.float32)


        #free space updates
        # Create a coordinate grid in meters relative to the map center
        y_idx, x_idx = np.ogrid[0:self.MAP_SIZE, 0:self.MAP_SIZE]
        y_m = (self.MAP_HALF - y_idx) * self.map_res
        x_m = (x_idx - self.MAP_HALF) * self.map_res
        
        # Shift coordinates to be relative to the CAMERA's position
        dx = x_m - cam_east
        dy = y_m - cam_north
        dist = np.hypot(dx, dy)
        
        # Calculate angle of each cell relative to the camera optical axis
        angle_deg = np.degrees(np.arctan2(dx, dy))
        diff_deg = (angle_deg - compass_angle + 180) % 360 - 180
        
        # inside max depth and inside FOV
        SAFE_FOV_DEG = self.FOV_DEG * 0.80  # 80% to prevent edge noise from doing shit
        clear_mask = (dist <= self.DEPTH_THRESH) & (np.abs(diff_deg) <= (SAFE_FOV_DEG / 2.0))
        
        MAX_CELL_WEIGHT = (self.map_res * self.FOCAL_LENGTH) ** 2
        DECAY_AMOUNT = MAX_CELL_WEIGHT * 0.07  # Slow decay (5% of max weight per frame)
        # decay needs to happen ONLY to counter act noise there should be no decay that can clear obstacles.
        # most of our map clearing needs to happen through movement
        
        free_updates_2d = np.where(clear_mask, DECAY_AMOUNT, 0.0).astype(np.float32)

        # Obstacles win ties If a cell has an obstacle hit THIS frame, DO NOT decay it.
        free_updates_2d[obs_updates_2d > 0] = 0.0

        # Apply the updates
        self.map += obs_updates_2d
        self.map -= free_updates_2d

        # Prevent infinity and negative confidence
        MAP_CEILING = MAX_CELL_WEIGHT * 3.0  
        np.clip(self.map, 0.0, MAP_CEILING, out=self.map)

        # Elevation reset: ONLY reset height if the cell has been eroded to 0 confidence
        cleared_mask = self.map <= 0.01
        self.elevation_map[cleared_mask] = float(self.ground_thresh)

        # APPLY FOOTPRINT
        self.map[self.footprint_mask]           = 0.0
        self.elevation_map[self.footprint_mask] = float(self.ground_thresh)
        self.elevation_map[self.map < 0.05] = self.ground_thresh
        return ground_mask, bush_mask

    def _find_valleys(self, compass_angle):
        raw_hist = self.map_to_histogram(compass_angle=compass_angle)
        window = np.array([1.0, 1.25, 1.75, 1.25, 1.0])
        window /= window.sum()
        
        pad_size = len(window) // 2
        padded_hist = np.pad(raw_hist, pad_size, mode='wrap')
        hist_smooth = np.convolve(padded_hist, window, mode='valid')
        # raw_hist *= 0

        # Hysteresis thresholding
        for i in range(self.NUM_SECTORS):
            if hist_smooth[i] > self.T_HIGH:
                self.sector_state[i] = 1   # blocked
            elif hist_smooth[i] < self.T_LOW:
                self.sector_state[i] = 0   # open
                
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
        
        return hist_smooth, valleys

    def _calculate_steering(self, valleys, compass_angle):
        target_gps        = self.waypoint 
        rover_gps         = (self.lat, self.lon)
        # Target bearing is already a global compass angle (0=N, 90=E)
        bearing_to_target = self.compute_bearing(rover_gps, target_gps)
        target_sector     = int(bearing_to_target / self.DEG_PER_SECT) % self.NUM_SECTORS
        
        # Rover heading as a sector
        robot_heading_sector = int(compass_angle / self.DEG_PER_SECT) % self.NUM_SECTORS

        # Cost-based Valley Selection
        best_steering_sector = None
        best_cost            = float('inf')

        for v_start, v_end in valleys:
            size = v_end - v_start + 1
            margin = self.SMAX // 2
            candidates = [v_start + margin, v_end - margin, target_sector] if size > self.SMAX else [(v_start + v_end) // 2]
            
            for c in candidates:
                c_norm = c % self.NUM_SECTORS

                if c == target_sector and (target_sector - v_start) % self.NUM_SECTORS > size:
                    continue

                cost = 5.0 * self.angular_dist(c_norm, target_sector, self.NUM_SECTORS) + \
                       1.0 * self.angular_dist(c_norm, robot_heading_sector, self.NUM_SECTORS)
                       
                if cost < best_cost:
                    best_cost, best_steering_sector = cost, c_norm

        # Potentially going into recovery states?
        is_recovery = best_steering_sector is None
        smoothed_theta_cw = 0.0

        if is_recovery:
            self.recovery_frame_count += 1
            error_deg = 45.0 # Force a spin to scan
        else:
            self.recovery_frame_count = 0
            
            # Global angle of chosen sector
            best_angle_deg = best_steering_sector * self.DEG_PER_SECT
            
            # Difference from current heading (Clockwise diff: Positive = Target is to the Right)
            diff_deg = (best_angle_deg - compass_angle + 180) % 360 - 180
            best_theta_cw = math.radians(diff_deg)

            if self.previous_best_theta is None:
                smoothed_theta_cw = best_theta_cw
            else:
                diff = (best_theta_cw - self.previous_best_theta + math.pi) % (2 * math.pi) - math.pi
                smoothed_theta_cw = self.previous_best_theta + (self.EWA_ALPHA * diff)
                smoothed_theta_cw = (smoothed_theta_cw + math.pi) % (2 * math.pi) - math.pi
                
            self.previous_best_theta = smoothed_theta_cw
            
            # Your wheel code expects positive error_deg to turn LEFT, so we flip the sign here
            error_deg = math.degrees(-smoothed_theta_cw)

        # Remove is_aligning lock on the swerve drive
        # VFH proportional speeds again
        speed_factor = max(0.2, 1.0 - (abs(error_deg) / 45.0))
        forward_speed = self.MAX_FORWARD * speed_factor if not is_recovery else 0.0 # Only have recover state i.e no best sector found
        #kp = 0.015
        turn_speed = max(min(abs(error_deg) * self.kP, 2.0), 0.15) - 1.0 if abs(error_deg) > 2.0 else 0.0 # 2 degree deadband

        if error_deg > 0: turn_speed = turn_speed
        else: turn_speed = -turn_speed
        # if self.is_aligning:
        #     # If we are currently turning, keep turning until we are highly accurate 
        #     if abs(error_deg) <= 6.0: self.is_aligning = False
        # elif abs(error_deg) > 11.0:
        #     # If we are driving, allow some slop. Only stop to fix it if we drift past 8°
        #     self.is_aligning = True

        # if self.is_aligning:
        #     forward_speed, turn_speed = 0.0, max(min(abs(error_deg) * self.kP, 1.8), 0.42) - 1.0
        # else:
        #     # Pure Forward Drive
        #     forward_speed, turn_speed = self.MAX_FORWARD * 0.8, -1.0

        steering_context = {
            "is_recovery": is_recovery,
            "error_deg": error_deg,
            "smoothed_theta_cw": smoothed_theta_cw,
            "bearing_to_target": bearing_to_target
        }

        cmd = [0.0, 0.0, 0.7, -1.0] if is_recovery else (
            [forward_speed, 0.0, turn_speed, -1.0] if error_deg > 0 else
            [forward_speed, 0.0, -1.0, turn_speed]
        )

        print(f"Aligning: {self.is_aligning} | error: {error_deg:.1f}°")

        return cmd, steering_context
        


    def _render_visualization(self, depth_full, ground_mask, compass_angle, hist_smooth, ctx, bush_mask):
        H, W = depth_full.shape
        depth_vis = cv2.normalize(np.nan_to_num(depth_full), None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)
        # ground — blue
        depth_vis[ground_mask] = [255, 80, 0]
        depth_vis[bush_mask] = [30,255,30]

        # Draw sector borders cleanly without looping over every pixel
        half_sectors = int((self.FOV_DEG / 2) / self.DEG_PER_SECT)
        max_rad = math.radians(self.FOV_DEG / 2.0) - 0.05 # Cap the angles so they stick to the edge of the screen instead of vanishing

        # Draw Bounds
        for s_offset in range(-half_sectors, half_sectors + 1):
            s = int((compass_angle + s_offset * self.DEG_PER_SECT) / self.DEG_PER_SECT) % self.NUM_SECTORS
            # Project the sector boundary angle into a pixel column
            x_col = int(math.tan(math.radians((s_offset - 0.5) * self.DEG_PER_SECT)) * self.FOCAL_LENGTH + self.X_PIXEL_OFFSET)
            if 0 <= x_col < W:
                color = (0, 0, 200) if self.sector_state[s] == 1 else (0, 200, 0)
                cv2.line(depth_vis, (x_col, 0), (x_col, H - 1), color, 1)
        
        # steering line — purple
        if not ctx["is_recovery"]:
            viz_theta_cw = max(-max_rad, min(max_rad, ctx["smoothed_theta_cw"]))
            chosen_pixel = int(math.tan(viz_theta_cw) * self.FOCAL_LENGTH + self.X_PIXEL_OFFSET)
            chosen_pixel = max(0, min(W - 1, chosen_pixel))
            cv2.line(depth_vis, (chosen_pixel, 0), (chosen_pixel, H - 1), (203, 192, 255), 3)

        # target bearing line — yellow
        target_diff_deg = (ctx["bearing_to_target"] - compass_angle + 180) % 360 - 180
        viz_target_cw = max(-max_rad, min(max_rad, math.radians(target_diff_deg)))
        target_pixel = int(math.tan(viz_target_cw) * self.FOCAL_LENGTH + self.X_PIXEL_OFFSET)
        target_pixel = max(0, min(W - 1, target_pixel))
        cv2.line(depth_vis, (target_pixel, 0), (target_pixel, H - 1), (0, 255, 255), 2)
        
        if ctx["is_recovery"]:
            cv2.putText(depth_vis, f"RECOVER", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

        # magnitude print
        #print("hist:", " ".join(f"[{s:02d}]{'X' if self.sector_state[s] else 'O'}{hist_smooth[s]:.2f}"for s in range(self.NUM_SECTORS)))
        # ── In-view histogram bar chart ───────────────────────────────────
        BAR_MAX_H, BAR_BASE = 80, H - 4
        scale = BAR_MAX_H / max(hist_smooth.max(), self.T_HIGH * 1.5, 0.001)
        cv2.rectangle(depth_vis, (0, H - BAR_MAX_H - 10), (W, H), (15, 15, 15), -1)

        t_high_y = BAR_BASE - int(self.T_HIGH * scale)
        t_low_y  = BAR_BASE - int(self.T_LOW  * scale)
        cv2.line(depth_vis, (0, t_high_y), (W, t_high_y), (0, 50, 220), 1)
        cv2.line(depth_vis, (0, t_low_y),  (W, t_low_y),  (0, 200, 255), 1)
        cv2.putText(depth_vis, f"T_H={self.T_HIGH:.2f}", (4, t_high_y - 3),
                    cv2.FONT_HERSHEY_PLAIN, 0.8, (0, 50, 220), 1)
        cv2.putText(depth_vis, f"T_L={self.T_LOW:.2f}",  (4, t_low_y  - 3),
                    cv2.FONT_HERSHEY_PLAIN, 0.8, (0, 200, 255), 1)

        for s_offset in range(-half_sectors, half_sectors + 1):
            s = int((compass_angle + s_offset * self.DEG_PER_SECT) / self.DEG_PER_SECT) % self.NUM_SECTORS
            # Project sector edges into pixel columns using the same math as the camera
            x_left  = max(0, int(math.tan(math.radians((s_offset - 0.5) * self.DEG_PER_SECT)) * self.FOCAL_LENGTH + self.X_PIXEL_OFFSET))
            x_right = min(W-1, int(math.tan(math.radians((s_offset + 0.5) * self.DEG_PER_SECT)) * self.FOCAL_LENGTH + self.X_PIXEL_OFFSET))

            bar_h = int(hist_smooth[s] * scale)
            # Color is red, orange and green respectively
            color = (0, 50, 220) if hist_smooth[s] >  self.T_HIGH else (0, 165, 255) if hist_smooth[s] > self.T_LOW else (30, 180, 60)
            cv2.rectangle(depth_vis, (x_left + 1, BAR_BASE - bar_h), (x_right - 1, BAR_BASE), color, -1)

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

    def angular_dist(self, s1, s2, max_val):
        """Calculates shortest distance between two sectors in a circular array."""
        diff = abs(s1 - s2) % max_val
        return min(diff, max_val - diff)
    

    def remove_upslopes(self, box_size=3, smooth_size=3, slope_thresh_deg=40.0):
        """
        Slope-based obstacle removal pass. 
        Gentle slopes (< 40°) are traversable ramps -> removed from obstacle map.
        Steep slopes  (> 40°) are walls             -> kept in obstacle map.
        """
        valid_mask = self.elevation_map > -np.inf
        elev_input = np.where(valid_mask, self.elevation_map, self.ground_thresh).astype(np.float32)
        smoothed = cv2.blur(elev_input, (smooth_size, smooth_size))
        elev_for_max = np.where(valid_mask, smoothed, -1e9).astype(np.float32)
        elev_for_min = np.where(valid_mask, smoothed,  1e9).astype(np.float32)
        kernel = np.ones((box_size, box_size), dtype=np.uint8)
        local_max = cv2.dilate(elev_for_max, kernel)
        local_min = cv2.erode(elev_for_min, kernel)
        dz = local_max - local_min
        dx = (box_size - 1) * self.map_res
        tan_thresh = math.tan(math.radians(slope_thresh_deg))
        remove_mask = (dz < dx * tan_thresh) & valid_mask

        self.map[remove_mask] = 0.0

        self.debug_removed_slopes = remove_mask


    def map_to_histogram(self, compass_angle):
        hist = np.zeros(self.NUM_SECTORS)

        #self.remove_upslopes(box_size=3, smooth_size=3, slope_thresh_deg=40.0)

        MAX_CELL_WEIGHT = (self.map_res * self.FOCAL_LENGTH) ** 2  
        OBJ_DENSITY     = 0.5          
        BUSH_THRESH     = MAX_CELL_WEIGHT * OBJ_DENSITY
        
        filtered_map = np.where(self.map >= BUSH_THRESH, self.map, 0.0).astype(np.float32)
        self.inflated = cv2.dilate(filtered_map, self.dilation_kernel)
        self.inflated[self.footprint_mask] = 0.0
        inflated = self.inflated

        cell_i, cell_j = np.where(inflated > 0)
        if len(cell_i) == 0:
            return hist

        # Global offsets in metres
        d_east  = (cell_j - self.MAP_HALF).astype(np.float32) * self.map_res
        d_north = (self.MAP_HALF - cell_i).astype(np.float32) * self.map_res

        dist_m = np.clip(np.hypot(d_east, d_north), 0.01, self.DEPTH_THRESH)

        a = 1.0
        b = a / self.DEPTH_THRESH

        certainty = np.clip(inflated[cell_i, cell_j] / MAX_CELL_WEIGHT, 0.0, 1.0)
        magnitude = (certainty ** 2) * (a - b * dist_m)
        magnitude = np.clip(magnitude, 0.0, None)

        # Global compass angle (0=N, 90=E, Clockwise)
        angle_deg  = (np.degrees(np.arctan2(d_east, d_north)) + 360) % 360.0
        sector_idx = (angle_deg / self.DEG_PER_SECT).astype(int) % self.NUM_SECTORS

        hist = np.bincount(sector_idx, weights=magnitude, minlength=self.NUM_SECTORS)

        return hist

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

        # if hasattr(self, 'debug_removed_slopes'):
        #     # Paint removed gentle slopes bright Magenta (BGR format)   
        #     cell_img[self.debug_removed_slopes] = (255, 0, 255)

        scaled = cv2.resize(cell_img, (SZ, SZ), interpolation=cv2.INTER_NEAREST)
        canvas[PAD:PAD+SZ, PAD:PAD+SZ] = scaled

        # ── Sector overlay — VECTORIZED, replaces 90x fillPoly + line loops ──
        # Rotate precomputed angle grid so 0° aligns with current compass heading
        sector_idx    = (self._sector_angle_deg / self.DEG_PER_SECT).astype(int) % self.NUM_SECTORS
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
