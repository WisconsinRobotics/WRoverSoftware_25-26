import depthai as dai
import cv2
import numpy as np
import time
import math
import zmq
import base64

from ground_detect import backproject_depth_sub
from ground_detect import ransac_plane
from ground_detect import apply_plane_to_full_depth
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray
from gps_heading import HeadingVerifier


class SwervePublisher(Node):
    def __init__(self):
        super().__init__("swerve_publisher")
        self.pub = self.create_publisher(Float32MultiArray, "swerve", 10)

    def send(self, swrv):
        msg = Float32MultiArray()
        msg.data = swrv
        self.pub.publish(msg)


class GPSNode(Node):
    def __init__(self):
        super().__init__("gps_listener")
        self.latest_gps = (0, 0)
        self.create_subscription(NavSatFix, "fix", self.gps_callback, 10)

    def gps_callback(self, msg):
        self.latest_gps = (msg.latitude, msg.longitude)


class SectorDepthClassifier():
    def __init__(self):
        self.debug   = True
        self.onRover = False

        # ── ZMQ video stream ───────────────────────────────────────────────
        self.context = zmq.Context()
        self.socket  = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.bind("tcp://*:9876")
        print("Video streamer ready on port 9876")

        # ── Camera intrinsics ──────────────────────────────────────────────
        self.X_PIXEL_OFFSET = np.float32(640)
        self.Y_PIXEL_OFFSET = np.float32(360)
        self.FOCAL_LENGTH   = np.float32(563.33333)
        self.W = 1280
        self.H = 720

        # ── Obstacle avoidance params ──────────────────────────────────────
        self.DEPTH_THRESH = np.float32(4.2)   # metres — beyond this = ignore
        self.MAX_FORWARD  = 0.8
        self.kP           = 0.02

        # ── VFH sector setup ───────────────────────────────────────────────
        self.DEG_PER_SECT = 3.0
        self.FOV_DEG      = 2 * np.degrees(np.arctan(self.W / (2 * self.FOCAL_LENGTH)))
        self.NUM_SECTORS  = int(self.FOV_DEG / self.DEG_PER_SECT)

        # map every column to a sector index — computed once, reused every frame
        cols      = np.arange(self.W)
        angle_deg = np.degrees(np.arctan((cols - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH))
        self.col_to_sector = np.clip(
            ((angle_deg + self.FOV_DEG / 2) / self.DEG_PER_SECT).astype(int),
            0, self.NUM_SECTORS - 1
        )

        # expand to full image shape (H, W) — every pixel knows its sector
        self.pixel_to_sector = np.tile(self.col_to_sector, (self.H, 1))

        # total pixels per sector — denominator for density normalisation
        self.pixels_per_sector = np.bincount(
            self.pixel_to_sector.ravel(),
            minlength=self.NUM_SECTORS
        ).astype(float)
        self.pixels_per_sector[self.pixels_per_sector == 0] = 1

        # ── VFH histogram state ────────────────────────────────────────────
        self.sector_state    = np.zeros(self.NUM_SECTORS, dtype=int)
        self.hist_persistent = np.zeros(self.NUM_SECTORS)
        self.T_HIGH          = 0.15
        self.T_LOW           = 0.10
        self.HIST_ALPHA      = 0.16
        self.MIN_VALLEY_SECTS = 4
        self.SMAX             = 10

        # ── Valley commitment — prevents oscillation between nearby valleys ─
        self.committed_valley     = None
        self.VALLEY_SWITCH_THRESH = 3

        # ── Steering smoothing ─────────────────────────────────────────────
        self.previous_best_theta = None
        self.EWA_ALPHA           = 0.27

        # ── Recovery state ─────────────────────────────────────────────────
        self.recovery_direction   = None   # -1=left  +1=right  None=not recovering
        self.recovery_frame_count = 0

        # ── RANSAC ground plane cache ──────────────────────────────────────
        self.cached_n        = None
        self.cached_d        = None
        self.ransac_counter  = 0
        self.RANSAC_INTERVAL = 3

    # ──────────────────────────────────────────────────────────────────────────
    # Ground removal
    # ──────────────────────────────────────────────────────────────────────────
    def get_ground_mask(self, depth):
        self.ransac_counter += 1
        if self.cached_n is None or self.ransac_counter >= self.RANSAC_INTERVAL:
            pts_sub, _, _ = backproject_depth_sub(depth)
            if len(pts_sub) >= 100:
                success, n, d, mask_best = ransac_plane(
                    pts=pts_sub,
                    iters=300,
                    dist_thresh=0.13,
                    angle_thresh_deg=30,
                    min_inliers=150,
                )
                if success:
                    inlier_ratio = mask_best.sum() / len(pts_sub)
                    if inlier_ratio > 0.35:
                        self.cached_n       = n
                        self.cached_d       = d
                        self.ransac_counter = 0

        if self.cached_n is not None:
            return apply_plane_to_full_depth(
                depth, self.cached_n, self.cached_d, dist_thresh=0.13
            )
        return None

    # ──────────────────────────────────────────────────────────────────────────
    # Main callback
    # ──────────────────────────────────────────────────────────────────────────
    def cb(self, depth_full, compass_angle, rover_gps):
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
        ground_mask = np.nan_to_num(depth_full) * rows[:, None] < -0.3

        obstacle_mask = (
            ~ground_mask         &
            ~np.isnan(depth_full) &
            (depth_full > 0.001)   &
            (depth_full < self.DEPTH_THRESH)
        )

        # Magnitude per pixel
        magnitudes = np.zeros_like(depth_full)
        magnitudes[obstacle_mask] = (
            (self.DEPTH_THRESH - depth_full[obstacle_mask]) / self.DEPTH_THRESH
        ) ** 2

        # Accumulate into polar histogram 
        raw_hist = np.bincount(
            self.pixel_to_sector.ravel(),
            weights=magnitudes.ravel(),
            minlength=self.NUM_SECTORS
        ).astype(float)

        # normalise to obstacle density (0-1 scale)
        hist_norm = raw_hist / self.pixels_per_sector
        
        # print(f"obstacle pixels: {obstacle_mask.sum()}")
        # print(f"ground pixels: {ground_mask.sum()}")
        # print(f"nan pixels: {np.isnan(depth_full).sum()}")
        # print(f"nan%: {np.isnan(depth_full).mean():.1%}  "
        # f"close (<1.5m)%: {((depth_full>0.1)&(depth_full<1.5)).mean():.1%}")
        min_list = np.nanpercentile(depth_full, 4, axis=0)
        print("min_list = ",min_list)

        # depth map diagnostic
        valid = depth[depth > 0.01]
        # if len(valid) > 0:
        #     print(f"depth stats — "
        #         f"min: {valid.min():.2f}m  "
        #         f"max: {valid.max():.2f}m  "
        #         f"mean: {valid.mean():.2f}m  "
        #         f"median: {np.median(valid):.2f}m  "
        #         f"valid px: {len(valid)}")
            
        #     # histogram of depth values in 0.5m buckets
        #     buckets = [0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5]
        #     counts = [np.sum((valid >= buckets[i]) & (valid < buckets[i+1])) 
        #             for i in range(len(buckets)-1)]
        #     print("depth distribution:", 
        #         " | ".join(f"{buckets[i]:.1f}-{buckets[i+1]:.1f}m:{c}" 
        #                     for i, c in enumerate(counts)))

        # sectors with too few valid readings  treat as blocked
        valid_pixel_mask = ~np.isnan(depth_full) & ~ground_mask
        valid_per_sector = np.bincount(
            self.pixel_to_sector[valid_pixel_mask].ravel(),
            minlength=self.NUM_SECTORS
        ).astype(float)
        valid_per_sector[valid_per_sector < 10] = np.nan  # too sparse = unknown

        # normalize by valid pixels
        hist_norm = raw_hist / np.where(np.isnan(valid_per_sector), 1, valid_per_sector)

        # mark sparse sectors as unknown (not enough evidence either way)
        hist_norm[np.isnan(valid_per_sector)] = self.T_HIGH + 0.1


        # Temporal smoothing 
        self.hist_persistent = (
            self.HIST_ALPHA * hist_norm +
            (1 - self.HIST_ALPHA) * self.hist_persistent
        )

        window      = np.array([1.0, 2.0, 3.0, 2.0, 1.0])
        window     /= window.sum()
        hist_smooth = np.convolve(self.hist_persistent, window, mode='same')

        # ysteresis thresholding
        for i in range(self.NUM_SECTORS):
            if hist_smooth[i] > self.T_HIGH:
                self.sector_state[i] = 1   # blocked
            elif hist_smooth[i] < self.T_LOW:
                self.sector_state[i] = 0   # open
            # between T_LOW and T_HIGH: keep previous state — dead band

        # Valley detection
        valleys   = []
        in_valley = False
        v_start   = 0

        for i in range(self.NUM_SECTORS):
            if self.sector_state[i] == 0 and not in_valley:
                in_valley = True
                v_start   = i
            elif self.sector_state[i] == 1 and in_valley:
                in_valley = False
                if (i - v_start) >= self.MIN_VALLEY_SECTS:
                    valleys.append((v_start, i - 1))

        # catch valley that runs to the right edge
        if in_valley and (self.NUM_SECTORS - v_start) >= self.MIN_VALLEY_SECTS:
            valleys.append((v_start, self.NUM_SECTORS - 1))

        #Target bearing
        target_gps        = (43.0724831900561, -89.41245993537561)  # TODO: dynamic waypoints
        bearing_to_target = self.compute_bearing(rover_gps, target_gps)
        target_angle_deg  = (360 - (compass_angle - bearing_to_target)) % 360
        if target_angle_deg > 180:
            target_angle_deg -= 360           # remap to [-180, 180]
        target_angle_rad  = math.radians(target_angle_deg)

        # clamp to FOV before converting handles targets outside camera view
        target_angle_clamped = max(-self.FOV_DEG / 2,
                                   min(self.FOV_DEG / 2, target_angle_deg))
        target_sector = int(np.clip(
            int((target_angle_clamped + self.FOV_DEG / 2) / self.DEG_PER_SECT),
            0, self.NUM_SECTORS - 1
        ))

        #  Valley selection with commitment
        best_steering_sector = None
        best_dist            = float('inf')
        best_valley          = None

        for v_start, v_end in valleys:
            if (v_end - v_start) > self.SMAX:
                # wide valley — steer toward GPS target, no reason to deviate
                clamped = int(np.clip(target_sector, v_start, v_end))
            else:
                # narrow valley — aim for center, maximize clearance
                clamped = (v_start + v_end) // 2

            dist = abs(target_sector - clamped)
            if dist < best_dist:
                best_dist            = dist
                best_steering_sector = clamped
                best_valley          = (v_start, v_end)

        # only switch committed valley if new one is meaningfully better
        if best_valley is not None and self.committed_valley is not None:
            committed_still_valid = any(
                v_start <= self.committed_valley[0] and
                v_end   >= self.committed_valley[1]
                for v_start, v_end in valleys
            )
            if committed_still_valid:
                cv_start, cv_end = self.committed_valley
                if (cv_end - cv_start) > self.SMAX:
                    committed_sector = int(np.clip(target_sector, cv_start, cv_end))
                else:
                    committed_sector = (cv_start + cv_end) // 2
                committed_dist = abs(target_sector - committed_sector)

                if best_dist < committed_dist - self.VALLEY_SWITCH_THRESH:
                    self.committed_valley = best_valley          # switch — worth it
                else:
                    best_steering_sector = committed_sector      # stay committed
                    best_valley          = self.committed_valley
            else:
                self.committed_valley = best_valley              # old valley gone
        elif best_valley is not None:
            self.committed_valley = best_valley                  # first valley


        is_recovery = False

        if best_steering_sector is None:
            # ── no valid valleys — recovery spin ──────────────────────────
            is_recovery = True
            self.recovery_frame_count += 1

            if self.recovery_direction is None:
                # pick direction once per recovery episode, then stick to it
                if abs(target_angle_deg) > 1.0:
                    # GPS target is off to one side — spin that way
                    self.recovery_direction = math.copysign(1.0, target_angle_rad)
                else:
                    # GPS is straight ahead but blocked — spin toward more open side
                    left_density  = np.mean(hist_smooth[:self.NUM_SECTORS // 2])
                    right_density = np.mean(hist_smooth[self.NUM_SECTORS // 2:])
                    self.recovery_direction = -1.0 if left_density < right_density else 1.0

            smoothed_theta           = self.recovery_direction * math.radians(45)
            self.previous_best_theta = None
            self.committed_valley    = None

            if self.recovery_frame_count > 200:
                print(f"WARNING: stuck in recovery for {self.recovery_frame_count} frames — physically trapped?")
            else:
                direction_str = ">>>" if self.recovery_direction > 0 else "<<<"
                print(f"RECOVERY {direction_str} frame {self.recovery_frame_count}")

        else:
            self.recovery_direction   = None
            self.recovery_frame_count = 0

            # convert sector index to angle in radians
            best_theta = math.radians(
                best_steering_sector * self.DEG_PER_SECT
                + self.DEG_PER_SECT / 2
                - self.FOV_DEG / 2
            )

            # exponential weighted average — smooth output angle
            if self.previous_best_theta is None:
                smoothed_theta = best_theta
            else:
                smoothed_theta = (
                    self.EWA_ALPHA * best_theta +
                    (1 - self.EWA_ALPHA) * self.previous_best_theta
                )
            self.previous_best_theta = smoothed_theta

        error_deg  = math.degrees(smoothed_theta)
        error_norm = min(abs(error_deg) / 45.0, 1.0)

        # forward speed drops quadraticaly to 0 at 45° error
        forward_speed = self.MAX_FORWARD * (1.0 - error_norm) ** 2
        forward_speed = max(forward_speed, 0.0)

        # turn speed proportional to error, minimum authority enforced
        turn_speed = min(abs(error_deg) * self.kP, 1.0)
        if turn_speed < 0.2 and abs(error_deg) > 1.0:
            turn_speed = 0.2

        print(f"error: {error_deg:.1f}°  fwd: {forward_speed:.2f}  turn: {turn_speed:.2f}")

        # ── Visualization ──────────────────────────────────────────────
        if self.debug:
            depth_vis = cv2.normalize(
                np.nan_to_num(depth_full), None, 0, 255, cv2.NORM_MINMAX
            ).astype(np.uint8)
            depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

            # ground — blue
            depth_vis[ground_mask] = [255, 80, 0]

            # sector borders only — green if open, red if blocked
            prev_sector = self.col_to_sector[0]
            for col in range(1, W):
                curr_sector = self.col_to_sector[col]
                if curr_sector != prev_sector:
                    color = (0, 0, 200) if self.sector_state[prev_sector] == 1 else (0, 200, 0)
                    cv2.line(depth_vis, (col, 0), (col, H - 1), color, 2)
                prev_sector = curr_sector

            # steering line — red
            chosen_pixel = int(math.tan(smoothed_theta) * self.FOCAL_LENGTH + self.X_PIXEL_OFFSET)
            chosen_pixel = max(0, min(W - 1, chosen_pixel))
            cv2.line(depth_vis, (chosen_pixel, 0), (chosen_pixel, H - 1), (203, 192, 255), 3)

            # target bearing line — yellow
            target_pixel = int(math.tan(target_angle_rad) * self.FOCAL_LENGTH + self.X_PIXEL_OFFSET)
            target_pixel = max(0, min(W - 1, target_pixel))
            cv2.line(depth_vis, (target_pixel, 0), (target_pixel, H - 1), (0, 255, 255), 2)

            if is_recovery:
                direction_str = ">>>" if self.recovery_direction and self.recovery_direction > 0 else "<<<"
                cv2.putText(depth_vis, f"RECOVERY {direction_str}",
                            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

            elapsed = time.time() - start_time
            cv2.putText(depth_vis, f"{1 / max(elapsed, 0.001):.0f} fps",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # magnitude print
            print("hist:", " ".join(
                f"[{s:02d}]{'X' if self.sector_state[s] else 'O'}{hist_smooth[s]:.2f}"
                for s in range(self.NUM_SECTORS)
            ))

            if not self.onRover:
                cv2.imshow("VFH Obstacle Avoidance", depth_vis)
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

        if is_recovery:
            if self.recovery_direction and self.recovery_direction < 0:
                return [0.0, 0.0, 0.5, -1.0]   # spin left
            else:
                return [0.0, 0.0, -1.0, 0.5]   # spin right

        if smoothed_theta < 0:
            return [forward_speed, 0.0, turn_speed, -1.0]  # curve left
        else:
            return [forward_speed, 0.0, -1.0, turn_speed]  # curve right

    @staticmethod
    def compute_bearing(p1, p2):
        lat1 = math.radians(p1[0]);  lon1 = math.radians(p1[1])
        lat2 = math.radians(p2[0]);  lon2 = math.radians(p2[1])
        dlon = lon2 - lon1
        x    = math.sin(dlon) * math.cos(lat2)
        y    = (math.cos(lat1) * math.sin(lat2)
                - math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        return (math.degrees(math.atan2(x, y)) + 360) % 360



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
    config.postProcessing.median                   = dai.MedianFilter.KERNEL_7x7
    config.postProcessing.thresholdFilter.maxRange = 8000
    config.setConfidenceThreshold(0)
    #config.setSubpixel(True)
    config.setExtendedDisparity(False)
    config.setLeftRightCheck(True)


    monoLeftOut  = monoLeft.requestOutput((1280, 720))
    monoRightOut = monoRight.requestOutput((1280, 720))
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    stereoOut = stereo.depth.createOutputQueue()

    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ARVR_STABILIZED_ROTATION_VECTOR, 100)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)
    imuQueue = imu.out.createOutputQueue(maxSize=10, blocking=False)

    obj         = SectorDepthClassifier()
    rclpy.init()
    gps_node    = GPSNode()
    swerve_node = SwervePublisher()
    verifier    = HeadingVerifier(min_move_dist=1.0, alpha=0.2)

    pipeline.start()
    current_heading = 0.0

    while pipeline.isRunning():
        rclpy.spin_once(gps_node,    timeout_sec=0.0)
        rclpy.spin_once(swerve_node, timeout_sec=0.0)

        imuData = imuQueue.tryGet()
        if imuData:
            rv              = imuData.packets[-1].rotationVector
            current_heading = (-quaternion_to_yaw(rv.i, rv.j, rv.k, rv.real)) % 360
            print(f"heading: {current_heading:.1f}°")

        stereoFrame = stereoOut.get()
        assert stereoFrame.validateTransformations()

        depth      = stereoFrame.getCvFrame().astype(np.float32) / 1000.0
        swerve_cmd = obj.cb(depth, current_heading, gps_node.latest_gps)
        swerve_node.send(swerve_cmd)

    pipeline.stop()

cv2.destroyAllWindows()