import cv2 as cv
import zmq
import numpy as np
import threading
import signal
import time
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32, Float32MultiArray
from ublox_ubx_msgs.msg import UBXNavPVT
import json

HAS_ROS = True

 
def receive_camera_data(ip: str, port: int, stop_event: threading.Event, frames: dict, lock: threading.Lock, stats: dict, flip_vertical: bool = False):
    """Subscribe to a single publisher at ip:port and display frames until stop_event is set."""
    context = zmq.Context()
    footage_socket = context.socket(zmq.SUB)
    footage_socket.setsockopt(zmq.CONFLATE, 1)
    
    window_name = f"Stream-{port}"
    try:
        print(f"[{window_name}] Attempting to connect to tcp://{ip}:{port}")
        footage_socket.connect(f'tcp://{ip}:{port}')
        footage_socket.setsockopt_string(zmq.SUBSCRIBE, '')
        # set a receive timeout so we can check stop_event periodically
        footage_socket.setsockopt(zmq.RCVTIMEO, 500)
        print(f"[{window_name}] Connected successfully")
    except Exception as e:
        print(f"[{window_name}] Failed to connect: {e}")
        return

    try:
        while not stop_event.is_set():
            try:
                frame_bytes = footage_socket.recv()
                # print(f"[{window_name}] Received {len(frame_bytes)} bytes")
            except zmq.Again:
                continue
            except Exception as e:
                print(f"[{window_name}] Receive error: {e}")
                continue

            try:
                with lock:
                    stats[window_name] = stats.get(window_name, 0) + len(frame_bytes)
            except Exception:
                pass

            try:
                # frame_bytes is raw image data, not base64
                npimg = np.frombuffer(frame_bytes, dtype=np.uint8)
                source = cv.imdecode(npimg, cv.IMREAD_COLOR)
                if source is None:
                    print(f"[{window_name}] Failed to decode image")
                    continue
                if flip_vertical:
                    source = cv.flip(source, 0)
                # store the latest frame for the main thread to display
                try:
                    with lock:
                        frames[window_name] = source.copy()
                        # print(f"[{window_name}] Frame stored successfully ({source.shape[0]}x{source.shape[1]})")
                except Exception as e:
                    print(f"[{window_name}] Failed to store frame: {e}")
                    continue
            except Exception as e:
                print(f"[{window_name}] Failed to decode frame: {e}")
                continue
    finally:
        # remove any stored frame for this window
        try:
            with lock:
                if window_name in frames:
                    del frames[window_name]
                if window_name in stats:
                    del stats[window_name]
        except Exception:
            pass
        try:
            footage_socket.close()
        except Exception:
            pass
        try:
            context.term()
        except Exception:
            pass
        print(f"[{window_name}] Receiver thread stopped")


class SciencePhotographer(Node):
    def __init__(self):
        super().__init__('science_photographer')

        broadcast_ip = '192.168.1.192'
        port = 5556

        self.get_logger().info(
            f"Starting Science Photographer. Connecting to ZMQ at {broadcast_ip}:{port}"
        )
        self.selected_camera_port = 4444
        self.stop_event = threading.Event()
        self.frames: dict = {}
        self.lock = threading.Lock()
        self.stats = {}

        # Start a single receiver thread for port 4444 and flip the image
        self.receiver_threads = []
        t = threading.Thread(
            target=receive_camera_data,
            args=(broadcast_ip, port, self.stop_event, self.frames, self.lock, self.stats, True),
            daemon=True
        )
        t.start()
        self.receiver_threads.append(t)
        self.get_logger().info(f"Started receiver thread for port {port} (flip_vertical=True)")

        # Telemetry stuff
        self.nav_data = {
            "lat":0.0, "lon":0.0, "elevation_mm": 0.0,
            "hAcc_mm": 0, "vAcc_mm": 0, "heading": 0.0
        }

        # GPS stuff
        self.current_pos_sub = self.create_subscription(UBXNavPVT, '/rover1/ubx_nav_pvt', self.current_pos_cb, 10)
        self.heading_sub = self.create_subscription(Float32, '/heading', self.heading_cb, 10)
        self.drive_pub = self.create_publisher(Float32MultiArray, '/swerve', 10)

        self.ROTATE_CMD = [0.0, 0.0, -1.0, -0.4]
        self.STOP_CMD = [0.0, 0.0, -1.0, -1.0]


        self.output_dir = "science_data"
        os.makedirs(self.output_dir, exist_ok=True)

        # Panaroma state
        self.pano_active = False
        self.pano_state = "ROTATING" # ROTATING -> SETTLING -> CAPTURING
        self.pano_settle_start_time = 0.0
        self.pano_current_target = ""
        self.pano_captured = set()
        self.pano_images = {}

        # Check if display is available
        self.has_display = os.environ.get('DISPLAY') is not None
        if not self.has_display:
            self.get_logger().warn("No display available - frames will not be shown with cv.imshow()")

        self.timer = self.create_timer(0.1, self._timer_callback)
        

    def send_drive_cmd(self, vector):
        msg = Float32MultiArray()
        msg.data = [float(v) for v in vector]
        self.drive_pub.publish(msg)

    def get_display_key(self):
        preferred = f"Stream-{self.selected_camera_port}"
        if preferred in self.frames:
            return preferred
        return next(iter(self.frames.keys()), None)

    def current_pos_cb(self, msg):
        self.nav_data["lat"] = msg.lat * 1e-7
        self.nav_data["lon"] = msg.lon * 1e-7
        self.nav_data["elevation_mm"] = msg.hmsl # Height above mean sea level
        self.nav_data["hAcc_mm"] = msg.h_acc # Horizontal accuracy
        self.nav_data["vAcc_mm"] = msg.v_acc # Vertical accuracy
    
    def heading_cb(self, msg):
        self.nav_data["heading"] = msg.data

    def render_telemetry(self, frame):
        "Draw GNSS and heading data onto image. This is for objective 4"
        hud = frame.copy()
        text = [
            f"Lat: {self.nav_data['lat']:.6f} Lon: {self.nav_data['lon']:.6f}",
            f"Alt: {self.nav_data['elevation_mm']/1000.0:.2f}m",
            f"Acc: H(+-{self.nav_data['hAcc_mm']/1000.0:.2f}m) V(+-{self.nav_data['vAcc_mm']/1000.0:.2f}m)",
            f"Heading: {self.nav_data['heading']:.1f} deg"
        ]
        y_offset = 30
        for line in text:
            cv.putText(hud, line, (10, y_offset), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2) # Can change acc to image
            y_offset += 30
        
        return hud
    
    def save_capture(self, frame, prefix):
        "Saves image + telemetry overlay. For any given capture taken (except panaroma)"
        timestamp = int(time.time())
        filename = f"{self.output_dir}/{prefix}_{timestamp}.jpg"
        json_file = f"{self.output_dir}/{prefix}_{timestamp}.json"

        hud_frame = self.render_telemetry(frame)
        cv.imwrite(filename=filename, img=hud_frame)
        
        with open(json_file, 'w') as file:
            json.dump(self.nav_data, file, indent=4)
        
        self.get_logger().info(f"Saved {prefix} caputre to {filename} under {self.output_dir}")


    def capture_panaroma(self, frame):
        "Monitor heading and save 4 cardinal directions, then stich them together automatically"
        
        if self.pano_state == "ROTATING":
            self.send_drive_cmd(self.ROTATE_CMD)
        
            heading = self.nav_data["heading"] % 360
            targets = {
                "North": (0, 360), # N wraps around 0/360
                "East": 90,
                "South": 180,
                "West": 270
            }
            tolerance = 5.0 # Error margin for photograph
            for direction, target in targets.items():
                if direction in self.pano_captured:
                    continue
                
                match = False
                if direction == "North":
                    match = (heading < tolerance or heading > 360 - tolerance)
                else:
                    match = abs(heading-target) < tolerance
                
                if match:
                    self.send_drive_cmd(self.STOP_CMD)
                    self.pano_state = "SETTLING"
                    self.pano_settle_start_time = time.time()
                    self.pano_current_target = direction
                    self.get_logger().info(f"Reached {direction} (+/- 5 deg). Stopping to let camera settle...")
                    break
        elif self.pano_state == "SETTLING":
            # Wait 1 full second
            self.send_drive_cmd(self.STOP_CMD)
            if time.time() - self.pano_settle_start_time >= 1.0:
                self.pano_state = "CAPTURING"
        elif self.pano_state == "CAPTURING":
            direction = self.pano_current_target
            heading = self.nav_data["heading"] % 360
            pano_slice = frame.copy()
            cv.putText(pano_slice, f"{direction} ({heading:.1f} deg)", (50, 80), 
                       cv.FONT_HERSHEY_DUPLEX, 1.5, (0, 165, 255), 3) # Amber color, can be modified.
            self.pano_images[direction] = pano_slice
            self.pano_captured.add(direction)
            self.get_logger().info(f"Captured {direction} Pano slice!")

            if len(self.pano_captured) == 4:
                "We have our image!"
                self.get_logger().info("Panaroma sequence complete, stiching together panaroma")
                collage = np.hstack((self.pano_images["North"], self.pano_images["East"], self.pano_images["South"], self.pano_images["West"])) # Rover must be rotated clockwise from North, starting from North
                self.save_capture(collage, "panaroma_collage") # Already renders telemetry
                self.pano_active = False
                self.send_drive_cmd(self.STOP_CMD) # Ensure it stops
            else:
                self.pano_state = "ROTATING" # Resume turning

    def _timer_callback(self):
        """periodic timer to resend current key if we're waiting for arm response"""
   
        #receive camera data
        with self.lock:
            if not self.frames:
                return
            
            key = self.get_display_key()
            if key is None:
                return
            if key != f"Stream-{self.selected_camera_port}":
                self.get_logger().warn(
                    f"Selected camera stream Stream-{self.selected_camera_port} is unavailable; "
                    f"falling back to {key}"
                )
            frame = self.frames.get(key)

            if frame is None:
                return
            
            display_frame = self.render_telemetry(frame.copy())
            
            if self.pano_active:
                cv.putText(display_frame, "PANO_ACTIVE - ROTATE ROVER", (10, 400), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                self.capture_panaroma(frame)
            
            # Only display if we have a display available
            if self.has_display:
                try:
                    cv.imshow("Science Camera", display_frame)
                    k = cv.waitKey(1) & 0xFF

                    if k == ord('h'):
                        self.get_logger().info("Capturing hillside image...")
                        self.save_capture(frame, "hillside")

                    elif k == ord('p'):
                        if not self.pano_active:
                            self.get_logger().info("Started Cardinal Panorama Sequence.")
                            self.pano_active = True
                            self.pano_state = "ROTATING"
                            self.pano_captured = set()
                            self.pano_images = {}
                        else:
                            self.pano_active = False
                            self.send_drive_cmd(self.STOP_CMD)
                            self.get_logger().info("Panaroma sequence cancelled")
                except Exception as e:
                    self.get_logger().error(f"cv.imshow() failed: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = SciencePhotographer()
    
    def _sig(signum, frame):
        node.stop_event.set()
        rclpy.shutdown()
    
    signal.signal(signal.SIGINT, _sig)
    print("Science Photographer Node Started")
    print("Controls (requires display via cv.imshow):")
    print("  [h] - Capture Hillside")
    print("  [p] - Start/Stop Pano Capture Sequence")
    print("\nWaiting for camera frames...")
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    try:
        cv.destroyAllWindows()
    except:
        pass

if __name__ == "__main__":
    main()


#####################
# 1) Wide angle panaroma showing full context of site. Panorma must have all cardinal directions and indication of scale.
#    Solution: Just keep rotating the rover, maybe create some sort of video stream, and then convert that to a panaromic shot (or directly photograph a panaroma if CV allows that). 
#              Then because we will be subscribing to heading in this ROS node, can render heading on top of this by first aligning with North before beginning panaroma, and then for every 90 degree turn made by swerve (very very slow turn for good photo), we add a cardinal direction using CV?
#              Indication of scale will be done by pre measuring an object with a known length a fixed distance away from the rover camera, and then aligning the rover closely
# 2) Close up, well focused, high resolution picture with some indication of scale at sampling site.
#    Solution: Will likely have a camera facing straight down. Can capture shots at different exposure levels using CV, and then combine them together to create the high resolution image (pretty sure CV allows you to do this, but I am not sure if the ZMQ bridge to the camera will be able to support this). Again indication of scale not sure how it can be done.
# 3) Photo of stratigraphic profile of a nearby hillside (I think this is just a photo of the hillside with the webcam)
#    Solution: As we can teleop, I think this will just be simple, after the operators manually align the rover's camera to a nearby hillside, they can just call my node, and then it will take and save a photo of the hillside.
# 4) GNSS coordinates of each site, to include elevation and accuracy range - look at the msg type ublox, then write up here
#
#####################
