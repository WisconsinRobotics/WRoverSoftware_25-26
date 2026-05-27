import cv2 as cv
import zmq
import base64
import numpy as np
import argparse
import threading
import signal
from typing import List
import time
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float32, Float32MultiArray
from ublox_ubx_msgs.msg import UBXNavPVT


import json
import threading
import argparse


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
HAS_ROS = True




def start_multiple_receivers(ip: str, ports: List[int]):
    stop_event = threading.Event()
    threads: List[threading.Thread] = []
    frames: dict = {}
    lock = threading.Lock()
    stats = {}

    for port in ports:
        t = threading.Thread(
            target=receive_camera_data, 
            args=(ip, port, stop_event, frames, lock, stats), 
            daemon=True
        )
        t.start()
        threads.append(t)
        print(f"Started receiver for {ip}:{port}")

    return stop_event, threads, frames, lock, stats

 
def receive_camera_data(ip: str, port: int, stop_event: threading.Event, frames: dict, lock: threading.Lock, stats: dict):
    """Subscribe to a single publisher at ip:port and display frames until stop_event is set."""
    context = zmq.Context()
    footage_socket = context.socket(zmq.SUB)
    footage_socket.setsockopt(zmq.CONFLATE, 1)
    footage_socket.connect(f'tcp://{ip}:{port}')
    footage_socket.setsockopt_string(zmq.SUBSCRIBE, '')
    # set a receive timeout so we can check stop_event periodically
    footage_socket.setsockopt(zmq.RCVTIMEO, 500)

    window_name = f"Stream-{port}"
    print(f"Receiving data on {ip}:{port} -> window '{window_name}'")
    print("Press 'q' in any window to quit")

    try:
        while not stop_event.is_set():
            try:
                frame_bytes = footage_socket.recv()
            except zmq.Again:
                continue

            try:
                with lock:
                    stats[window_name] = stats.get(window_name, 0) + len(frame_bytes)
            except Exception:
                pass

            try:
                img = base64.b64decode(frame_bytes)
                npimg = np.frombuffer(img, dtype=np.uint8)
                source = cv.imdecode(npimg, cv.IMREAD_COLOR)
                if source is None:
                    continue
                # store the latest frame for the main thread to display
                try:
                    with lock:
                        frames[window_name] = source.copy()
                except Exception:
                    # if storing fails, skip this frame
                    continue
            except Exception:
                # skip malformed frames
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


class SciencePhotographer(Node):
    def __init__(self, broadcast_ip, port):
        super().__init__('science_photographer')

        self.stop_event = threading.Event()
        self.frames: dict = {}
        self.lock = threading.Lock()
        self.stats = {}

        
        t = threading.Thread(
            target=receive_camera_data, 
            args=(broadcast_ip, port, self.stop_event, self.frames, self.lock, self.stats), 
            daemon=True
        )
        t.start()

        # Telemetry stuff
        self.nav_data = {
            "lat":0.0, "lon":0.0, "elevation_m": 0.0,
            "hAcc_m": 0, "vAcc_m": 0, "heading": 0.0
        }

        # GPS stuff
        self.current_pos_sub = self.create_subscription(UBXNavPVT, '/rover1/ubx_nav_pvt', self.current_pos_cb, 10)
        self.heading_sub = self.create_subscription(Float32, '/heading', self.heading_cb, 10)
        self.drive_pub = self.create_publisher(Float32MultiArray, '/swerve', 10)

        # Publisher for HDR images
        self.camera_cmd_pub = self.create_publisher(String, '/camera_commands', 10)
        self.hdr_active = False
        self.hdr_images = []
        self.hdr_exposures = [50, 150, 300] # Can be modified acc to exposure shots
        self.hdr_step = 0

        self.output_dir = "science_data"
        os.makedirs(self.output_dir, exist_ok=True)

        # Panaroma state
        self.pano_active = False
        self.pano_captured = set()
        self.pano_images = {}

        # # Camera stuff
        # parser = argparse.ArgumentParser(prog='camera_receiver', description='Receive one or more camera streams over ZMQ')
        # parser.add_argument('--broadcast-ip', type=str, default="0.0.0.0", help='IP of the publisher(s)')
        # parser.add_argument('--base-port', type=int, default=5555, help='Starting port for the first camera. Subsequent cameras use base-port+index')
        # parser.add_argument('--count', type=int, default=1, help='Number of sequential ports to subscribe to starting at base-port')
        # parser.add_argument('--show-stats', type=str, choices=['on', 'off'], default='off', help='Show streaming statistics')

        # args = parser.parse_args()
        # broadcast_ip = "192.168.1.192"
        # port = "5555"
        # self.show_stats = args.show_stats == 'on'

        # #self.stop_event, self.threads, self.frames, self.lock, self.stats = start_multiple_receivers(broadcast_ip, ports)
     
        # print(f"Started receiver for {broadcast_ip}:{port}")

        # self.get_logger().info(f"AAAAAAA: {self.frames}")
        # def _signal_handler(signum, frame):
        #     print('Stopping receivers...')
        #     self.stop_event.set()
        #     rclpy.shutdown()

        # signal.signal(signal.SIGINT, _signal_handler)
        # signal.signal(signal.SIGTERM, _signal_handler)

        # self.prev_bytes = {}
        # prev_time = time.time()
        # last_data_rate_update_time = time.time()
        # data_rate_text = "0 KB/s"
        # self.frame = None


        self.timer = self.create_timer(0.1, self._timer_callback)
        

    def current_pos_cb(self, msg):
        self.nav_data["lat"] = msg.lat * 1e-7
        self.nav_data["lon"] = msg.lon * 1e-7
        self.nav_data["elevation_mm"] = msg.hMSL # Height above mean sea level
        self.nav_data["hAcc_mm"] = msg.hAcc # Horizontal accuracy
        self.nav_data["vAcc_mm"] = msg.vAcc # Vertical accuracy
    
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
        heading = self.nav_data["heading"] % 360
        targets = {
            "North": (0, 360), # N wraps around 0/360
            "East": 90,
            "South": 180,
            "West": 270
        }
        tolerance = 3.0 # Error margin for photograph
        for direction, target in targets.items():
            if direction in self.pano_captured:
                continue
            
            match = False
            if direction == "North":
                match = (heading < tolerance or heading > 360 - tolerance)
            else:
                match = abs(heading-target) < tolerance
            
            if match:
                self.pano_captured.add(direction)
                # hud_frame = self.render_telemetry(frame) - repetition in all 4 images???
                pano_slice = frame.copy()
                cv.putText(pano_slice, f"{direction} ({heading:.1f} deg)", (50, 80), 
                           cv.FONT_HERSHEY_DUPLEX, 1.5, (0, 165, 255), 3) # Amber color, can be modified.
                self.pano_images[direction] = pano_slice
                self.get_logger().info(f"Captured {direction} Pano slice!")

        if len(self.pano_captured) == 4:
            "We have our image!"
            self.get_logger().info("Panaroma sequence complete, stiching together panaroma")
            collage = np.hstack((self.pano_images["North"], self.pano_images["East"], self.pano_images["South"], self.pano_images["West"])) # Rover must be rotated clockwise from North, starting from North
            self.save_capture(collage, "panaroma_collage") # Already renders telemetry
            self.pano_active = False

    def trigger_hdr_sequence(self):
        "Start capturing exposure shots"
        self.get_logger().info("Starting HDR Capture sequence")
        self.hdr_active = True
        self.hdr_images = []
        self.hdr_step = 0
        self.request_next_exposure()
    
    def request_next_exposure(self):
        if self.hdr_step < len(self.hdr_exposures):
            exposure = self.hdr_exposures[self.hdr_step]

            msg = String()
            msg.data = f"EXP:{exposure}"
            self.camera_cmd_pub.publish(msg)
            threading.Timer(0.5, self.snap_hdr_slice).start()
        else:
            self.finish_hdr()
    
    def snap_hdr_slice(self):
        "Grab current ZMQ frame after exposure adjustment"
        with self.lock:
            key = list(self.frames.keys())[0]
            frame = self.frames.get(key)
        
        if frame is not None:
            self.hdr_images.append(frame.copy())
            self.get_logger().info(f"Captured image at {self.hdr_step + 1}/3")
        
        self.hdr_step += 1
        self.request_next_exposure()

    def finish_hdr(self):
        self.get_logger().info("Fusing HDR images into a single high quality image")

        merge_mertens = cv.createMergeMertens()
        hdr_fusion = merge_mertens.process(self.hdr_images)
        hdr_8bit = np.clip(hdr_fusion * 255, 0, 255).astype('uint8')

        self.save_capture(hdr_8bit, "closeup_hdr")
        self.hdr_active = False

        # Reset camera back to auto-exposure
        msg = String()
        msg.data = f"EXP:AUTO"
        self.camera_cmd_pub.publish(msg)


    def _timer_callback(self):
        """periodic timer to resend current key if we're waiting for arm response"""
   
        #receive camera data
        with self.lock:
            if not self.frames:
                return
            key = list(self.frames.keys())[0]
            frame = self.frames.get(key)

            if frame is None:
                return
            
            display_frame = self.render_telemetry(frame.copy())

            if self.pano_active:
                cv.putText(display_frame, "PANO_ACTIVE - ROTATE ROVER", (10, 400), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                self.capture_panaroma(frame)
            
            #cv.imshow("Science Camera", display_frame) - TODO USE FOR TESTING PURPOSES
            k = cv.waitKey(1) & 0xFF

            if k == ord('h'):
                # Mission objective 3
                self.get_logger().info("Capturing hillside image...")
                self.save_capture(frame, "hillside")
            
            elif k == ord('c'):
                #Mission objective 2
                self.get_logger().info("Capturing Close-up image...")
                if not self.hdr_active:
                    self.trigger_hdr_sequence()
            elif k == ord('p'): # NOTE Pressing p twice will stop the sequence
                if not self.pano_active:
                    self.get_logger().info("Started Cardinal Panorama Sequence. Please teleop spin the rover 360 degrees.")
                    self.pano_active = True
                    self.pano_captured = set()
                    self.pano_images = {}
                else:
                    self.pano_active = False
                    self.get_logger().info("Panaroma sequence cancelled")


def main(args=None):
    rclpy.init(args=args)

    node = SciencePhotographer()
    
    def _sig(signum, frame):
        node.stop_event.set()
        rclpy.shutdown()
    
    signal.signal(signal.SIGINT, _sig)
    print("Controls:\n[h] - Capture Hillside\n[c] - Capture Close-up (Enhanced)\n[p] - Start/Stop Pano Capture Sequence\n")
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv.destroyAllWindows()

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