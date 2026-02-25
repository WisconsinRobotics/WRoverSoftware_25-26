# TODO Test using rover camera and actual keyboard
# TODO Give to state machine team to let them implement task
# Basically detects the 4 aruco markers on the K552 keyboard and figures out where
# all the keys are so the arm can type on them. Publishes the homography
# and a key positions topic so whatever controls the arm can just ask
# where is the R key and get center mm coordinates back for typing task. I made it
#vizualize the bounding boxes of the reddragon keys but I don't believe thats needed.
#

import numpy as np
import cv2
import cv_bridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, String
import json

# -----------keyboard hardcoded stuff(mostly standard ANSI so could not be perfect for reddragon)------------
KEYBOARD_WIDTH_MM = 354.0
KEYBOARD_DEPTH_MM = 123.0
PITCH = 19.05
H_KEY = PITCH
ROW_Y = {
    'fn':   6.5,
    'num':  25.55,
    'tab':  44.60,
    'caps': 63.65,
    'shft': 82.70,
    'spc':  101.75,
}
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
# keyboard corners for homography
DST_PTS = np.array([
    [0.0,               0.0], #Bl
    [KEYBOARD_WIDTH_MM, 0.0], #BR
    [KEYBOARD_WIDTH_MM, KEYBOARD_DEPTH_MM], #TR
    [0.0,               KEYBOARD_DEPTH_MM], #TL
], dtype=np.float32)
#Table to lookup keys with label. w_mm is for diff key sizes.
KEY_LAYOUT = []  # (label, cx_mm, cy_mm, w_mm)


#Builds the keyboard layout using ANSI standadized mapping on reddragon
def u(n):
    return n * PITCH
def add_row(keys, y, start_x):
    x = start_x
    for label, w in keys:
        w_mm = u(w)
        KEY_LAYOUT.append((label, x + w_mm / 2, y, w_mm))
        x += w_mm
# number row
add_row([
    ('`',1.0),('1',1.0),('2',1.0),('3',1.0),('4',1.0),('5',1.0),
    ('6',1.0),('7',1.0),('8',1.0),('9',1.0),('0',1.0),('-',1.0),
    ('=',1.0),('Bksp',2.0),
], ROW_Y['num'], 4.5)
for i, lbl in enumerate(['Ins','Home','PgUp']):
    KEY_LAYOUT.append((lbl, u(14.75)+4.5+i*u(1.0)+u(0.5), ROW_Y['num'], u(1.0)))
# qwerty row
add_row([
    ('Tab',1.5),('Q',1.0),('W',1.0),('E',1.0),('R',1.0),('T',1.0),
    ('Y',1.0),('U',1.0),('I',1.0),('O',1.0),('P',1.0),
    ('[',1.0),(']',1.0),('\\',1.5),
], ROW_Y['tab'], 4.5)
for i, lbl in enumerate(['Del','End','PgDn']):
    KEY_LAYOUT.append((lbl, u(14.75)+4.5+i*u(1.0)+u(0.5), ROW_Y['tab'], u(1.0)))
# asdf row
add_row([
    ('Caps',1.75),('A',1.0),('S',1.0),('D',1.0),('F',1.0),('G',1.0),
    ('H',1.0),('J',1.0),('K',1.0),('L',1.0),(';',1.0),("'",1.0),
    ('Enter',2.25),
], ROW_Y['caps'], 4.5)
# zxcv row
add_row([
    ('LShft',2.25),('Z',1.0),('X',1.0),('C',1.0),('V',1.0),('B',1.0),
    ('N',1.0),('M',1.0),(',',1.0),('.',1.0),('/',1.0),('RShft',2.75),
], ROW_Y['shft'], 4.5)
KEY_LAYOUT.append(('Up', u(14.75)+4.5+u(1.5), ROW_Y['shft'], u(1.0)))
# bottom row TODO do we need this row?
add_row([
    ('LCtrl',1.25),('LWin',1.25),('LAlt',1.25),('Space',6.25),
    ('RAlt',1.25),('Fn',1.25),('RCtrl',1.25),
], ROW_Y['spc'], 4.5)
KEY_LAYOUT.append(('Left',  u(14.75)+4.5+u(0.5), ROW_Y['spc'], u(1.0)))
KEY_LAYOUT.append(('Down',  u(14.75)+4.5+u(1.5), ROW_Y['spc'], u(1.0)))
KEY_LAYOUT.append(('Right', u(14.75)+4.5+u(2.5), ROW_Y['spc'], u(1.0)))

# dict for arm controller lookup
KEY_MAP = {
    lbl: {'cx_mm': cx, 'cy_mm': cy, 'w_mm': w, 'h_mm': H_KEY}
    for lbl, cx, cy, w in KEY_LAYOUT
}

#--------------Helper functions and callback logic--------------

def marker_center(corners_one):
    pts = corners_one.reshape((4, 2))
    return np.array([pts[:, 0].mean(), pts[:, 1].mean()], dtype=np.float32)

# sort found markers into TL, TR, BR, BL order. Fixes hardcoded aruco issue
def sort_corners_geometric(centers):
    pts = np.array(centers, dtype=np.float32)
    top    = pts[np.argsort(pts[:, 1])[:2]]
    bottom = pts[np.argsort(pts[:, 1])[2:]]
    tl = top[np.argmin(top[:, 0])]
    tr = top[np.argmax(top[:, 0])]
    bl = bottom[np.argmin(bottom[:, 0])]
    br = bottom[np.argmax(bottom[:, 0])]
    return np.array([tl, tr, br, bl], dtype=np.float32)

# takes a pixel coordinate and returns where it is on the keyboard in mm
def pixel_to_keyboard_mm(H, u_px, v_px):
    H = np.asarray(H).reshape(3, 3)
    pt = np.array([[[u_px, v_px]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, H)
    return float(out[0, 0, 0]), float(out[0, 0, 1])

#Topic publishing. TODO make sure these are the right topic names for arm and camera
class KeyboardArucoNode(Node):
    def __init__(self):
        #Setup and bridge
        super().__init__("keyboard_aruco")
        self.declare_parameter("image_topic", "camera_data_topic")
        image_topic = self.get_parameter("image_topic").value
        self.bridge = cv_bridge.CvBridge()
        self.sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10, #TODO should it be every ten?
        )
        # publishes the 3x3 homography matrix as 9 floats, all zeros if no lock yet
        self.homography_pub = self.create_publisher(
            Float64MultiArray,
            "keyboard_plane_homography",
            10,
        )

        # publishes all key positions as json so arm controller can look up any key
        self.key_positions_pub = self.create_publisher(
            String,
            "keyboard_key_positions",
            10,
        )
        self.get_logger().info(
            f"keyboard_aruco started on {image_topic} | {len(KEY_MAP)} keys mapped"
        )

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        corners, ids, _ = cv2.aruco.detectMarkers(img, ARUCO_DICT, parameters=ARUCO_PARAMS)

        # need at least 4 markers to compute the plane
        if ids is None or len(ids) < 4:
            self.publish_no_plane()
            return

        centers = [marker_center(corners[i]) for i in range(4)]
        src_pts = sort_corners_geometric(centers)

        H, _ = cv2.findHomography(src_pts, DST_PTS)
        if H is None:
            self.publish_no_plane()
            return

        # publish raw homography for anything that wants to do its own transforms
        h_msg = Float64MultiArray()
        h_msg.data = H.flatten().tolist()
        self.homography_pub.publish(h_msg)

        # publish key positions - arm controller subscribes to this and looks up keys by name
        # format: {key_label: {cx_mm, cy_mm, w_mm, h_mm}, ...}
        key_msg = String()
        key_msg.data = json.dumps(KEY_MAP)
        self.key_positions_pub.publish(key_msg)

        self.get_logger().info(
            "keyboard locked, homography + key positions published",
            throttle_duration_sec=1.0,
        )

    def publish_no_plane(self):
        # zeros = no valid plane, subscribers should wait before trying to type
        h_msg = Float64MultiArray()
        h_msg.data = [0.0] * 9
        self.homography_pub.publish(h_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()