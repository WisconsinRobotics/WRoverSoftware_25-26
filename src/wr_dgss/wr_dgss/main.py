import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger 

class SinglePointCorrection(Node):
    def __init__(self):
        super().__init__("Single_Point_Correction")
        self.fix_sub = self.create_subscription(
            NavSatFix,
            "/fix",
            self.gps_callback,
            10
        )

        self.gnss_calibrate = self.create_service(
            Trigger,
            "/gnss/calibrate",
            self.gnss_calibrate
        )

        self.gnss_reset = self.create_service(
            Trigger,
            "/gnss/reset",
            self.gnss_reset
        )

        msg = Trigger()


    def gps_callback(self, msg: NavSatFix):
        pass

    def gnss_calibrate(self, request, response):
        pass

    def gnss_reset(self, request, response):
        pass
