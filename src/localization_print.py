import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class SwervePublisher(Node):
    def __init__(self):
        super().__init__("swerve_publisher")
        self.pub = self.create_publisher(Imu, "imu/data", 10)

        # --- Sensor Syncing Cache ---
        # DepthAI sends packets asynchronously. We cache the raw sensors (Accel/Gyro)
        # and publish the full ROS message only when the Orientation (Rotation Vector) arrives.
        self.cached_accel = None
        self.cached_gyro = None

    def process_packet(self, packet):
        """
        Ingests a single DepthAI IMU packet.
        1. Updates cache if raw data is present.
        2. Publishes ROS message if Rotation Vector is present.
        """
        # 1. Cache Raw Data
        if packet.acceleroMeter:
            self.cached_accel = packet.acceleroMeter
        
        if packet.gyroscope:
            self.cached_gyro = packet.gyroscope

        # 2. Trigger Publish on Orientation Update
        if packet.rotationVector:
            self.publish_msg(packet.rotationVector)

    def publish_msg(self, rv):
        msg = Imu()
        
        # --- Header ---
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # --- Orientation (DepthAI i,j,k,real -> ROS x,y,z,w) ---
        msg.orientation.x = rv.i
        msg.orientation.y = rv.j
        msg.orientation.z = rv.k
        msg.orientation.w = rv.real
        
        # Covariance: Low variance (High confidence in Sensor Fusion)
        msg.orientation_covariance = [
            0.001, 0.0, 0.0, 
            0.0, 0.001, 0.0, 
            0.0, 0.0, 0.001
        ]

        # --- Angular Velocity (Gyro) ---
        if self.cached_gyro:
            msg.angular_velocity.x = self.cached_gyro.x
            msg.angular_velocity.y = self.cached_gyro.y
            msg.angular_velocity.z = self.cached_gyro.z
            msg.angular_velocity_covariance = [
                0.02, 0.0, 0.0, 
                0.0, 0.02, 0.0, 
                0.0, 0.0, 0.02
            ]
        else:
            msg.angular_velocity_covariance[0] = -1.0 # Data invalid

        # --- Linear Acceleration (Accel) ---
        if self.cached_accel:
            msg.linear_acceleration.x = self.cached_accel.x
            msg.linear_acceleration.y = self.cached_accel.y
            msg.linear_acceleration.z = self.cached_accel.z
            msg.linear_acceleration_covariance = [
                0.04, 0.0, 0.0, 
                0.0, 0.04, 0.0, 
                0.0, 0.0, 0.04
            ]
        else:
            msg.linear_acceleration_covariance[0] = -1.0 # Data invalid

        self.pub.publish(msg)

# --- DEPTHAI V3 PIPELINE SETUP ---

def main():
    pipeline = dai.Pipeline()

    # 1. Create IMU Node
    imu = pipeline.create(dai.node.IMU)

    # 2. Enable Sensors
    # We enable all three. The device will send them in batches.
    # ARVR_STABILIZED_ROTATION_VECTOR: 9-axis fusion (internal mag/gyro/accel)
    imu.enableIMUSensor([
        dai.IMUSensor.ARVR_STABILIZED_ROTATION_VECTOR,
        dai.IMUSensor.ACCELEROMETER_RAW,
        dai.IMUSensor.GYROSCOPE_CALIBRATED
    ], 100) # 100Hz

    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    # 3. V3 Change: Create Output Queue DIRECTLY from the node
    # No "XLinkOut" node is required in v3.
    imuQueue = imu.out.createOutputQueue(maxSize=10, blocking=False)

    # 4. Initialize ROS
    rclpy.init()
    swerve_node = SwervePublisher()
    pipeline.start()
    while rclpy.ok() and pipeline.isRunning():
        # Spin ROS (non-blocking)
        rclpy.spin_once(swerve_node, timeout_sec=0.001)

        # Fetch data from v3 queue
        imuData = imuQueue.tryGet()
        
        if imuData:
            # Iterate through the batch of packets
            for packet in imuData.packets:
                swerve_node.process_packet(packet)

    swerve_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()