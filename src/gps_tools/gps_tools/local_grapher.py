import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

# IMPORT FIX: Import the custom U-blox message type instead of NavSatFix!
from ublox_ubx_msgs.msg import UBXNavPVT

import matplotlib.pyplot as plt

class LocalGrapher(Node):
    def __init__(self):
        super().__init__('local_grapher')
        
        # TOPIC FIX: Subscribe back to the native U-blox PVT topic
        self.subscription = self.create_subscription(
            UBXNavPVT,
            '/rover1/ubx_nav_pvt',   
            self.fix_callback,
            10)
            
        # Publish the new X, Y coordinates
        self.publisher_ = self.create_publisher(Point, '/rover1/local_xy', 10)
        
        # Variables to store our (0,0) origin
        self.origin_lat = None
        self.origin_lon = None
        
        # Earth radius in meters
        self.earth_radius = 6378137.0 

        # --- Plotting Variables ---
        self.x_history = []
        self.y_history =[]
        self.latest_x = 0.0
        self.latest_y = 0.0
        self.new_data = False
        
        # --- Matplotlib Setup ---
        plt.ion()  # Enable interactive mode so it doesn't block the ROS node
        self.fig, self.ax = plt.subplots()
        
        # Create two plot elements: a line for the path, and a dot for the current location
        self.path_line, = self.ax.plot([],[], 'b-', label='Rover Path')
        self.current_dot, = self.ax.plot([],[], 'ro', label='Current Position')
        
        self.ax.set_title("Rover Real-Time Localization")
        self.ax.set_xlabel("X (1 unit = 5cm)")
        self.ax.set_ylabel("Y (1 unit = 5cm)")
        self.ax.grid(True)
        self.ax.legend()
        
        # Create a timer to update the graph 10 times a second (0.1s)
        self.plot_timer = self.create_timer(0.1, self.update_plot)

    def fix_callback(self, msg):

        # MATH EXPLOSION FIX:
        # U-blox sends coordinates multiplied by 10,000,000 to avoid decimals.
        # We must divide by 10,000,000 (1e7) to convert them back to true degrees!
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7

        # 1. SET THE ORIGIN (0,0)
        if self.origin_lat is None:
            self.origin_lat = current_lat
            self.origin_lon = current_lon
            self.get_logger().info(f"ORIGIN SET AT: Lat {self.origin_lat}, Lon {self.origin_lon}")
            return

        # 2. CONVERT LAT/LON DIFFERENCE TO METERS
        lat1 = math.radians(self.origin_lat)
        lon1 = math.radians(self.origin_lon)
        lat2 = math.radians(current_lat)
        lon2 = math.radians(current_lon)

        dx_meters = (lon2 - lon1) * math.cos(lat1) * self.earth_radius
        dy_meters = (lat2 - lat1) * self.earth_radius

        # 3. CONVERT TO CUSTOM SCALE (1 unit = 5 cm)
        x_scaled = dx_meters / 0.05
        y_scaled = dy_meters / 0.05

        # 4. PUBLISH
        pt = Point()
        pt.x = x_scaled
        pt.y = y_scaled
        pt.z = 0.0  
        self.publisher_.publish(pt)
        
        # 5. SAVE DATA FOR PLOTTING
        self.x_history.append(x_scaled)
        self.y_history.append(y_scaled)
        self.latest_x = x_scaled
        self.latest_y = y_scaled
        self.new_data = True

    def update_plot(self):
        # If no new data has arrived, we still call pause to keep the GUI window responsive
        if not self.new_data:
            plt.pause(0.001)
            return

        # Update the line with the history arrays
        self.path_line.set_xdata(self.x_history)
        self.path_line.set_ydata(self.y_history)
        
        # Update the red dot with the latest single point
        self.current_dot.set_xdata([self.latest_x])
        self.current_dot.set_ydata([self.latest_y])

        # Recalculate graph limits so the view scales dynamically
        self.ax.relim()
        self.ax.autoscale_view()
        
        # Force a 1:1 aspect ratio so moving 1 unit X looks the same as 1 unit Y
        self.ax.set_aspect('equal', 'datalim')

        # Redraw the canvas
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        # plt.pause allows the Matplotlib GUI to process its events
        plt.pause(0.001)
        
        self.new_data = False

def main(args=None):
    rclpy.init(args=args)
    node = LocalGrapher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Keep the plot open even after shutting down the node until the user closes it manually
        plt.ioff()
        plt.show()
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()