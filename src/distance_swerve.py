"""
Contains redundant code from object detection.py but may still be useful??
"""

    # Calculate swerve velocities based on detection position
def calculate_vel(self, x_offset, distance, linear_velocity):
    x_center = 320 # Center of the 640 crop
    angular_vel = 0.0
    current_linear = linear_velocity
    
    # Slow down as we get closer
    # TODO: test distance thresholds
    if 1.0 < distance < 2.5:
        current_linear = linear_velocity / 2
    elif distance <= 1.0:
        current_linear = 0.0

    max_error = 320 # Half width of crop
    error = x_offset - x_center
    
    # Deadband logic
    if abs(error) > 20:
        angular_vel = 1.0 * (error / max_error)
        # If extremely offset, stop moving forward and just rotate
        if abs(error) > 200:
            current_linear = 0.0
            
    return current_linear, angular_vel

# Publish swerve control parameters linear_y, linear_x, angular_pos, and angular_neg
def publish_swerve(self, linear_y, linear_x, angular_pos, angular_neg):
    swerve_msg = Float32MultiArray()
    
    swerve_msg.data = [linear_y, linear_x, angular_pos, angular_neg]
    
    self.swerve_publisher.publish(swerve_msg)


# EXAMPLE USAGE: # Publish swerve msg
                # self.publish_swerve(float(linear_vel), 0.0, float(-angular_vel), float(angular_vel))