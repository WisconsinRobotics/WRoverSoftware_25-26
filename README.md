# Object Detection

## Overview

- [YOLOv8n](https://docs.ultralytics.com/models/yolov8/) is used to detect bottle, mallet, and hammer objects

- Three models were trained (one for each object) on custom datasets

- The object_detection node publishes detection information, detected image, LED color, and swerve control parameters to respective topics

- A custom ```Detection``` message is used to publish detection information

## Install

- Install [ROS2](https://docs.ros.org/en/humble/Installation.html)

- Install python dependencies
    ```
    pip install numpy
    pip install depthai==3.1.0
    pip install opencv-python
    ```

## Run

- Source the global ROS2 workspace
    ```
    source /opt/ros/$ROS_DISTRO/setup.bash
    ```

- Build and source the package
    ```
    colcon build
    source install/setup.bash
    ```

- Run the publisher for different objects
    ```
    ros2 run object_detection object_detection --ros-args -p model:="bottle"
    ros2 run object_detection object_detection --ros-args -p model:="mallet"
    ros2 run object_detection object_detection --ros-args -p model:="hammer"
    ```

- Testing
    ```
    # Open another terminal and source again
    ros2 topic echo detection_msg
    ```

## Publisher

- Publishes a custom ```Detection``` message to the topic ```detection_msg``` with the following format:
    ```
    # Normalized bounding box coordinates (0.0 to 1.0)
    float64 x1
    float64 y1
    float64 x2
    float64 y2

    # Confidence level (0.0 to 1.0)
    float64 conf

    # Distance in meters
    float64 distance
    ```

- Publishes the detection image of type ```CompressedImage``` to the topic ```detection_image```

- Publishes the LED color to the topic ```led``` with the following format:
    ```
    # [R, G, B] (0.0 to 255.0)
    float32[3] data
    ```
    
    - When the LED is red, no objects are detected
    - When the LED is blue, the rover is currently navigating towards an object
    - When the LED is green and blinking, the rover has successfully reached the object

- Publishes the swerve control parameters to the topic ```swerve``` with the following format:
    ```
    # Translation and rotation velocities
    float32 linear_y     # Forward/backward velocity (calculated from distance)
    float32 linear_x     # Lateral velocity (hardcoded to 0.0)
    float32 angular_pos  # Counter-clockwise rotation
    float32 angular_neg  # Clockwise rotation
    ```
