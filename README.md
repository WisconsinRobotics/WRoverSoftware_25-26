# WRoverSoftware_25-26
Official Wisconsin Robotics software repository for the 2026 University Rover Challenge, containing autonomy, control, and base station code.


# How To Set Up Simulation

## 1. Install Dependencies
starting from the root of the wisconsin robotics directory do:
`sudo apt-get update`
`rosdep install --from-path src`

## 2. build project
`colcon build`

## 3. Run Simulation
`source install/setup.sh`
`ros2 launch simulation simulation.launch.py` 
