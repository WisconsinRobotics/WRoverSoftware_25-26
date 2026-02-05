# Telemetry README

# ESP32 Libraries
we are using the following packages that need to be installed on the Arduino IDE in order to work.

## MicroROS arduino library
[uROS arduinp](https://github.com/micro-ROS/micro_ros_arduino)
## ESP32 Board Library
[ESP32 board library guide](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)
Raw: ```https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json, http://arduino.esp8266.com/stable/package_esp8266com_index.json```

# MicroROS agent
this is installed on the computer that wants to read the ESP32. This is what allows the ESP32 to communicate with ROS2 as a stand alone topic.
[Docker MicroROS link](https://hub.docker.com/r/microros/micro-ros-agent)

Use the following command to start the docker container:
```docker run -it --device=/dev/ttyUSB0  microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0```
Replace dev/ttyUSB0 with the respective port the ESP32 decides to us...
