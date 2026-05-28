# Science Photographer Node (`wr_science`)

This ROS 2 node connects to a live ZMQ camera stream from the rover's Raspberry Pi to capture High-Definition science photography. It automatically reads GNSS/Compass data, overlays telemetry onto the images, saves JSON metadata, and features a fully autonomous "Stop-and-Shoot" panorama mode.

## 🛠️ 1. Building the Workspace
This node runs on the **Base Station Laptop** (must have a display for OpenCV).

1. Navigate to your ROS 2 workspace:
   ```bash
   cd ~/workspace/WRoverSoftware_25-26
   ```
2. Make sure you are in the 'dev/science'

3. Build the `wr_science` package:
   ```bash
   colcon build --packages-select wr_science
   ```
4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---

## 🚀 2. Running the Node
By default, the node looks for a camera streaming at `192.168.1.192` on port `5555`.

### Standard Start (Default Camera)
```bash
ros2 run wr_science science_photographer
```

### Advanced Start (Specific Camera / IP)
If you need to connect to a different camera (e.g., the mast camera on the other Pi `192.168.1.169`), use ROS parameters:
```bash
ros2 run wr_science science_photographer --ros-args -p broadcast_ip:="192.168.1.169" -p port:=5555
```

---

## 🎮 3. Operator Controls (OpenCV Window)
When the node starts, an OpenCV window titled **"Science Camera"** will appear. 
**CRITICAL:** You *must* click on the camera window to give it focus before pressing keys.

*   `h` **- Capture Hillside / Stratigraphic Image**
    *   Takes a single snapshot of the current view, overlays GNSS telemetry, and saves it.
*   `p` **- Start/Stop Autonomous Panorama**
    *   Initiates the automated 360-degree panorama sequence.
    *   Pressing `p` again during a sequence acts as an **Emergency Abort**, stopping the rover immediately.
*   *   For capturing a high quality HDR image please see section 6 of the documentation

---

## 🤖 4. Autonomous Panorama (How it works)
When hitting `p`, the node takes control of the rover drive system via `/swerve`. 
*Ensure manual teleop is stopped so you don't fight the camera script for control.*

**The State Machine Sequence:**
1.  **ROTATING:** The rover turns slowly in place.
2.  **SETTLING:** When it reaches within ±5° of a cardinal direction (N, E, S, W), it stops and waits **1.0 second** for the chassis to stop shaking and the Wi-Fi stream to catch up.
3.  **CAPTURING:** It takes a crisp picture without motion blur, stamps the cardinal direction in Amber text, and resumes rotating.
4.  **STITCHING:** Once all 4 directions are captured, it automatically stitches them together into a 1x4 wide-angle collage and saves it to disk.

---

## 📂 5. Output Data
All captures are stored relative to where you ran the command, inside a directory called `science_data/`.

For every capture, **two** files are generated using a UNIX timestamp:
1.  **Image:** `hillside_1716820000.jpg` *(Contains visual overlay of Lat, Lon, Alt, Acc, Heading)*
2.  **JSON:** `hillside_1716820000.json` *(Raw data dump of the UBXNavPVT arrays for precise reporting)*

---

## ⚠️ Mission Day Checklists & Sanity Checks
*   **Blank/Grey Screen?** Check your IPs. Are you connected to the rover network? Is the Pi at `.169` or `.192`?
*   **Rover Jittering during Panorama?** You have a "Teleop Fight". Make sure the operator driving the rover lets go of the joystick and that the joystick script isn't spamming "STOP" commands while the camera script is trying to send "TURN" commands.

## 6. Capturing a HDR Image of the sampling site
"""
A close-up, well focused, high-resolution picture with some indication of scale at
the sampling site.
"""
1) Align the Rover's camera to point to the area whose photo you want.
2) Then you will have to ssh into the raspberry pi which is connected to the camera, pointing in the direction of the site to be photographed.
3) Kill the currently ongoing launched process of the camera, so that the photos with diff exposure shots can be taken. (Nico knows how to do this)
4) Then run `science_script.py` under WRecorder on the pi which you are ssh'd into.
5) This will save the HDR image under photos/*.jpg . There will be multiple images on there, each one belonging to a different combination of exposure shots, choose the one which looks the best.
6) scp the photos directory (or one particular picture) out of the raspberry pi's
7) Run sudo reboot to restart the pi's launch file