
# TurtleBot3 Custom ROS Noetic Setup (Arduino + LDS-02 Lidar)

This project demonstrates how to interface **wheel encoders** and an **LDS-02 Lidar** with **ROS Noetic** for a differential drive robot.  
It publishes wheel encoder data from an Arduino and scan data from the Lidar, which can then be used for odometry and navigation tasks.

> **Note:** This setup provides encoder and scan data for use in ROS. Mapping and full SLAM require additional configuration.

---

## Hardware Used
- **Arduino** (Uno, Mega, or compatible)
- **Wheel encoders** (connected to Arduino for tick counting)
- **LDS-02 Lidar** – [Robotis LDS-02 Lidar Setup Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#add-swap-space)
- **Raspberry Pi or PC running ROS Noetic**
- Differential drive robot platform

---

## ROS Topics Published
The system publishes the following ROS topics:

| Topic Name       | Message Type            | Description                              |
|------------------|-------------------------|------------------------------------------|
| `/encoder_ticks` | `std_msgs/Int16MultiArray` | Wheel encoder ticks for left and right wheels |
| `/scan`          | `sensor_msgs/LaserScan`    | Lidar scan data from LDS-02 |

---

## Why Odom and Scan Topics Are Needed
- **Odometry (`/encoder_ticks`)**  
  - Wheel encoder ticks are used to estimate the robot's movement (distance traveled and rotation).
  - This is essential for navigation, localization, and combining data with the Lidar for SLAM.

- **Laser Scan (`/scan`)**  
  - The LDS-02 Lidar publishes 360° distance measurements.
  - This allows the robot to detect obstacles and understand its surroundings.

Together, these data sources allow ROS to track where the robot is and navigate intelligently.

---

## How the Arduino Code Works
The Arduino code:
1. Reads signals from two wheel encoders (left and right).
2. Counts ticks and determines direction of wheel rotation.
3. Packages the counts into a ROS message (`Int16MultiArray`).
4. Publishes the data to the `/encoder_ticks` topic every **100 ms** using `rosserial`.

Pin connections:

| Arduino Pin | Purpose            |
|-------------|--------------------|
| 2           | Right encoder A    |
| 3           | Left encoder A     |
| 6           | Left encoder B     |
| 11          | Right encoder B    |

---

## Installation & Setup
### 1. Setup ROS Noetic
Follow the official [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation).

### 2. Install LDS-02 Lidar Driver
Install the LDS-02 Lidar driver from the TurtleBot3 guide:  
> [Robotis LDS-02 Lidar Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#add-swap-space)

This will provide the `/scan` topic.

### 3. Upload Arduino Code
- Open the provided `.ino` file in the Arduino IDE.
- Install the `rosserial_arduino` library.
- Set the baud rate to **115200** in the code.
- Upload the code to the Arduino.

### 4. Run rosserial
On your ROS computer:
```bash
roscore
```
In another terminal, connect the Arduino:
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
```

### 5. Verify Topics
Check that both encoder and scan data are publishing:
```bash
rostopic list
```
You should see:
```
/encoder_ticks
/scan
```

View live data:
```bash
rostopic echo /encoder_ticks
rostopic echo /scan
```

---

## Output Example
When both components are running, you should see:

- `/encoder_ticks`  
  ```
  data: [15, 16]
  ```
  *(Left wheel ticks, Right wheel ticks)*

- `/scan`  
  ```
  angle_min: -3.1415926
  angle_max: 3.1415926
  ranges: [1.2, 1.15, 1.18, ...]
  ```

---

## Visualization
You can visualize both odometry and Lidar data in **RViz**:
```bash
rviz
```
Add the following displays:
- **LaserScan** → Topic: `/scan`
- **TF** → To view coordinate frames like `base_link`

---

## Notes
- This setup provides raw encoder and Lidar data.
- Additional steps are required to implement full **SLAM** and **mapping**.
- Ensure wheel encoders are correctly mounted and calibrated for accurate odometry.

---

## Repository Contents
```
├── Arduino_Code.ino      # Arduino code for publishing encoder ticks
├── scan_sample.png       # Example Lidar scan image
├── README.md             # Documentation (this file)
```

---

## References
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [LDS-02 Lidar Setup Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#add-swap-space)
- [rosserial_arduino](http://wiki.ros.org/rosserial_arduino)
