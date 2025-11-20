# Sensor Simulator and Driver System

This workspace contains sensor simulators and drivers for IMU, Camera, and LiDAR sensors, designed for SLAM robot testing.

## Prerequisites

- ROS 2 Jazzy
- Python 3
- Required packages: `pyserial`, `opencv-python`, `numpy`
- `socat` (for virtual serial ports): `sudo apt-get install socat`

## Quick Start - All Sensors

**Launch all simulators:**
```bash
ros2 launch sensor_simulator all_sensors_simulation.launch.py
```

This launches only the simulators:
- IMU simulator (writes to `/tmp/vserial1`)
- Camera simulator (writes to `/tmp/camera_pipe`)
- LiDAR simulator (writes to `/tmp/vserial3`)

**Then launch all drivers separately:**
```bash
# In a separate terminal
ros2 launch imu_driver imu_driver.launch.py port:=/tmp/vserial2
ros2 launch camera_driver camera_driver.launch.py pipe_path:=/tmp/camera_pipe
ros2 launch lidar_driver lidar_driver.launch.py port:=/tmp/vserial4
```

## Individual Sensor Launch

### IMU

**Simulator only:**
```bash
ros2 launch sensor_simulator imu_simulation.launch.py
```

**Driver (launch in separate terminal after simulator):**
```bash
ros2 launch imu_driver imu_driver.launch.py port:=/tmp/vserial2
```

**Visualization:**
The IMU visualizer is included in the simulation launch. For standalone visualization:
```bash
ros2 run imu_driver imu_visualizer
```

**View IMU data:**
```bash
ros2 topic echo /imu/data
```

### Camera

**Simulator only:**
```bash
ros2 launch sensor_simulator camera_simulation.launch.py
```

**Driver (launch in separate terminal after simulator):**
```bash
ros2 launch camera_driver camera_driver.launch.py pipe_path:=/tmp/camera_pipe
```

**Visualization:**
```bash
# Using rqt_image_view
rqt_image_view

# Or using rviz2
rviz2
# Then add Image display and set topic to: /camera/image_raw
```

**View camera topics:**
```bash
ros2 topic echo /camera/image_raw
ros2 topic echo /camera/camera_info
```

### LiDAR

**Simulator only:**
```bash
ros2 launch sensor_simulator lidar_simulation.launch.py
```

**Driver (launch in separate terminal after simulator):**
```bash
ros2 launch lidar_driver lidar_driver.launch.py port:=/tmp/vserial4
```

**Visualization:**
```bash
rviz2
# Add LaserScan display and set topic to: /scan
```

**View LiDAR data:**
```bash
ros2 topic echo /scan
```

## Launch Arguments

### All Sensors Launch

```bash
ros2 launch sensor_simulator all_sensors_simulation.launch.py \
    imu_data_file:=/path/to/imu_data.txt \
    imu_playback_speed:=0.5 \
    imu_loop:=true \
    video_file:=/path/to/video.MOV \
    camera_fps:=30.0 \
    camera_playback_speed:=1.0 \
    camera_loop:=true \
    lidar_synthetic_mode:=true \
    lidar_data_file:=/path/to/lidar_data.txt \
    lidar_playback_speed:=1.0 \
    lidar_loop:=true \
    lidar_scan_rate:=10.0
```

### IMU Launch Arguments

```bash
ros2 launch sensor_simulator imu_simulation.launch.py \
    imu_data_file:=/path/to/imu_data.txt \
    imu_playback_speed:=0.5 \
    imu_loop:=true
```

### Camera Launch Arguments

```bash
ros2 launch sensor_simulator camera_simulation.launch.py \
    video_file:=/path/to/video.MOV \
    fps:=30.0 \
    playback_speed:=1.0 \
    loop:=true
```

### LiDAR Launch Arguments

```bash
ros2 launch sensor_simulator lidar_simulation.launch.py \
    data_file:=/path/to/lidar_data.txt \
    synthetic_mode:=true \
    playback_speed:=1.0 \
    loop:=true \
    scan_rate:=10.0 \
    angle_min:=-3.14159 \
    angle_max:=3.14159 \
    range_min:=0.1 \
    range_max:=10.0
```

## Data File Formats

### IMU Data Format
CSV file with format: `timestamp,roll,pitch,yaw,wx,wy,wz,ax,ay,az`
- Angles in radians
- Angular velocities in rad/s
- Accelerations in m/s²

### LiDAR Data Format
CSV file with format: `timestamp,range1,range2,...,rangeN`
- Timestamp in seconds
- Ranges in meters

## Topics Published

### IMU
- `/imu/data` (sensor_msgs/Imu)

### Camera
- `/camera/image_raw` (sensor_msgs/Image)
- `/camera/camera_info` (sensor_msgs/CameraInfo)

### LiDAR
- `/scan` (sensor_msgs/LaserScan)

## Troubleshooting

### Virtual Serial Ports Not Created
If you see errors about serial ports, ensure `socat` is installed:
```bash
sudo apt-get install socat
```

The launch files automatically create virtual serial ports using `socat`.

### Camera Pipe Issues
If camera simulator fails, check that the pipe path is accessible:
```bash
ls -l /tmp/camera_pipe
```

### Permission Errors
Virtual serial ports should have permissions set automatically. If issues occur:
```bash
sudo chmod 666 /tmp/vserial*
```

### Port Already in Use
If you see "port already in use" errors, kill existing processes:
```bash
pkill -f socat
pkill -f imu_simulator
pkill -f camera_simulator
pkill -f lidar_simulator
```

## Architecture

```
┌─────────────┐         ┌─────────────┐         ┌─────────────┐
│   Simulator │────────▶│   Driver    │────────▶│   ROS2      │
│             │ Serial  │             │         │   Topics    │
└─────────────┘         └─────────────┘         └─────────────┘
```

- **Simulators**: Generate sensor data and send via serial ports (IMU, LiDAR) or named pipes (Camera)
- **Drivers**: Read from hardware interfaces and publish ROS2 messages
- **Visualizers**: Display sensor data (IMU text output, Camera/LiDAR in RViz)

## Package Structure

- `sensor_simulator`: Simulates physical sensors (IMU, Camera, LiDAR)
- `imu_driver`: Reads IMU data and publishes to ROS2
- `camera_driver`: Reads camera frames and publishes to ROS2
- `lidar_driver`: Reads LiDAR scans and publishes to ROS2

