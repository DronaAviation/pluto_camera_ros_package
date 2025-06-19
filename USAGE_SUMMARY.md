# PlutoCamera ROS Integration - Corrected Usage

## Package Structure (Corrected)

```
pluto_camera_ros_package/          # This is just the parent directory
├── pluto_camera_sense/             # This is the actual ROS package for camera
│   ├── scripts/
│   │   └── plutocam_publisher.py   # Python publisher using PlutoCam library
│   ├── launch/
│   │   ├── plutocam_stream.launch  # Publisher only
│   │   └── plutocam_complete.launch # Publisher + Viewer
│   ├── package.xml
│   └── CMakeLists.txt
└── pluto_image_sub/                # This is the ROS package for image viewing
    ├── src/imagepronode.cpp        # C++ image viewer (unchanged)
    ├── package.xml
    └── CMakeLists.txt
```

## Correct Usage Commands

### Complete Setup (Recommended)
```bash
roslaunch pluto_camera_sense plutocam_complete.launch
```

### Publisher Only
```bash
roslaunch pluto_camera_sense plutocam_stream.launch
```

### Manual Nodes
```bash
# Terminal 1: Camera publisher
rosrun pluto_camera_sense plutocam_publisher.py

# Terminal 2: Image viewer (optional)
rosrun pluto_image_sub imagepronode
```

## Key Corrections Made

1. **Package naming**: `pluto_camera_sense` is the ROS package, not `pluto_camera_ros_package`
2. **Launch file location**: All launch files are in `pluto_camera_sense/launch/`
3. **roslaunch commands**: Use `pluto_camera_sense` as the package name
4. **File organization**: Python script in `scripts/`, launch files in `launch/`

## What Was Changed

### pluto_camera_sense package:
- **REMOVED**: `src/plutocamera.cpp` (C++ FFmpeg implementation)
- **ADDED**: `scripts/plutocam_publisher.py` (Python PlutoCam implementation)
- **UPDATED**: `package.xml` (Python dependencies)
- **UPDATED**: `CMakeLists.txt` (Python build setup)
- **ADDED**: Launch files for different usage scenarios

### pluto_image_sub package:
- **NO CHANGES**: Still uses original C++ imagepronode.cpp
- **COMPATIBILITY**: Works with new publisher topics

## Topics Published

- `/plutocamera/image_raw` (sensor_msgs/Image)
- `/plutocamera/image_raw/compressed` (sensor_msgs/CompressedImage)

## Installation & Build

```bash
cd ~/distrobox/uav/drona_ws
catkin_make
source devel/setup.bash
```

Now you can run:
```bash
roslaunch pluto_camera_sense plutocam_complete.launch
```

