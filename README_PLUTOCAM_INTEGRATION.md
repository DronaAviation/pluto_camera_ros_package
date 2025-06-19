# PlutoCamera ROS Integration with PlutoCam Python Library

This package has been adapted to use the PlutoCam Python library instead of the original C++ FFmpeg-based implementation.

## Key Changes

### pluto_camera_sense package:
- **OLD**: C++ FFmpeg-based TCP stream reader (plutocamera.cpp)
- **NEW**: Python PlutoCam publisher (plutocam_publisher.py)

### Benefits:
- Direct communication with PlutoCamera using native protocol
- No need for FFmpeg dependencies
- Simpler H.264 stream handling
- Better error handling and reconnection

## Installation

1. **Install PlutoCam library dependencies:**
```bash
pip install opencv-python numpy
```

2. **Ensure PlutoCam library is accessible:**
   - The script expects PlutoCam at `/home/malay/distrobox/uav/PlutoCam`
   - Adjust the path in `plutocam_publisher.py` line 16 if needed

3. **Build the workspace:**
```bash
cd ~/distrobox/uav/drona_ws
catkin_make
source devel/setup.bash
```

## Usage

### Option 1: Complete setup (Publisher + Viewer)
```bash
roslaunch pluto_camera_sense plutocam_complete.launch
```

### Option 2: Publisher only
```bash
roslaunch pluto_camera_sense plutocam_stream.launch
```

### Option 3: Manual nodes
```bash
# Terminal 1: Start publisher
rosrun pluto_camera_sense plutocam_publisher.py

# Terminal 2: Start viewer (optional)
rosrun pluto_image_sub imagepronode
```

## Parameters

### Camera Connection:
- `camera_ip`: PlutoCamera IP address (default: "192.168.1.1")
- `cmd_port`: Command port (default: 8065)
- `stream_port`: Stream port (default: 7065)

### Stream Settings:
- `high_def`: Enable high definition mode (default: true)
- `publish_rate`: Publishing rate in Hz (default: 30.0)

## Topics

### Published:
- `/plutocamera/image_raw` (sensor_msgs/Image): Raw camera frames
- `/plutocamera/image_raw/compressed` (sensor_msgs/CompressedImage): JPEG compressed frames

### Subscribed by image viewer:
- `/plutocamera/image_raw`: Displays in OpenCV window

## Code Mappings

### C++ to Python Equivalents:

| C++ Component | Python Equivalent | Description |
|---------------|-------------------|-------------|
| `avformat_open_input()` | `LWDrone()` | Camera connection |
| `av_read_frame()` | `drone.start_video_stream()` | Frame reading |
| `avcodec_decode_video2()` | `decode_h264_with_opencv()` | H.264 decoding |
| `sws_scale()` | OpenCV color conversion | Format conversion |
| `FrameQueue` | `queue.Queue()` | Frame buffering |
| `processing_thread` | `frame_streaming_thread()` | Background processing |

### Topic Compatibility:
- Same topic names maintained: `/plutocamera/image_raw`
- Same message types: `sensor_msgs/Image`
- `pluto_image_sub` works unchanged

## Troubleshooting

### Connection Issues:
```bash
# Check camera IP and connectivity
ping 192.168.1.1

# Verify PlutoCam library path
python3 -c "import sys; sys.path.append('/home/malay/distrobox/uav/PlutoCam'); from plutocam.lwdrone import LWDrone"
```

### Build Issues:
```bash
# Clean build
cd ~/distrobox/uav/drona_ws
catkin_clean
catkin_make
```

### Runtime Issues:
```bash
# Check topics
rostopic list | grep plutocamera

# Monitor frame rate
rostopic hz /plutocamera/image_raw

# View debug logs
rqt_console
```

## Migration Summary

This migration replaces the complex C++ FFmpeg pipeline with a direct Python interface to the PlutoCamera, providing:

1. **Simplified architecture**: Direct camera protocol communication
2. **Better maintenance**: Pure Python vs C++/FFmpeg
3. **Improved reliability**: Native camera error handling
4. **Preserved compatibility**: Same ROS topics and messages

The `pluto_image_sub` package remains unchanged and continues to work with the new publisher.

