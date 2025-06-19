#!/usr/bin/env python3

import sys
import os
import signal
import argparse
import subprocess
import threading
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from contextlib import contextmanager


# Import PlutoCam classes
from lwdrone import LWDrone
from defaults import CAM_IP

@contextmanager
def output_stream(filename=None):
    """Context manager to handle output to file or stdout"""
    if filename and filename != '-':
        with open(filename, 'wb') as f:
            print(f"Streaming to file: {filename}", file=sys.stderr)
            yield f
    else:
        # In Python 3, sys.stdout is already in text mode, so we need to use buffer
        if hasattr(sys.stdout, 'buffer'):
            # Python 3 with buffer attribute
            yield sys.stdout.buffer
        else:
            # Python 2 or other cases
            yield sys.stdout

class PlutoCameraNode:
    def __init__(self, ip, low_def=False, display=False, out_file='-', width=1280, height=720):
        self.ip = ip
        self.low_def = low_def
        self.display = display
        self.out_file = out_file
        self.width = width
        self.height = height
        self.frame_size = width * height * 3  # 3 bytes per pixel for BGR24
        
        # Initialize ROS node and publisher
        rospy.init_node('pluto_camera_publisher', anonymous=True)
        self.image_pub = rospy.Publisher('plutocamera/image_raw', Image, queue_size=10)
        self.bridge = CvBridge()
        
        # Process handles
        self.ffmpeg_decoder = None
        self.ffplay_process = None
        self.drone = None
        self.running = False
        
        # Set up signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def setup_ffmpeg_decoder(self):
        """Setup FFmpeg process for H.264 decoding with 180-degree flip"""
        cmd = [
            'ffmpeg',
            '-fflags', 'nobuffer',
            '-flags', 'low_delay',
            '-probesize', '32',
            '-i', 'pipe:0',           # Input from stdin
            '-vf', 'vflip,hflip',     # Flip video 180 degrees (vertical + horizontal flip)
            '-f', 'rawvideo',         # Output raw video
            '-pix_fmt', 'bgr24',      # Output format
            '-vcodec', 'rawvideo',
            '-',                     # Output to stdout
          ]
        
        return subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=10**8  # Larger buffer for better performance
        )
    
    def setup_ffplay_display(self):
        """Setup ffplay for displaying the decoded video"""
        cmd = [
            'ffplay',
            '-fflags', 'nobuffer',
            '-flags', 'low_delay',
            '-probesize', '32',
            '-sync', 'ext',
            '-framedrop',
            '-f', 'h264',
            '-i', 'pipe:0'
        ]
        
        return subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
    
    def signal_handler(self, sig, frame):
        print("\nShutting down...", file=sys.stderr)
        self.running = False
    
    def process_frames(self):
        """Process frames from the FFmpeg decoder"""
        while self.running and self.ffmpeg_decoder.poll() is None:
            try:
                # Read decoded frame from FFmpeg
                raw_frame = self.ffmpeg_decoder.stdout.read(self.frame_size)
                if len(raw_frame) != self.frame_size:
                    print(f"Incomplete frame: expected {self.frame_size} bytes, got {len(raw_frame)}", file=sys.stderr)
                    continue
                
                # Convert to numpy array
                frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((self.height, self.width, 3))
                
                # Publish to ROS
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    ros_image.header.stamp = rospy.Time.now()
                    self.image_pub.publish(ros_image)
                except Exception as e:
                    print(f"Error publishing image: {e}", file=sys.stderr)
                
                # Optional: Can add frame processing here if needed
                
            except Exception as e:
                print(f"Error processing frame: {e}", file=sys.stderr)
                break
    
    def run(self):
        try:
            # Setup FFmpeg decoder
            self.ffmpeg_decoder = self.setup_ffmpeg_decoder()
            
            # Start ffplay display if requested
            if self.display:
                self.ffplay_process = self.setup_ffplay_display()
            
            # Connect to the drone
            print(f"Connecting to PlutoCamera at {self.ip}...", file=sys.stderr)
            self.drone = LWDrone(ip=self.ip)
            print(f"Connected to PlutoCamera at {self.ip}", file=sys.stderr)
            
            resolution = '1080p' if not self.low_def else '720p'
            print(f"Starting {resolution} video stream... (Press Ctrl+C to stop)", file=sys.stderr)
            
            self.running = True
            out_file_handle = None
            
            try:
                # Open output file if specified
                if self.out_file != '-':
                    out_file_handle = open(self.out_file, 'wb')
                
                # Start frame processing thread
                import threading
                process_thread = threading.Thread(target=self.process_frames, daemon=True)
                process_thread.start()
                
                # Main loop - feed H.264 data to FFmpeg
                for frame in self.drone.start_video_stream(not self.low_def):
                    if not self.running:
                        break
                        
                    frame_data = frame.frame_bytes
                    
                    # Write to FFmpeg decoder
                    try:
                        self.ffmpeg_decoder.stdin.write(frame_data)
                        self.ffmpeg_decoder.stdin.flush()
                    except (BrokenPipeError, OSError) as e:
                        print(f"Error writing to FFmpeg: {e}", file=sys.stderr)
                        break
                    
                    # Write to output file if specified
                    if out_file_handle is not None:
                        out_file_handle.write(frame_data)
                        out_file_handle.flush()
                    
                    # Write to ffplay if display is enabled
                    if self.display and self.ffplay_process and self.ffplay_process.poll() is None:
                        try:
                            self.ffplay_process.stdin.write(frame_data)
                            self.ffplay_process.stdin.flush()
                        except (BrokenPipeError, OSError):
                            print("\nDisplay closed", file=sys.stderr)
                            self.display = False
                
                # Wait for processing thread to finish
                process_thread.join(timeout=1.0)
                if process_thread.is_alive():
                    print("Warning: Frame processing thread did not exit cleanly", file=sys.stderr)
            
            except Exception as e:
                print(f"\nError in main loop: {e}", file=sys.stderr)
                raise
                
        except Exception as e:
            print(f"Error: {e}", file=sys.stderr)
            return 1
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        try:
            if hasattr(self, 'drone') and self.drone:
                self.drone.stop_video_stream()
            
            if self.ffplay_process and self.ffplay_process.poll() is None:
                self.ffplay_process.terminate()
                self.ffplay_process.wait()
                
            if hasattr(self, 'out_file_handle') and self.out_file_handle:
                self.out_file_handle.close()
                
        except Exception as e:
            print(f"Error during cleanup: {e}", file=sys.stderr)

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Stream video from PlutoCamera and publish as ROS topic')
    parser.add_argument('--ip', default=CAM_IP, help=f'Camera IP address (default: {CAM_IP})')
    parser.add_argument('--low-def', action='store_true', help='Use 720p instead of 1080p')
    parser.add_argument('--display', '-d', action='store_true', help='Display video using ffplay')
    parser.add_argument('--out-file', '-o', default='-', help='Output file (default: stdout)')
    
    args = parser.parse_args()
    
    # Create and run the camera node
    node = PlutoCameraNode(
        ip=args.ip,
        low_def=args.low_def,
        display=args.display,
        out_file=args.out_file
    )
    
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    
    print("Stream stopped", file=sys.stderr)
    return 0

if __name__ == '__main__':
    sys.exit(main())
