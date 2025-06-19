# pluto_camera_ros_package
This pacakge can be used to stream camera feed of Pluto on ros platfrom for linux 64 bit system. 


## Getting Started 
Use following instructions on how to use this package:

###### Prerequisites
Run [install_required_lib.sh](/install_required_lib.sh) script to install necessary libraries. 

###### Run Package
Navigate to pluto_camera_ros_package/pluto_camera_sense/scripts and run the python script to get camera frame and publish to ros topic

```
# To get the frame from camera
python3 plutocam_publisher.py

# To start streaming camera feed
rosrun pluto_image_sub imagepronode 
