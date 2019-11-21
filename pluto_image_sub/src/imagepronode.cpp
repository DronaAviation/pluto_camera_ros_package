

#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "plutodrone/PlutoMsgAP.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;



ros::Publisher command_ap_pub;

cv_bridge::CvImagePtr cv_ptr;



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


 try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


  try
  {
    imshow("view", cv_ptr->image);

    waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }


}


int main(int argc, char **argv) {


  ros::init(argc, argv, "imagepronode");

  ros::NodeHandle n;


  namedWindow("view");

  startWindowThread();
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("plutocamera/image_raw", 1, imageCallback);

  command_ap_pub = n.advertise<plutodrone::PlutoMsgAP>("drone_ap_command", 1000);
  destroyWindow("view");


  ros::spin();
  
  return 0;

}
