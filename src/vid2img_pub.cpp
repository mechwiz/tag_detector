#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "tag_detector/WindowSize.h"
#include <iostream>

using namespace cv;
// static const std::string OPENCV_WINDOW = "Image window";
static int window_w = 0;

bool set_window_size(tag_detector::WindowSize::Request  &req,
                     tag_detector::WindowSize::Response &res)
{
  if (req.w == 0)
  {
    if (window_w == 0)
      return false;
    res.w = window_w;
  }else{
    if (req.w < 1 || req.w % 2 == 0)
      return false;
    window_w = req.w;
    res.w = req.w;
  }

  return true;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image_raw", 1);
  ros::ServiceServer service = nh.advertiseService("set_blur_window_size", set_window_size);

  ROS_INFO("Waiting for GaussianBlur service call...");
  ros::Rate loop_rate(30);
  while (nh.ok()){
    if (window_w > 0)
      break;
    ros::spinOnce();
    loop_rate.sleep();
  }
  auto kernel = Size(window_w,window_w);

  const std::string video_source = ros::package::getPath("tag_detector")+"/vid/tag_vid.MOV";

  VideoCapture cap(video_source);

  // Check if video device can be opened with the given name
  if(!cap.isOpened()) return 1;
  Mat frame;
  sensor_msgs::ImagePtr msg;

  // namedWindow(OPENCV_WINDOW);
  while (nh.ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      resize(frame, frame, Size(640, 480), 0, 0, CV_INTER_CUBIC);
      Mat blurr;
      if (kernel.width != window_w){
        kernel.height = window_w;
        kernel.width = window_w;
      }
      GaussianBlur(frame,blurr,kernel,0);
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", blurr).toImageMsg();
      // imshow(OPENCV_WINDOW, blurr);
      pub.publish(msg);
      // waitKey(1);
    } else {
      // destroyAllWindows();
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
}
