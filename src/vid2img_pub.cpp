#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "tag_detector/WindowSize.h"
#include <iostream>

using namespace cv;

class ImagePublisher
{
private:
  ros::NodeHandle nh;
  image_transport::Publisher pub;
  ros::ServiceServer service;
  ros::Rate loop_rate;
  image_transport::ImageTransport it;
  int window_w;

public:
  ImagePublisher()
  : it(nh), window_w(0), loop_rate(30)
  {
    pub = it.advertise("image_raw", 1);
    service = nh.advertiseService("set_blur_window_size", &ImagePublisher::set_window_size, this);

    ROS_INFO("Waiting for GaussianBlur service call...");

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
    if(cap.isOpened())
    {
      Mat frame;
      sensor_msgs::ImagePtr msg;

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
          // imshow("blurred image", blurr);
          pub.publish(msg);
          // waitKey(1);
        }

        ros::spinOnce();
        loop_rate.sleep();
      }
    }
  }

  ~ImagePublisher()
  {
    destroyAllWindows();
  }

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
};


int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_publisher");
  ImagePublisher ip;
  ros::spin();
  return 0;
}
