#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

static const std::string OPENCV_WINDOW = "Image window";

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image_raw", 1);

  const std::string video_source = ros::package::getPath("tag_detector")+"/vid/tag_vid.MOV";

  cv::VideoCapture cap(video_source);

  // Check if video device can be opened with the given name
  if(!cap.isOpened()) return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  cv::namedWindow(OPENCV_WINDOW);
  ros::Rate loop_rate(30);
  while (nh.ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      cv::resize(frame, frame, cv::Size(640, 480), 0, 0, CV_INTER_CUBIC);
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      cv::imshow(OPENCV_WINDOW, frame);
      pub.publish(msg);
      cv::waitKey(1);
    } else {
      cv::destroyAllWindows();
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
}
