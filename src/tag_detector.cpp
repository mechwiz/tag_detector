#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <numeric>
#include <iostream>
#include <chrono>

using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  static vector<chrono::microseconds> times;
  try
  {
    auto start = chrono::high_resolution_clock::now();
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    // imshow("view", cv_ptr->image);
    Mat gray, edge, draw;
    cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
    Canny( gray, edge, 50, 150, 3);
    dilate(edge,edge,Mat(),Point(-1,-1));
    edge.convertTo(draw, CV_8U);
    // imshow("canny",draw);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(draw,contours,hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    Rect boundRect;
    double ar;

    vector<Point> approxSquare;
    for(size_t i = 0; i < contours.size(); i++){
        approxPolyDP(contours[i], approxSquare, arcLength(Mat(contours[i]), true)*0.02, true);
        if(approxSquare.size() == 4 && isContourConvex(approxSquare) && contourArea(approxSquare)>100){
          boundRect = boundingRect( approxSquare );
          ar = boundRect.width/static_cast<double>(boundRect.height);
          if (ar>0.5 && ar <1.5)
            drawContours(cv_ptr->image, contours, i, Scalar(0, 255, 0),3); // fill GREEN
        }
    }
    auto stop = chrono::high_resolution_clock::now();
    imshow("overlay", cv_ptr->image);
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    if (times.size() <= 100)
      times.push_back(duration);

    if (times.size() == 100){
      chrono::microseconds sum;
      for (auto &n : times)
        sum += n;
      auto avg = sum/times.size();
      ROS_INFO("Average computation time taken by function over 100 calls: %ld microseconds",avg.count());
    }
    waitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  // namedWindow("view");
  // namedWindow("canny");
  namedWindow("overlay");
  startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image_raw", 1, imageCallback);
  ros::spin();
  destroyAllWindows();
}
