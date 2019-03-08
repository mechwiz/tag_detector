#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <numeric>
#include <iostream>
#include <chrono>
#include "tag_detector/WindowSize.h"

using namespace cv;
using namespace std;

static ros::ServiceClient client;

vector<Mat> fft(Mat &I,int i_row, int i_col){
//! [expand]
    Mat padded;                            //expand input image to optimal size
    int m = getOptimalDFTSize( i_row );
    int n = getOptimalDFTSize( i_col ); // on the border add zero values
    copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));
//! [expand]

//! [complex_and_real]
    Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
    Mat complexI;
    merge(planes, 2, complexI);         // Add to the expanded another plane with zeros
//! [complex_and_real]

//! [dft]
    dft(complexI, complexI);            // this way the result may fit in the source matrix
//! [dft]

    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
//! [magnitude]
    split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
    Mat magI = planes[0];
//! [magnitude]
    vector<Mat> res = {complexI,magI};

    return res;
}

int findSharpness(vector<Mat> &res){

    Mat complexI = res.at(0);
    Mat magI = res.at(1);

//! [log]
    magI += Scalar::all(1);                    // switch to logarithmic scale
    log(magI, magI);
//! [log]

//! [crop_rearrange]
    // crop the spectrum, if it has an odd number of rows or columns
    magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));

    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    int cx = magI.cols/2;
    int cy = magI.rows/2;

    Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
    Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
    Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right

    Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);
//! [crop_rearrange]

//! [normalize]
    normalize(magI, magI, 0, 1, NORM_MINMAX);  // Transform the matrix with float values into a
                                              // viewable image form (float between values 0 and 1)
    // imshow("spectrum magnitude", magI);
//! [threshold]
    threshold(magI,magI, 0.2, 1, THRESH_BINARY);

    erode(magI,magI,Mat(),Point(-1,1));

//! [sharpness]
    double sharpness = countNonZero(magI)/static_cast<double>(magI.rows*magI.cols);

    return static_cast<int>(sharpness*100);
}

Mat deblur(Mat &complexI,int ksize, int i_row, int i_col){
  Mat kw = getGaussianKernel(ksize,0);//
  Mat k = kw*kw.t();
  vector<Mat> res = fft(k,i_row,i_col);
  Mat kernel = res.at(0);
  Mat r;
  // kernel.setTo(0.01,abs(kernel)<0.01);
  divide(complexI,kernel,r);
  Mat inverseTransform;
  idft(r,inverseTransform,DFT_REAL_OUTPUT);
  normalize(inverseTransform, inverseTransform, 0, 1, NORM_MINMAX);
  // inverseTransform.convertTo(inverseTransform, CV_8U);
  // imshow("Reconstructed", inverseTransform);
  return inverseTransform;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  static vector<chrono::microseconds> times;
  static vector<int> sharps;
  static int num_rect = 35;
  tag_detector::WindowSize srv;
  srv.request.w = 0;


  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    // imshow("view", cv_ptr->image);
    Mat gray, edge, draw;
    cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
    vector<Mat> res = fft(gray,gray.rows,gray.cols);
    int sharpness = findSharpness(res);
    // if (sharpness < 50 && num_rect < 35){
    //   client.call(srv);
    //   int winSize = static_cast<int>(srv.response.w);
    //   Mat sharp_image = deblur(res.at(0),srv.response.w,gray.rows,gray.cols);
    //   // imshow("deblur",sharp_image);
    // }
    auto start = chrono::high_resolution_clock::now();
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
    num_rect = 0;
    for(size_t i = 0; i < contours.size(); i++){
        approxPolyDP(contours[i], approxSquare, arcLength(Mat(contours[i]), true)*0.02, true);
        if(approxSquare.size() == 4 && isContourConvex(approxSquare) && contourArea(approxSquare)>100){
          boundRect = boundingRect( approxSquare );
          ar = boundRect.width/static_cast<double>(boundRect.height);
          if (ar>0.5 && ar <1.5){
            drawContours(cv_ptr->image, contours, i, Scalar(0, 255, 0),3); // fill GREEN
            num_rect++;
          }
        }
    }
    auto stop = chrono::high_resolution_clock::now();
    imshow("overlay", cv_ptr->image);
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    if (times.size() <= 100)
      times.push_back(duration);
      sharps.push_back(sharpness);

    if (times.size() == 100){
      chrono::microseconds sum;
      for (auto &n : times)
        sum += n;
      auto avg = sum/times.size();
      int sharp_avg = accumulate(sharps.begin(),sharps.end(),0.0)/sharps.size();
      ROS_INFO("Average computation time taken by function over 100 calls: %ld microseconds",avg.count());
      ROS_INFO("Average percent sharpness over 100 calls: %d%%",sharp_avg);
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
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image_raw", 1, imageCallback);
  client = nh.serviceClient<tag_detector::WindowSize>("set_blur_window_size");

  ros::spin();
  destroyAllWindows();
}
