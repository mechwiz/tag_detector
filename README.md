**Michael Wiznitzer**

Coding Challenge


## Introduction
####  Objective
The goal of this project is to develop a ROS package to detect 2D tags in C++.

#### How it works
- A video should be taken of the 2D tags from your phone and placed into the [vid](vid/) directory. Make sure it is named tag_vid.MOV. Two sample vids are currently in there for test purposes.
- During implementation, the video is loaded into a ROS node and split into a sequence of images that are published to a second node. In this latter node, image processing is done using the OpenCV package to detect the tags. See the implementation section for a discussion of how the package architecture works to accomplish this task.
- A service allows the user to change the window size of the Gaussian Kernel applied to the published image message in real time.
- The average computation time to run the image processing functions for the tag detector is taken over 100 iterations and displayed on the terminal screen.
- The average _sharpness_ of the incoming image is evaluated over 100 iterations and displayed on the terminal screen.

## Installation
#### Prerequisits
- Linux running on Ubuntu 16.04 with ROS Kinetic

#### Dependencies
To install this package, clone it into your favorite workspace and run `catkin_make` in the root directory or make a new workspace and clone it into there as described below:
```bash
mkdir -p tag_ws/src; cd tag_ws/src; catkin_init_workspace
git clone git@github.com:mechwiz/tag_detector.git
cd ..; catkin_make; source devel/setup.bash
```

All of the required dependencies should be included (OpenCV, CV_Bridge, Image_Transport, etc...). This should hopefully just be a plug and play package.

## Implementation
#### Running The Demo
Currently the setup of the ROS package implements tag detection without illumination or deblurring correction. If you would like to turn either of them on, then set their corresponding parameters to true in the [launch](launch/tag_detector.launch) file. While the illumination correction will work (albeit not so well yet), the resulting image of the deblurring correction (even when turned on) will not be incorporated into the program flow until further development. As mentioned earlier, make sure that the video you want to feed into the program is located in the [vid](vid/) directory and is named **tag_vid.MOV**.

To launch the package, open up a terminal and run:
```bash
roslaunch tag_detector tag_detector.launch
```
A message will be displayed on the terminal saying
```
Waiting for GaussianBlur service call...
```
At this point, you should open up another terminal, and run the following rosservice command:
```bash
rosservice call /set_blur_window_size num
```
where **num** should be a user defined odd integer above zero for setting the GaussianBlur window size.

A live feed of the video should pop up with a rectangle detection overlay highlighted in green over the tags. You will also notice the average computation-time (just for implementing the rectangle detector) and sharpness printed out to the terminal as shown below after 100 iterations.

```
[ INFO] [1552076835.462454493]: Average computation time taken by function over 100 calls: 2481 microseconds
[ INFO] [1552076835.462519880]: Average percent sharpness over 100 calls: 89%
```
#### Nodes
#### Video to Image Publisher Node
[`vid2img_pub.cpp`](src/vid2img_pub.cpp)

This node takes in a video file named `tag_vid.MOV` from the [vid](vid/) directory , resizes it to a 640x480 image, and converts it into a stream of images. Each image is blurred using a Gaussian Blur with the window size specified from the `/set_blur_window_size` server and then published onto the `/image_raw` topic.

#### Image Processing Node
[`tag_detector.cpp`](src/tag_detector.cpp)

This node subscribes to the `/image_raw` topic and performs image processing on each image as follows:

- First, illumination correction is implemented if it is turned on. The algorithm implemented is **adaptive histogram equalization** to the lumincance channel of the image converted into the Lab color-space as described [here](https://stackoverflow.com/questions/24341114/simple-illumination-correction-in-images-opencv-c). The benefit of this over histogram equalization is that it computes several histograms over distinct sections of the image (instead of the whole image) and then uses them to redistribute the lightness values of the image. OpenCV's [CLAHE](https://docs.opencv.org/3.1.0/d5/daf/tutorial_py_histogram_equalization.html).
- The resulting image is converted to grayscale for further processing.
- The sharpness of the image is then determined by taking the FFT of the image. OpenCV's [DFT tutorial](https://docs.opencv.org/3.1.0/d8/d01/tutorial_discrete_fourier_transform.html) (Note, FFT is just an efficient implementation of DFT) about how to do this proved to be exremely helpful for accomplishing this. The normalized logarithmic-magnitude plot is really helpful for understanding the relationship between frequency and blurriness of an image. The lower the amount of high frequencies (denoted by brightness on the normalized plot), the more blurred the image is. Determining the thre exampleshold of what's considered to be high or a low frequency is kind of fuzzy, but using guess-check-and-revise, I discovered that anything above 0.2 seemed to be a good threshold in the normalized plot. Hence, my sharpness value is calculated by taking all the pixels in the nomalized image above that threshold and dividing them by the total amount of pixels in the image to get a percentage. While this relationship is definitely not linear with blurriness, the trend is what we'd expect. More blurriness means a lower shaprnerss percentage.For example, a GaussianBlur kernel size of 1 and 3 resulted in an average sharpness of about 95% and 76% for the video stream tested. Based on trial and error, sharpness values below 50% tended to be when the rectangle detector could no longer detect all of the tags, but only some.
- At this point, if the deblur parameter was turned on and the image was considered to be blurry (i.e. less than 35 rectangles were detected and also had a sharpness value less than 50%), then a deblur procedure followed. Much time was spent researching different filters (Wiener, Richardson-Lucy, etc...), but as we know what the windowSize of the GassianBlur filter was, I figured I could try using an inverse filter. The method I attempted to follow is described [here](http://www.owlnet.rice.edu/~elec539/Projects99/BACH/proj2/inverse.html) with an attempted python implementation [here](https://stackoverflow.com/questions/7930803/inverse-filter-of-spatially-convolved-versus-frequency-convolved-image). The idea is that if we applied some low-pass filter on the original image (in this case a Gaussian) to get a blurry image, then to get back the original image from the blurred image, we would need some sort of high-pass filter. The idea implemented to find this high-pass filter is to take the FFT of the low-pass filter and then invert all the elements. The issue is that many of the elements are close to 0, so inverting them would result in infinity which is not what we want. The solution was to do a psuedo-inverse where we only invert those elements that are not significantly close to 0, and set everything else to some low number such as 0.0001. While this seemed to work in the article, I was having issues implementing it that I still am in the process of debugging.
- After the above process, edge detection was implemented using OpenCV's **Canny Edge detector**. The resulting image was dilated to fill in any missing gaps and then further processed as described in the next step.
- The countours within the edge plot were found using OpenCV and then filtered such that only those with 4 points (determined by using OpenCV's **approxPolyDP** function), a convex contour, a contour area above a minimum value, and a bounded Rectangle with an aspect ratio between 0.5 and 1.5 were considered. Those contours were then drawn and overlayed on top of the original image.

## Demo & Further Improvements
#### Video
Two videos of this package in action is shown below. The left image demos detection done with a good lighting background and the right one in a dark room with the phone's flashlight.

Nice Background                                                                          | Using a flashlight
:---------------------------------------------------------------------------------:|:---------------------------------------------------------------------------------:
<img src="imgs/good_det.gif" width="400px" alt="" /> | <img src="imgs/flashlight.gif" width="400px" alt="" />

#### Further Improvements
- Improving the illumination correction algorithm
- Debugging the inverse filter for deblurring a blurred image
- Restructure code to use classes
