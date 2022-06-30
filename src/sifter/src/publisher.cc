#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

// https://riptutorial.com/opencv/example/21401/get-image-from-webcam
cv::Mat getWebcamImg(const int cam = 0)
{
  cv::VideoCapture camera(cam);
  if (!camera.isOpened()) {
    std::cerr << "ERROR: Could not open camera\n";
    return 1;
  }

  cv::Mat frame;
  camera >> frame;

  return frame;
}

cv::Mat getSIFTedImg(cv::Mat input)
{
  cv::Ptr<cv::SIFT> siftPtr = cv::SIFT::create();
  std::vector<cv::KeyPoint> keypoints;
  siftPtr->detect(input, keypoints);

  cv::Mat output;
  cv::drawKeypoints(input, keypoints, output);

  return output;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "draw_circle");
  ros::NodeHandle n;
  ros::Rate lr(1);
  ros::Time time = ros::Time::now();

  image_transport::ImageTransport it_(n);
  image_transport::Publisher image_pub_ = it_.advertise("traj_output", 1);

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  cv_ptr->encoding = "bgr8";    // TODO: which encoding to use?
  cv_ptr->header.stamp = time;
  cv_ptr->header.frame_id = "traj_output";

  while (ros::ok()) {
    cv::Mat input = getWebcamImg(0);
    cv::Mat output = getSIFTedImg(input);
    cv_ptr->image = output;
    image_pub_.publish(cv_ptr->toImageMsg());
    ROS_INFO("ImageMsg Send.");
    ros::spinOnce();
    lr.sleep();
  }

  return 0;
}
