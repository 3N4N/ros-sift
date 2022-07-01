#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#define ERRSIFT "ERROR: Could not SIFT image\n"
#define ERRCAM "ERROR: Could not open camera\n"

// https://riptutorial.com/opencv/example/21401/get-image-from-webcam
int getWebcamImg(cv::Mat& frame, const int cam = 0)
{
  cv::VideoCapture camera(cam);
  if (!camera.isOpened()) {
    std::cerr << ERRCAM;
    return 1;
  }

  camera >> frame;

  return 0;
}

int getSIFTedImg(const cv::Mat& image,
                 cv::Mat& descriptors,
                 std::vector<cv::KeyPoint>& keypoints)
{
  if (image.empty()) {
    std::cerr << ERRSIFT;
    return 1;
  }

  // TODO:
  // learn why detectAndCompute() takes mask in the middle
  // while detect and compute separately defaults the mask to zero
  cv::Ptr<cv::SIFT> siftPtr = cv::SIFT::create();
  siftPtr->detect(image, keypoints);
  siftPtr->compute(image, keypoints, descriptors);

  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vis_sift_img");
  ros::NodeHandle n;
  ros::Rate lr(1);
  ros::Time time = ros::Time::now();

  image_transport::ImageTransport it_(n);
  image_transport::Publisher image_pub_ = it_.advertise("traj_output", 1);

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  cv_ptr->encoding = "bgr8";    // TODO: which encoding to use?
  cv_ptr->header.stamp = time;
  cv_ptr->header.frame_id = "traj_output";

  std::string fp_input = ros::package::getPath("sifter") + "/assets/input.jpg";
  const cv::Mat image = cv::imread(fp_input); //, cv::IMREAD_GRAYSCALE);
  cv::Mat desc_image;
  std::vector<cv::KeyPoint> kp_image;

  if (getSIFTedImg(image, desc_image, kp_image)) {
    std::cerr << ERRSIFT;
    return 1;
  }

  cv::Mat frame;
  cv::Mat desc_frame;
  std::vector<cv::KeyPoint> kp_frame;
  cv::Mat img_matches;

  while (ros::ok()) {
    if (!getWebcamImg(frame, 0)) {
      if (!getSIFTedImg(frame, desc_frame, kp_frame)) {
        std::vector<cv::DMatch> matches;
        cv::BFMatcher matcher(cv::NORM_L1, true);
        matcher.match(desc_frame, desc_image, matches);
        cv::drawMatches(frame, kp_frame, image, kp_image, matches, img_matches);

        cv_ptr->image = img_matches;
        image_pub_.publish(cv_ptr->toImageMsg());
        ROS_INFO("ImageMsg Sent");

        ros::spinOnce();
        lr.sleep();
      }
    }
  }

  return 0;
}
