#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

int main()
{
  string fp_inimg = ros::package::getPath("sifter") + "/assets/input.jpg";
  string fp_outimg = ros::package::getPath("sifter") + "/assets/output.jpg";
  cout << fp_inimg << "\n";

  cout << "OpenCV Version: " << CV_VERSION << "\n";
  const cv::Mat input = cv::imread(fp_inimg, cv::IMREAD_GRAYSCALE);

  cv::Ptr<cv::SIFT> siftPtr = cv::SIFT::create();
  std::vector<cv::KeyPoint> keypoints;
  siftPtr->detect(input, keypoints);

  cv::Mat output;
  cv::drawKeypoints(input, keypoints, output);

  // cv::imwrite(fp_outimg, output);

  cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  cv::imshow("image", output);
  cv::waitKey(0);
  cv::destroyWindow("image");

  return 0;
}
