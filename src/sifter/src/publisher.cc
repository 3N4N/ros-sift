#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>


// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "talker");
//   ros::NodeHandle n;

//   ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);
//   ros::Rate loop_rate(10);

//   int count = 0;
//   while (ros::ok()) {
//     std_msgs::String msg;

//     std::stringstream ss;
//     ss << "hello world " << count;
//     msg.data = ss.str();

//     ROS_INFO("%s", msg.data.c_str());

//     pub.publish(msg);

//     ros::spinOnce();

//     loop_rate.sleep();
//     ++count;
//   }

//   return 0;
// }

int main(int argc, char **argv)
{
  int trajectory[3] = { 512, 612, 712 };
  cv::Mat image(1024, 1024, CV_8UC3);

  for (int i = 0; i < 3; i++) {
    cv::circle(image, cv::Point(trajectory[i], trajectory[i]), 100, CV_RGB(255, 0, 0));
  }

#if 0
  cv::namedWindow("image", CV_WINDOW_NORMAL);
  cv::resizeWindow("image", 1024, 1024);
  cv::imshow("image", image);
  cv::waitKey(0);
  cv::destroyWindow("image");
#endif

#if 1
  ros::init(argc, argv, "draw_circle");
  ros::NodeHandle n;
  image_transport::ImageTransport it_(n);
  image_transport::Publisher image_pub_ = it_.advertise("traj_output", 1);
  ros::Rate lr(1);

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

# if 1
  ros::Time time = ros::Time::now();
  cv_ptr->encoding = "bgr8";
  cv_ptr->header.stamp = time;
  cv_ptr->header.frame_id = "/traj_output";
# endif

  while (ros::ok()) {
    cv_ptr->image = image;
    image_pub_.publish(cv_ptr->toImageMsg());
    ROS_INFO("ImageMsg Send.");
    ros::spinOnce();
    lr.sleep();
  }

#endif
  return 0;
}
