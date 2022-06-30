cmake_minimum_required(VERSION 3.0.2)
project(sifter)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp rospy roslib image_transport cv_bridge
)

catkin_package(
  # INCLUDE_DIRS include
  # CATKIN_DEPENDS roscpp rospy roslib
)

include_directories()


set(OpenCV_DIR /usr/local/lib/cmake/opencv4/OpenCVConfig.cmake)
find_package(OpenCV 4.6.0 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(sift src/sifter.cc)
target_link_libraries(
  sift
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(publish src/publisher.cc)
target_link_libraries(
  publish
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# set_property(TARGET sift PROPERTY CXX_STANDARD 17)
