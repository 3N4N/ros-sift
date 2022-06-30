cmake_minimum_required(VERSION 3.0.2)
project(sifter)

set(EXE "sifter")

find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy roslib
)

include_directories()


set(OpenCV_DIR /usr/local/lib/cmake/opencv4/OpenCVConfig.cmake)
find_package(OpenCV 4.6.0 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${EXE} src/sifter.cc)
target_link_libraries(
  ${EXE}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

set_property(TARGET ${EXE} PROPERTY CXX_STANDARD 17)

file(COPY assets DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
