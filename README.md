# ROS SIFT

This project matches a saved image with the frames captured with
webcam and publishes the matches as an image to RViz.

## Requirements

- OpenCV
- ROS Noetic
- RViz
- Catkin tools


## Instructions

```bash

git clone git@github.com:3N4N/ros-sift-surf.git
cd ros-sift-surf

catkin build
source devel/setup.bash

rosrun sifter publish
```

### Part 1: RViz
1. Open RViz

    ```bash
    rosrun rviz rviz
    ```

2. Add an Image display from the Add button on the bottom left corner
   of RViz.

3. The publisher is publishing on the topic `traj_output`. Set the
   topic of the Image display to this topic.

### Part 2: Publish

1. Clone the repository

    ```bash
    git clone git@github.com:3N4N/ros-sift-surf.git
    cd ros-sift-surf
    ```

2. Build the ROS package

    ```bash
    catkin build
    source devel/setup.bash
    ```

3. Run the publisher node

    ```bash
    rosrun sifter publish /path/to/image
    ```


## Demo

In the following demo, the image captured from webcam (on the left) is
matched with the image saved in my computer (on the right).

https://user-images.githubusercontent.com/32037751/176884721-f3c053b1-154b-470f-9a38-49819c8ae87a.mp4
