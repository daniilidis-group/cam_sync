# Camera synchronization server

This repo contains a ROS node for recording sychronized images from
several Pointgrey Cameras.

## Installation

Install the ROS flea3 drivers:


    git clone https://github.com/berndpfrommer/flea3.git
    git clone https://github.com/KumarRobotics/camera_base.git

Install and compile the code:


    git clone https://github.com/daniilidis-group/cam_sync.git
	catkin config -DCMAKE_BUILD_TYPE=Release
	catkin build


## Getting maximum USB3 performance:

If you run cam_sync with USB3 cameras, you will find that it's
sometimes difficult getting full frame
rate. Look [here](docs/usb_performance.md) for performance debugging
tips.
