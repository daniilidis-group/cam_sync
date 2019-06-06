# Camera synchronization server

This repo contains a ROS node for recording sychronized images from
several Pointgrey/FLIR Cameras.

## Installation


Install the FlyCapture SDK from FLIR. If the installer fails because
of missing dependencies, run ``apt --fix-broken install``, and retry
the installation. Do this several times, and eventually the
installation will succeed.

Install the ROS flea3 drivers:


    git clone https://github.com/berndpfrommer/flea3.git
    git clone https://github.com/berndpfrommer/camera_base.git

Install and compile the code:

    git clone https://github.com/daniilidis-group/cam_sync.git
	catkin config -DCMAKE_BUILD_TYPE=Release
	catkin build

VERY IMPORTANT: you MUST set the cameras to provide embedded
image information, or else ``cam_sync`` will not work.

For that, start the ``flycap`` GUI, go to the Configuration section,
find Advanced Camera Settings, select Memory Channels = 1, and then
under Embedded Image Information enable Timestamp, Gain, Shutter,
Brightness, Exposure, White balance and Frame counter. Hit Save before
exiting. This must be done for each camera, but the settings will
survive camera power cycling.

## FAQ

1) Why does ``cam_sync`` crash when I run multiple USB3 cameras with it?
This is often caused by [insufficient USB3 memory limits in the kernel](
https://www.flir.com/globalassets/support/iis/application-notes/tan2012007-using-linux-usb3.pdf).
On Ubuntu, edit ``/etc/default/grub`` to have a line like this:

    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"

then run ``sudo update grub``. Then, for immediate relief:

    sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'

2) Does ``cam_sync`` need calibration files to work?
No, but you want to provide them such that a correct camera_info topic
can be published.

3) Why are the cameras dropping frames and not working as expected?

- Check that you enabled embedded image information as described
  above.
- Is there excessive load on the host?
- Check that you are getting full USB3 performance. You need to have
  each camera on a separate host chip, they cannot
  share. [Here](docs/usb_performance.md) are some instructions for
  performance debugging USB3 performance issues.

