# autonomous_ws
Autonomous tests performed on laptop (use autonomous reference in jetson)

This software is for the autonomous task in URC. Sponsored by Eagle X developers.

## v1 software aranaia arco 

- [ZED SDK](https://download.stereolabs.com/zedsdk/2.4/ubuntu_cuda9)



### Ros Packages compiled

- [depthimage_to_laserscan](https://github.com/ros-perception/depthimage_to_laserscan)
- [zed_wrapper](https://github.com/stereolabs/zed-ros-wrapper)
- [navigation](http://wiki.ros.org/navigation?distro=kinetic)

For navigation package to function properly, be sure to install the following dependencies

```
sudo apt-get install libasound2-dev libbullet-dev libbulletsoftbody2.83.6 libcaca-dev libcppunit-1.13-0v5 libcppunit-dev libpulse-dev libsdl-image1.2 libsdl-image1.2-dev libsdl1.2-dev libsdl1.2debian libslang2-dev

```