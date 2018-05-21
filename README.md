# autonomous_ws
Autonomous tests performed on laptop (use autonomous reference in jetson)

This software is for the autonomous task in URC. Sponsored by Eagle X developers.

## v1 software aranaia arco 

- [CUDA 9.0](https://developer.nvidia.com/compute/cuda/9.0/Prod/local_installers/cuda-repo-ubuntu1604-9-0-local_9.0.176-1_amd64-deb)

Commands
```
sudo dpkg -i cuda-repo-ubuntu1604-9-0-local_9.0.176-1_amd64.deb 
sudo apt-key add /var/cuda-repo-9-0-local/7fa2af80.pub
sudo apt-get update
sudo apt-get install cuda

```
- Nvidia Drivers

Commands
```
sudo apt-get install nvidia-375
```

- [ZED SDK](https://download.stereolabs.com/zedsdk/2.4/ubuntu_cuda9)
- [ROS Kinetic Ubuntu 16.04](http://wiki.ros.org/kinetic/Installation/Ubuntu)


### Ros Packages compiled

- [depthimage_to_laserscan](https://github.com/ros-perception/depthimage_to_laserscan)
- [zed_wrapper](https://github.com/stereolabs/zed-ros-wrapper)
- [navigation](http://wiki.ros.org/navigation?distro=kinetic)
- [navigation_2d](https://github.com/skasperski/navigation_2d)

For navigation package to function properly, be sure to install the required dependencies with the following commands:

```
rosdep install --from-paths ~/catkin_ws/src/navigation/ --ignore-src --rosdistro=kinetic -y
rosdep install --from-paths ~/catkin_ws/src/navigation_2d/ --ignore-src --rosdistro=kinetic -y
```

## Ariania-Arco for Jetson-Tx2 (Tegra)

-For [Ros](http://wiki.ros.org/kinetic/Installation/Ubuntu) in Tegra:

Commands
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt-get update

sudo apt-get install ros-kinetic-ros-base

sudo c_rehash /etc/ssl/certs

sudo update-ca-certificates

sudo rosdep init

rosdep update

sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```
