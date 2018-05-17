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

For navigation package to function properly, be sure to install the required dependencies with the following command:

```
rosdep install --from-paths ~/catkin_ws/src/navigation/ --ignore-src --rosdistro=kinetic -y
```