# CWRU ROS Camera Plugin #

This plugin is intended to resolve some of the functional problems with the old [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) camera plugin:

+ Can specify fx and fy directly instead of calculating to FOV.
+ Independant horizontal and vertical fx/fy can now be entered.
+ cx and cy no longer automatically snap to half the resolution (+0.5 for some reason).
+ Camera info and image topics can be named arbitrarily.
+ Redundant entry of intrinsics for the actual camera and the camera info (potentially allowing mismatched camera info) has been eliminated.
+ Camera coordinates now follow [the OpenCV axial convention](https://docs.opencv.org/master/pinhole_camera_model.png) instead of some random other convention.
+ Distortion model now matches [that used by OpenCV](https://docs.opencv.org/master/d9/d0c/group__calib3d.html) instead of the normalized model Gazebo uses.

### Installation ###

The improved camera plugin requires Gazebo 11 to function fully. Despite the dire warnings on [the Gazebo versioning page](http://gazebosim.org/tutorials/?tut=ros_wrapper_versions), anything in cwru\_robotics that worked will Gazebo 9 should work with 11, and installation is a one-time process. Simply follow [the instructions from OSRF](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install):

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install ros-melodic-gazebo11-dev ros-melodic-gazebo11-plugins ros-melodic-gazebo11-ros ros-melodic-gazebo11-ros-control ros-melodic-gazebo11-ros-pkgs
```

The package also requires OpenCV, cv_bridge, and image_manip:

```
sudo apt-get install ros-melodic-opencv-apps ros-melodic-cv-bridge
git clone https://github.com/lucasw/image_manip.git
```

### Use ###

The utility includes a self-contained camera macro to which all of the necessary parameters are passed as arguments. Simply add `<xacro:include filename="$(find cwru_ros_camera)/modular_camera.urdf" />` to the beginning of your file to make the macro available, and then call it:

```
<xacro:cwru_camera
	parent="camera_optical_link"
	image_topic="/camera/image_raw_color"
	info_topic="/camera/camera_info"
	k1="-0.433351"
	k2=" 0.257538"
	k3="-0.090683"
	p1=" 0.000512"
	p2=" 0.000263"
	
	f_x="1600.0"
	f_y="1600.0"
	c_x="1152"
	c_y="648"
	r_u="2304"
	r_v="1296"
/>
```

|  Argument | Description |
|----------:|-------------|
|parent     |The link you want the camera to attach to.|
|image_topic|The ROS topic you want the camera output published to.|
|info_topic |The ROS topic you want the camera info published to (optional).|
|k1         |K1 distortion parameter (optional).|
|k1         |K1 distortion parameter (optional).|
|k2         |K2 distortion parameter (optional).|
|k3         |K3 distortion parameter (optional).|
|p1         |P1 distortion parameter (optional).|
|p2         |P2 distortion parameter (optional).|
|f_x        |Horizontal focal length.|
|f_y        |Vertical focal length.|
|c_x        |Horizontal image center (px).|
|c_y        |Vertical image center (px).|
|r_u        |Horizontal image size (px).|
|r_v        |Horizontal image size (px).|

An example use can be found in `cwru_ros_camera/urdf/test_camera.urdf`. It is run automatically by `cwru_ros_camera/launch/test_camera.launch`.

### References ###
+ [vimjay](https://github.com/lucasw/vimjay)
+ [gazebo-ros-pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs)
