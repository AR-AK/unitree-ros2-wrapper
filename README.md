# ROS2 Wrapper of Unitree SDK for Go1 Robot

# About
This is a ROS2 Wraper package to replace the developer api for the no-longer supported Unitree_Legged_SDK. The wrppaer is streamlined to focus only on motion.

# Dependencies
- ROS2 (desktop full)
- Boost (Version 16.5, newer versions seem to build with errors or break stuff with UnitreeLeggedSDK)

```
Boost install instructions:

Step 1:
https://www.boost.org/users/history/version_1_65_0.html
-- Download: boost_1_65_0.tar.gz

Step 2:
Open terminal and run:
cd Downloads/
tar -xzf boost_1_65_0.tar.gz 

Step 3:
Once unpackaged:
cd boost_1_65_0/

Step 4:
./bootstrap.sh

Step 5:
sudo ./b2

Step 6:
sudo ./b2 install
```

- OpenCV2
```
pip install opencv-python
```
- udev
```
# In terminal:
sudo apt update
sudo apt install libudev-dev
```
# How to install
- Install the package to your home directory.

- Ensure all dependencies are installed

- In the terminal navigate to the head directory
```
cd ~/Ros2-Humble-interface-for-Unitree-Go1
```
- Build the package
```
Colcon Build
```

- Source bash.rc
```
source install/setup.bash
```

- Connect to the Go1 wireless network

- Run the package launch file
```
ros2 launch unitree_ros2_cpp go1.launch.py
```
