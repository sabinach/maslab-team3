# Useful Guides

# ROS
### ROS Tutorials
http://wiki.ros.org/ROS/Tutorials
### Install ROS Melodic on Ubuntu 18.04
http://wiki.ros.org/melodic/Installation/Ubuntu (Desktop-Full Install)
### Create catkin_ws
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment (use catkin_make, NOT rosbuild/catkin build)
### Git clone a new ROS package       
```cd ~/team-3/catkin_ws/src/```       
```git clone <github_link_to_ros_package>```       
```cd ~/team-3/catkin_ws/```       
```catkin_make``` (make sure it builds without errors)       
```source ~/team-3/catkin_ws/devel/setup.bash```       


# OpenCV Tutorials       
### Online tutorials:       
https://docs.opencv.org/4.2.0/d6/d00/tutorial_py_root.html       
### What I found useful:       
https://github.com/sabinach/opencv_tutorials


# How To...
### SSH into NUC
ssh username@ip_address       
(username) team3       
(password) balsam3       
(ip_address) http://maslab.mit.edu/pollmemaybe/        
### Make a new script
```cd ~/team-3/catkin_ws/src/autobot/src```       
```touch script_name.py```       
```chmod +x script_name.py ```       
### Run your new script
```roscore ```       
```rosrun autobot _____.py```       
### Use git
```git pull```         
```git add <file_name>```          
```git commit -m "your message"```         
```git push```      
### Use vncserver
**On NUC**         
```vncserver```         
```vncserver -kill :<display_number>```         
**On your computer**         
Make sure yourcomputer is on the same wifi network as NUC         
Open VNC Viewer         
Get NUC IP: http://maslab.mit.edu/pollmemaybe/         
```<ip_address>:<display_number>```         


# Example Code (first run: ```roscore```)
### Run example publisher/subscriber
```rosrun autobot talker.py```       
```rosrun autobot listener.py```       
### Run camera publisher (specify front, left, right, dispenser cameras)
```rosrun autobot camera_pub.py```       
### Run camera editor (for opencv edits)
```rosrun autobot camera_edit.py```       
### Run marker publisher (display markers on rviz map)
```rosrun autobot markers_pub.py```       
### Run kitbot (WASD control)
```rosrun autobot kitbot.py```       
```rosrun autobot kbd_driver.py```       
### Run semibot (Gamepad control)
```rosrun autobot semibot.py```       
```rosrun autobot gamepad_driver.py```       
```rosrun autobot gamepad_mux.py```       
```rosrun autobot auto_driver.py```       
### Run digital electronics (for inner intake/sorting system)
```rosrun autobot intake_motor.py```       
```rosrun autobot color_sensor.py```       
```rosrun autobot dispenser_servo.py```       
```rosrun autobot dispenser_motor.py```       
```rosrun autobot pusher_motor.py```       


# Example Launch Files
### Launch camera_pub and camera_edit
```roslaunch autobot camera.launch```       
### Launch lidar
```roslaunch autobot lidar.launch```       
### Launch lidar with rviz (make sure plugged into data port, NOT power; add /scan topic to rviz)
```roslaunch autobot lidar_view.launch```       
### Launch hectorslam with rviz (add /map topic to rviz)
https://archit0994.wixsite.com/architshah/post/manage-your-blog-from-your-live-site       
```roslaunch autobot hector_slam.launch```       
### Launch planning visualization with hectorslam on rviz (add /markers topic to rviz)
```roslaunch autobot viz_markers_with_slam.launch```       
### Drive kitbot (WASD control)
```roslaunch autobot kitware.launch```       
### Drive semibot (Gamepad control)
```roslaunch autobot teleop.launch```       
### Launch onboard digital electronics
```roslaunch autobot digital_electronics.launch```       
### Launch cartographer
(https://github.com/mit-rss/cartographer_config)       
```roslaunch cartographer_config cartographer.launch```       
### Launch cartographer example (super cool)
(https://google-cartographer-ros.readthedocs.io/en/latest/demos.html)       
```roslaunch cartographer_ros demo_revo_lds.launch bag_filename:=${HOME}/Downloads/cartographer_paper_revo_lds.bag```       
```roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag```       




# Gamepad Mappings
Move Forward/Backward:	Left Joystick Vertical       
Turn Left/Right:	Right Joystick Horizontal       
Manual: 		LB       
Auto:			RB       
Servo:			Button-UP       
Intake:			Button-LEFT       
Dispenser Left:		Button-X       
Dispenser Right:	Button-B       
Pusher Left:		Button-Y       
Pusher Right:		Button-A       


# Visual Debugging Tools
### View images
```rqt```       
Go to Plugins -> Visualization -> Image View       
Select /images topic
### View ros graph
```rqt```       
Go to Plugins -> Introspection -> Node Graph       
Select “Nodes/Topics (all)”
Deselect "Hide: Dead sinks, Leaf topics"
### View topics
```rqt```       
Go to Plugins -> Topics -> Toic Monitor       


# ROS Topics
### /images
bgr images from camera
### /scan
laser scan points
### /map
map created via hector slam
### /markers
markers to be visualized on rviz






