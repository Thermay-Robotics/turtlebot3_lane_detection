# turtlebot3_lane_detection

Detect a road lane using RGB camera

# Notice

* The node subscribes to the camera topic detect the lane. A region of interest is defined at the bottom of the image. The ROI is used to detect lane. To do so, we detect each sides of the lane and compute a mean lane. Then, we compute the angle between the ordonate axis and the mid-lane. 
This value is then published. 

### Subscribed Topics

* ```/camera/color/image_raw``` ([sensor_msgs/Image])
    THe camera measurements

### Published Topics

* ``` /lane_detection/angle``` ([turtlebot3_lane_detection::line_msg])
    Publishes the angle betwenn the lane and the robot

* ```/lane_detection/image/image_lane``` ([sensor_msgs/Image])
    Publishes the detected lane and the mid lane 

# How to build
```
cd ~/catkin_ws/src/
git clone https://github.com/Thermay-Robotics/turtlebot3_lane_detection.git
cd ~/catkin_ws
catkin_make
```
# Run

Launch the camera node (please update the  subscribed topic camera name in the launch file)

Launch detection node

``` roslaunch turtlebot3_lane_detection turtlebot3_lane_detection.launch ```

if you want to run it with the camera, you can launch

``` roslaunch turtlebot3_lane_detection turtlebot3_detection_realsense.launch```


# Test environment

``` 
Ubuntu 20.04 LTS
ROS Noetic
```

``` 
Jetson Nano
Ubuntu 18.04 
ROS Melodic
```

