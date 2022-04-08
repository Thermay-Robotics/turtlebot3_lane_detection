#include "turtlebot3_lane_detection/lane_detection.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_detection");

  LaneDetection line_detector;

  ros::spin();

  return 0;
}
