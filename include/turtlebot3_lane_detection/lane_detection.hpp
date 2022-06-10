#ifndef LANE_DETECTION_H
#define LANE_DETECTION_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <algorithm>
#include <numeric>
#include <cstdlib>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/image_encodings.h>

#include "turtlebot3_lane_detection/line_msg.h"

class LaneDetection
{
private:
    ros::NodeHandle nh;
    ros::Publisher lane_pub;

    image_transport::Publisher image_lane_pub;
    image_transport::Subscriber camera_sub;
    image_transport::ImageTransport it;

    std::string camera_topic;

    std::vector<cv::Vec2d> mid_lane;

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    cv::Mat CannyThreshold(cv::Mat &im);
    cv::Mat defineROI(cv::Mat &im);

    void detectLines(cv::Mat &im, std::vector<cv::Vec4i> &lines);
    void displayLines(cv::Mat &im, std::string name, std::vector<cv::Vec4i> &lines);

    std::vector<cv::Vec4i> computeAverageLines(cv::Mat &im, std::vector<cv::Vec4i> &lines);
    cv::Vec4i makePoint(cv::Mat &im, cv::Vec2d &r);
    void polyfit(cv::Vec4i &p, cv::Vec2d &param);
    cv::Vec2d computeMeanLane(std::vector<cv::Vec2d> &);

    cv::Mat computeMeamLaneBoundingBox(cv::Mat &im);

public:
    LaneDetection();
    ~LaneDetection(){};
};

#endif