#include "turtlebot3_lane_detection/lane_detection.hpp"

LaneDetection::LaneDetection() : it(nh)
{
    nh.getParam("/lane_detection_node/camera_topic", camera_topic);

    lane_pub = nh.advertise<turtlebot3_lane_detection::line_msg>("/lane_detection/angle", 1000);
    camera_sub = it.subscribe(camera_topic, 1, &LaneDetection::imageCallback, this);

    // Image publishers
    image_lane_pub = it.advertise("/lane_detection/image/image_lane", 1);
}

void LaneDetection::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat image_canny = CannyThreshold(image);

    cv::Mat imageRoi = defineROI(image_canny);
    cv::Mat img_LaneDetection = defineROI(image);

    cv::Mat image_lane = computeMeamLaneBoundingBox(imageRoi);

    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr->encoding = "bgr8";
    cv_ptr->header.stamp = ros::Time::now();
    cv_ptr->header.frame_id = "camera_color_frame";
    cv_ptr->image = image_lane;

    cv::waitKey(30);

    image_lane_pub.publish(cv_ptr->toImageMsg());
}

/**
 * @brief Canny Threshold camera image
 *
 * // TODO find a better way to add parameters
 *
 * @param im
 * @return cv::Mat
 */
cv::Mat LaneDetection::CannyThreshold(cv::Mat &im)
{
    cv::Mat imgGrayScale;
    cv::Mat imgBlurred;
    cv::Mat imgCanny;
    cv::cvtColor(im, imgGrayScale, CV_BGR2GRAY); // convert to grayscale

    cv::GaussianBlur(imgGrayScale,   // input image
                     imgBlurred,     // output image
                     cv::Size(5, 5), // smoothing window width and height in pixels
                     1.5);           // sigma value, determines how much the image will be blurred

    cv::Canny(imgBlurred, // input image
              imgCanny,   // output image
              50,         // low threshold
              150);       // high threshold

    return imgCanny;
}

/**
 * @brief Define region of Interest
 *
 * // TODO parameters parsing
 *
 * @param im
 * @return cv::Mat
 */
cv::Mat LaneDetection::defineROI(cv::Mat &im)
{
    cv::Mat imgROI(im);
    int heigth = im.size().height;
    int width = im.size().width;

    cv::Rect Rec((width / 2) - 200, heigth - 200, 400, 200);

    cv::Mat ROI = imgROI(Rec);

    return ROI;
}

/**
 * @brief Display line based on OpenCv example
 *
 * @param im
 * @param lines
 */
void LaneDetection::displayLines(cv::Mat &im, std::string name, std::vector<cv::Vec4i> &lines)
{
    for (size_t i = 0; i < lines.size(); i++)
    {
        line(im, cv::Point(lines[i][0], lines[i][1]),
             cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
    }

    cv::imshow(name, im);
}

void LaneDetection::polyfit(cv::Vec4i &p, cv::Vec2d &param)
{
    param[0] = (double)(p[3] - p[1]) / (double)(p[2] - p[0]);
    param[1] = p[1] - param[0] * p[0];
}

/**
 * @brief Compute the average for left ang right lanes
 *
 * @param lines
 */
std::vector<cv::Vec4i> LaneDetection::computeAverageLines(cv::Mat &im, std::vector<cv::Vec4i> &lines)
{
    mid_lane.clear();

    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec2d parameters;
        polyfit(lines[i], parameters);

        mid_lane.push_back(parameters);
    }
    cv::Vec2d mid_avg = computeMeanLane(mid_lane);

    std::vector<cv::Vec4i> v = {makePoint(im, mid_avg)};

    return v;
}

/**
 * @brief Compute of one lane
 *
 * @param l lane
 * @return cv::Vec2d
 */
cv::Vec2d LaneDetection::computeMeanLane(std::vector<cv::Vec2d> &l)
{
    cv::Vec2d mean;

    double sum_slope = 0;
    double sum_y = 0;

    for (auto param : l)
    {
        sum_slope += param[0];
        sum_y += param[1];
    }

    mean[0] = sum_slope / l.size();
    mean[1] = sum_y / l.size();

    return mean;
}

cv::Vec4i LaneDetection::makePoint(cv::Mat &im, cv::Vec2d &r)
{
    double slope = r[0];
    double y_int = r[1];

    cv::Vec4i point;

    point[1] = im.size().height;
    point[3] = (int)(point[1] * (3 / 5));

    point[0] = (int)((point[1] - y_int) / slope);
    point[2] = (int)((point[3] - y_int) / slope);

    return point;
}

/**
 * @brief This method is used to compute the average lane from the bounding boxes
 *
 * @param im
 * @return cv::Vec2d
 */
cv::Mat LaneDetection::computeMeamLaneBoundingBox(cv::Mat &im)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(im, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    std::vector<cv::RotatedRect> boundRect(contours.size());

    // //Draw the contours
    cv::Mat contourImage(im.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Scalar colors[3];
    colors[0] = cv::Scalar(255, 0, 0);
    colors[1] = cv::Scalar(0, 255, 0);
    colors[2] = cv::Scalar(0, 0, 255);

    cv::Point2f mean_center;
    float mean_angle = 0;

    int line_lenght = 100;

    for (size_t idx = 0; idx < contours.size(); idx++)
    {
        boundRect[idx] = cv::minAreaRect(contours[idx]);

        mean_center += boundRect[idx].center;

        // This condition is used to overcome strange opencv's behaviour
        // see https://namkeenman.wordpress.com/2015/12/18/open-cv-determine-angle-of-rotatedrect-minarearect/ to understand
        if (boundRect[idx].size.width < boundRect[idx].size.height)
            mean_angle += abs(boundRect[idx].angle);
        else
            mean_angle -= 90 - abs(boundRect[idx].angle);

        cv::Point2f rect_points[4];
        boundRect[idx].points(rect_points);

        for (int j = 0; j < 4; j++)
        {
            line(contourImage, rect_points[j], rect_points[(j + 1) % 4], colors[idx % 3]);
        }
    }

    mean_center.x /= contours.size();
    mean_center.y /= contours.size();
    mean_angle /= contours.size();

    // Create line msg and publish it
    turtlebot3_lane_detection::line_msg msg;
    msg.angle = mean_angle;
    lane_pub.publish(msg);

    // Create line for display
    cv::Point mean_line;
    mean_line.x = (int)round(mean_center.x - line_lenght * sin(mean_angle * CV_PI / 180.0));
    mean_line.y = (int)round(mean_center.y - line_lenght * cos(mean_angle * CV_PI / 180.0));

    // Display line
    line(contourImage, mean_center, mean_line, cv::Scalar(0, 0, 255));

    return contourImage;
}