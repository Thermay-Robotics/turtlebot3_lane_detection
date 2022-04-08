#include "turtlebot3_lane_detection/lane_detection.hpp"


LaneDetection::LaneDetection(): it(nh)
{
    cv::namedWindow("view");
    cv::namedWindow("imgCanny");
    cv::namedWindow("imgROI",cv::WINDOW_AUTOSIZE);
    cv::namedWindow("imgLines");
    cv::namedWindow("LaneDetection");
    cv::namedWindow("contour");
    //qr_pub = nh.advertise<LaneDetection::line_msg>("/line_detection", 1000);
    camera_sub = it.subscribe("/camera/color/image_raw", 1, &LaneDetection::imageCallback, this);
}

LaneDetection::~LaneDetection()
{
    cv::destroyAllWindows(); 
}

void LaneDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::Mat image_canny = CannyThreshold(image);

    cv::Mat imageRoi = defineROI(image_canny);
    cv::Mat line_img;
    cv::Mat img_LaneDetection = defineROI(image);

    std::vector<cv::Vec4i> lines; // will hold the results of the detection
    detectLines(imageRoi, lines);
    cv::cvtColor(imageRoi, line_img, cv::COLOR_GRAY2BGR );
    displayLines(line_img,"imgLines", lines);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(imageRoi,contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::RotatedRect> boundRect( contours.size() );

    for( size_t i = 0; i < contours.size(); i++ )
    {
        //approxPolyDP( contours[i], contours_poly[i], 3, true );
        //boundRect[i] = boundingRect( contours_poly[i] );
        boundRect[i] = minAreaRect( contours[i] );
    }    

    //Draw the contours
    cv::Mat contourImage(imageRoi.size(), CV_8UC3, cv::Scalar(0,0,0));
    cv::Scalar colors[3];
    colors[0] = cv::Scalar(255, 0, 0);
    colors[1] = cv::Scalar(0, 255, 0);
    colors[2] = cv::Scalar(0, 0, 255);

    cv::Vec4i meanLaneCoordinates(0,0,0,0);
    for (size_t idx = 0; idx < contours.size(); idx++) {
        cv::drawContours(contourImage, contours, idx, colors[idx % 3]);
        //cv::rectangle(contourImage, boundRect[idx].tl(), boundRect[idx].br(), colors[idx % 3], 2 );
    
        cv::Point2f rect_points[4];
        boundRect[idx].points( rect_points );

        for ( int j = 0; j < 4; j++ )
        {
            std::cout << rect_points[j] << " ";
            bool error_correction = false;
            if(j == 0){
                // To manage opencv vector error 
                if(rect_points[j].x - rect_points[j+1].x < rect_points[j+3].x - rect_points[j].x){
                    meanLaneCoordinates[j] +=  (rect_points[j].x + rect_points[j+1].x)/2;
                    meanLaneCoordinates[j+1] +=  (rect_points[j].y + rect_points[j+1].y)/2;
                    error_correction = true;
                }
                else{
                    meanLaneCoordinates[j] +=  (rect_points[j+3].x + rect_points[j].x)/2;
                    meanLaneCoordinates[j+1] +=  (rect_points[j+3].y + rect_points[j].y)/2;
                }
            }
            
            if(j==2){
                if((rect_points[j].x - rect_points[j+1].x < rect_points[j+3].x - rect_points[j].x)){
                    meanLaneCoordinates[j] +=  (rect_points[j].x + rect_points[j+1].x)/2;
                    meanLaneCoordinates[j+1] +=  (rect_points[j].y + rect_points[j+1].y)/2;
                }
                else{
                    meanLaneCoordinates[j] +=  (rect_points[j-1].x + rect_points[j].x)/2;
                    meanLaneCoordinates[j+1] +=  (rect_points[j-1].y + rect_points[j].y)/2;
                }
            }

            line(contourImage, rect_points[j], rect_points[(j+1)%4], colors[idx%3]);
        }
        std::cout << std::endl;
    }

    for (int i = 0; i < 4; i++)
    {
        meanLaneCoordinates[i] /= contours.size();
    }

    line( contourImage, cv::Point(meanLaneCoordinates[0],meanLaneCoordinates[1]),cv::Point(meanLaneCoordinates[2],meanLaneCoordinates[3]),cv::Scalar(0,0,255) );
    


    cv::imshow("contour", contourImage);

    std::vector<cv::Vec4i> lines_avg = computeAverageLines(imageRoi,lines);
    displayLines(img_LaneDetection,"LaneDetection",lines_avg);


    cv::imshow("imgCanny", image_canny);
    cv::imshow("imgROI", imageRoi);
    cv::imshow("view",image);

    cv::waitKey(30);
}

/**
 * @brief Canny Threshold camera image
 * 
 * // TODO find a better way to add parameters
 * 
 * @param im 
 * @return cv::Mat 
 */
cv::Mat LaneDetection::CannyThreshold(cv::Mat &im){
    cv::Mat imgGrayScale;
    cv::Mat imgBlurred;
    cv::Mat imgCanny;
    cv::cvtColor(im, imgGrayScale, CV_BGR2GRAY);        // convert to grayscale


    cv::GaussianBlur(imgGrayScale,  // input image
        imgBlurred,                 // output image
        cv::Size(5, 5),             // smoothing window width and height in pixels
        1.5);                       // sigma value, determines how much the image will be blurred

    cv::Canny(imgBlurred,           // input image
        imgCanny,                   // output image
        50,                        // low threshold
        150);                       // high threshold    

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
cv::Mat LaneDetection::defineROI(cv::Mat &im){
    cv::Mat imgROI(im);
    int heigth = im.size().height;
    int width = im.size().width;

    cv::Rect Rec((width/2) - 200, heigth - 200, 400, 200);

    cv::Mat ROI = imgROI(Rec);

    return ROI;
}

/**
 * @brief Standard Hough Line Transform
 * 
 * @param im 
 * @param lines 
 */
void LaneDetection::detectLines(cv::Mat &im, std::vector<cv::Vec4i> &lines){
    HoughLinesP(im, lines, 1, CV_PI/180, 5, 40, 10); // runs the actual detection
}

/**
 * @brief Display line based on OpenCv example
 * 
 * @param im 
 * @param lines 
 */
void LaneDetection::displayLines(cv::Mat &im,std::string name,std::vector<cv::Vec4i> &lines){
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line(im, cv::Point(lines[i][0], lines[i][1]),
        cv::Point( lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8 );
    }

    cv::imshow(name, im);
}

void LaneDetection::polyfit(cv::Vec4i &p,cv::Vec2d &param){
    param[0]= (double)(p[3] - p[1])/(double)(p[2]-p[0]);
    param[1] = p[1] - param[0] * p[0];
}

/**
 * @brief Compute the average for left ang right lanes 
 * 
 * @param lines 
 */
std::vector<cv::Vec4i> LaneDetection::computeAverageLines(cv::Mat &im,std::vector<cv::Vec4i> &lines){
    mid_lane.clear();

    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec2d parameters;
        polyfit(lines[i],parameters);

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
cv::Vec2d LaneDetection::computeMeanLane(std::vector<cv::Vec2d> &l){
    cv::Vec2d mean;

    double sum_slope = 0;
    double sum_y = 0;

    for(auto param : l){
        sum_slope += param[0];
        sum_y += param[1];
    }

    mean[0] = sum_slope / l.size();
    mean[1] = sum_y / l.size();
    
    return mean;
}

cv::Vec4i LaneDetection::makePoint(cv::Mat &im, cv::Vec2d &r){
    double slope = r[0];
    double y_int = r[1];

    cv::Vec4i point;

    point[1] = im.size().height;
    point[3] = (int)(point[1]*(3/5));

    point[0] = (int)((point[1] - y_int) / slope);
    point[2] = (int)((point[3] - y_int) / slope);

    return point;
}
