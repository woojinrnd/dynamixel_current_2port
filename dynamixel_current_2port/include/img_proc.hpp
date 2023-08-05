#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <librealsense2/rs.hpp>



class Img_proc
{
public:
    Img_proc();
    // ~Img_proc();
    void topic_publish(const std::string &topic_name)
    {
        pub = nh.advertise<std_msgs::Float32>(topic_name, 1000);
    }

    void publish_message(float msg_data)
    {
        std_msgs::Float32 msg;
        msg.data = msg_data;
        pub.publish(msg);
    }

    // Line Processing
    // delta_x :화면의 중앙.x - 마지막으로 포착한 라인의 중심점.x
    // delta_x > 0 : LEFT
    // delta_x < 0 : RIGHT
    // static double delta_x = 0;

    // check the variable sharing with multi thread
    int aaaa = -25;

    int threshold_value_white = 127;
    int threshold_value_yellow = 127;
    const int max_value = 255;
    int hue_lower = 0;
    int hue_upper = 179;
    int saturation_lower = 0;
    int saturation_upper = 255;
    int value_lower = 0;
    int value_upper = 255;

    cv::Scalar blue_color = (255, 0, 0);
    cv::Scalar green_color = (0, 255, 0);
    cv::Scalar red_color = (0, 0, 255);

    cv::Scalar lower_bound_yellow = (20, 20, 100); // HSV에서 노란색의 하한값
    cv::Scalar upper_bound_yellow = (32, 255, 255);

    cv::Scalar lower_bound_white = (0, 0, 0);
    cv::Scalar upper_bound_white = (179, 255, 255);

    // cv::Scalar blue_color;
    // cv::Scalar green_color;
    // cv::Scalar red_color;

    // cv::Scalar lower_bound_yellow; // HSV에서 노란색의 하한값
    // cv::Scalar upper_bound_yellow;

    // cv::Scalar lower_bound_white;
    // cv::Scalar upper_bound_white;

    static void on_trackbar(int, void *);
    void create_threshold_trackbar_W(const std::string &window_name);
    void create_threshold_trackbar_Y(const std::string &window_name);
    void create_color_range_trackbar(const std::string &window_name);
    cv::Mat extract_color(const cv::Mat &input_frame, const cv::Scalar &lower_bound, const cv::Scalar &upper_bound);
    cv::Mat detect_color_areas(const cv::Mat &input_frame, const cv::Scalar &contour_color, int threshold_value);
    // void webcam_thread();
    // void realsense_thread();

    // ********************************************** GETTERS ************************************************** //
    cv::Scalar getBlueColor() const
    {
        return blue_color;
    }

    cv::Scalar getGreenColor() const
    {
        return green_color;
    }

    cv::Scalar getRedColor() const
    {
        return red_color;
    }

    cv::Scalar getLowerBoundYellow() const
    {
        return lower_bound_yellow;
    }

    cv::Scalar getUpperBoundYellow() const
    {
        return upper_bound_yellow;
    }

    cv::Scalar getLowerBoundWhite() const
    {
        return lower_bound_white;
    }

    cv::Scalar getUpperBoundWhite() const
    {
        return upper_bound_white;
    }

    // ********************************************** SETTERS ************************************************** //
    void setBlueColor(const cv::Scalar &color)
    {
        blue_color = color;
    }

    void setGreenColor(const cv::Scalar &color)
    {
        green_color = color;
    }

    void setRedColor(const cv::Scalar &color)
    {
        red_color = color;
    }

    void setLowerBoundYellow(const cv::Scalar &lower)
    {
        lower_bound_yellow = lower;
    }

    void setUpperBoundYellow(const cv::Scalar &upper)
    {
        upper_bound_yellow = upper;
    }

    void setLowerBoundWhite(const cv::Scalar &lower)
    {
        lower_bound_white = lower;
    }

    void setUpperBoundWhite(const cv::Scalar &upper)
    {
        upper_bound_white = upper;
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
};
