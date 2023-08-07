#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <librealsense2/rs.hpp>
#include <vector>

using namespace cv;
using namespace std;



class Img_proc
{
public:
    Img_proc();
    ~Img_proc() {};
    
    // ********************************************** 2D THREAD************************************************** //

    void webcam_thread();

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

    //Cam set
    const int webcam_width = 640;
    const int webcam_height = 480;
    const int webcam_fps = 30;
    const int webcam_id = 1;

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

    cv::Scalar HSV_lower_bound_yellow = (20, 20, 100); // HSV에서 노란색의 하한값
    cv::Scalar HSV_upper_bound_yellow = (32, 255, 255);

    cv::Scalar HSV_lower_bound_white = (0, 0, 0);
    cv::Scalar HSV_upper_bound_white = (179, 255, 255);



    static void on_trackbar(int, void *);
    void create_threshold_trackbar_W(const std::string &window_name);
    void create_threshold_trackbar_Y(const std::string &window_name);
    void create_color_range_trackbar(const std::string &window_name);
    cv::Mat extract_color(const cv::Mat &input_frame, const cv::Scalar &lower_bound, const cv::Scalar &upper_bound);
    cv::Mat detect_color_areas(const cv::Mat &input_frame, const cv::Scalar &contour_color, int threshold_value);



    // ********************************************** 3D THREAD************************************************** //

    void realsense_thread();

    //Cam set
    const int realsense_width = 640;
    const int realsense_height = 480;
    const int realsense_fps = 30;

    // ********************************************** GETTERS ************************************************** //

    bool Get_img_proc_line_det() const;
    
    // ********************************************** SETTERS ************************************************** //

    void Set_img_proc_line_det(bool img_proc_line_det_);


    // ********************************************** running ************************************************** //

    cv::VideoCapture vcap;
    Mat Origin_img;

    void RGB2HSV(const cv::Mat& rgb_image, cv::Mat& hsv_image);
    void RGB2LAB(const cv::Mat& rgb_image, cv::Mat& lab_image);
    void extractAndDisplayObject();
    // void extractAndDisplayObject2(cv::VideoCapture& cap, const cv::Scalar& hsv_lower, const cv::Scalar& hsv_upper, const cv::Scalar& lab_lower, const cv::Scalar& lab_upper);

    void init();



private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    const int SPIN_RATE;

    //LINE Determine flg from img_proc
    bool img_proc_line_det_ = false;

};
