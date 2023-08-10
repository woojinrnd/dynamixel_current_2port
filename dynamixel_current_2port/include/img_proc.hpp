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
#include <mutex>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace cv;
using namespace std;

class Img_proc
{
public:
    Img_proc();
    ~Img_proc(){};

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

    // Cam set
    const int webcam_width = 640;
    const int webcam_height = 480;
    const int webcam_fps = 30;
    const int webcam_id = 0;

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

    static void on_trackbar(int, void *);
    void create_threshold_trackbar_W(const std::string &window_name);
    void create_threshold_trackbar_Y(const std::string &window_name);
    void create_color_range_trackbar(const std::string &window_name);
    cv::Mat extract_color(const cv::Mat &input_frame, const cv::Scalar &lower_bound, const cv::Scalar &upper_bound);
    cv::Mat detect_color_areas(const cv::Mat &input_frame, const cv::Scalar &contour_color, int threshold_value);

    // ********************************************** 3D THREAD************************************************** //

    void realsense_thread();

    // Cam set
    const int realsense_width = 640;
    const int realsense_height = 480;
    const int realsense_fps = 30;

    // ********************************************** GETTERS ************************************************** //

    bool Get_img_proc_line_det() const;
    bool Get_img_proc_no_line_det() const;
    bool Get_img_proc_goal_line_det() const;
    bool Get_img_proc_huddle_det() const;
    bool Get_img_proc_wall_det() const;
    bool Get_img_proc_stop_det() const;
    
    double Get_gradient() const;


    // ********************************************** SETTERS ************************************************** //

    void Set_img_proc_line_det(bool img_proc_line_det);
    void Set_img_proc_no_line_det(bool img_proc_no_line_det);
    void Set_img_proc_goal_line_det(bool img_proc_goal_line_det);
    void Set_img_proc_huddle_det(bool img_proc_huddle_det);
    void Set_img_proc_wall_det(bool img_proc_wall_det);
    void Set_img_proc_stop_det(bool img_proc_stop_det);
    
    void Set_gradient(double gradient);


    // ********************************************** running ************************************************** //

    cv::VideoCapture vcap;
    Mat Origin_img;

    void RGB2HSV(const cv::Mat &rgb_image, cv::Mat &hsv_image);
    void RGB2LAB(const cv::Mat &rgb_image, cv::Mat &lab_image);
    void saveParameters(const std::string &filename);
    void loadParameters(const std::string &filename);


    void extractAndDisplayObject();
    // void extractAndDisplayObject2(cv::VideoCapture& cap, const cv::Scalar& hsv_lower, const cv::Scalar& hsv_upper, const cv::Scalar& lab_lower, const cv::Scalar& lab_upper);

    void init();
    void LINE_imgprocessing();

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    const int SPIN_RATE;

    // HSV and LAB parameter values
    int h_min, h_max, s_min, s_max, v_min, v_max;
    int l_min, l_max, a_min, a_max, b_min, b_max;

    // LINE Determine flg from img_proc
    bool img_proc_line_det_;
    bool img_proc_no_line_det_;
    bool img_proc_goal_det_;
    bool img_proc_huddle_det_;
    bool img_proc_wall_det_;
    bool img_proc_stop_det_;


    //Line mode
    int8_t gradient_ = 0; // Line_angle


    //Mutex
    mutable std::mutex mtx_img_proc_line_det_;
    mutable std::mutex mtx_img_proc_no_line_det_;
    mutable std::mutex mtx_img_proc_goal_det_;
    mutable std::mutex mtx_img_proc_huddle_det_;
    mutable std::mutex mtx_img_proc_wall_det_;
    mutable std::mutex mtx_img_proc_stop_det_;

    mutable std::mutex mtx_gradient;


    std::vector<std::vector<cv::Point>> contours_;
};