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


#define TOP_BORDER_LINE   240	// == (IMG_H / 2)
#define BOTTOM_BORDER_LINE 460  // == (IMG_H - 20) change this value when Up-Down Neck Angle changed or to adjust with RV value
#define LEFT_BORDER_LINE  300	// == (IMG_W/2 - 20)
#define RIGHT_BORDER_LINE 260	// == (IMG_W/2 + 20)

#define CIRCLE_RADIUS 100 // 50 -> 60 -> 100

#define LEFT_EDGE_BORDER_LINE 0 + 15
#define RIGHT_EDGE_BORDER_LINE 640 - 15

#define RR_LINE_CURVATURE 0.004 // 0.005 -> 0.004
#define Y_VERTEX		  90

#define NOISE_DELETE_DELTA_X 120

#define IMG_W 640
#define IMG_H 480




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
    double Get_delta_x() const;
    

    // ********************************************** SETTERS ************************************************** //

    void Set_img_proc_line_det(bool img_proc_line_det);
    void Set_img_proc_no_line_det(bool img_proc_no_line_det);
    void Set_img_proc_goal_line_det(bool img_proc_goal_line_det);
    void Set_img_proc_huddle_det(bool img_proc_huddle_det);
    void Set_img_proc_wall_det(bool img_proc_wall_det);
    void Set_img_proc_stop_det(bool img_proc_stop_det);

    void Set_gradient(double gradient);
    void Set_delta_x(double delta_x);

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




    /////////////////////LINE////////////////////
    Point tmp_point_target = Point(IMG_W/2,IMG_H/2);
    Point point_target = Point(IMG_W/2,IMG_H);
    std::vector<std::vector<cv::Point>> contours_;
    bool roi_line_flg = true;
    double delta_x_list[3] = {0.f, 0.f, 0.f};
    double delta_x_ = 0;
    // cv::Mat final_binary_mask;
    cv::Mat final_binary_mask = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC1); 


    

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

    // Line mode
    int8_t gradient_ = 0; // Line_angle

    // No Line mode
    // delta_x : Center of window.x - Center of last captured line.x
    // delta_x > 0 : LEFT
    // delta_x < 0 : RIGHT

    /////////////////////////////////////////// Mutex ///////////////////////////////////////////
    // LINE Determine flg from img_proc
    mutable std::mutex mtx_img_proc_line_det_;
    mutable std::mutex mtx_img_proc_no_line_det_;
    mutable std::mutex mtx_img_proc_goal_det_;
    mutable std::mutex mtx_img_proc_huddle_det_;
    mutable std::mutex mtx_img_proc_wall_det_;
    mutable std::mutex mtx_img_proc_stop_det_;

    //Line Mode
    mutable std::mutex mtx_gradient;
    //No Line Mode
    mutable std::mutex mtx_delta_x;

};
