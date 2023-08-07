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

#define c_img_num	2
#define Field Color_inf[0] // ORIGIN
#define Field_hsv Color_inf[0].img_hsv
#define Field_lab Color_inf[0].img_lab
#define Field_gray Color_inf[0].img_gray
#define Line Color_inf[1]
#define Line_hsv Color_inf[1].img_hsv
#define Line_lab Color_inf[1].img_lab

#define IMG_W 320
#define IMG_H 240
#define IMG_PIX_NUM 320 * 240
#define RL_CENTER 0
#define UD_CENTER 0

#define ZEROS_1 Mat::zeros(Size(IMG_W, IMG_H), CV_8UC1)
#define ZEROS_3 Mat::zeros(Size(IMG_W, IMG_H), CV_8UC3)

#define COLOR1 0
#define COLOR2 1
#define COLOR3 2
#define COLOR4 3

#define TOP_BORDER_LINE 10     // == (240 / 2) ����: 120, å�󿡼� ����� ��: 30
#define BOTTOM_BORDER_LINE 230 // == (240 - 20) change this value when Up-Down Neck Angle changed or to adjust with RV value
#define LEFT_BORDER_LINE 140   // == (160 - 20)
#define RIGHT_BORDER_LINE 180  // == (160 + 20)

#define CIRCLE_RADIUS 100 // 50 -> 60 -> 100

#define LEFT_EDGE_BORDER_LINE 0 + 25    // ����: 15 å�󿡼� ����� ��: 75
#define RIGHT_EDGE_BORDER_LINE 320 - 25 // ����: 15 å�󿡼� ����� ��: 75

#define RR_LINE_CURVATURE 0.0025f // 0.003// 0.005 -> 0.004
#define Y_VERTEX 55               // 50//30//90

#define NOISE_DELETE_DELTA_X 120

#define CONTOUR_AREA 200
#define MIN_CONTOUR_AREA 50
#define NO_LINE_DETECT_DX 160

typedef struct COLOR
{
    int H_max;
    int S_max;
    int V_max;
    int H_min;
    int S_min;
    int V_min;
    int H_Range;
    int S_Range;
    int V_Range;
    int H_val, S_val, V_val;
    int L_max;
    int L_min;
    int A_max;
    int A_min;
    int B_max;
    int B_min;

    int Label_num;

    Mat img_field;
    Mat img_hsv;
    Mat img_lab;
    Mat img_gray; // gray 이미지
    Mat Label;

#ifdef __cplusplus
    COLOR(int hmax = 180, int smax = 255, int vmax = 255, int hmin = 0, int smin = 0, int vmin = 0, int h_rng = 0, int h_rng2 = 0, int s_rng = 0, int v_rng = 0, int h_val = 0, int s_val = 0, int v_val = 0, int label_num = 0,
          int lmax = 255, int lmin = 0, int amax = 255, int amin = 0, int bmax = 255, int bmin = 0,
          Mat _img_1 = ZEROS_1, Mat _img_2 = ZEROS_1, Mat _img_3 = ZEROS_1, Mat _label = Mat::zeros(Size(IMG_W, IMG_H), CV_16UC1)) : H_max(hmax), S_max(smax), V_max(vmax), H_min(hmin), S_min(smin), V_min(vmin), H_Range(h_rng), S_Range(s_rng), V_Range(v_rng), H_val(h_val), S_val(s_val), V_val(v_val),
                                                                                                                                     L_max(lmax), L_min(lmin), A_max(amax), A_min(amin), B_max(bmax), B_min(bmin), Label_num(label_num), img_hsv(_img_1), img_lab(_img_2), img_field(_img_3), Label(_label)
    {
    }
#endif
} COLOR;

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

    int img_processing();
    void img_view(const cv::Mat &img, int mode);
    void RGBtoLAB();
    void RGBtoHSV();
    void FIELD_imgprocessing();
    void LINE_imgprocessing();
    double find_line_pt(Mat ss_img);
    void DRAW_OBJ();
    void MINMAX_SET();
    void COLOR_SET(int COLOR_mode);

    // ********************************************** 3D THREAD************************************************** //

    void realsense_thread();

    // Cam set
    const int realsense_width = 640;
    const int realsense_height = 480;
    const int realsense_fps = 30;

    // ********************************************** GETTERS ************************************************** //

    bool Get_img_proc_line_det() const;

    // ********************************************** SETTERS ************************************************** //

    void Set_img_proc_line_det(bool img_proc_line_det_);

    // ********************************************** running ************************************************** //

    std::vector<COLOR> Color_inf;

    int H_Range, H_Range2, H_max, H_min;
    int S_Range, S_max, S_min;
    int V_Range, V_max, V_min;
    int COLOR_MODE;

    int L_max, L_min;
    int A_max, A_min;
    int B_max, B_min;

    int H_val;
    int S_val;
    int V_val;

    Mat cam_img_1;
    Mat Capture_img;
    Mat Origin_img;
    Mat Origin_img2;
    Mat Depth_img;
    Mat Draw_img;

    // double tmp_slope;
    // double intercept_y;
    // double intercept_x;
    // double intercept_x_2;
    // int standard_prob_tt;
    // int standard_length;
    // int x_l, x_r, x_l_2, x_r_2;
    // int total_chk_pix_num;
    // int pass_pix_num;
    // double det_ratio;
    // double det_ratio_all;

    vector<bool> pix_num_chk;
    vector<double> vec_intercept_x;

    bool roi_line_flg;

    float dx;
    float noLineDetect_x;
    float noLineDetect_y;
    int REFERNCE_X = 160;
    int REFERNCE_Y;
    float target_x;
    Point Point_vector_x_y_save;
    Point Point_tmp_point_target_save;

    double delta_x;
    double DX;
    double G_DX;
    double direction_LR;
    double delta_x_list[3];
    double second_order_coefficient;
    // int tmp_i;
    Point point_target;
    Point tmp_point_target;

    bool line_det_flg;
    bool goal_line_detect_flg;
    bool no_line_det_flg;
    VideoCapture vcap_1;
    int hsv_mode = 1;

    bool cam_1_btn_flg;
    bool cam_1_finish;
    bool img_thread_finish_flg;
    bool load_st_flg;
    bool rec_st_flg;

    bool load_img_num_up_flg;
    bool load_img_num_down_flg;
    bool load_img_play_flg;

    int load_img_num;
    int rec_img_num;
    int cam_1_idx;
    int cam_2_idx;
    double cam_1_zoom; //

    bool CAMMODE;
    bool ROI_flg;
    bool mouse_flg;
    bool get_hsv_flg;
    bool capture_end_flg;

    void init();





private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    const int SPIN_RATE;

    // LINE Determine flg from img_proc
    bool img_proc_line_det_ = false;
};
