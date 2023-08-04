#ifndef MOVE_DECISION_H
#define MOVE_DECISION_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
// #include <mutex>
#include <string.h>

#include "dynamixel_current_2port/Select_Motion.h"
#include "dynamixel_current_2port/Turn_Angle.h"
#include "dynamixel_current_2port/UD_NeckAngle.h"
#include "dynamixel_current_2port/RL_NeckAngle.h"
#include "img_proc.hpp"

using namespace std;

class Move_Decision
{
public:
    enum Motion_Index
    {
        InitPose = 0,
        Forward_4step = 1,
        Left_2step = 2,
        Step_in_place = 3,
        Right_2step = 4,
        Back_4step = 5,
        FWD_UP = 6,
        BWD_UP = 7,
    };

    enum Running_Mode
    {
        LINE_MODE = 0,
        NO_LINE_MODE = 1,
        STOP_MODE = 2,
        WAKEUP_MODE = 3,
        GOAL_MODE = 4,
        HUDDLE_MODE = 5,
        WALL_MODE = 6,
    };

    enum Stand_Status
    {
        Stand = 0,
        Fallen_Forward = 1,
        Fallen_Back = 2,
    };
    
    string Str_InitPose = "InitPose";
    string Str_Forward_4step = "Forward_4step";
    string Str_Left_2step = "Left_2step";
    string Str_Step_in_place = "Step_in_place";
    string Str_Right_2step = "Right_2step";
    string Str_Back_4step = "Back_4step";
    string Str_FWD_UP = "FWD_UP";
    string Str_BWD_UP = "BWD_UP";


    string Str_LINE_MODE = "LINE_MODE";
    string Str_NO_LINE_MODE = "NO_LINE_MODE";
    string Str_STOP_MODE = "STOP_MODE";
    string Str_WAKEUP_MODE = "WAKEUP_MODE";
    string Str_GOAL_MODE = "GOAL_MODE";
    string Str_HUDDLE_MODE = "HUDDLE_MODE";
    string Str_WALL_MODE = "WALL_MODE";

    
    Move_Decision();
    ~Move_Decision();


    // ********************************************** PROCESS THREAD************************************************** //

    void process();
    void processThread();
    void LINE_mode();
    void NOLINE_mode();
    void STOP_mode();
    void WAKEUP_mode();
    void GOAL_LINE_mode();
    void HUDDLE_mode();
    void WALL_mode();

    // ********************************************** MoveDecision THREAD ************************************************** //

    void Running_Mode_Decision();
    void MoveDecisionThread();

    // ********************************************** CALLBACK THREAD ************************************************** //

    void callbackThread();
    void startMode();
    // void stopMode();
    // void playMotion(float motion_index);
    void EmergencyPublish(bool _emergency);

    bool playMotion(dynamixel_current_2port::Select_Motion::Request &req, dynamixel_current_2port::Select_Motion::Response &res);
    bool turn_angle(dynamixel_current_2port::Turn_Angle::Request &req, dynamixel_current_2port::Turn_Angle::Response &res);
    bool Move_UD_NeckAngle(dynamixel_current_2port::UD_NeckAngle::Request &req, dynamixel_current_2port::UD_NeckAngle::Response &res);
    bool Move_RL_NeckAngle(dynamixel_current_2port::RL_NeckAngle::Request &req, dynamixel_current_2port::RL_NeckAngle::Response &res);

    void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg);

    // Publish & Subscribe
    ros::Publisher Emergency_pub_;
    ros::Subscriber imu_data_sub_;

    // Server && Client
    ros::ServiceServer motion_index_server_;
    ros::ServiceServer turn_angle_server_;
    ros::ServiceServer UD_NeckAngle_server_;
    ros::ServiceServer RL_NeckAngle_server_;


    // ********************************************** FUNCTION ************************************************** //

    Eigen::Vector3d convertRotationToRPY(const Eigen::Matrix3d &rotation);
    Eigen::Vector3d convertQuaternionToRPY(const Eigen::Quaterniond &quaternion);
    void Motion_Info();
    void Running_Info();


    // ********************************************** GETTERS ************************************************** //

    bool Get_Emergency_() const;
    int8_t Get_motion_index_() const;
    int8_t Get_stand_status_() const;
    int8_t Get_running_mode_() const;
    double Get_turn_angle_() const;

    bool Get_ProcessON() const;
    bool Get_MoveDecisionON() const;
    bool Get_CallbackON() const;

    bool Get_goal_line_det_flg() const;
    bool Get_line_det_flg() const;
    bool Get_no_line_det_flg() const;
    bool Get_huddle_det_flg() const;
    bool Get_wall_det_flg() const;
    bool Get_stop_det_flg() const;


    double Get_gradient() const;
    double Get_delta_x() const;

    double Get_RL_NeckAngle() const;
    double Get_UD_NeckAngle() const;


    // ********************************************** SETTERS ************************************************** //

    void Set_Emergency_(bool Emergency_);
    void Set_motion_index_(int8_t motion_index_);
    void Set_stand_status_(int8_t stand_status_);
    void Set_running_mode_(int8_t running_mode_);
    void Set_turn_angle_(double turn_angle_);

    void Set_ProcessON(bool ProcessON_);
    void Set_MoveDecisionON(bool MoveDecisionON_);
    void Set_CallbackON(bool CallbackON_);

    void Set_goal_line_det_flg(bool goal_line_det_flg);
    void Set_line_det_flg(bool line_det_flg);
    void Set_no_line_det_flg(bool no_line_det_flg);
    void Set_huddle_det_flg(bool huddle_det_flg);
    void Set_wall_det_flg(bool wall_det_flg);
    void Set_stop_det_flg(bool stop_det_flg);


    void Set_gradient(double gradient);
    void Set_delta_x(double delta_x);
    void Set_RL_NeckAngle(double RL_NeckAngle_);
    void Set_UD_NeckAngle(double UD_NeckAngle_);


    // ********************************************** IMG_PROC ************************************************** //

    // StraightLine
    bool straightLine;
    double margin_gradient = 5; // margin of straight line
    void StraightLineDecision(double gra, double mg_gra);
    double Angle_toBeStraight = 40; //max or min
    
    //If no find line (NO_LINE_MODE)
    //delta_x : Center of window.x - Center of last captured line.x 
    //delta_x > 0 : LEFT
    //delta_x < 0 : RIGHT
    //Out of Range -> A straight trun walking
    double Angle_ToFindLine = 10; //max or min

    //Actural send turn angle
    double Actural_angle = 0;

    //check the variable sharing with multi thread
    // int aaaa = -25;

private:
    const double FALL_FORWARD_LIMIT;
    const double FALL_BACK_LIMIT;
    const int SPIN_RATE;

    int8_t motion_index_;
    int8_t stand_status_;
    int8_t running_mode_;

    //Body Angle
    //Counter Clock Wise(+)
    //LEFT(+) / RIGHT(-)
    double turn_angle_ = 0;

    //Neck 
    //Counter Clock Wise(+)
    //LEFT(+) / RIGHT(-)
    double RL_NeckAngle_ = 0;
    //Counter Clock Wise(+)
    //UP(+) / DOWN(-)
    double UD_NeckAngle_ = 0;

    //Running mode
    bool goal_line_det_flg = false;
    bool line_det_flg = false;
    bool no_line_det_flg = false;
    bool huddle_det_flg = false;
    bool wall_det_flg = false;
    bool stop_det_flg = false;

    bool stop_fallen_check_;
    double present_pitch_;
    double present_roll_;

    bool Emergency_;

    /// Thread switch ///
    bool ProcessON_;
    bool MoveDecisionON_;
    bool CallbackON_;

    /// Img_Proc ///
    int8_t gradient; // Line_angle
    double delta_x = 0;

};

#endif // MOVE_DECISION_H