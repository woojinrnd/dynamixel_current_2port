#ifndef MOVE_DECISION_H
#define MOVE_DECISION_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <boost/thread.hpp>


#include "callback.hpp"
#include "dynamixel.hpp"
#include "Walkingpattern_generator.hpp"
#include "dynamixel_current_2port/Select_Motion.h"
#include "dynamixel_current_2port/Turn_Angle.h"


extern Motions motion;
extern Callback callback;

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


    Move_Decision();
    ~Move_Decision();

    void process();
    void processThread();
    void callbackThread();

    void startMode();
    // void stopMode();
    // void playMotion(float motion_index);
    void EmergencyPublish(bool emergency_);

    bool playMotion(dynamixel_current_2port::Select_Motion::Request &req, dynamixel_current_2port::Select_Motion::Response &res);
    bool turn_angle(dynamixel_current_2port::Turn_Angle::Request &req, dynamixel_current_2port::Turn_Angle::Response &res);
    
    //Publish & Subscribe
    ros::Publisher Emergency_pub_;

    //Server && Client
    ros::ServiceServer motion_index_server_;
    ros::ServiceServer turn_angle_server_;

    bool emergency_ = 1 ;


private:
    const double FALL_FORWARD_LIMIT;
    const double FALL_BACK_LIMIT;
    const int SPIN_RATE;

    int8_t stand_status_;
    int8_t motion_index_;
    int8_t running_mode_;
    int8_t turn_angle_;

    bool stop_fallen_check_;
};

#endif // MOVE_DECISION_H