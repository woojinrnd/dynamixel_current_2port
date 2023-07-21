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

extern Motions motion;
extern Callback callback;

class Move_Decision
{
public:

    enum Motion_Index 
    {
        InitPose = 0,
        Foward_4step = 1,
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
    void stopMode();
    void playMotion(float motion_index);
    

    ros::Publisher motion_index_pub_;


private:
  const double FALL_FORWARD_LIMIT;
  const double FALL_BACK_LIMIT;
  const int SPIN_RATE;

  int stand_state_;
  int robot_status_;
  bool stop_fallen_check_;
};

#endif // MOVE_DECISION_H