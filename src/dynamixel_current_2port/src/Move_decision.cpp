#include "Move_decision.hpp"
#include "dynamixel_current_2port/Select_Motion.h"

// Constructor
Move_Decision::Move_Decision()
    : FALL_FORWARD_LIMIT(60),
      FALL_BACK_LIMIT(-60),
      SPIN_RATE(500),
      stand_status_(0),
      running_mode_(0),
      motion_index_(0)
{
    // Init ROS
    ros::NodeHandle nh(ros::this_node::getName());

    boost::thread process_thread = boost::thread(boost::bind(&Move_Decision::processThread, this));
    boost::thread queue_thread = boost::thread(boost::bind(&Move_Decision::callbackThread, this));

}

Move_Decision::~Move_Decision()
{
}


///////////////////////////  Process Thread   /////////////////////////
void Move_Decision::process()
{
    motion_index_ = 3;
    turn_angle_ = 39;
    // cout << a << endl;
}

void Move_Decision::processThread()
{
    bool result = false;

    // set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    // node loop
    while (ros::ok())
    {
        process();

        // relax to fit output rate
        loop_rate.sleep();
    }
}


///////////////////////////  Callback Thread   /////////////////////////

void Move_Decision::callbackThread()
{
    ros::NodeHandle nh(ros::this_node::getName());

    //Subscriber & Publisher
    Emergency_pub_ = nh.advertise<std_msgs::Bool>("Emergency", 0);

    // Server
    motion_index_server_ = nh.advertiseService("Select_Motion", &Move_Decision::playMotion, this);
    turn_angle_server_ = nh.advertiseService("Turn_Angle", &Move_Decision::turn_angle, this);

    ros::Rate loop_rate(1);
    while (nh.ok())
    {
        startMode();

        ros::spinOnce();
        loop_rate.sleep();
        //usleep(1000);
    }
}

//Server part
bool Move_Decision::playMotion(dynamixel_current_2port::Select_Motion::Request &req, dynamixel_current_2port::Select_Motion::Response &res)
{
    if ((req.finish == true) && (stand_status_ == Stand_Status::Stand))
    {
        switch (motion_index_)
        {
            case 0:
            res.select_motion = Motion_Index::InitPose;
            break;
            
            case 1:
            res.select_motion = Motion_Index::Forward_4step;
            break;
            
            case 2:
            res.select_motion = Motion_Index::Left_2step;
            break;
            
            case 3:
            res.select_motion = Motion_Index::Step_in_place;
            break;
            
            case 4:
            res.select_motion = Motion_Index::Right_2step;
            break;
            
            case 5:
            res.select_motion = Motion_Index::Back_4step;
            break;
        }
    }

    ROS_INFO("[MESSAGE] SM Request : %d ", req.finish);
    ROS_INFO("[MESSAGE] SM Response : %d", res.select_motion);
    return true;
}

bool Move_Decision::turn_angle(dynamixel_current_2port::Turn_Angle::Request &req, dynamixel_current_2port::Turn_Angle::Response &res)
{
    if ((req.finish == true) && (stand_status_ == Stand_Status::Stand))
    {
        //img_procssing
        res.turn_angle = turn_angle_;
    }

    ROS_INFO("[MESSAGE] TA Request : %d ", req.finish);
    ROS_INFO("[MESSAGE] TA Response : %d", res.turn_angle);
    return true;
}


///////////////////////////////////////// About Publish /////////////////////////////////////////
void Move_Decision::startMode()
{
    // emergency_ = 0;
    EmergencyPublish(emergency_);
}


// void Move_Decision::stopMode()
// {
//     playMotion(Motion_Index::Foward_4step);
// }


//Emergency Stop
//0 : Stop
//1 : Keep Going (Option)
void Move_Decision::EmergencyPublish(bool _Emergency)
{
    std_msgs::Bool emergency;
    emergency.data = _Emergency;

    Emergency_pub_.publish(emergency);
    // ROS_INFO("%d",emergency);
}

