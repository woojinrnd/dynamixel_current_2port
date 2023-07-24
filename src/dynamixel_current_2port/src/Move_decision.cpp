#include "Move_decision.hpp"
#include "dynamixel_current_2port/Select_Motion.h"

// Constructor
Move_Decision::Move_Decision()
    : FALL_FORWARD_LIMIT(60),
      FALL_BACK_LIMIT(-60),
      SPIN_RATE(500)
{
    // Init ROS
    ros::NodeHandle nh(ros::this_node::getName());

    boost::thread process_thread = boost::thread(boost::bind(&Move_Decision::processThread, this));
    boost::thread queue_thread = boost::thread(boost::bind(&Move_Decision::callbackThread, this));

}

Move_Decision::~Move_Decision()
{
}


void Move_Decision::process()
{
    int a = 0;
    cout << a << endl;
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

void Move_Decision::callbackThread()
{
    ros::NodeHandle nh(ros::this_node::getName());

    //Subscriber & Publisher
    // motion_index_pub_ = nh.advertise<std_msgs::Float32>("Select_Motion", 0);

    motion_index_server_ = nh.advertiseService("Select_Motion", playMotion);

    
    ros::Rate loop_rate(SPIN_RATE);
    while (nh.ok())
    {
        startMode();
        ros::spinOnce();
        loop_rate.sleep();
        //usleep(1000);
    }
}

//
// static bool playMotion(Move_Decision::Select_Motion::Request &req, Move_Decision::Select_Motion::Response &res)
// {
//     switch (req.num)
//     {
//     case 1:
//         res.ans = "Hello";
//         break;

//     case 2:
//         res.ans = "ROS";
//         break;

//     case 3:
//         res.ans = "World!";
//         break;
//     }
//     ROS_INFO("[MESSAGE] Select Number: %d ", req.num);
//     ROS_INFO_STREAM("[MESSAGE] " << res.ans);
//     return true;
// }


////////////About Publish
void Move_Decision::startMode()
{
    playMotion(Motion_Index::InitPose);
}


void Move_Decision::stopMode()
{
    playMotion(Motion_Index::Foward_4step);
}


void Move_Decision::playMotion(float motion_index)
{
    std_msgs::Float32 motion_msgs;
    motion_msgs.data = motion_index;

    motion_index_pub_.publish(motion_msgs);
    // ROS_INFO("%d",motion_msgs);
}

