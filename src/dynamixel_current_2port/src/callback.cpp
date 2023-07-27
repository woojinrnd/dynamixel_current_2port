#include "callback.hpp"

extern Dxl dxl;
extern Motions motion;

Callback::Callback()
{
    // ros::NodeHandle nh(ros::this_node::getName());

    // boost::thread queue_thread = boost::thread(boost::bind(&Callback::callbackThread, this));
}

sensor_msgs::JointState joint_state;

void Callback::JointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_command)
{
    for (int i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        Goal_joint_[i] = joint_command->position[i];
        dxl.SetThetaRef(Goal_joint_);
    }
}

void Callback::FSRsensorCallback(const std_msgs::UInt8::ConstPtr &FSR)
{
    L_value = FSR->data; // Left_foot_FSR
    R_value = FSR->data; // Right_foot_FSR
}

void Callback::IMUsensorCallback(const sensor_msgs::Imu::ConstPtr &IMU)
{
    // ROS_INFO("Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
    //  IMU->linear_acceleration.x, IMU->linear_acceleration.y, IMU->linear_acceleration.z,
    //  IMU->angular_velocity.x, IMU->angular_velocity.y, IMU->angular_velocity.z,
    //  IMU->orientation.x, IMU->orientation.y, IMU->orientation.z, IMU->orientation.w);
    IMU->linear_acceleration.x, IMU->linear_acceleration.y, IMU->linear_acceleration.z,
        IMU->angular_velocity.x, IMU->angular_velocity.y, IMU->angular_velocity.z,
        IMU->orientation.x, IMU->orientation.y, IMU->orientation.z, IMU->orientation.w;
    
    Accel(0) = IMU->linear_acceleration.x;
    Accel(1) = IMU->linear_acceleration.y;
    Accel(2) = IMU->linear_acceleration.z;

    Gyro(0) = IMU->angular_velocity.x;
    Gyro(1) = IMU->angular_velocity.y;
    Gyro(2) = IMU->angular_velocity.z;


    quaternion(0) = IMU->orientation.x;
    quaternion(1) = IMU->orientation.y;
    quaternion(2) = IMU->orientation.z;
    quaternion(3) = IMU->orientation.w;
   
}


// void Callback::callbackThread()
// {
//     ros::NodeHandle nh(ros::this_node::getName());

//     // joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("KWJ_joint_states", 100);
//     // joint_state_subscriber_ = nh.subscribe("KWJ_desired_joint_states", 1000, &Callback::JointStatesCallback, this);
//     // FSR_L_sensor_subscriber_ = nh.subscribe("FSR_L", 1000, &Callback::FSRsensorCallback, this);
//     // FSR_R_sensor_subscriber_ = nh.subscribe("FSR_R", 1000, &Callback::FSRsensorCallback, this);
//     // IMU_sensor_subscriber_ = nh.subscribe("/imu/data", 1000, &Callback::IMUsensorCallback, this);
//     // Emergency_subscriber_ = nh.subscribe("/Move_decision/Emergency", 1000, &Callback::Emergencycallback, this);


//     // //Client (재민이형 코드에 들어감)
//     // ros::ServiceClient client_SM = nh.serviceClient<dynamixel_current_2port::Select_Motion>("Select_Motion");
//     // ros::ServiceClient client_TA = nh.serviceClient<dynamixel_current_2port::Turn_Angle>("Turn_Angle");

//     // dynamixel_current_2port::Select_Motion srv_SM;
//     // dynamixel_current_2port::Turn_Angle srv_TA;

//     // srv_SM.request.finish = 1;
//     // srv_TA.request.finish = 1;

//     ros::Rate loop_rate(1);
//     while (nh.ok())
//     {
//         // startMode();
//         if (client_SM.call(srv_SM))
//         {
//             ROS_INFO("#Motion# Service call success!");
//             ROS_INFO("Response: %d", srv_SM.response.select_motion);
//         }
//         else
//         {
//             ROS_ERROR("Failed to call service");
//         }
        
//         if (client_TA.call(srv_TA))
//         {
//             ROS_INFO("#Angle# Service call success!");
//             ROS_INFO("Response: %d", srv_TA.response.turn_angle);
//         }
//         else
//         {
//             ROS_ERROR("Failed to call service");
//         }

//         ros::spinOnce();
//         loop_rate.sleep();
//         //usleep(1000);
//     }
// }

void Callback::Emergencycallback(const std_msgs::Bool &msg)
{
    ROS_INFO("%d", msg.data);
}

// void Callback::SelectMotion(const std_msgs::Float32Ptr &msg)
// {
//     mode = msg ->data;
//     ROS_INFO("mode(%f)",mode);
//     if (mode == 0)
//     {
//         RL_motion = RL_motion0;
//         LL_motion = LL_motion0;
//     }
//     else if (mode == 1)
//     {
//         indext = 0;
//         RL_motion = RL_motion1;
//         LL_motion = LL_motion1;
//     }
//     else if (mode == 2)
//     {
//         indext = 0;
//         RL_motion = RL_motion2;
//         LL_motion = LL_motion2;
//     }
//     else if (mode == 3)
//     {
//         indext = 0;
//         RL_motion = RL_motion3;
//         LL_motion = LL_motion3;
//     }
//     else if (mode == 4)
//     {
//         indext = 0;
//         RL_motion = RL_motion4;
//         LL_motion = LL_motion4;
//     }
//     else if (mode == 5)
//     {
//         indext = 0;
//         RL_motion = RL_motion5;
//         LL_motion = LL_motion5;
//     }
//     else if (mode == 6)
//     {
//         indext = 0;
//         RL_motion = RL_motion6;
//         LL_motion = LL_motion6;
//     }
//     else if (mode == 7)
//     {
//         indext = 0;
//         RL_motion = RL_motion7;
//         LL_motion = LL_motion7;
//     }
//     else
//     {
//         indext = 0;
//         RL_motion = RL_motion0;
//         LL_motion = LL_motion0;
//     }
// }

void Callback::MotionMaker(){
    

    motion.Motion0();
    LL_motion0 = motion.Return_Motion0_LL();
    RL_motion0 = motion.Return_Motion0_RL();

    motion.Motion1();
    LL_motion1 = motion.Return_Motion1_LL();
    RL_motion1 = motion.Return_Motion1_RL();

    motion.Motion2();
    LL_motion2 = motion.Return_Motion2_LL();
    RL_motion2 = motion.Return_Motion2_RL();

    motion.Motion3();
    LL_motion3 = motion.Return_Motion3_LL();
    RL_motion3 = motion.Return_Motion3_RL();

    motion.Motion4();
    LL_motion4 = motion.Return_Motion4_LL();
    RL_motion4 = motion.Return_Motion4_RL();

    motion.Motion5();
    LL_motion5 = motion.Return_Motion5_LL();
    RL_motion5 = motion.Return_Motion5_RL();

    motion.Motion6();
    LL_motion6 = motion.Return_Motion6_LL();
    RL_motion6 = motion.Return_Motion6_RL();

    motion.Motion7();
    LL_motion7 = motion.Return_Motion7_LL();
    RL_motion7 = motion.Return_Motion7_RL();

    LL_motion = LL_motion0;
    RL_motion = RL_motion0;


}

void Callback::Write_Leg_Theta(){

    ////////////////////////////////////////////////////////////////////
    All_Theta[0] = RL_motion(indext, 0); //Right Waist
    All_Theta[1] = RL_motion(indext, 1) - 2 * DEG2RAD;//Left Waist
    All_Theta[2] = RL_motion(indext, 2) -  10.74 * DEG2RAD;//Right Waist
    All_Theta[3] = -RL_motion(indext, 3) + 38.34 * DEG2RAD;//Left
    All_Theta[4] = -RL_motion(indext, 4) +24.22 * DEG2RAD;
    All_Theta[5] = -RL_motion(indext, 5);
    All_Theta[6] = LL_motion(indext, 0);
    All_Theta[7] = LL_motion(indext, 1);
    All_Theta[8] = -LL_motion(indext, 2) +  10.74 * DEG2RAD;
    All_Theta[9] = LL_motion(indext, 3) - 38.34 * DEG2RAD;
    All_Theta[10] = LL_motion(indext, 4) - 24.22 * DEG2RAD;
    All_Theta[11] = -LL_motion(indext, 5);
    // if (indext > simt * 1.74 && indext < simt * 1.75 && R_value < 3)
    // {
    // indext = indext;
    // }
    // else if (indext > simt * 2.74 && indext < simt * 2.75 && R_value < 3)
    // {
    // indext = indext;
    // }
    // else if (indext > simt * 3.74 && indext < simt * 3.75 && R_value < 3)
    // {
    // indext = indext;
    // }
    // else if (indext > simt * 1.24 && indext < simt * 1.25 && L_value < 3)
    // {
    // indext = indext;
    // }
    // else if (indext > simt * 2.24 && indext < simt * 2.25 && L_value < 3)
    // {
    // indext = indext;
    // }
    // else if (indext > simt * 3.24 && indext < simt * 3.25 && L_value < 3)
    // {
    // indext = indext;
    // }
    // else
    // {
    // indext += 1;
    // }
    // if (indext >= RL_motion.rows() - 1)
    // {
    //     if (L_value > 1 && R_value > 1)
    //         indext = 0
    //          RL_motion = RL_motion0;
    //          LL_motion = LL_motion0;;
    //     else
    //         indext = indext - 1;
    // }
       indext = 843;
    // indext = 1181;
    // indext += 1;
    if (indext >= RL_motion.rows() - 1)
    {
    indext = 0;
    RL_motion = RL_motion0;
    LL_motion = LL_motion0;
    }
}

void Callback::Write_Arm_Theta(){
    All_Theta[12] = 0; // 허리
    All_Theta[13] = -90 * DEG2RAD;
    All_Theta[14] = 90 * DEG2RAD;
    All_Theta[15] = -60 * DEG2RAD;
    All_Theta[16] = 60 * DEG2RAD;
    All_Theta[17] = -90 * DEG2RAD;
    All_Theta[18] = 90 * DEG2RAD;
    All_Theta[19] = 0 * DEG2RAD;
    All_Theta[20] = 0 * DEG2RAD;
}


