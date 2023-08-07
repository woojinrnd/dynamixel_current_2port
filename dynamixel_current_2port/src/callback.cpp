#include "callback.hpp"


// Callback::callback()
Callback::Callback(Motions *motionPtr, Dxl *dxlPtr) : motionPtr(motionPtr), dxlPtr(dxlPtr), SPIN_RATE(100)
{
    ros::NodeHandle nh(ros::this_node::getName());

    boost::thread queue_thread = boost::thread(boost::bind(&Callback::callbackThread, this));
}


void Callback::JointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_command)
{
    for (int i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        Goal_joint_[i] = joint_command->position[i];
        dxlPtr->SetThetaRef(Goal_joint_);
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

void Callback::callbackThread()
{
    ros::NodeHandle nh(ros::this_node::getName());

    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("KWJ_joint_states", 100);
    joint_state_subscriber_ = nh.subscribe("KWJ_desired_joint_states", 1000, &Callback::JointStatesCallback, this);
    FSR_L_sensor_subscriber_ = nh.subscribe("FSR_L", 1000, &Callback::FSRsensorCallback, this);
    FSR_R_sensor_subscriber_ = nh.subscribe("FSR_R", 1000, &Callback::FSRsensorCallback, this);
    IMU_sensor_subscriber_ = nh.subscribe("/imu/data", 1000, &Callback::IMUsensorCallback, this);
    Emergency_subscriber_ = nh.subscribe("/Move_decision/Emergency", 1000, &Callback::Emergencycallback, this);

    srv_SM.request.finish = 1;
    srv_TA.request.finish = 1;
    srv_UD_Neck.request.finish = 1;
    srv_RL_Neck.request.finish = 1;


    ros::Rate loop_rate(SPIN_RATE);
    while (nh.ok())
    {
        // startMode();
        if (client_SM.call(srv_SM))
        {
            ROS_INFO("#[MESSAGE] SM Request : %d#", srv_SM.request.finish);
            ROS_INFO("[MESSAGE] SM Response : %d", srv_SM.response.select_motion);
            SelectMotion();
        }

        if (client_TA.call(srv_TA))
        {
            ROS_INFO("#[MESSAGE] TA Request : %d#", srv_TA.request.finish);
            ROS_INFO("[MESSAGE] TA Response : %f", srv_TA.response.turn_angle);
            // Turn Body Angle에 모션 해당하는 부분
        }

        if (client_UD_Neck.call(srv_UD_Neck))
        {
            ROS_INFO("#[MESSAGE] UD Request : %d#", srv_UD_Neck.request.finish);
            ROS_INFO("[MESSAGE] UD Response : %f", srv_UD_Neck.response.ud_neckangle);
            Move_UD_NeckAngle();            
        }

        if (client_RL_Neck.call(srv_RL_Neck))
        {
            ROS_INFO("#[MESSAGE] RL Request : %d#", srv_RL_Neck.request.finish);
            ROS_INFO("[MESSAGE] RL Response : %f", srv_RL_Neck.response.rl_neckangle);
            Move_RL_NeckAngle();            
        }

        else
        {
            ROS_ERROR("Failed to call service");
        }

        ros::spinOnce();
        loop_rate.sleep();
        // usleep(1000);
    }
}

void Callback::Emergencycallback(const std_msgs::Bool &msg)
{
    // ROS_INFO("%d", msg.data);
    if (msg.data == false)
    {
        ROS_ERROR("Emergency");
    }
}

// About Subscribe
//  void Callback::SelectMotion(const std_msgs::UInt8::ConstPtr &msg)
//  {
//      mode = msg ->data;
//      ROS_INFO("mode(%f)",mode);
//      if (mode == 0)
//      {
//          RL_motion = RL_motion0;
//          LL_motion = LL_motion0;
//      }
//      else if (mode == 1)
//      {
//          indext = 0;
//          RL_motion = RL_motion1;
//          LL_motion = LL_motion1;
//      }
//      else if (mode == 2)
//      {
//          indext = 0;
//          RL_motion = RL_motion2;
//          LL_motion = LL_motion2;
//      }
//      else if (mode == 3)
//      {
//          indext = 0;
//          RL_motion = RL_motion3;
//          LL_motion = LL_motion3;
//      }
//      else if (mode == 4)
//      {
//          indext = 0;
//          RL_motion = RL_motion4;
//          LL_motion = LL_motion4;
//      }
//      else if (mode == 5)
//      {
//          indext = 0;
//          RL_motion = RL_motion5;
//          LL_motion = LL_motion5;
//      }
//      else if (mode == 6)
//      {
//          indext = 0;
//          RL_motion = RL_motion6;
//          LL_motion = LL_motion6;
//      }
//      else if (mode == 7)
//      {
//          indext = 0;
//          RL_motion = RL_motion7;
//          LL_motion = LL_motion7;
//      }
//      else
//      {
//          indext = 0;
//          RL_motion = RL_motion0;
//          LL_motion = LL_motion0;
//      }
//  }

/////////////////////////////////////////////// About Client Callback ///////////////////////////////////////////////
void Callback::Move_RL_NeckAngle()
{
    if (client_RL_Neck.call(srv_RL_Neck))
    {
        double res_rl_neck = srv_RL_Neck.response.rl_neckangle;
        rl_neckangle = res_rl_neck;
        // All_Theta[21] = rl_neckangle;
        All_Theta[1] = rl_neckangle * DEG2RAD;
    }
}

void Callback::Move_UD_NeckAngle()
{
    if (client_UD_Neck.call(srv_UD_Neck))
    {
        double res_ud_neck = srv_UD_Neck.response.ud_neckangle;
        ud_neckangle = res_ud_neck;
        // All_Theta[22] = ud_neckangle;
        All_Theta[0] = ud_neckangle * DEG2RAD;
    }
}

void Callback::SelectMotion()
{
    if (client_SM.call(srv_SM))
    {
        int8_t res_mode = srv_SM.response.select_motion;
        mode = res_mode;
        ROS_INFO("mode(%d)", mode);
        if (mode == 0)
        {
            RL_motion = RL_motion0;
            LL_motion = LL_motion0;
        }
        else if (mode == 1)
        {
            indext = 0;
            RL_motion = RL_motion1;
            LL_motion = LL_motion1;
        }
        else if (mode == 2)
        {
            indext = 0;
            RL_motion = RL_motion2;
            LL_motion = LL_motion2;
        }
        else if (mode == 3)
        {
            indext = 0;
            RL_motion = RL_motion3;
            LL_motion = LL_motion3;
        }
        else if (mode == 4)
        {
            indext = 0;
            RL_motion = RL_motion4;
            LL_motion = LL_motion4;
        }
        else if (mode == 5)
        {
            indext = 0;
            RL_motion = RL_motion5;
            LL_motion = LL_motion5;
        }
        else if (mode == 6)
        {
            indext = 0;
            RL_motion = RL_motion6;
            LL_motion = LL_motion6;
        }
        else if (mode == 7)
        {
            indext = 0;
            RL_motion = RL_motion7;
            LL_motion = LL_motion7;
        }
        else
        {
            indext = 0;
            RL_motion = RL_motion0;
            LL_motion = LL_motion0;
        }
    }
}

void Callback::MotionMaker()
{
    motionPtr->Motion0();
    LL_motion0 = motionPtr->Return_Motion0_LL();
    RL_motion0 = motionPtr->Return_Motion0_RL();

    motionPtr->Motion1();
    LL_motion1 = motionPtr->Return_Motion1_LL();
    RL_motion1 = motionPtr->Return_Motion1_RL();

    motionPtr->Motion2();
    LL_motion2 = motionPtr->Return_Motion2_LL();
    RL_motion2 = motionPtr->Return_Motion2_RL();

    motionPtr->Motion3();
    LL_motion3 = motionPtr->Return_Motion3_LL();
    RL_motion3 = motionPtr->Return_Motion3_RL();

    motionPtr->Motion4();
    LL_motion4 = motionPtr->Return_Motion4_LL();
    RL_motion4 = motionPtr->Return_Motion4_RL();

    motionPtr->Motion5();
    LL_motion5 = motionPtr->Return_Motion5_LL();
    RL_motion5 = motionPtr->Return_Motion5_RL();

    motionPtr->Motion6();
    LL_motion6 = motionPtr->Return_Motion6_LL();
    RL_motion6 = motionPtr->Return_Motion6_RL();

    motionPtr->Motion7();
    LL_motion7 = motionPtr->Return_Motion7_LL();
    RL_motion7 = motionPtr->Return_Motion7_RL();

    LL_motion = LL_motion0;
    RL_motion = RL_motion0;
}

void Callback::Write_Leg_Theta()
{

    ////////////////////////////////////////////////////////////////////
    All_Theta[0] = RL_motion(indext, 0);                    // Right Waist
    All_Theta[1] = RL_motion(indext, 1) - 2 * DEG2RAD;      // Left Waist
    All_Theta[2] = RL_motion(indext, 2) - 10.74 * DEG2RAD;  // Right Waist
    All_Theta[3] = -RL_motion(indext, 3) + 38.34 * DEG2RAD; // Left
    All_Theta[4] = -RL_motion(indext, 4) + 24.22 * DEG2RAD;
    All_Theta[5] = -RL_motion(indext, 5);
    All_Theta[6] = LL_motion(indext, 0);
    All_Theta[7] = LL_motion(indext, 1);
    All_Theta[8] = -LL_motion(indext, 2) + 10.74 * DEG2RAD;
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
    // if (indext >= RL_motionPtr->rows() - 1)
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

void Callback::Write_Arm_Theta()
{
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
