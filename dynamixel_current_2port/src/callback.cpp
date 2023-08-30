#include "callback.hpp"

// Callback::callback()
Callback::Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr)
    : trajectoryPtr(trajectoryPtr),
      IK_Ptr(IK_Ptr),
      dxlPtr(dxlPtr),
      SPIN_RATE(1)
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

    // srv_SM.request.finish = 1;
    // srv_TA.request.finish = 1;
    // srv_UD_Neck.request.finish = 1;
    // srv_RL_Neck.request.finish = 1;
    // srv_Emergency.request.finish = 1;

    srv_SendMotion.request.SM_finish = 0;
    srv_SendMotion.request.TA_finish = 1;
    srv_SendMotion.request.UD_finish = 0;
    srv_SendMotion.request.RL_finish = 0;
    srv_SendMotion.request.EM_finish = 0;

    ros::Rate loop_rate(SPIN_RATE);
    while (nh.ok())
    {
        // if (client_SM.call(srv_SM))
        // {
        //     ROS_INFO("\n");
        //     ROS_INFO("------------------------- Select Motion ----------------------------");
        //     ROS_INFO("#[MESSAGE] SM Request : %s#", srv_SM.request.finish ? "true" : "false");
        //     SelectMotion();
        //     Motion_Info();
        // }

        // if (client_TA.call(srv_TA))
        // {
        //     ROS_INFO("\n");
        //     ROS_INFO("------------------------- Turn Angle ----------------------------");
        //     ROS_INFO("#[MESSAGE] TA Request : %s#", srv_TA.request.finish ? "true" : "false");
        //     ROS_WARN("[MESSAGE] TA Response : %f", srv_TA.response.turn_angle);
        //     // Turn Body Angle에 모션 해당하는 부분
        //     TATA();
        //     ROS_INFO("------------------------- Turn Angle ----------------------------");
        // }

        // if (client_UD_Neck.call(srv_UD_Neck))
        // {
        //     ROS_INFO("\n");
        //     ROS_INFO("------------------------- UD NECK Angle ----------------------------");
        //     ROS_INFO("#[MESSAGE] UD Request : %s#", srv_UD_Neck.request.finish ? "true" : "false");
        //     // ROS_INFO("[MESSAGE] UD Response : %f", srv_UD_Neck.response.ud_neckangle);
        //     Move_UD_NeckAngle();
        // }

        // if (client_RL_Neck.call(srv_RL_Neck))
        // {
        //     ROS_INFO("\n");
        //     ROS_INFO("------------------------- RL NECK Angle ----------------------------");
        //     ROS_INFO("#[MESSAGE] RL Request : %s#", srv_RL_Neck.request.finish ? "true" : "false");
        //     // ROS_INFO("[MESSAGE] RL Response : %f", srv_RL_Neck.response.rl_neckangle);
        //     Move_RL_NeckAngle();
        // }

        // if (client_Emergency.call(srv_Emergency))
        // {
        //     ROS_INFO("\n");
        //     ROS_INFO("------------------------- EMERGENCY ----------------------------");
        //     ROS_ERROR("#[MESSAGE] EMG Request : %s#", srv_Emergency.request.finish ? "true" : "false");
        //     // ROS_INFO("[MESSAGE] EMG Response : %s", srv_Emergency.response.emergency ? "true" : "false");
        //     Emergency();
        // }

        if (client_SendMotion.call(srv_SendMotion))
        {
            RecieveMotion();
            Motion_Info();
        }

        else
        {
            ROS_INFO("\n");
            ROS_ERROR("Failed to call service");
            ROS_INFO("\n");
        }

        ros::spinOnce();
        loop_rate.sleep();
        // usleep(1000);
    }
}

/////////////////////////////////////////////// About Client Callback ///////////////////////////////////////////////

void Callback::RecieveMotion()
{
    if (client_SendMotion.call(srv_SendMotion))
    {
        ROS_INFO("#[MESSAGE] SM Request : %s#", srv_SendMotion.request.SM_finish ? "true" : "false");
        ROS_INFO("#[MESSAGE] TA Request : %s#", srv_SendMotion.request.TA_finish ? "true" : "false");
        ROS_INFO("#[MESSAGE] UD Request : %s#", srv_SendMotion.request.UD_finish ? "true" : "false");
        ROS_INFO("#[MESSAGE] RL Request : %s#", srv_SendMotion.request.RL_finish ? "true" : "false");
        ROS_INFO("#[MESSAGE] EM Request : %s#", srv_SendMotion.request.EM_finish ? "true" : "false");
        SelectMotion();
        TATA();
        Move_UD_NeckAngle();
        Move_RL_NeckAngle();
        Emergency();
    }
}

void Callback::Move_RL_NeckAngle()
{
    if (client_SendMotion.call(srv_SendMotion))
    {
        double res_rl_neck = srv_SendMotion.response.rl_neckangle;
        rl_neckangle = res_rl_neck;
        // All_Theta[21] = rl_neckangle;
        All_Theta[1] = rl_neckangle * DEG2RAD;
        ROS_WARN("RL_NECK : %f", rl_neckangle);
        ROS_INFO("------------------------- RL NECK Angle ----------------------------");
        ROS_INFO("\n");
    }
}

// -90[deg] : Initial Propose (Look straight)
// CCW (+)
void Callback::Move_UD_NeckAngle()
{
    if (client_SendMotion.call(srv_SendMotion))
    {
        double res_ud_neck = srv_SendMotion.response.ud_neckangle;
        ud_neckangle = -90 + res_ud_neck;
        // All_Theta[22] = ud_neckangle;
        All_Theta[0] = ud_neckangle * DEG2RAD;
        ROS_WARN("UD_NECK : %f", ud_neckangle);
        ROS_INFO("------------------------- UD NECK Angle ----------------------------");
        ROS_INFO("\n");
    }
}

void Callback::TATA()
{
    if (client_SendMotion.call(srv_SendMotion))
    {
        double res_turn_angle = srv_SendMotion.response.turn_angle;
        turn_angle = res_turn_angle;
        // All_Theta[22] = ud_neckangle;
        All_Theta[2] = turn_angle * DEG2RAD;
        ROS_WARN("TURN_ANGLE : %f", turn_angle);
        ROS_INFO("------------------------- TURN_ANGLE ----------------------------");
        ROS_INFO("\n");
    }
}

/// 재민이형 긴급정지에 대한 코드 여기다가 넣으면 됨 ///
void Callback::Emergency()
{
    if (client_SendMotion.call(srv_SendMotion))
    {
        bool res_emergency = srv_SendMotion.response.emergency;
        emergency_ = res_emergency;
        ROS_ERROR("EMERGENCY : %s", emergency_ ? "True" : "False");
        ROS_INFO("------------------------- EMERGENCY ----------------------------");
        ROS_INFO("\n");
    }
}

void Callback::Check_FSR()
{
    if (IK_Ptr->check_index == 0.05 && L_value == 0) // 오른발 들때 왼발 착지 확인
    {
        indext -= 1;
    }
    else if (IK_Ptr->check_index > 0.55 && R_value == 0) // 왼발 들때 오른발 착지 확인
    {
        indext -= 1;
    }
}

void Callback::SelectMotion()
{
    if (client_SendMotion.call(srv_SendMotion))
    {
        int8_t res_mode = srv_SendMotion.response.select_motion;
        float res_distance = srv_SendMotion.response.distance;
        mode = res_mode;
        // ROS_INFO("mode(%d)", mode);
        // ROS_WARN("Distance(%f)", res_distance);
        if (mode == 0)
        {
            IK_Ptr->RL_th[0] = 0;
            IK_Ptr->RL_th[1] = 0;
            IK_Ptr->RL_th[2] = -0.44567;
            IK_Ptr->RL_th[3] = 0.99918;
            IK_Ptr->RL_th[4] = -0.55351;
            IK_Ptr->RL_th[5] = 0;
            IK_Ptr->LL_th[0] = 0;
            IK_Ptr->LL_th[1] = 0;
            IK_Ptr->LL_th[2] = -0.44567;
            IK_Ptr->LL_th[3] = 0.99918;
            IK_Ptr->LL_th[4] = -0.55351;
            IK_Ptr->LL_th[5] = 0;
        }
        else if (mode == 1)
        {
            trajectoryPtr->Go_Straight(0.05, 0.5);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Set_Angle_Compensation();
            indext = 0;
        }
        else if (mode == 2)
        {
            trajectoryPtr->Side_Left2();
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Set_Angle_Compensation();
            indext = 0;
        }
        else if (mode == 3)
        {
            trajectoryPtr->Step_in_place(0.05, 0.5);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Set_Angle_Compensation();
            indext = 0;
        }
        else if (mode == 4)
        {
            trajectoryPtr->Side_Right2();
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Set_Angle_Compensation();
            indext = 0;
        }
        else if (mode == 5)
        {
            trajectoryPtr->Fast_Straight(0.05, 0.5);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Set_Angle_Compensation();
            indext = 0;
        }
        else
        {
            IK_Ptr->RL_th[0] = 0;
            IK_Ptr->RL_th[1] = 0;
            IK_Ptr->RL_th[2] = -0.44567;
            IK_Ptr->RL_th[3] = 0.99918;
            IK_Ptr->RL_th[4] = -0.55351;
            IK_Ptr->RL_th[5] = 0;
            IK_Ptr->LL_th[0] = 0;
            IK_Ptr->LL_th[1] = 0;
            IK_Ptr->LL_th[2] = -0.44567;
            IK_Ptr->LL_th[3] = 0.99918;
            IK_Ptr->LL_th[4] = -0.55351;
            IK_Ptr->LL_th[5] = 0;
        }
    }
}

void Callback::Motion_Info()
{
    if (client_SendMotion.call(srv_SendMotion))
    {
        int8_t res_mode = srv_SendMotion.response.select_motion;
        float res_distance = srv_SendMotion.response.distance;
        string tmp_motion;
        switch (res_mode)
        {
        case Motion_Index::InitPose:
            tmp_motion = Str_InitPose;
            break;

        case Motion_Index::Forward_4step:
            tmp_motion = Str_Forward_4step;
            break;

        case Motion_Index::Left_2step:
            tmp_motion = Str_Left_2step;
            break;

        case Motion_Index::Step_in_place:
            tmp_motion = Str_Step_in_place;
            break;

        case Motion_Index::Right_2step:
            tmp_motion = Str_Right_2step;
            break;

        case Motion_Index::Back_4step:
            tmp_motion = Str_Back_4step;
            break;

        case Motion_Index::Forward_Nstep:
            tmp_motion = Str_Forward_Nstep;
            break;

        case Motion_Index::Huddle_Jump:
            tmp_motion = Str_Huddle_Jump;
            break;

        case Motion_Index::FWD_UP:
            tmp_motion = Str_FWD_UP;
            break;

        case Motion_Index::BWD_UP:
            tmp_motion = Str_BWD_UP;
            break;

        case Motion_Index::NONE:
            tmp_motion = Str_NONE;
            break;
        }

        if (Motion_Index::Forward_Nstep)
        {
            ROS_WARN("Motion_Index : %s", tmp_motion.c_str());
            ROS_WARN("Distance : %f", res_distance);
            ROS_INFO("------------------------- Select Motion ----------------------------");
        }
        else
        {
            ROS_WARN("Motion_Index : %s", tmp_motion.c_str());
            ROS_INFO("------------------------- Select Motion ----------------------------");
        }
    }
}

// 0[deg] : Initial Propose (Look Straight)
// CCW (+)
// void Callback::Move_RL_NeckAngle()
// {
//     if (client_RL_Neck.call(srv_RL_Neck))
//     {
//         double res_rl_neck = srv_RL_Neck.response.rl_neckangle;
//         rl_neckangle = res_rl_neck;
//         // All_Theta[21] = rl_neckangle;
//         All_Theta[1] = rl_neckangle * DEG2RAD;
//         ROS_WARN("RL_NECK : %f", rl_neckangle);
//         ROS_INFO("------------------------- RL NECK Angle ----------------------------");
//         ROS_INFO("\n");
//     }
// }

// // -90[deg] : Initial Propose (Look straight)
// // CCW (+)
// void Callback::Move_UD_NeckAngle()
// {
//     if (client_UD_Neck.call(srv_UD_Neck))
//     {
//         double res_ud_neck = srv_UD_Neck.response.ud_neckangle;
//         ud_neckangle = -90 + res_ud_neck;
//         // All_Theta[22] = ud_neckangle;
//         All_Theta[0] = ud_neckangle * DEG2RAD;
//         ROS_WARN("UD_NECK : %f", ud_neckangle);
//         ROS_INFO("------------------------- UD NECK Angle ----------------------------");
//         ROS_INFO("\n");
//     }
// }

// void Callback::TATA()
// {
//     if (client_TA.call(srv_TA))
//     {
//         double res_turn_angle = srv_TA.response.turn_angle;
//         turn_angle = res_turn_angle;
//         // All_Theta[22] = ud_neckangle;
//         All_Theta[2] = turn_angle * DEG2RAD;
//         ROS_WARN("TURN_ANGLE : %f", turn_angle);
//         ROS_INFO("------------------------- TURN_ANGLE ----------------------------");
//         ROS_INFO("\n");
//     }
// }

// /// 재민이형 긴급정지에 대한 코드 여기다가 넣으면 됨 ///
// void Callback::Emergency()
// {
//     if (client_Emergency.call(srv_Emergency))
//     {
//         bool res_emergency = srv_Emergency.response.emergency;
//         emergency_ = res_emergency;
//         ROS_ERROR("EMERGENCY : %s", emergency_ ? "True" : "False");
//         ROS_INFO("------------------------- EMERGENCY ----------------------------");
//         ROS_INFO("\n");
//     }
// }

// void Callback::Check_FSR()
// {
//     if (IK_Ptr->check_index == 0.05 && L_value == 0) // 오른발 들때 왼발 착지 확인
//     {
//         indext -= 1;
//     }
//     else if (IK_Ptr->check_index > 0.55 && R_value == 0) // 왼발 들때 오른발 착지 확인
//     {
//         indext -= 1;
//     }
// }

// void Callback::SelectMotion()
// {
//     if (client_SM.call(srv_SM))
//     {
//         int8_t res_mode = srv_SM.response.select_motion;
//         float res_distance = srv_SM.response.distance;
//         mode = res_mode;
//         // ROS_INFO("mode(%d)", mode);
//         // ROS_WARN("Distance(%f)", res_distance);
//         if (mode == 0)
//         {
//             IK_Ptr->RL_th[0] = 0;
//             IK_Ptr->RL_th[1] = 0;
//             IK_Ptr->RL_th[2] = -0.44567;
//             IK_Ptr->RL_th[3] = 0.99918;
//             IK_Ptr->RL_th[4] = -0.55351;
//             IK_Ptr->RL_th[5] = 0;
//             IK_Ptr->LL_th[0] = 0;
//             IK_Ptr->LL_th[1] = 0;
//             IK_Ptr->LL_th[2] = -0.44567;
//             IK_Ptr->LL_th[3] = 0.99918;
//             IK_Ptr->LL_th[4] = -0.55351;
//             IK_Ptr->LL_th[5] = 0;
//         }
//         else if (mode == 1)
//         {
//             trajectoryPtr->Go_Straight(0.05, 0.5);
//             trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
//             IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
//             IK_Ptr->Set_Angle_Compensation();
//             indext = 0;
//         }
//         else if (mode == 2)
//         {
//             trajectoryPtr->Side_Left2();
//             IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
//             IK_Ptr->Set_Angle_Compensation();
//             indext = 0;
//         }
//         else if (mode == 3)
//         {
//             trajectoryPtr->Step_in_place(0.05, 0.5);
//             trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
//             IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
//             IK_Ptr->Set_Angle_Compensation();
//             indext = 0;
//         }
//         else if (mode == 4)
//         {
//             trajectoryPtr->Side_Right2();
//             IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
//             IK_Ptr->Set_Angle_Compensation();
//             indext = 0;
//         }
//         else if (mode == 5)
//         {
//             trajectoryPtr->Fast_Straight(0.05, 0.5);
//             IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
//             IK_Ptr->Set_Angle_Compensation();
//             indext = 0;
//         }
//         else
//         {
//             IK_Ptr->RL_th[0] = 0;
//             IK_Ptr->RL_th[1] = 0;
//             IK_Ptr->RL_th[2] = -0.44567;
//             IK_Ptr->RL_th[3] = 0.99918;
//             IK_Ptr->RL_th[4] = -0.55351;
//             IK_Ptr->RL_th[5] = 0;
//             IK_Ptr->LL_th[0] = 0;
//             IK_Ptr->LL_th[1] = 0;
//             IK_Ptr->LL_th[2] = -0.44567;
//             IK_Ptr->LL_th[3] = 0.99918;
//             IK_Ptr->LL_th[4] = -0.55351;
//             IK_Ptr->LL_th[5] = 0;
//         }
//     }
// }

// void Callback::Motion_Info()
// {
//     if (client_SM.call(srv_SM))
//     {
//         int8_t res_mode = srv_SM.response.select_motion;
//         float res_distance = srv_SM.response.distance;
//         string tmp_motion;
//         switch (res_mode)
//         {
//         case Motion_Index::InitPose:
//             tmp_motion = Str_InitPose;
//             break;

//         case Motion_Index::Forward_4step:
//             tmp_motion = Str_Forward_4step;
//             break;

//         case Motion_Index::Left_2step:
//             tmp_motion = Str_Left_2step;
//             break;

//         case Motion_Index::Step_in_place:
//             tmp_motion = Str_Step_in_place;
//             break;

//         case Motion_Index::Right_2step:
//             tmp_motion = Str_Right_2step;
//             break;

//         case Motion_Index::Back_4step:
//             tmp_motion = Str_Back_4step;
//             break;

//         case Motion_Index::Forward_Nstep:
//             tmp_motion = Str_Forward_Nstep;
//             break;

//         case Motion_Index::Huddle_Jump:
//             tmp_motion = Str_Huddle_Jump;
//             break;

//         case Motion_Index::FWD_UP:
//             tmp_motion = Str_FWD_UP;
//             break;

//         case Motion_Index::BWD_UP:
//             tmp_motion = Str_BWD_UP;
//             break;
//         }

//         if (Motion_Index::Forward_Nstep)
//         {
//             ROS_WARN("Motion_Index : %s", tmp_motion.c_str());
//             ROS_WARN("Distance : %f", res_distance);
//             ROS_INFO("------------------------- Select Motion ----------------------------");
//         }
//         else
//         {
//             ROS_WARN("Motion_Index : %s", tmp_motion.c_str());
//             ROS_INFO("------------------------- Select Motion ----------------------------");
//         }
//     }
// }

void Callback::Write_Leg_Theta()
{
    if (emergency == 1)
    {
        stop_indext += 1;
        IK_Ptr->BRP_Simulation(trajectoryPtr->rsRef_RL_x, trajectoryPtr->rsRef_RL_y, trajectoryPtr->rsRef_RL_z, trajectoryPtr->rsRef_LL_x, trajectoryPtr->rsRef_LL_y, trajectoryPtr->rsRef_LL_z, stop_indext);
        if (stop_indext > 134)
        {
            stop_indext -= 1;
        }
    }
    else if (emergency == 2)
    {
        stop_indext += 1;
        IK_Ptr->BRP_Simulation(trajectoryPtr->lsRef_RL_x, trajectoryPtr->lsRef_RL_y, trajectoryPtr->lsRef_RL_z, trajectoryPtr->lsRef_LL_x, trajectoryPtr->lsRef_LL_y, trajectoryPtr->lsRef_LL_z, stop_indext);
        if (stop_indext > 134)
        {
            stop_indext -= 1;
        }
        cout << stop_indext;
    }
    else if (emergency == 0)
    {
        indext += 1;
        if (mode > 0 && mode < 6)
        {

            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext);
        }
    }
    ////////////////////////////////////////////////////////////////////
    All_Theta[0] = IK_Ptr->RL_th[0]; //- 2 * DEG2RAD;
    All_Theta[1] = IK_Ptr->RL_th[1] - 2 * DEG2RAD;
    All_Theta[2] = IK_Ptr->RL_th[2] - 10.74 * DEG2RAD;
    All_Theta[3] = -IK_Ptr->RL_th[3] + 38.34 * DEG2RAD;
    All_Theta[4] = -IK_Ptr->RL_th[4] + 24.22 * DEG2RAD;
    All_Theta[5] = -IK_Ptr->RL_th[5];
    All_Theta[6] = IK_Ptr->LL_th[0] + 1 * DEG2RAD;
    All_Theta[7] = IK_Ptr->LL_th[1];
    All_Theta[8] = -IK_Ptr->LL_th[2] + 8.74 * DEG2RAD;
    All_Theta[9] = IK_Ptr->LL_th[3] - 36.34 * DEG2RAD;
    All_Theta[10] = IK_Ptr->LL_th[4] - 26.22 * DEG2RAD;
    All_Theta[11] = -IK_Ptr->LL_th[5];
    if (indext >= trajectoryPtr->Ref_RL_x.cols())
    {
        indext -= 1;
    }
    // Check_FSR();
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
