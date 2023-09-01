#include "Move_decision.hpp"

// Constructor
Move_Decision::Move_Decision(Img_proc *img_procPtr)
    : img_procPtr(img_procPtr),
      FALL_FORWARD_LIMIT(60),
      FALL_BACK_LIMIT(-60),
      SPIN_RATE(1),
      stand_status_(Stand_Status::Stand),
      motion_index_(Motion_Index::NONE),
      stop_fallen_check_(false),
      Emergency_(false),
      turn_angle_(0),
      straightLine(true)
{
    // Init ROS
    ros::NodeHandle nh(ros::this_node::getName());

    boost::thread process_thread = boost::thread(boost::bind(&Move_Decision::processThread, this));
    boost::thread web_process_thread = boost::thread(boost::bind(&Img_proc::webcam_thread, img_procPtr));
    boost::thread depth_process_thread = boost::thread(boost::bind(&Img_proc::realsense_thread, img_procPtr));
    // boost::thread move_thread = boost::thread(boost::bind(&Move_Decision::MoveDecisionThread, this));
    boost::thread queue_thread = boost::thread(boost::bind(&Move_Decision::callbackThread, this));
}

Move_Decision::~Move_Decision()
{
}

// ********************************************** PROCESS THREAD************************************************** //

void Move_Decision::process()
{
    //////////////////////////////////////   DEBUG WINDOW    //////////////////////////////////////
    //// Switch line_det_flg | no_line_det_flg
    // if (aaaa % 2 == 0)
    // {
    //     Set_line_det_flg(true);
    //     Set_no_line_det_flg(false);
    // }
    // else
    // {
    //     Set_no_line_det_flg(true);
    //     Set_line_det_flg(false);
    // }
    // aaaa++;

    //// No-straight line - set gradient //aaaa = -25
    // Set_line_det_flg(true);
    // aaaa++;
    // Set_gradient(aaaa);

    //// no_line_det_flg
    // Set_no_line_det_flg(true);
    // aaaa++;
    // Set_delta_x(aaaa);

    //// wake up mode
    // Set_running_mode_(Running_Mode::WAKEUP_MODE);
    // Set_stand_status_(Stand_Status::Fallen_Forward);
    // Motion_Info();

    // stop mode
    // rostopic echo /Move_decision/Emergency
    // Set_stop_det_flg(false);

    // goal mode
    // Set_goal_line_det_flg(true);
    // Set_line_det_flg(true);

    // huddle mode
    // Set_huddle_det_flg(true);
    // Set_UD_Neck_on_flg(true);

    //////////////////////////////////////   DEBUG WINDOW    //////////////////////////////////////

    // bool tmp_img_proc_line_det_flg_ = img_procPtr->Get_img_proc_line_det();
    // bool tmp_img_proc_no_line_det_flg_ = img_procPtr->Get_img_proc_no_line_det();
    // bool tmp_img_proc_huddle_det_flg_ = img_procPtr->Get_img_proc_huddle_det();
    // bool tmp_img_proc_wall_det_flg_ = img_procPtr->Get_img_proc_wall_det();
    // bool tmp_img_proc_goal_det_flg_ = img_procPtr->Get_img_proc_goal_line_det();
    // bool tmp_img_proc_corner_det_flg_ = img_procPtr->Get_img_proc_corner_det();

    tmp_img_proc_line_det_flg_ = img_procPtr->Get_img_proc_line_det();
    tmp_img_proc_no_line_det_flg_ = img_procPtr->Get_img_proc_no_line_det();
    tmp_img_proc_huddle_det_flg_ = img_procPtr->Get_img_proc_huddle_det();
    tmp_img_proc_wall_det_flg_ = img_procPtr->Get_img_proc_wall_det();
    tmp_img_proc_goal_det_flg_ = img_procPtr->Get_img_proc_goal_line_det();
    tmp_img_proc_corner_det_flg_ = img_procPtr->Get_img_proc_corner_det();

    //////////////////////////////////////   LINE MODE    //////////////////////////////////////

    if (tmp_img_proc_line_det_flg_)
    {
        if (tmp_img_proc_huddle_det_flg_)
        {
            Set_huddle_det_flg(true);
            Set_line_det_flg(false);
            Set_no_line_det_flg(false);
        }
        else if (tmp_img_proc_goal_det_flg_)
        {
            Set_goal_line_det_flg(true);
            Set_line_det_flg(false);
            Set_no_line_det_flg(false);
        }
        else if (tmp_img_proc_corner_det_flg_)
        {
            Set_corner_det_flg(true);
            Set_line_det_flg(false);
            Set_no_line_det_flg(false);
        }
        else
        {
            Set_line_det_flg(true);
            Set_no_line_det_flg(false);
        }
    }

    //////////////////////////////////////   NO LINE MODE    //////////////////////////////////////

    else if (tmp_img_proc_no_line_det_flg_)
    {
        if (tmp_img_proc_huddle_det_flg_)
        {
            Set_huddle_det_flg(true);
            Set_line_det_flg(false);
            Set_no_line_det_flg(false);
        }
        else if (tmp_img_proc_corner_det_flg_)
        {
            Set_corner_det_flg(true);
            Set_line_det_flg(false);
            Set_no_line_det_flg(false);
        }
        else if (tmp_img_proc_wall_det_flg_)
        {
            Set_wall_det_flg(true);
            Set_line_det_flg(false);
            Set_no_line_det_flg(false);
        }
        else if (tmp_img_proc_goal_det_flg_)
        {
            Set_goal_line_det_flg(true);
            Set_line_det_flg(false);
            Set_no_line_det_flg(false);
        }
        else
        {
            Set_no_line_det_flg(true);
        }
    }

    //////////////////////////////////////   HUDDLE MODE    //////////////////////////////////////

    else if (tmp_img_proc_huddle_det_flg_)
    {
        if (tmp_img_proc_line_det_flg_)
        {
            Set_line_det_flg(false);
        }
        else if (tmp_img_proc_no_line_det_flg_)
        {
            Set_no_line_det_flg(false);
        }
        else
        {
            Set_huddle_det_flg(true);
        }
    }

    //////////////////////////////////////   CORNER MODE    //////////////////////////////////////

    else if (tmp_img_proc_corner_det_flg_)
    {
        if (tmp_img_proc_line_det_flg_)
        {
            Set_line_det_flg(false);
        }
        else if (tmp_img_proc_no_line_det_flg_)
        {
            Set_no_line_det_flg(false);
        }
        else if (tmp_img_proc_wall_det_flg_)
        {
            Set_wall_det_flg(false);
        }
        else
        {
            Set_corner_det_flg(true);
        }
    }

    //////////////////////////////////////   WALL MODE    //////////////////////////////////////

    else if (tmp_img_proc_wall_det_flg_)
    {
        if (tmp_img_proc_line_det_flg_)
        {
            Set_line_det_flg(false);
        }
        else if (tmp_img_proc_no_line_det_flg_)
        {
            Set_no_line_det_flg(false);
        }
        else if (tmp_img_proc_corner_det_flg_)
        {
            Set_corner_det_flg(true);
        }
        else
        {
            Set_wall_det_flg(true);
        }
    }

    //////////////////////////////////////   GOAL MODE    //////////////////////////////////////

    else if (tmp_img_proc_goal_det_flg_)
    {
        if (tmp_img_proc_line_det_flg_)
        {
            Set_line_det_flg(false);
        }
        else if (tmp_img_proc_no_line_det_flg_)
        {
            Set_no_line_det_flg(true);
        }
        else
        {
            Set_goal_line_det_flg(true);
        }
    }

    ///////////////////////// LINE_MODE --- line_det_flg = true /////////////////////////
    // if (tmp_img_proc_line_det_flg_) // line mode
    // {
    //     Set_no_line_det_flg(false);
    //     Set_line_det_flg(true);
    //     ROS_ERROR("2번 : %d", Get_line_det_flg());

    //     if (tmp_img_proc_corner_det_flg_)
    //     {
    //         Set_corner_det_flg(true);
    //     }

    //     else
    //     {
    //         Set_corner_det_stop_flg(true);
    //         Set_corner_det_flg(false);
    //     }
    // }

    // //////////////////////////// No LINE_MODE --- no_line_det_flg = true ////////////////////////////
    // else if (!tmp_img_proc_line_det_flg_ && !tmp_img_proc_wall_det_flg_) // no line mode
    // {
    //     Set_no_line_det_flg(true);
    //     Set_line_det_flg(false);
    //     ROS_ERROR("1번 : %d", Get_line_det_flg());

    //     if (tmp_img_proc_corner_det_flg_)
    //     {
    //         Set_corner_det_flg(true);
    //     }

    //     else
    //     {
    //         Set_corner_det_stop_flg(true);
    //         Set_corner_det_flg(false);
    //     }
    // }

    // /////////////////////////STOP MODE --- no_line_det_flg = true /////////////////////////
    // // else if (장애물과 로봇이 접촉해 정지가 필요할 때)
    // else if (img_procPtr->Get_img_proc_stop_det())
    // {
    //     Set_stop_det_flg(true);
    // }

    // /////////////////////////GOAL LINE MODE --- goal_line_det_flg = true /////////////////////////
    // // else if (골 라인 인식 == true)
    // // {
    // // Set_goal_line_det_flg(true);
    // // }
    // else if (tmp_img_proc_goal_det_flg_)
    // {
    //     Set_goal_line_det_flg(true);
    //     Set_line_det_flg(true);
    // }

    // ///////////////////////// WALL MODE --- wall_det_flg = true /////////////////////////
    // if (tmp_img_proc_wall_det_flg_ && Get_no_line_det_flg() == true)
    // {
    //     Set_wall_det_flg(true);
    // }

    // ///////////////////////// CORNER MODE --- corner_det_flg = true /////////////////////////
    // else if (tmp_img_proc_corner_det_flg_ && !Get_no_line_det_flg())
    // {
    //     Set_corner_det_flg(true);
    //     // Set_line_det_flg(true);
    // }
}

void Move_Decision::processThread()
{
    // set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    // node loop
    while (ros::ok())
    {
        ROS_INFO("\n");
        ROS_INFO("-------------------------PROCESSTHREAD----------------------------");
        process();
        Running_Info();
        Motion_Info();
        // ProccessThread(gradient) = callbackThread(turn_angle)
        ROS_INFO("Gradient : %f", img_procPtr->Get_gradient());
        ROS_INFO("delta_x : %f", img_procPtr->Get_delta_x());
        ROS_INFO("distance : %f", img_procPtr->Get_distance());
        ROS_INFO("Move_decision img_proc_line_det : %d", img_procPtr->Get_img_proc_line_det());
        ROS_INFO("-------------------------PROCESSTHREAD----------------------------");
        ROS_INFO("\n");
        // relax to fit output rate
        loop_rate.sleep();
    }
}

// ********************************************** CALLBACK THREAD ************************************************** //

// void Move_Decision::MoveDecisionThread()
// {
//     // set node loop rate
//     ros::Rate loop_rate(SPIN_RATE);

//     // node loop
//     while (ros::ok())
//     {
//         // relax to fit output rate
//         loop_rate.sleep();
//     }
// }

void Move_Decision::Running_Mode_Decision()
{
    if (running_mode_ == WAKEUP_MODE || Get_stand_status_() == Fallen_Forward || Get_stand_status_() == Fallen_Back)
    {
        running_mode_ = WAKEUP_MODE;
    }

    else if (running_mode_ != WAKEUP_MODE)
    {
        if (goal_line_det_flg_ && line_det_flg_) // goal_line_detect_flg is true: goal_line detection mode
        {
            running_mode_ = GOAL_MODE;
        }
        else if (no_line_det_flg_ && !wall_det_flg_)
        {
            running_mode_ = NO_LINE_MODE;
        }
        else if (line_det_flg_ && !corner_det_flg_)
        {
            running_mode_ = LINE_MODE;
        }
        else if (huddle_det_flg_)
        {
            running_mode_ = HUDDLE_MODE;
        }
        else if (wall_det_flg_)
        {
            running_mode_ = WALL_MODE;
        }
        else if (stop_det_flg_)
        {
            running_mode_ = STOP_MODE;
        }
        else if (corner_det_flg_)
        {
            running_mode_ = CORNER_MODE;
        }
    }

    switch (running_mode_)
    {
    case LINE_MODE:
        LINE_mode();
        break;
    case NO_LINE_MODE:
        NOLINE_mode();
        break;
    case STOP_MODE:
        STOP_mode();
        break;
    case WAKEUP_MODE:
        WAKEUP_mode();
        break;
    case GOAL_MODE:
        GOAL_LINE_mode();
        break;
    case HUDDLE_MODE:
        HUDDLE_mode();
        break;
    case WALL_MODE:
        WALL_mode();
        break;
    case CORNER_MODE:
        CORNER_mode();
        break;
    }
}

void Move_Decision::LINE_mode()
{
    line_gradient = img_procPtr->Get_gradient();
    StraightLineDecision(line_gradient, margin_gradient);
    line_actual_angle = Get_turn_angle_();
    line_motion = Get_motion_index_();

    // If SM_req_finish = false -> InitPose
    // Straight Line
    if (straightLine == true)
    {
        if (!Get_turn_angle_on_flg() && Get_TA_req_finish())
        {
            // Left turn
            // To be zero
            if (line_actual_angle > 0)
            {
                line_actual_angle -= 1;
                if (line_actual_angle < 0)
                    line_actual_angle = 0;
                Set_turn_angle_(line_actual_angle);
                Set_turn_angle_on_flg(true);
            }

            // Right turn
            // To be zero
            else if (line_actual_angle < 0)
            {
                line_actual_angle += 1;
                if (line_actual_angle > 0)
                    line_actual_angle = 0;
                Set_turn_angle_(line_actual_angle);
                Set_turn_angle_on_flg(true);
            }
            line_motion = Motion_Index::Forward_4step;
            Set_select_motion_on_flg(true);
            Set_motion_index_(line_motion);
        }

        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            line_motion = Motion_Index::Forward_4step;
            Set_motion_index_(line_motion);
            Set_select_motion_on_flg(true);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }
        ROS_ERROR("STRAIGHT LINE");

        // TEST
        //  Set_RL_Neck_on_flg(true);
        //  Set_RL_NeckAngle(Actual_angle);
    }

    // Non Straight Line
    else if (straightLine == false)
    {
        if (!Get_turn_angle_on_flg() && Get_TA_req_finish())
        {
            // Increase Actual_angle more quickly for larger line_gradient values
            // Counter Clock wise(+) (Turn Angle sign)
            // Gradient : Angle from center of window.x to center of line.x
            // LEFT TURN
            if (line_gradient >= margin_gradient * 5)
            {
                increment = 4;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient >= margin_gradient * 4)
            {
                increment = 3;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient >= margin_gradient * 3)
            {
                increment = 2;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient >= margin_gradient * 2)
            {
                increment = 2;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient > margin_gradient * 1)
            {
                increment = 2;
                ROS_WARN("LEFT_TURN");
            }

            // Decrease Actual_angle relatively slowly for smaller line_gradient values
            // Right Turn
            else if (line_gradient <= -margin_gradient * 5)
            {
                increment = -4;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient <= -margin_gradient * 4)
            {
                increment = -3;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient <= -margin_gradient * 3)
            {
                increment = -2;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient <= -margin_gradient * 2)
            {
                increment = -2;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient < -margin_gradient * 1)
            {
                increment = -2;
                ROS_WARN("RIGHT TURN");
            }
            else
            {
                increment = 0;
            }

            line_actual_angle += increment;
            Set_turn_angle_on_flg(true);
            Set_turn_angle_(line_actual_angle);
        }

        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            line_motion = Motion_Index::Forward_4step;
            Set_motion_index_(line_motion);
            Set_select_motion_on_flg(true);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }
        ROS_ERROR("NO STRAIGHT LINE");

        // TEST
        //  Set_RL_Neck_on_flg(true);
        //  Set_RL_NeckAngle(Actual_angle);
    }
}

void Move_Decision::NOLINE_mode()
{
    // Counter Clock wise(+) (Turn Angle sign)
    // delta_x > 0 : LEFT Window  ->  Left turn (-)
    // delta_x < 0 : RIGHT window ->  Right turn  (+)
    tmp_delta_x = img_procPtr->Get_delta_x();
    noline_actual_angle = Get_turn_angle_();
    noline_motion = Get_motion_index_();

    if (tmp_delta_x < 0) // Right
    {
        if (!Get_turn_angle_on_flg() && Get_TA_req_finish())
        {
            noline_actual_angle -= 3;
            if (noline_actual_angle < -Angle_ToFindLine)
            {
                noline_actual_angle = -Angle_ToFindLine;
            }
            Set_turn_angle_(noline_actual_angle);
            Set_turn_angle_on_flg(true);
        }

        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }

        ROS_WARN("RIGHT TURN");
        // ROS_INFO("turn angle : %d", Get_turn_angle_());
    }

    else if (tmp_delta_x > 0) // LEFT
    {
        if (!Get_turn_angle_on_flg() && Get_TA_req_finish())
        {
            noline_actual_angle += 3;
            if (noline_actual_angle > Angle_ToFindLine)
            {
                noline_actual_angle = Angle_ToFindLine;
            }
            Set_turn_angle_(noline_actual_angle);
            Set_turn_angle_on_flg(true);
        }

        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }
        ROS_WARN("LEFT_TURN");
    }
}

void Move_Decision::STOP_mode()
{
    Set_Emergency_(true);
}

void Move_Decision::WAKEUP_mode()
{
    // WAKEUP_MODE
    // WakeUp_seq = 0 : Initial
    // WakeUp_seq = 1 : FWD_UP or BWD_UP
    // WakeUp_seq = 2 : Motion_Index : Initial_pose
    // WakeUp_seq = 3 : Line_mode()

    wakeup_motion = Get_motion_index_();
    wakeup_running = 0;

    tmp_stand_status = Get_stand_status_();

    if (WakeUp_seq == 0)
    {
        WakeUp_seq++;
    }

    else if (WakeUp_seq == 1)
    {
        if (tmp_stand_status == Stand_Status::Fallen_Back)
        {
            if (!Get_select_motion_on_flg() && Get_SM_req_finish())
            {
                wakeup_motion = Motion_Index::FWD_UP;
                Set_motion_index_(wakeup_motion);
                Set_select_motion_on_flg(true);
                WakeUp_seq++;
            }
        }

        else if (tmp_stand_status == Stand_Status::Fallen_Forward)
        {
            if (!Get_select_motion_on_flg() && Get_SM_req_finish())
            {
                wakeup_motion = Motion_Index::BWD_UP;
                Set_motion_index_(wakeup_motion);
                Set_select_motion_on_flg(true);
                WakeUp_seq++;
            }
        }
    }

    else if (WakeUp_seq == 2)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            wakeup_motion = InitPose;
            Set_motion_index_(wakeup_motion);
            Set_select_motion_on_flg(true);
            WakeUp_seq++;
        }
    }

    else if (WakeUp_seq == 3)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            wakeup_motion = Motion_Index::InitPose;
            Set_motion_index_(wakeup_motion);
            Set_select_motion_on_flg(true);
            WakeUp_seq++;
        }
    }

    else if (WakeUp_seq == 4)
    {
        wakeup_running = Running_Mode::LINE_MODE;
        Set_running_mode_(wakeup_motion);
        WakeUp_seq = 0;
    }
    ROS_ERROR("WAKE_UP SEQ : %d", WakeUp_seq);
    // Motion_Info();
}

void Move_Decision::GOAL_LINE_mode()
{
    // longer width 활용하고 싶음
    if (!Get_select_motion_on_flg() && Get_SM_req_finish())
    {
        Set_motion_index_(Motion_Index::Forward_4step);
        Set_select_motion_on_flg(true);
    }
    else if (!Get_SM_req_finish())
    {
        Set_motion_index_(Motion_Index::NONE);
    }
}

void Move_Decision::HUDDLE_mode()
{
    // 고개를 들고 허들의 거리값 받아와 걸음 수 계산
    // 걸음수 전달 후, LineMode로 진입
    // 허들을 다시 본다면 멈추고 다시 걸음 수 계산
    // LineMode

    // Huddle Sequence
    // 0 : Motion : InitPose (for Getting distance) (Depth)
    // 1 : Motion : Forward_Nstep (Far)
    // 2 : Motion : InitPose (for Getting distance) (Depth)
    // 3 : Motion : Forward_Nstep (Approach)
    // 4 : Motion : Step in place (Pose Control)
    // 5 : Motion : InitPose
    // 6 : Motion : Huddle Jump
    // 7 : Initializing

    huddle_actual_angle = Get_turn_angle_();
    huddle_motion = Get_motion_index_();

    // 0 : Motion : InitPose (For getting distance) (Depth)
    if (tmp_huddle_seq == 0)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            huddle_motion = Motion_Index::InitPose;
            Set_motion_index_(huddle_motion);
            huddle_distance = img_procPtr->Get_distance();
            Set_select_motion_on_flg(true);
            tmp_huddle_seq++;
            // Set_huddle_det_flg(false);
        }

        if (Get_select_motion_on_flg() && !Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }
    }

    // 1 : Motion : Forward_Nstep (Far) --> Line_mode()
    else if (tmp_huddle_seq == 1)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            huddle_motion = Motion_Index::Forward_Nstep;
            Set_distance_(huddle_distance);
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);
            tmp_huddle_seq++;
            // Set_line_det_flg(true);
            Set_huddle_det_flg(false);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
            // tmp_huddle_seq++;
        }
    }

    // 2 : Motion : InitPose (For getting distance) (Depth)
    else if (tmp_huddle_seq == 2)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            huddle_motion = Motion_Index::InitPose;
            Set_motion_index_(huddle_motion);
            huddle_distance = img_procPtr->Get_distance();
            Set_select_motion_on_flg(true);
            tmp_huddle_seq++;
            Set_huddle_det_flg(false);
        }
        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }
    }

    // 3 : Motion : Forward_Nstep (Approach) --> Line_mode()
    else if (tmp_huddle_seq == 3)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            huddle_motion = Motion_Index::Forward_Nstep;
            Set_distance_(huddle_distance);
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);
            tmp_huddle_seq++;
            // Set_line_det_flg(true);
            Set_huddle_det_flg(false);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }
    }

    // 4 : Motion : Step in place (Pose Control)
    else if (tmp_huddle_seq == 4)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            huddle_motion = Motion_Index::Step_in_place;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);
            Set_huddle_det_flg(false);
        }

        if (!Get_turn_angle_on_flg() && Get_TA_req_finish())
        {
            huddle_actual_angle = img_procPtr->Get_gradient();
            Set_turn_angle_(huddle_actual_angle);
            Set_turn_angle_on_flg(true);
            Set_huddle_det_flg(false);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }
        tmp_huddle_seq++;
    }

    // 5 : Motion : InitPose
    else if (tmp_huddle_seq == 5)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            huddle_motion = Motion_Index::InitPose;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);
            tmp_huddle_seq++;
            Set_huddle_det_flg(false);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }
    }

    // 6 : Motion : Huddle Jump
    else if (tmp_huddle_seq == 6)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            huddle_motion = Motion_Index::Huddle_Jump;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);
            tmp_huddle_seq++;
            Set_huddle_det_flg(false);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }
    }

    // 7 : Initializeing
    else if (tmp_huddle_seq == 7)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            // Set_select_motion_on_flg(true);
            tmp_huddle_seq = 0;
            Set_huddle_det_flg(false);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }
    }

    if (Get_huddle_det_stop_flg() == true)
    {
        Set_huddle_det_flg(false);
        tmp_huddle_seq = 0;
    }

    ROS_ERROR("HUDDLE_SEQ : %d", tmp_huddle_seq);
}

void Move_Decision::WALL_mode()
{
    wall_motion = Get_motion_index_();
    img_wall_number_case = img_procPtr->Get_img_proc_wall_number();
    switch (img_wall_number_case)
    {
    case 1:
        // 처음 벽 인식 후 일정 거리 안까지 직진
        if (!Get_select_motion_on_flg() && Get_SM_req_finish() && wall_number_seq == 0)
        {
            Set_motion_index_(Motion_Index::Forward_Nstep);
            Set_distance_(img_procPtr->Get_distance());
            Set_select_motion_on_flg(true);
            wall_number_seq++;
            Set_wall_det_flg(false);
        }
        else if (!Get_select_motion_on_flg() && Get_SM_req_finish() && wall_number_seq == 1)
        {
            Set_motion_index_(Motion_Index::InitPose);
            Set_select_motion_on_flg(true);
            wall_number_seq++;
            Set_wall_det_flg(false);
        }
        ROS_ERROR("FLAG 1 : START Straight");
        break;

    case 2:
        // 처음 벽 인식 후 일정 거리 안까지 직진
        if (!Get_select_motion_on_flg() && Get_SM_req_finish() && wall_number_seq == 2)
        {
            Set_motion_index_(Motion_Index::Forward_Nstep);
            Set_distance_(img_procPtr->Get_distance());
            Set_select_motion_on_flg(true);
            wall_number_seq++;
            Set_wall_det_flg(false);
        }

        else if (!Get_select_motion_on_flg() && Get_SM_req_finish() && wall_number_seq == 3)
        {
            Set_motion_index_(Motion_Index::InitPose);
            Set_select_motion_on_flg(true);
            wall_number_seq++;
            Set_wall_det_flg(false);
        }
        ROS_ERROR("FLAG 2 : Straight");
        break;

    case 3:
        // 일정 거리 앞에서 정지 후 멀리 있는 벽이 보일 때 까지 좌보행
        if (!Get_select_motion_on_flg() && Get_SM_req_finish() && wall_number_seq == 4)
        {
            Set_motion_index_(Motion_Index::Left_2step);
            Set_select_motion_on_flg(true);
            wall_number_seq++;
            Set_wall_det_flg(false);
        }

        else if (!Get_select_motion_on_flg() && Get_SM_req_finish() && wall_number_seq == 5)
        {
            Set_motion_index_(Motion_Index::InitPose);
            Set_select_motion_on_flg(true);
            wall_number_seq++;
            Set_wall_det_flg(false);
        }
        ROS_ERROR("FLAG 3 : LEFT");
        break;

    case -2:
        // 멀리 있는 벽의 일정 거리 앞까지 직진
        if (!Get_select_motion_on_flg() && Get_SM_req_finish() && wall_number_seq == 6)
        {
            Set_motion_index_(Motion_Index::Forward_Nstep);
            Set_distance_(img_procPtr->Get_distance());
            Set_select_motion_on_flg(true);
            wall_number_seq++;
            Set_wall_det_flg(false);
        }

        else if (!Get_select_motion_on_flg() && Get_SM_req_finish() && wall_number_seq == 7)
        {
            Set_motion_index_(Motion_Index::InitPose);
            Set_select_motion_on_flg(true);
            wall_number_seq++;
            Set_wall_det_flg(false);
        }

        ROS_ERROR("FLAG -2 : Straight");
        break;

    case -3:
        // 일정 거리 앞에서 정지 후 멀리 있는 벽이 보일 때 까지 우보행
        if (!Get_select_motion_on_flg() && Get_SM_req_finish() && wall_number_seq == 8)
        {
            Set_motion_index_(Motion_Index::Right_2step);
            Set_select_motion_on_flg(true);
            wall_number_seq++;
            Set_wall_det_flg(false);
        }

        else if (!Get_select_motion_on_flg() && Get_SM_req_finish() && wall_number_seq == 9)
        {
            Set_motion_index_(Motion_Index::InitPose);
            Set_select_motion_on_flg(true);
            wall_number_seq++;
            Set_wall_det_flg(false);
        }
        ROS_ERROR("FLAG -3 : RIGHT");
        break;

    case 10:
        // 멀리 있는 벽의 일정 거리 앞까지 직진
        if (!Get_select_motion_on_flg() && Get_SM_req_finish() && wall_number_seq == 10)
        {
            Set_motion_index_(Motion_Index::Forward_Nstep);
            Set_distance_(img_procPtr->Get_distance());
            Set_select_motion_on_flg(true);
            wall_number_seq++;
            Set_wall_det_flg(false);
        }
        else if (!Get_select_motion_on_flg() && Get_SM_req_finish() && wall_number_seq == 11)
        {
            Set_motion_index_(Motion_Index::InitPose);
            Set_select_motion_on_flg(true);
            wall_number_seq++;
            Set_wall_det_flg(false);
        }
        ROS_ERROR("FLAG 10 : END Straight");

        if (img_procPtr->Get_img_proc_line_det())
        {
            Set_line_det_flg(true);
        }
        break;

    default:
        Set_motion_index_(Motion_Index::InitPose);
        break;
    }

    Set_wall_det_flg(false);

    ROS_ERROR("WALL_SEQ : %d", wall_number_seq);
    ROS_ERROR("IMG_WALL_NUMBER_CASE : %d", img_wall_number_case);
}

void Move_Decision::CORNER_mode()
{
    // corner shape ㅓ / ㅜ
    // tmp_corner_shape ㅓ(1)
    // tmp_corner_shape ㅜ(2)

    // Corner seq
    // 0 : corner_shape dicision (From img_proc_corner_number) (Depth)
    // 1 : Motion : InitPose (For getting distance) (Depth)
    // 2 : Motion : Forward_Nstep (Far)
    // 3 : Motion : InitPose (For getting distance) (Depth)
    // 4 : Motion : Forward_Nstep (Approach)
    // 5 : Motion : Step in place
    // 6 : Motion : Turn Angle 90(ㅓ) or -90(ㅜ)
    // 7 : Initializing

    corner_actual_angle = Get_turn_angle_();
    tmp_corner_shape = img_procPtr->Get_img_proc_corner_number();
    corner_motion = Get_motion_index_();

    // 0 : corner_shape dicision (From img_procPtr) (Depth)
    if (tmp_corner_seq == 0)
    {
        if (tmp_corner_shape == 1)
        {
            ROS_WARN("ㅓ Type : CORNER NUMBER 1");
            tmp_corner_seq++;
        }
        else if (tmp_corner_shape == 2)
        {
            ROS_WARN("ㅜ Type : CORNER NUMBER 2");
            tmp_corner_seq++;
        }
        Set_corner_det_flg(false);
    }

    // 1 : Motion : InitPose (For getting distance) (Depth)
    else if (tmp_corner_seq == 1)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::InitPose);
            corner_distance = img_procPtr->Get_distance();
            Set_select_motion_on_flg(true);
            tmp_corner_seq++;
            Set_corner_det_flg(false);
        }
    }

    // 2 : Motion : Forward_Nstep (Far)
    else if (tmp_corner_seq == 2)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            Set_select_motion_on_flg(true);
            Set_motion_index_(Motion_Index::Forward_Nstep);
            Set_distance_(corner_distance);
            tmp_corner_seq++;
            Set_select_motion_on_flg(true);
            // Set_line_det_flg(true);
            Set_corner_det_flg(false);
        }
    }

    // 3 : Motion : InitPose (For getting distance) (Depth)
    else if (tmp_corner_seq == 3)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            Set_select_motion_on_flg(true);
            Set_motion_index_(Motion_Index::InitPose);
            corner_distance = img_procPtr->Get_distance();
            tmp_corner_seq++;
            Set_corner_det_flg(false);
        }
    }

    // 4 : Motion : Forward_Nstep (Approach)
    else if (tmp_corner_seq == 4)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            Set_select_motion_on_flg(true);
            Set_motion_index_(Motion_Index::Forward_Nstep);
            Set_distance_(corner_distance);
            tmp_corner_seq++;
            // Set_line_det_flg(true);
            Set_corner_det_flg(false);
        }
    }

    // 5 : Motion : Step in place
    else if (tmp_corner_seq == 5)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
            tmp_corner_seq++;
            Set_corner_det_flg(false);
        }
    }

    // 6 : Motion : Turn Angle 90(ㅓ) or -90(ㅜ)
    else if (tmp_corner_seq == 6)
    {
        // Rotate 90
        if (tmp_turn90 == 0)
        {
            if (!Get_turn_angle_on_flg() && Get_TA_req_finish())
            {
                corner_actual_angle += 5;
                Set_turn_angle_(corner_actual_angle);
                Set_turn_angle_on_flg(true);

                if (corner_actual_angle > Angle_ToStartWall)
                {
                    corner_actual_angle = Angle_ToStartWall;

                    if (corner_actual_angle == Angle_ToStartWall)
                    {
                        Turn90 = true;
                        tmp_turn90++;
                    }
                }
            }
        }

        // Rotate 90->0
        else if (tmp_turn90 == 1)
        {
            if (Turn90)
            {
                if (!Get_turn_angle_on_flg() && Get_TA_req_finish())
                {
                    corner_actual_angle = 0;
                    Set_turn_angle_(corner_actual_angle);
                    Turn90 = false;
                }
            }
            // Set_turn_angle_(corner_actual_angle);
            tmp_corner_seq++;
        }
        Set_corner_det_flg(false);
    }

    // 7 : Initializing
    else if (tmp_corner_seq == 7)
    {
        tmp_turn90 = 0;
        tmp_corner_seq = 0;
        Set_corner_det_flg(false);
    }

    if (Get_corner_det_stop_flg() == true)
    {
        Set_corner_det_flg(false);
    }

    ROS_ERROR("CORNER_SEQ : %d", tmp_corner_seq);
}
// Set_line_det_flg(true);

void Move_Decision::callbackThread()
{
    ros::NodeHandle nh(ros::this_node::getName());

    // Subscriber & Publisher
    // Emergency_pub_ = nh.advertise<std_msgs::Bool>("Emergency", 0);
    // imu_data_sub_ = nh.subscribe("imu", 1, &Move_Decision::imuDataCallback, this);
    // IMU_Gryo_x_publisher_ = nh_.advertise<std_msgs::Float32>("/Gyro/x", 100);
    // IMU_sensor_x_subscriber_ = nh.subscribe("/Angle/x", 1000, &Move_Decision::IMUsensorCallback, this);
    IMU_sensor_y_subscriber_ = nh.subscribe("/Angle/y", 1000, &Move_Decision::IMUsensorCallback, this);
    // IMU_sensor_z_subscriber_ = nh.subscribe("/Angle/z", 1000, &Move_Decision::IMUsensorCallback, this);

    // Server
    // motion_index_server_ = nh.advertiseService("Select_Motion", &Move_Decision::playMotion, this);
    // turn_angle_server_ = nh.advertiseService("Turn_Angle", &Move_Decision::turn_angle, this);
    // UD_NeckAngle_server_ = nh.advertiseService("UD_NeckAngle", &Move_Decision::Move_UD_NeckAngle, this);
    // RL_NeckAngle_server_ = nh.advertiseService("RL_NeckAngle", &Move_Decision::Move_RL_NeckAngle, this);
    // Emergency_server_ = nh.advertiseService("Emergency", &Move_Decision::Emergency, this);
    SendMotion_server_ = nh.advertiseService("SendMotion", &Move_Decision::SendMotion, this);

    ros::Rate loop_rate(SPIN_RATE);
    startMode();
    while (nh.ok())
    {
        ROS_INFO("-------------------------CALLBACKTHREAD----------------------------");
        ROS_INFO("-------------------------------------------------------------------");
        // StatusCheck();
        Running_Mode_Decision();
        Running_Info();
        Motion_Info();
        ROS_INFO("angle : %f", Get_turn_angle_());
        ROS_INFO("RL_Neck : %f", Get_RL_NeckAngle());
        ROS_INFO("UD_Neck : %f", Get_UD_NeckAngle());
        ROS_INFO("Distance : %f", img_procPtr->Get_distance());
        ROS_INFO("EMG : %s", Get_Emergency_() ? "true" : "false");
        ROS_INFO("-------------------------------------------------------------------");
        ROS_INFO("-------------------------CALLBACKTHREAD----------------------------");

        ros::spinOnce();
        loop_rate.sleep();
        // usleep(1000);
    }
}

//////////////////////////////////////////////////////////// Server Part ////////////////////////////////////////////////////////////////
bool Move_Decision::SendMotion(dynamixel_current_2port::SendMotion::Request &req, dynamixel_current_2port::SendMotion::Response &res)
{
    // Finish Check
    bool req_SM_finish = req.SM_finish;
    bool req_TA_finish = req.TA_finish;
    bool req_UD_finish = req.UD_finish;
    bool req_RL_finish = req.RL_finish;
    bool req_EM_finish = req.EM_finish;

    Set_SM_req_finish(req_SM_finish);
    Set_TA_req_finish(req_TA_finish);
    Set_UD_req_finish(req_UD_finish);
    Set_RL_req_finish(req_RL_finish);
    Set_EM_req_finish(req_EM_finish);

    // Response
    auto res_SM = playMotion();
    double res_TA = turn_angle();
    double res_UD = Move_UD_NeckAngle();
    double res_RL = Move_RL_NeckAngle();
    bool res_EM = Emergency();

    res.select_motion = std::get<0>(res_SM);
    res.distance = std::get<1>(res_SM);
    res.turn_angle = res_TA;
    res.ud_neckangle = res_UD;
    res.rl_neckangle = res_RL;
    res.emergency = res_EM;

    return true;
}

std::tuple<int8_t, double> Move_Decision::playMotion()
{
    int8_t res_select_motion = 0;
    double res_distance = 0;
    int8_t total = Get_TA_req_finish() + Get_UD_req_finish() + Get_RL_req_finish() + Get_EM_req_finish();
    if (Get_SM_req_finish())
    {
        if ((Get_stand_status_() == Stand_Status::Stand) && (Get_select_motion_on_flg() == true) /*&& total <=4*/ )
        {
            switch (Get_motion_index_())
            {
            case Motion_Index::InitPose:
                res_select_motion = Motion_Index::InitPose;
                break;

            case Motion_Index::Forward_4step:
                res_select_motion = Motion_Index::Forward_4step;
                break;

            case Motion_Index::Left_2step:
                res_select_motion = Motion_Index::Left_2step;
                break;

            case Motion_Index::Step_in_place:
                res_select_motion = Motion_Index::Step_in_place;
                break;

            case Motion_Index::Right_2step:
                res_select_motion = Motion_Index::Right_2step;
                break;

            case Motion_Index::Back_4step:
                res_select_motion = Motion_Index::Back_4step;
                break;

            case Motion_Index::Forward_Nstep:
                res_select_motion = Motion_Index::Forward_Nstep;
                res_distance = img_procPtr->Get_distance();
                break;

            case Motion_Index::Huddle_Jump:
                res_select_motion = Motion_Index::Huddle_Jump;
                break;

            case Motion_Index::NONE:
                res_select_motion = Motion_Index::NONE;
                break;
            }
            Set_select_motion_on_flg(false);
        }

        else if (((Get_stand_status_() == Stand_Status::Fallen_Back)) || (Get_stand_status_() == Stand_Status::Fallen_Forward) && (Get_select_motion_on_flg() == true))
        {
            switch (Get_motion_index_())
            {
            case Motion_Index::FWD_UP:
                res_select_motion = Motion_Index::FWD_UP;
                break;

            case Motion_Index::BWD_UP:
                res_select_motion = Motion_Index::BWD_UP;
                break;
            }
            Set_select_motion_on_flg(false);
        }
    }

    else if (!Get_SM_req_finish())
    {
        res_select_motion = Motion_Index::NONE;
    }
    ROS_WARN("[MESSAGE] SM Request :   %s ", Get_SM_req_finish() ? "true" : "false");
    ROS_INFO("#[MESSAGE] SM Motion :   %d#", res_select_motion);
    ROS_INFO("#[MESSAGE] SM Distance : %f#", res_distance);

    return std::make_tuple(res_select_motion, res_distance);
}

double Move_Decision::turn_angle()
{
    double res_turn_angle = 0;
    if (Get_TA_req_finish())
    {
        if ((Get_stand_status_() == Stand_Status::Stand) && Get_turn_angle_on_flg())
        {
            // img_procssing
            res_turn_angle = this->Get_turn_angle_();
            Set_turn_angle_on_flg(false);
        }
    }

    ROS_WARN("[MESSAGE] TA Request :   %s ", Get_TA_req_finish() ? "true" : "false");
    ROS_INFO("#[MESSAGE] TA Response : %f#", res_turn_angle);

    return res_turn_angle;
}

double Move_Decision::Move_UD_NeckAngle()
{
    double res_ud_neckangle = 0;
    if (Get_UD_req_finish())
    {
        if ((Get_stand_status_() == Stand_Status::Stand) && (Get_UD_Neck_on_flg() == true))
        {
            // img_procssing
            res_ud_neckangle = Get_UD_NeckAngle();
            Set_UD_Neck_on_flg(false);
        }
    }

    ROS_WARN("[MESSAGE] UD Request :   %s ", Get_UD_req_finish() ? "true" : "false");
    ROS_INFO("#[MESSAGE] UD Response : %f#", res_ud_neckangle);

    return res_ud_neckangle;
}

double Move_Decision::Move_RL_NeckAngle()
{
    double res_rl_neckangle = 0;
    if (Get_RL_req_finish())
    {
        if ((Get_stand_status_() == Stand_Status::Stand) && (Get_RL_Neck_on_flg() == true))
        {
            // img_procssing
            res_rl_neckangle = Get_RL_NeckAngle();
            Set_RL_Neck_on_flg(false);
        }
    }

    ROS_WARN("[MESSAGE] RL Request :   %s ", Get_RL_req_finish() ? "true" : "false");
    ROS_INFO("#[MESSAGE] RL Response : %f#", res_rl_neckangle);

    return res_rl_neckangle;
}

bool Move_Decision::Emergency()
{
    bool res_emergency = false;
    if (Get_EM_req_finish())
    {
        if ((Get_stand_status_() == Stand_Status::Stand) && Get_emergency_on_flg())
        {
            // 1 : Stop
            // 0 : Keep Going (Option)
            res_emergency = Get_Emergency_();
            Set_emergency_on_flg(false);
        }
    }

    ROS_ERROR("[MESSAGE] EMG Request :   %s ", Get_EM_req_finish() ? "true" : "false");
    ROS_ERROR("#[MESSAGE] EMG Response : %s#", res_emergency ? "true" : "false");

    return res_emergency;
}

///////////////////////////////////////// About Publish & Subscribe /////////////////////////////////////////
void Move_Decision::startMode()
{
    bool send_emergency = Get_Emergency_();
    // EmergencyPublish(send_emergency);
    Set_motion_index_(Motion_Index::InitPose);
}

// void Move_Decision::stopMode()
// {
//     playMotion(Motion_Index::Foward_4step);
// }

// Emergency Stop
// 1 : Stop
// 0 : Keep Going (Option)
// void Move_Decision::EmergencyPublish(bool _emergency)
// {
//     std_msgs::Bool A;
//     A.data = _emergency;

//     Emergency_pub_.publish(A);
//     // ROS_INFO("%d",Emergency_);
// }

void Move_Decision::IMUsensorCallback(const std_msgs::Float32::ConstPtr &IMU)
{
    if (stop_fallen_check_ == true)
        return;

    // RPY(0) = IMU->data;
    RPY(1) = IMU->data;
    // RPY(2) = IMU->data;

    double pitch = RPY(1) * 57.2958;

    // Quaternino2RPY();
    // sensorPtr->RPY *= (180 / M_PI);
    // double roll = sensorPtr->RPY(0);
    // double pitch = sensorPtr->RPY(1);
    // double yaw = sensorPtr->RPY(2);

    double alpha = 0.4;
    if (present_pitch_ == 0)
        present_pitch_ = pitch;
    // else
    //     present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

    if (present_pitch_ > FALL_FORWARD_LIMIT)
        Set_stand_status_(Stand_Status::Fallen_Forward);
    else if (present_pitch_ < FALL_BACK_LIMIT)
        Set_stand_status_(Stand_Status::Fallen_Back);
    else
        Set_stand_status_(Stand_Status::Stand);

    // ROS_ERROR("STAND_STAUS : %d", Get_stand_status_());
    // ROS_ERROR("PITCH : %f", pitch);
}

// ********************************************** FUNCTION ************************************************** //

// // // check fallen states
// void Move_Decision::StatusCheck()
// {
//     if (stop_fallen_check_ == true)
//         return;

//     // Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
//     Eigen::Quaterniond orientation(quaternion(3), quaternion(0), quaternion(1), quaternion(2));
//     Eigen::MatrixXd rpy_orientation = convertQuaternionToRPY(orientation);
//     rpy_orientation *= (180 / M_PI);

//     ROS_ERROR("Roll : %3.2f, Pitch : %2.2f", rpy_orientation.coeff(0, 0), rpy_orientation.coeff(1, 0));

//     double pitch = rpy_orientation.coeff(1, 0);

//     // Quaternino2RPY();
//     // sensorPtr->RPY *= (180 / M_PI);
//     // double roll = sensorPtr->RPY(0);
//     // double pitch = sensorPtr->RPY(1);
//     // double yaw = sensorPtr->RPY(2);

//     double alpha = 0.4;
//     if (present_pitch_ == 0)
//         present_pitch_ = pitch;
//     else
//         present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

//     if (present_pitch_ > FALL_FORWARD_LIMIT)
//         Set_stand_status_(Stand_Status::Fallen_Forward);
//     else if (present_pitch_ < FALL_BACK_LIMIT)
//         Set_stand_status_(Stand_Status::Fallen_Back);
//     else
//         Set_stand_status_(Stand_Status::Stand);

//     ROS_ERROR("STAND_STAUS : %d", Get_stand_status_());
// }

// void Move_Decision::Quaternino2RPY()
// {
//     tf::Quaternion q(
//         quaternion(0),
//         quaternion(1),
//         quaternion(2),
//         quaternion(3));
//     tf::Matrix3x3 m(q);
//     m.getRPY(RPY(0), RPY(1), RPY(2));
//     // ROS_INFO("roll : %.3f", sensorPtr->RPY(0) * 57.2958);
//     // ROS_INFO("pitch : %.3f", sensorPtr->RPY(1) * 57.2958);
//     // ROS_INFO("yaw : %.3f", sensorPtr->RPY(2) * 57.2958);
// }

// Eigen::Vector3d Move_Decision::convertRotationToRPY(const Eigen::Matrix3d &rotation)
// {
//     Eigen::Vector3d rpy; // = Eigen::MatrixXd::Zero(3,1);

//     rpy.coeffRef(0, 0) = atan2(rotation.coeff(2, 1), rotation.coeff(2, 2));
//     rpy.coeffRef(1, 0) = atan2(-rotation.coeff(2, 0), sqrt(pow(rotation.coeff(2, 1), 2) + pow(rotation.coeff(2, 2), 2)));
//     rpy.coeffRef(2, 0) = atan2(rotation.coeff(1, 0), rotation.coeff(0, 0));

//     return rpy;
// }

// Eigen::Vector3d Move_Decision::convertQuaternionToRPY(const Eigen::Quaterniond &quaternion)
// {
//     Eigen::Vector3d rpy = convertRotationToRPY(quaternion.toRotationMatrix());

//     return rpy;
// }

/*            -mg_gra        mg_gra              */
/*      False    |     True    |     False       */
/*  <----------------------------------------->  */
void Move_Decision::StraightLineDecision(double gra, double mg_gra)
{
    if ((gra > mg_gra) || (gra < -mg_gra)) // judged to be a straight line. If it exists between the slopes, it is a straight line.
    {
        straightLine = false;
    }

    // Straight Line Decision
    else /*if ((gradient < margin_gradient && (gradient > -margin_gradient)))*/
    {
        straightLine = true;
    }
}

void Move_Decision::Motion_Info()
{
    string tmp_motion;
    switch (Get_motion_index_())
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
    ROS_INFO("Motion_Index : %s", tmp_motion.c_str());
}

void Move_Decision::Running_Info()
{
    string tmp_running;
    switch (Get_running_mode_())
    {
    case Running_Mode::LINE_MODE:
        tmp_running = Str_LINE_MODE;
        break;

    case Running_Mode::NO_LINE_MODE:
        tmp_running = Str_NO_LINE_MODE;
        break;

    case Running_Mode::STOP_MODE:
        tmp_running = Str_STOP_MODE;
        break;

    case Running_Mode::WAKEUP_MODE:
        tmp_running = Str_WAKEUP_MODE;
        break;

    case Running_Mode::GOAL_MODE:
        tmp_running = Str_GOAL_MODE;
        break;

    case Running_Mode::HUDDLE_MODE:
        tmp_running = Str_HUDDLE_MODE;
        break;

    case Running_Mode::WALL_MODE:
        tmp_running = Str_WALL_MODE;
        break;

    case Running_Mode::CORNER_MODE:
        tmp_running = Str_CORNER_MODE;
        break;
    }
    ROS_INFO("Running_Mode : %s", tmp_running.c_str());
}

// ********************************************** GETTERS ************************************************** //

bool Move_Decision::Get_Emergency_() const
{
    std::lock_guard<std::mutex> lock(mtx_Emergency_);
    return Emergency_;
}

bool Move_Decision::Get_emergency_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_emergency_on_flg_);
    return emergency_on_flg_;
}

int8_t Move_Decision::Get_motion_index_() const
{
    std::lock_guard<std::mutex> lock(mtx_motion_index_);
    return motion_index_;
}

int8_t Move_Decision::Get_stand_status_() const
{
    std::lock_guard<std::mutex> lock(mtx_stand_status_);
    return stand_status_;
}

int8_t Move_Decision::Get_running_mode_() const
{
    std::lock_guard<std::mutex> lock(mtx_running_mode_);
    return running_mode_;
}

double Move_Decision::Get_turn_angle_() const
{
    std::lock_guard<std::mutex> lock(mtx_turn_angle_);
    return turn_angle_;
}

double Move_Decision::Get_distance_() const
{
    std::lock_guard<std::mutex> lock(mtx_distance_);
    return distance_;
}

bool Move_Decision::Get_ProcessON() const
{
    std::lock_guard<std::mutex> lock(mtx_ProcessON_);
    return ProcessON_;
}

bool Move_Decision::Get_MoveDecisionON() const
{
    std::lock_guard<std::mutex> lock(mtx_MoveDecisionON_);
    return MoveDecisionON_;
}

bool Move_Decision::Get_CallbackON() const
{
    std::lock_guard<std::mutex> lock(mtx_CallbackON_);
    return CallbackON_;
}

bool Move_Decision::Get_line_det_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_line_det_flg);
    return line_det_flg_;
}

bool Move_Decision::Get_no_line_det_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_no_line_det_flg);
    return no_line_det_flg_;
}

bool Move_Decision::Get_goal_line_det_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_goal_line_det_flg);
    return goal_line_det_flg_;
}

bool Move_Decision::Get_huddle_det_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_flg);
    return huddle_det_flg_;
}

bool Move_Decision::Get_huddle_det_stop_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_stop_flg);
    return huddle_det_stop_flg_;
}

bool Move_Decision::Get_wall_det_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_wall_det_flg);
    return wall_det_flg_;
}

bool Move_Decision::Get_stop_det_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_stop_det_flg);
    return stop_det_flg_;
}

bool Move_Decision::Get_corner_det_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_corner_det_flg);
    return corner_det_flg_;
}

bool Move_Decision::Get_corner_det_stop_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_corner_det_stop_flg);
    return corner_det_stop_flg_;
}

bool Move_Decision::Get_select_motion_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_select_motion_on_flg_);
    return select_motion_on_flg_;
}

bool Move_Decision::Get_turn_angle_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_turn_angle_on_flg_);
    return turn_angle_on_flg_;
}

double Move_Decision::Get_UD_NeckAngle() const
{
    std::lock_guard<std::mutex> lock(mtx_UD_NeckAngle_);
    return UD_NeckAngle_;
}

double Move_Decision::Get_RL_NeckAngle() const
{
    std::lock_guard<std::mutex> lock(mtx_RL_NeckAngle_);
    return RL_NeckAngle_;
}

bool Move_Decision::Get_UD_Neck_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_UD_Neck_on_flg);
    return UD_Neck_on_flg_;
}

bool Move_Decision::Get_RL_Neck_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_RL_Neck_on_flg);
    return RL_Neck_on_flg_;
}

bool Move_Decision::Get_SM_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_SM_req_finish_);
    return SM_req_finish_;
}

bool Move_Decision::Get_TA_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_TA_req_finish_);
    return TA_req_finish_;
}

bool Move_Decision::Get_UD_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_UD_req_finish_);
    return UD_req_finish_;
}

bool Move_Decision::Get_RL_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_RL_req_finish_);
    return RL_req_finish_;
}

bool Move_Decision::Get_EM_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_EM_req_finish_);
    return EM_req_finish_;
}

// ********************************************** SETTERS ************************************************** //

void Move_Decision::Set_Emergency_(bool Emergency)
{
    std::lock_guard<std::mutex> lock(mtx_Emergency_);
    this->Emergency_ = Emergency;
}

void Move_Decision::Set_emergency_on_flg(bool emergency_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_emergency_on_flg_);
    this->emergency_on_flg_ = emergency_on_flg;
}

void Move_Decision::Set_motion_index_(int8_t motion_index)
{
    std::lock_guard<std::mutex> lock(mtx_motion_index_);
    this->motion_index_ = motion_index;
}

void Move_Decision::Set_stand_status_(int8_t stand_status)
{
    std::lock_guard<std::mutex> lock(mtx_stand_status_);
    this->stand_status_ = stand_status;
}

void Move_Decision::Set_running_mode_(int8_t running_mode)
{
    std::lock_guard<std::mutex> lock(mtx_running_mode_);
    this->running_mode_ = running_mode;
}

void Move_Decision::Set_turn_angle_(double turn_angle)
{
    std::lock_guard<std::mutex> lock(mtx_turn_angle_);
    this->turn_angle_ = turn_angle;
}

void Move_Decision::Set_distance_(double distance)
{
    std::lock_guard<std::mutex> lock(mtx_distance_);
    this->distance_ = distance;
}

void Move_Decision::Set_ProcessON(bool ProcessON)
{
    std::lock_guard<std::mutex> lock(mtx_ProcessON_);
    this->ProcessON_ = ProcessON;
}

void Move_Decision::Set_MoveDecisionON(bool MoveDecisionON)
{
    std::lock_guard<std::mutex> lock(mtx_MoveDecisionON_);
    this->MoveDecisionON_ = MoveDecisionON;
}

void Move_Decision::Set_CallbackON(bool CallbackON)
{
    this->CallbackON_ = CallbackON;
}

void Move_Decision::Set_goal_line_det_flg(bool goal_line_det_flg)
{
    this->goal_line_det_flg_ = goal_line_det_flg;
}

void Move_Decision::Set_line_det_flg(bool line_det_flg)
{
    std::lock_guard<std::mutex> lock(mtx_CallbackON_);
    this->line_det_flg_ = line_det_flg;
}

void Move_Decision::Set_no_line_det_flg(bool no_line_det_flg)
{
    std::lock_guard<std::mutex> lock(mtx_no_line_det_flg);
    this->no_line_det_flg_ = no_line_det_flg;
}

void Move_Decision::Set_huddle_det_flg(bool huddle_det_flg)
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_flg);
    this->huddle_det_flg_ = huddle_det_flg;
}

void Move_Decision::Set_wall_det_flg(bool wall_det_flg)
{
    std::lock_guard<std::mutex> lock(mtx_wall_det_flg);
    this->wall_det_flg_ = wall_det_flg;
}

void Move_Decision::Set_stop_det_flg(bool stop_det_flg)
{
    std::lock_guard<std::mutex> lock(mtx_stop_det_flg);
    this->stop_det_flg_ = stop_det_flg;
}

void Move_Decision::Set_corner_det_flg(bool corner_det_flg)
{
    std::lock_guard<std::mutex> lock(mtx_corner_det_flg);
    this->corner_det_flg_ = corner_det_flg;
}

void Move_Decision::Set_huddle_det_stop_flg(bool huddle_det_stop_flg)
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_stop_flg);
    this->huddle_det_stop_flg_ = huddle_det_stop_flg;
}

void Move_Decision::Set_select_motion_on_flg(bool select_motion_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_select_motion_on_flg_);
    this->select_motion_on_flg_ = select_motion_on_flg;
}

void Move_Decision::Set_turn_angle_on_flg(bool turn_angle_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_turn_angle_on_flg_);
    this->turn_angle_on_flg_ = turn_angle_on_flg;
}

void Move_Decision::Set_corner_det_stop_flg(bool corner_det_stop_flg)
{
    std::lock_guard<std::mutex> lock(mtx_corner_det_stop_flg);
    this->corner_det_stop_flg_ = corner_det_stop_flg;
}
void Move_Decision::Set_RL_NeckAngle(double RL_NeckAngle)
{
    std::lock_guard<std::mutex> lock(mtx_RL_NeckAngle_);
    this->RL_NeckAngle_ = RL_NeckAngle;
}

void Move_Decision::Set_UD_NeckAngle(double UD_NeckAngle)
{
    std::lock_guard<std::mutex> lock(mtx_UD_NeckAngle_);
    this->UD_NeckAngle_ = UD_NeckAngle;
}

void Move_Decision::Set_RL_Neck_on_flg(bool RL_Neck_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_RL_Neck_on_flg);
    this->RL_Neck_on_flg_ = RL_Neck_on_flg;
}

void Move_Decision::Set_UD_Neck_on_flg(bool UD_Neck_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_UD_Neck_on_flg);
    this->UD_Neck_on_flg_ = UD_Neck_on_flg;
}

void Move_Decision::Set_SM_req_finish(bool SM_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_SM_req_finish_);
    this->SM_req_finish_ = SM_req_finish;
}

void Move_Decision::Set_TA_req_finish(bool TA_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_TA_req_finish_);
    this->TA_req_finish_ = TA_req_finish;
}

void Move_Decision::Set_UD_req_finish(bool UD_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_UD_req_finish_);
    this->UD_req_finish_ = UD_req_finish;
}

void Move_Decision::Set_RL_req_finish(bool RL_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_RL_req_finish_);
    this->RL_req_finish_ = RL_req_finish;
}

void Move_Decision::Set_EM_req_finish(bool EM_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_EM_req_finish_);
    this->EM_req_finish_ = EM_req_finish;
}
