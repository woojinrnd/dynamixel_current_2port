#include "Move_decision.hpp"
#include <numeric>

// 
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
      straightLine(true),
      UD_NeckAngle_(UD_CENTER)
{
    // Init ROS
    ros::NodeHandle nh(ros::this_node::getName());

    boost::thread process_thread = boost::thread(boost::bind(&Move_Decision::processThread, this));
    boost::thread web_process_thread = boost::thread(boost::bind(&Img_proc::webcam_thread, img_procPtr));
    boost::thread depth_process_thread = boost::thread(boost::bind(&Img_proc::realsense_thread, img_procPtr));
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

    // corner mode
    // Set_no_line_det_flg(true);
    // Set_corner_det_flg_2d(true);


    // wall mode
    // tmp_img_proc_wall_det_flg_ = true;
    // Set_wall_det_flg(true);

    //////////////////////////////////////   DEBUG WINDOW    //////////////////////////////////////

    // AllModereset();

    tmp_img_proc_line_det_flg_ = img_procPtr->Get_img_proc_line_det();
    tmp_img_proc_no_line_det_flg_ = img_procPtr->Get_img_proc_no_line_det();
    tmp_img_proc_huddle_det_flg_2d_ = img_procPtr->Get_img_proc_huddle_det_2d();
    tmp_img_proc_wall_det_flg_ = img_procPtr->Get_img_proc_wall_det();
    // tmp_img_proc_wall_det_flg_ = Get_wall_det_flg();
    tmp_img_proc_goal_det_flg_ = img_procPtr->Get_img_proc_goal_line_det();
    tmp_img_proc_corner_det_flg_2d_ = img_procPtr->Get_img_proc_corner_det_2d();
    
    ROS_WARN("tmp_img_proc_line_det_flg_ : %d", tmp_img_proc_line_det_flg_);
    ROS_WARN("tmp_img_proc_no_line_det_flg_ : %d", tmp_img_proc_no_line_det_flg_);
    ROS_WARN("tmp_img_proc_huddle_det_flg_2d_ : %d", tmp_img_proc_huddle_det_flg_2d_);
    ROS_WARN("tmp_img_proc_corner_det_flg_2d_ : %d", tmp_img_proc_corner_det_flg_2d_);
    ROS_WARN("tmp_img_proc_wall_det_flg_ : %d", tmp_img_proc_wall_det_flg_);

    //////////////////////////////////////   LINE MODE    //////////////////////////////////////

    if (tmp_img_proc_line_det_flg_)
    {
        if (tmp_img_proc_huddle_det_flg_2d_)
        {
            // if (Get_huddle_det_stop_flg() == true)
            // {
            //     if (huddle_seq_finish)
            //     {
            //         Set_line_det_flg(true);
            //         Set_huddle_det_flg_2d(false);
            //         ROS_WARN("djdkfjdkfjd");
            //     }
            // }

            // else if (tmp_huddle_seq == 5 || huddle_seq_finish)
            // {
            //     Set_line_det_flg(true);
            //     Set_huddle_det_flg_2d(false);
            // }

            // else if (0 <= tmp_huddle_seq <= 4)
            // {
            //     Set_line_det_flg(false);
            //     Set_huddle_det_flg_2d(true);
            // }

            // else
            // {
            //     Set_huddle_det_flg_2d(true);
            //     Set_line_det_flg(false);
            //     Set_no_line_det_flg(false);
            // }

            Set_huddle_det_flg_2d(true);
            Set_line_det_flg(false);
            Set_no_line_det_flg(false);
        }

        else if (tmp_img_proc_goal_det_flg_)
        {
            Set_goal_line_det_flg(true);
            Set_line_det_flg(false);
            Set_no_line_det_flg(false);
        }

        else if (tmp_img_proc_corner_det_flg_2d_)
        {
            Set_huddle_det_flg_2d(false); //newwwwwww
            if (Get_corner_det_stop_flg() && corner_seq_finish)
            {
                Set_wall_det_flg(true);
                Set_corner_det_flg_2d(false);
            }

            else if (Get_corner_det_stop_flg() && tmp_corner_shape == 2 && corner_seq_finish)
            {
                Set_line_det_flg(true);
                Set_corner_det_flg_2d(false);
            }

            else
            {
                Set_corner_det_flg_2d(true);
                Set_line_det_flg(false);
                Set_no_line_det_flg(false);
            }
        }

        else if (tmp_img_proc_wall_det_flg_)
        {
            Set_wall_det_flg(true);
            Set_line_det_flg(false);
        }
        // // Keeping huddle mode
        if (tmp_huddle_seq == 1 || tmp_huddle_seq == 2 || tmp_huddle_seq == 3 || tmp_huddle_seq == 4)
        {
            Set_huddle_det_flg_2d(true);
            Set_line_det_flg(false);
            Set_no_line_det_flg(false);
        }

        // // Keeping corner mode
        if (tmp_corner_seq == 1 || tmp_corner_seq == 2 || tmp_corner_seq == 3)
        {
            Set_corner_det_flg_2d(true);
            Set_line_det_flg(false);
        }

        else
        {
            Set_line_det_flg(true);
            Set_no_line_det_flg(false);
        }
    }

    if (tmp_img_proc_line_det_flg_ && tmp_img_proc_huddle_det_flg_2d_)
    {
        Set_huddle_det_flg_2d(true);
        Set_line_det_flg(false);
        Set_no_line_det_flg(false);
    }

    if (tmp_img_proc_corner_det_flg_2d_ && tmp_img_proc_line_det_flg_)
    {
        Set_corner_det_flg_2d(true);
        Set_line_det_flg(false);
        Set_no_line_det_flg(false);
    }

    // // Keeping huddle mode
    if (tmp_huddle_seq == 2 || tmp_huddle_seq == 3 || tmp_huddle_seq == 4)
    {
        Set_huddle_det_flg_2d(true);
        Set_line_det_flg(false);
        Set_no_line_det_flg(false);
    }

    //////////////////////////////////////   NO LINE MODE    //////////////////////////////////////

    else if (tmp_img_proc_no_line_det_flg_)
    {
        if (tmp_img_proc_huddle_det_flg_2d_)
        {
            Set_huddle_det_flg_2d(true);
            Set_no_line_det_flg(false);
            Set_line_det_flg(false);

            // if (Get_huddle_det_stop_flg())
            // {
            //     if (huddle_seq_finish)
            //     {
            //         Set_no_line_det_flg(true);
            //         Set_huddle_det_flg_2d(false);
            //     }
            // }

            // else
            // {
            //     Set_huddle_det_flg_2d(true);
            //     Set_line_det_flg(false);
            //     Set_no_line_det_flg(false);
            // }
        }



        else if (tmp_img_proc_corner_det_flg_2d_)
        {
            if (Get_corner_det_stop_flg() && tmp_corner_shape == 1 && corner_seq_finish)
            {
                Set_wall_det_flg(true);
                Set_corner_det_flg_2d(false);
            }

            if (Get_corner_det_stop_flg() && tmp_corner_shape == 2 && corner_seq_finish)
            {
                Set_line_det_flg(true);
                Set_corner_det_flg_2d(false);
            }

            else
            {
                Set_corner_det_flg_2d(true);
                Set_line_det_flg(false);
                Set_no_line_det_flg(false);
            }
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

        // // Keeping huddle mode
        // else if (tmp_huddle_seq == 2 || tmp_huddle_seq == 3 || tmp_huddle_seq == 4)
        // {
        //     Set_huddle_det_flg_2d(true);
        //     Set_line_det_flg(false);
        // }

        // // Keeping corner mode
        if (tmp_corner_seq == 1 || tmp_corner_seq == 2 || tmp_corner_seq == 3)
        {
            ROS_INFO("KEEPING CORNER MODE");
            Set_corner_det_flg_2d(true);
            Set_line_det_flg(false);
        }
        
        else
        {
            Set_no_line_det_flg(true);
        }
    }

    if (tmp_img_proc_no_line_det_flg_ && tmp_img_proc_huddle_det_flg_2d_)
    {
        Set_huddle_det_flg_2d(true);
        Set_line_det_flg(false);
        Set_no_line_det_flg(false);
    }

    if (tmp_img_proc_no_line_det_flg_ && tmp_img_proc_wall_det_flg_)
    {
        Set_wall_det_flg(true);
        Set_no_line_det_flg(false);
        Set_line_det_flg(false);
    }

    //////////////////////////////////////   HUDDLE MODE    //////////////////////////////////////

    else if (tmp_img_proc_huddle_det_flg_2d_)
    {
        if (tmp_img_proc_line_det_flg_)
        {
            Set_line_det_flg(false);
            Set_huddle_det_flg_2d(true);
        }

        else if (tmp_img_proc_no_line_det_flg_)
        {
            Set_no_line_det_flg(false);
            Set_huddle_det_flg_2d(true);
        }

        // if (Get_huddle_det_stop_flg() && huddle_seq_finish)
        // {
        //     Set_line_det_flg(true);
        //     Set_huddle_det_flg_2d(false);
        // }

        else
        {
            Set_huddle_det_flg_2d(true);
        }
    }

    //////////////////////////////////////   CORNER MODE    //////////////////////////////////////

    else if (tmp_img_proc_corner_det_flg_2d_)
    {
        Set_no_line_det_flg(false);
        Set_huddle_det_flg_2d(false); //newwwwww
        if (tmp_img_proc_line_det_flg_)
        {
            Set_line_det_flg(false);
            Set_corner_det_flg_2d(true);
        }

        else if (tmp_img_proc_no_line_det_flg_)
        {
            Set_no_line_det_flg(false);
            Set_corner_det_flg_2d(true);
        }

        else if (tmp_img_proc_wall_det_flg_)
        {
            Set_wall_det_flg(false);
            Set_corner_det_flg_2d(true);
        }
        
        else if (tmp_img_proc_huddle_det_flg_2d_)
        {
            Set_huddle_det_flg_2d(false);
            Set_corner_det_flg_2d(true);
        }

        if (Get_corner_det_stop_flg() && tmp_corner_shape == 1 && corner_seq_finish)
        {
            Set_wall_det_flg(true);
            Set_corner_det_flg_2d(false);
        }

        if (Get_corner_det_stop_flg() && tmp_corner_shape == 2 && corner_seq_finish)
        {
            Set_line_det_flg(true);
            Set_corner_det_flg_2d(false);
            Set_corner_det_flg_2d(true);
        }

        else
        {
            Set_corner_det_flg_2d(true);
        }
    }

    //////////////////////////////////////   WALL MODE    //////////////////////////////////////

    if (tmp_img_proc_wall_det_flg_ && tmp_img_proc_no_line_det_flg_)
    {
        Set_wall_det_flg(true);
        Set_no_line_det_flg(false);
        Set_line_det_flg(false);
    }

    if (tmp_img_proc_wall_det_flg_ && tmp_img_proc_line_det_flg_)
    {
        Set_wall_det_flg(true);
        Set_no_line_det_flg(false);
        Set_line_det_flg(false);
    }

    if (tmp_img_proc_wall_det_flg_ && tmp_img_proc_corner_det_flg_2d_)
    {
        Set_wall_det_flg(true);
        Set_corner_det_flg_2d(false);
        Set_no_line_det_flg(false);
        Set_line_det_flg(false);
    }

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

        else if (tmp_img_proc_corner_det_flg_2d_)
        {
            Set_corner_det_flg_2d(true);
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
}

void Move_Decision::AllModereset()
{
    this->Set_corner_det_flg_2d(false);
    this->Set_huddle_det_flg_2d(false);
    this->Set_no_line_det_flg(false);
    this->Set_line_det_flg(false);
}

void Move_Decision::processThread()
{
    // set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    // node loop
    while (ros::ok())
    {
        // ROS_INFO("\n");
        // ROS_INFO("-------------------------PROCESSTHREAD----------------------------");
        if (!wall_seq_start)
        {
            process();
        }

        else if (wall_seq_start)
        {
            Set_wall_det_flg(true);
        }
        
        // Running_Info();
        // Motion_Info();
        // ROS_INFO("Gradient : %f", img_procPtr->Get_gradient());
        // ROS_INFO("delta_x : %f", img_procPtr->Get_delta_x());
        // ROS_INFO("distance : %f", img_procPtr->Get_distance());
        // ROS_INFO("Move_decision img_proc_line_det : %d", img_procPtr->Get_img_proc_line_det());
        // ROS_INFO("-------------------------PROCESSTHREAD----------------------------");
        // ROS_INFO("\n");
        // relax to fit output rate
        loop_rate.sleep();
    }
}

// ********************************************** CALLBACK THREAD ************************************************** //

void Move_Decision::Running_Mode_Decision()
{
    if (running_mode_ == WAKEUP_MODE || Get_stand_status_() == Fallen_Forward || Get_stand_status_() == Fallen_Back)
    {
        running_mode_ = WAKEUP_MODE;
    }

    else if (running_mode_ != WAKEUP_MODE)
    {
        if (Get_goal_line_det_flg() && Get_line_det_flg()) // goal_line_detect_flg is true: goal_line detection mode
        {
            running_mode_ = GOAL_MODE;
        }

        else if (Get_no_line_det_flg() && !Get_wall_det_flg())
        {
            running_mode_ = NO_LINE_MODE;
        }

        else if (Get_line_det_flg() /* && !Get_corner_det_flg_2d() */)
        {
            running_mode_ = LINE_MODE;
        }

        else if (Get_huddle_det_flg_2d() /* && !Get_huddle_det_stop_flg() */)
        {
            running_mode_ = HUDDLE_MODE;
        }

        else if (Get_wall_det_flg())
        {
            running_mode_ = WALL_MODE;
        }

        else if (Get_stop_det_flg())
        {
            running_mode_ = STOP_MODE;
        }

        else if (Get_corner_det_flg_2d() /* && !Get_corner_det_stop_flg() */)
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
        // Test_service();
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
        // HUDDLE_mode();
        HUDDLE_mode2();
        break;
    case WALL_MODE:
        WALL_mode();
        // WALL_MODE2();
        break;
    case CORNER_MODE:
        CORNER_mode();
        break;
    }
}

void Move_Decision::LINE_mode()
{
    line_gradient = img_procPtr->Get_gradient();
    StraightLineDecision(line_gradient, MARGIN_GRADIENT);
    line_actual_angle = Get_turn_angle_();
    // line_motion = Get_motion_index_();
    line_motion = Motion_Index::InitPose;
    line_ud_neckangle = Get_UD_NeckAngle();

    // Initializing
    corner_seq_finish = false;
    Set_corner_det_stop_flg(false);

    huddle_seq_finish = false;
    Set_huddle_det_stop_flg(false);

    // If SM_req_finish = false -> InitPose
    // Straight Line
    if (straightLine == true)
    {
        if (!Get_turn_angle_on_flg())
        {
            // Left turn
            // To be zero
            if (line_actual_angle > 0)
            {
                // line_actual_angle -= 1;
                // if (line_actual_angle < 0)
                line_actual_angle = 0;
                Set_turn_angle_(line_actual_angle);
                Set_turn_angle_on_flg(true);
            }

            // Right turn
            // To be zero
            else if (line_actual_angle < 0)
            {
                // line_actual_angle += 1;
                // if (line_actual_angle > 0)
                line_actual_angle = 0;
                Set_turn_angle_(line_actual_angle);
                Set_turn_angle_on_flg(true);
            }
        }

        if (!Get_select_motion_on_flg())
        {
            line_motion = Motion_Index::Forward_2step;
            Set_motion_index_(line_motion);
            Set_select_motion_on_flg(true);
        }

        if (!Get_UD_Neck_on_flg())
        {
            line_ud_neckangle = UD_CENTER;
            Set_UD_NeckAngle(line_ud_neckangle);
            Set_UD_Neck_on_flg(true);
        }

        ROS_ERROR("STRAIGHT LINE");

        // TEST
        //  Set_RL_Neck_on_flg(true);
        //  Set_RL_NeckAngle(Actual_angle);
    }

    // Non Straight Line
    else if (straightLine == false)
    {
        if (!Get_turn_angle_on_flg())
        {
            // Increase Actual_angle more quickly for larger line_gradient values
            // Counter Clock wise(+) (Turn Angle sign)
            // Gradient : Angle from center of window.x to center of line.x
            // LEFT TURN
            if (line_gradient >= MARGIN_GRADIENT * 5)
            {
                increment = 4;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient >= MARGIN_GRADIENT * 4)
            {
                increment = 3;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient >= MARGIN_GRADIENT * 3)
            {
                increment = 2;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient >= MARGIN_GRADIENT * 2)
            {
                increment = 2;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient > MARGIN_GRADIENT * 1)
            {
                increment = 2;
                ROS_WARN("LEFT_TURN");
            }

            // Decrease Actual_angle relatively slowly for smaller line_gradient values
            // Right Turn
            else if (line_gradient <= -MARGIN_GRADIENT * 5)
            {
                increment = -4;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient <= -MARGIN_GRADIENT * 4)
            {
                increment = -3;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient <= -MARGIN_GRADIENT * 3)
            {
                increment = -2;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient <= -MARGIN_GRADIENT * 2)
            {
                increment = -2;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient < -MARGIN_GRADIENT * 1)
            {
                increment = -2;
                ROS_WARN("RIGHT TURN");
            }
            else
            {
                increment = 0;
            }

            line_actual_angle += increment;
            if (line_actual_angle >= 15)
            {
                line_actual_angle = LINE_TURN;
            }
            else if (line_actual_angle <= -15)
            {
                line_actual_angle = -LINE_TURN;
            }

            Set_turn_angle_(line_actual_angle);
            Set_turn_angle_on_flg(true);
        }

        if (!Get_select_motion_on_flg())
        {
            if (line_gradient > MARGIN_GRADIENT * 2 || line_gradient < -MARGIN_GRADIENT * 2)
            {
                line_motion = Motion_Index::Step_in_place;
            }
            else
            {
                line_motion = Motion_Index::Forward_2step;
            }

            Set_motion_index_(line_motion);
            Set_select_motion_on_flg(true);
        }

        if (!Get_UD_Neck_on_flg())
        {
            line_ud_neckangle = UD_CENTER;
            Set_UD_NeckAngle(line_ud_neckangle);
            Set_UD_Neck_on_flg(true);
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

    // Initializing
    corner_seq_finish = false;
    Set_corner_det_stop_flg(false);

    huddle_seq_finish = false;
    Set_huddle_det_stop_flg(false);

    if (tmp_delta_x < 0) // Right
    {
        if (!Get_turn_angle_on_flg())
        {
            noline_actual_angle -= 3;
            if (noline_actual_angle < -Angle_ToFindLine)
            {
                noline_actual_angle = -Angle_ToFindLine;
            }
            Set_turn_angle_(noline_actual_angle);
            Set_turn_angle_on_flg(true);
        }

        if (!Get_select_motion_on_flg())
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
        }

        if (!Get_UD_Neck_on_flg())
        {
            noline_neckangle = UD_CENTER;
            Set_UD_NeckAngle(noline_neckangle);
            Set_UD_Neck_on_flg(true);
        }

        ROS_WARN("RIGHT TURN");
        // ROS_INFO("turn angle : %d", Get_turn_angle_());
    }

    else if (tmp_delta_x > 0) // LEFT
    {
        if (!Get_turn_angle_on_flg())
        {
            noline_actual_angle += 3;
            if (noline_actual_angle > Angle_ToFindLine)
            {
                noline_actual_angle = Angle_ToFindLine;
            }
            Set_turn_angle_(noline_actual_angle);
            Set_turn_angle_on_flg(true);
        }

        if (!Get_select_motion_on_flg())
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
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
            if (!Get_select_motion_on_flg())
            {
                wakeup_motion = Motion_Index::FWD_UP;
                Set_motion_index_(wakeup_motion);
                Set_select_motion_on_flg(true);
                WakeUp_seq++;
            }
        }

        else if (tmp_stand_status == Stand_Status::Fallen_Forward)
        {
            if (!Get_select_motion_on_flg())
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
        if (!Get_select_motion_on_flg())
        {
            wakeup_motion = InitPose;
            Set_motion_index_(wakeup_motion);
            Set_select_motion_on_flg(true);
            WakeUp_seq++;
        }
    }

    else if (WakeUp_seq == 3)
    {
        if (!Get_select_motion_on_flg())
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
        Set_running_mode_(wakeup_running);
        WakeUp_seq = 0;
    }
    ROS_ERROR("WAKE_UP SEQ : %d", WakeUp_seq);
    // Motion_Info();
}

void Move_Decision::GOAL_LINE_mode()
{
    // longer width 활용하고 싶음
    if (!Get_select_motion_on_flg())
    {
        Set_motion_index_(Motion_Index::Forward_2step);
        Set_select_motion_on_flg(true);
    }

    if (!Get_UD_Neck_on_flg() && Get_UD_Neck_on_flg())
    {
        Set_UD_NeckAngle(UD_CENTER);
        Set_UD_Neck_on_flg(true);
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
    // 6 : MOtion : Forward_halfstep (Aprroach Huddle)
    // 7 : Motion : Huddle Jump
    // 8 : Initializing

    huddle_actual_angle = img_procPtr->Get_huddle_angle();
    line_gradient = img_procPtr->Get_gradient();
    StraightLineDecision(line_gradient, MARGIN_GRADIENT);
    huddle_motion = Motion_Index::InitPose;
    huddle_ud_neck_angle = Get_UD_NeckAngle();

    // 0 : Motion : InitPose (For getting distance) (Depth)
    if (tmp_huddle_seq == 0)
    {
        ROS_ERROR(Str_HUDDLE_SEQUENCE_0.c_str());
        if (!Get_select_motion_on_flg())
        {
            int _size = 0;

            huddle_motion = Motion_Index::InitPose;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);

            huddle_distance = img_procPtr->Get_huddle_distance();
            ROS_ERROR("huddle_distance : %d", huddle_distance);

            huddle_distance_save.push_back(huddle_distance);
            ROS_WARN("huddle_distance_save SIZE : %d", huddle_distance_save.size());
            if (huddle_distance_save.size() == SPIN_RATE * 3) // 3sec Mean Distance Value
            {
                huddle_distance = accumulate(huddle_distance_save.begin(), huddle_distance_save.end(), 0.0) / huddle_distance_save.size();
                huddle_distance = std::floor(huddle_distance * 1000.0) / 1000.0;
                huddle_distance_save.clear();
            }
            Set_huddle_det_flg_3d(false);
        }

        // else if (!Get_SM_req_finish())
        // {
        //     huddle_ud_neck_angle = 30;
        //     Set_UD_NeckAngle(huddle_ud_neck_angle);
        //     Set_UD_Neck_on_flg(true);
        // }

        // if (!Get_UD_Neck_on_flg() )
        // {
        //     huddle_ud_neck_angle = 30;
        //     Set_UD_NeckAngle(huddle_ud_neck_angle);
        //     Set_UD_Neck_on_flg(true);
        // }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            req_finish_count++;
            finish_past = Get_SM_req_finish();
        }

        if (req_finish_count == 1)
        {
            req_finish_count = 0;
            tmp_huddle_seq++;
        }

        // ROS_ERROR("Get_select_motion_on_flg : ############### %d ###############", Get_select_motion_on_flg());
    }

    // 1 : Motion : Forward_Nstep (Far) --> Line_mode()
    else if (tmp_huddle_seq == 1)
    {
        ROS_ERROR(Str_HUDDLE_SEQUENCE_1.c_str());
        if (!Get_select_motion_on_flg())
        {
            huddle_motion = Motion_Index::Forward_Nstep;
            huddle_distance *= 0.8;
            ROS_ERROR("HUDDLE_DISTANCE : %f", huddle_distance);
            Set_distance_(huddle_distance);
            Set_motion_index_(huddle_motion);

            Set_select_motion_on_flg(true);
            Set_distance_on_flg(true);
        }

        // For LINE_TRACKING
        if (straightLine == true)
        {
            if (!Get_turn_angle_on_flg())
            {
                // Left turn
                // To be zero
                if (line_actual_angle > 0)
                {
                    // line_actual_angle -= 1;
                    // if (line_actual_angle < 0)
                    line_actual_angle = 0;
                    Set_turn_angle_(line_actual_angle);
                    Set_turn_angle_on_flg(true);
                }

                // Right turn
                // To be zero
                else if (line_actual_angle < 0)
                {
                    // line_actual_angle += 1;
                    // if (line_actual_angle > 0)
                    line_actual_angle = 0;
                    Set_turn_angle_(line_actual_angle);
                    Set_turn_angle_on_flg(true);
                }
            }
        }

        // Non Straight Line
        else if (straightLine == false)
        {
            if (!Get_turn_angle_on_flg())
            {
                // Increase Actual_angle more quickly for larger line_gradient values
                // Counter Clock wise(+) (Turn Angle sign)
                // Gradient : Angle from center of window.x to center of line.x
                // LEFT TURN
                if (line_gradient >= MARGIN_GRADIENT * 5)
                {
                    increment = 4;
                    ROS_WARN("LEFT_TURN");
                }
                else if (line_gradient >= MARGIN_GRADIENT * 4)
                {
                    increment = 3;
                    ROS_WARN("LEFT_TURN");
                }
                else if (line_gradient >= MARGIN_GRADIENT * 3)
                {
                    increment = 2;
                    ROS_WARN("LEFT_TURN");
                }
                else if (line_gradient >= MARGIN_GRADIENT * 2)
                {
                    increment = 2;
                    ROS_WARN("LEFT_TURN");
                }
                else if (line_gradient > MARGIN_GRADIENT * 1)
                {
                    increment = 2;
                    ROS_WARN("LEFT_TURN");
                }

                // Decrease Actual_angle relatively slowly for smaller line_gradient values
                // Right Turn
                else if (line_gradient <= -MARGIN_GRADIENT * 5)
                {
                    increment = -4;
                    ROS_WARN("RIGHT TURN");
                }
                else if (line_gradient <= -MARGIN_GRADIENT * 4)
                {
                    increment = -3;
                    ROS_WARN("RIGHT TURN");
                }
                else if (line_gradient <= -MARGIN_GRADIENT * 3)
                {
                    increment = -2;
                    ROS_WARN("RIGHT TURN");
                }
                else if (line_gradient <= -MARGIN_GRADIENT * 2)
                {
                    increment = -2;
                    ROS_WARN("RIGHT TURN");
                }
                else if (line_gradient < -MARGIN_GRADIENT * 1)
                {
                    increment = -2;
                    ROS_WARN("RIGHT TURN");
                }
                else
                {
                    increment = 0;
                }

                line_actual_angle += increment;
                if (line_actual_angle > 15)
                {
                    line_actual_angle = TURN_MAX;
                }
                else if (line_actual_angle < -15)
                {
                    line_actual_angle = TURN_MIN;
                }

                Set_turn_angle_(line_actual_angle);
                Set_turn_angle_on_flg(true);
            }

            // Set_line_det_flg(true);
            Set_huddle_det_flg_3d(false);
        }

        if (!Get_UD_Neck_on_flg())
        {
            huddle_ud_neck_angle = UD_CENTER;
            Set_UD_NeckAngle(huddle_ud_neck_angle);
            Set_UD_Neck_on_flg(true);
        }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            req_finish_count++;
            finish_past = Get_SM_req_finish();
        }
        if (req_finish_count == 1)
        {
            req_finish_count = 0;
            tmp_huddle_seq++;
        }
    }

    // 2 : Motion : InitPose (For getting distance) (Depth)
    else if (tmp_huddle_seq == 2)
    {
        ROS_ERROR(Str_HUDDLE_SEQUENCE_2.c_str());
        if (!Get_select_motion_on_flg())
        {
            int _size = 0;
            huddle_motion = Motion_Index::InitPose;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);
            // ROS_ERROR("huddle_distace : %f", huddle_distance);

            huddle_distance = img_procPtr->Get_huddle_distance();
            huddle_distance_save.push_back(huddle_distance);
            // ROS_ERROR("huddle_distance_save SIZE : %d", huddle_distance_save.size());
            if (huddle_distance_save.size() == SPIN_RATE * 3) // 3sec Mean Distance Value
            {
                huddle_distance = accumulate(huddle_distance_save.begin(), huddle_distance_save.end(), 0.0) / huddle_distance_save.size();
                huddle_distance = std::floor(huddle_distance * 1000.0) / 1000.0;
                huddle_distance_save.clear();
            }
            // Set_huddle_det_flg(false);
        }

        // if (!Get_UD_Neck_on_flg() )
        // {
        //     huddle_ud_neck_angle = 40;
        //     Set_UD_NeckAngle(huddle_ud_neck_angle);
        //     Set_UD_Neck_on_flg(true);
        // }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            req_finish_count++;
            finish_past = Get_SM_req_finish();
        }
        if (req_finish_count == 1)
        {
            req_finish_count = 0;
            tmp_huddle_seq++;
        }
    }

    // 3 : Motion : Forward_Nstep (Approach) --> Line_mode()
    else if (tmp_huddle_seq == 3)
    {
        ROS_ERROR(Str_HUDDLE_SEQUENCE_3.c_str());
        if (!Get_distance_on_flg())
        {
            if (huddle_distance < 0)
            {
                tmp_huddle_seq = 4;
            }

            huddle_distance *= 0.80;
            Set_distance_(huddle_distance);
            Set_distance_on_flg(true);
        }

        if (!Get_select_motion_on_flg())
        {
            huddle_motion = Motion_Index::Forward_Nstep;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);

            // if (!Get_UD_Neck_on_flg() )
            // {
            //     huddle_ud_neck_angle = 40;
            //     Set_UD_NeckAngle(huddle_ud_neck_angle);
            //     Set_UD_Neck_on_flg(true);
            // }

            // Set_line_det_flg(true);
            Set_huddle_det_flg_3d(false);
        }

        // For LINE_TRACKING
        if (straightLine == true)
        {
            if (!Get_turn_angle_on_flg())
            {
                // Left turn
                // To be zero
                if (line_actual_angle > 0)
                {
                    // line_actual_angle -= 1;
                    // if (line_actual_angle < 0)
                    line_actual_angle = 0;
                    Set_turn_angle_(line_actual_angle);
                    Set_turn_angle_on_flg(true);
                }

                // Right turn
                // To be zero
                else if (line_actual_angle < 0)
                {
                    // line_actual_angle += 1;
                    // if (line_actual_angle > 0)
                    line_actual_angle = 0;
                    Set_turn_angle_(line_actual_angle);
                    Set_turn_angle_on_flg(true);
                }
            }
        }

        // Non Straight Line
        else if (straightLine == false)
        {
            if (!Get_turn_angle_on_flg())
            {
                // Increase Actual_angle more quickly for larger line_gradient values
                // Counter Clock wise(+) (Turn Angle sign)
                // Gradient : Angle from center of window.x to center of line.x
                // LEFT TURN
                if (line_gradient >= MARGIN_GRADIENT * 5)
                {
                    increment = 4;
                    ROS_WARN("LEFT_TURN");
                }
                else if (line_gradient >= MARGIN_GRADIENT * 4)
                {
                    increment = 3;
                    ROS_WARN("LEFT_TURN");
                }
                else if (line_gradient >= MARGIN_GRADIENT * 3)
                {
                    increment = 2;
                    ROS_WARN("LEFT_TURN");
                }
                else if (line_gradient >= MARGIN_GRADIENT * 2)
                {
                    increment = 2;
                    ROS_WARN("LEFT_TURN");
                }
                else if (line_gradient > MARGIN_GRADIENT * 1)
                {
                    increment = 2;
                    ROS_WARN("LEFT_TURN");
                }

                // Decrease Actual_angle relatively slowly for smaller line_gradient values
                // Right Turn
                else if (line_gradient <= -MARGIN_GRADIENT * 5)
                {
                    increment = -4;
                    ROS_WARN("RIGHT TURN");
                }
                else if (line_gradient <= -MARGIN_GRADIENT * 4)
                {
                    increment = -3;
                    ROS_WARN("RIGHT TURN");
                }
                else if (line_gradient <= -MARGIN_GRADIENT * 3)
                {
                    increment = -2;
                    ROS_WARN("RIGHT TURN");
                }
                else if (line_gradient <= -MARGIN_GRADIENT * 2)
                {
                    increment = -2;
                    ROS_WARN("RIGHT TURN");
                }
                else if (line_gradient < -MARGIN_GRADIENT * 1)
                {
                    increment = -2;
                    ROS_WARN("RIGHT TURN");
                }
                else
                {
                    increment = 0;
                }

                line_actual_angle += increment;
                if (line_actual_angle > 15)
                {
                    line_actual_angle = TURN_MAX;
                }
                else if (line_actual_angle < -15)
                {
                    line_actual_angle = TURN_MIN;
                }

                Set_turn_angle_(line_actual_angle);
                Set_turn_angle_on_flg(true);
            }

            // Set_line_det_flg(true);
            Set_huddle_det_flg_3d(false);
        }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            req_finish_count++;
            finish_past = Get_SM_req_finish();
        }
        if (req_finish_count == 1)
        {
            req_finish_count = 0;
            tmp_huddle_seq++;
        }
    }

    // 4 : Motion : Step in place (Pose Control)
    else if (tmp_huddle_seq == 4)
    {
        ROS_ERROR(Str_HUDDLE_SEQUENCE_4.c_str());
        if (!Get_select_motion_on_flg())
        {
            huddle_motion = Motion_Index::Step_in_place;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);
            Set_huddle_det_flg_3d(false);
        }

        if (!Get_turn_angle_on_flg())
        {
            Set_turn_angle_(huddle_actual_angle);
            Set_turn_angle_on_flg(true);
            Set_huddle_det_flg_3d(false);
        }

        // if (!Get_UD_Neck_on_flg() )
        // {
        //     huddle_ud_neck_angle = UD_CENTER;
        //     Set_UD_NeckAngle(huddle_ud_neck_angle);
        //     Set_UD_Neck_on_flg(true);
        // }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            req_finish_count++;
            finish_past = Get_SM_req_finish();
        }
        if (req_finish_count == 1)
        {
            req_finish_count = 0;
            tmp_huddle_seq++;
        }
    }

    // 5 : Motion : InitPose
    else if (tmp_huddle_seq == 5)
    {
        ROS_ERROR(Str_HUDDLE_SEQUENCE_5.c_str());
        // if (!Get_UD_Neck_on_flg() )
        // {
        //     huddle_ud_neck_angle = UD_CENTER;
        //     Set_UD_NeckAngle(huddle_ud_neck_angle);
        //     Set_UD_Neck_on_flg(true);
        // }

        if (!Get_select_motion_on_flg())
        {
            huddle_motion = Motion_Index::InitPose;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);

            ROS_ERROR("huddle_actual_angle : %f", huddle_actual_angle);

            if (huddle_actual_angle > 10 || huddle_actual_angle < -10)
            {
                tmp_huddle_seq = 4;
            }

            else
            {
                // Sequence++
                tmp_huddle_seq++;
            }

            Set_huddle_det_flg_3d(false);
        }
    }

    // 6 : MOtion : Forward_halfstep (Aprroach Huddle)
    else if (tmp_huddle_seq == 6)
    {
        ROS_ERROR(Str_HUDDLE_SEQUENCE_6.c_str());
        if (!Get_UD_Neck_on_flg())
        {
            huddle_ud_neck_angle = UD_CENTER;
            Set_UD_NeckAngle(huddle_ud_neck_angle);
            Set_UD_Neck_on_flg(true);
        }

        contain_huddle_to_foot = img_procPtr->Get_contain_huddle_to_foot();
        if (!Get_select_motion_on_flg())
        {
            while (!contain_huddle_to_foot)
            {
                huddle_motion = Motion_Index::Forward_Halfstep;
                Set_motion_index_(huddle_motion);
                Set_select_motion_on_flg(true);
                Set_huddle_det_flg_3d(false);
                if (contain_huddle_to_foot)
                    break;
            }

            if (contain_huddle_to_foot)
            {
                huddle_motion = Motion_Index::InitPose;
                Set_motion_index_(huddle_motion);
                Set_select_motion_on_flg(true);
                Set_huddle_det_flg_3d(false);
                tmp_huddle_seq++;
            }
        }
    }

    // 7 : Motion : Huddle Jump
    else if (tmp_huddle_seq == 7)
    {
        ROS_ERROR(Str_HUDDLE_SEQUENCE_7.c_str());
        if (!Get_select_motion_on_flg())
        {
            huddle_motion = Motion_Index::Huddle_Jump;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);
            Set_huddle_det_flg_3d(false);
        }

        if (!Get_UD_Neck_on_flg())
        {
            huddle_ud_neck_angle = UD_CENTER;
            Set_UD_NeckAngle(huddle_ud_neck_angle);
            Set_UD_Neck_on_flg(true);
        }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            req_finish_count++;
            finish_past = Get_SM_req_finish();
        }
        if (req_finish_count == 1)
        {
            req_finish_count = 0;
            tmp_huddle_seq++;
        }
    }

    // 8 : Initializeing
    else if (tmp_huddle_seq == 8)
    {
        ROS_ERROR(Str_HUDDLE_SEQUENCE_8.c_str());
        if (!Get_select_motion_on_flg())
        {
            // Set_select_motion_on_flg(true);
            tmp_huddle_seq = 0;
            to_be_line_mode++;
            Set_huddle_det_flg_3d(false);
        }

        if (!Get_UD_Neck_on_flg())
        {
            huddle_ud_neck_angle = UD_CENTER;
            Set_UD_NeckAngle(huddle_ud_neck_angle);
            Set_UD_Neck_on_flg(true);
        }
    }

    if (to_be_line_mode == 3)
    {
        Set_huddle_det_stop_flg(true);
    }

    if (Get_huddle_det_stop_flg() == true)
    {
        Set_huddle_det_flg_3d(false);
        Set_running_mode_(Running_Mode::LINE_MODE);
        tmp_huddle_seq = 0;
    }

    ROS_ERROR("HUDDLE_SEQ : %d", tmp_huddle_seq);
}

void Move_Decision::HUDDLE_mode2()
{
    // 0 : Pose Control (Posture(Gradient))
    // --> Motion : Motion_Index::Step_In_Place && Turn Angle(Gradient)

    // 1 : Approach to the Huddle
    // --> Motion : Motion_Index::Forward_Halfstep (Until Huddle center) : About Y diff

    // 2 : Pose Control (Posture(Gradient))
    // --> Motion : Motion_Index::Step_In_Place && Turn Angle(Gradient)

    // 3 : Motion : HUDDLE_JUMP

    // 4 : Initializing

    huddle_actual_angle = Get_turn_angle_();
    huddle_ud_neck_angle = Get_UD_NeckAngle();
    huddle_motion = Get_motion_index_();
    img_proc_huddle_angle = img_procPtr->Get_huddle_angle();

    ROS_ERROR("img_proc_huddle_angle : %lf", img_proc_huddle_angle);
    ROS_WARN("Get_foot_huddle_distance : %d", img_procPtr->Get_foot_huddle_distance());

    // 0 : InitPose (Dummy)
    if (tmp_huddle_seq == 0)
    {
        if (!Get_select_motion_on_flg())
        {
            huddle_motion = Motion_Index::InitPose;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);
        }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            tmp_huddle_seq++;
        }
    }

    // 1 : Pose Control (Posture(Gradient))
    else if (tmp_huddle_seq == 1)
    {
        // Initializing
        huddle_seq_finish = false;
        Set_huddle_det_stop_flg(false);
        huddle_posture = false;

        // Keeping Huddle mode
        // Set_huddle_det_flg_2d(true);
        // Set_line_det_flg(false);
        // tmp_img_proc_huddle_det_flg_2d_ = true;
        // tmp_img_proc_line_det_flg_ = false;

        ROS_ERROR(Str_HUDDLE2_SEQUENCE_0.c_str());

        if (!Get_select_motion_on_flg())
        {
            huddle_motion = Motion_Index::Step_in_place;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);
            ROS_WARN("img_proc_huddle_angle : %lf", img_proc_huddle_angle);
        }

        if (!Get_turn_angle_on_flg())
        {
            if (img_proc_huddle_angle > 5 || img_proc_huddle_angle < -5)
            {
                if (!huddle_posture)
                {
                    if (img_proc_huddle_angle >= 15)
                    {
                        huddle_actual_angle = HUDDLE_TURN;
                    }
                    else if (img_proc_huddle_angle <= -15)
                    {
                        huddle_actual_angle = -HUDDLE_TURN;
                    }
                    Set_turn_angle_(huddle_actual_angle);
                    Set_turn_angle_on_flg(true);
                }
            }
            else
            {
                huddle_posture = true;
            }
        }

        if (huddle_posture == true)
        {
            tmp_huddle_seq++;
            huddle_posture = false;
        }
    }

    // 2 : Approach to the Huddle + Pose Control (Position)
    else if (tmp_huddle_seq == 2)
    {
        // Keeping Huddle mode
        // Set_huddle_det_flg_2d(true);
        // Set_line_det_flg(false);
        // tmp_img_proc_huddle_det_flg_2d_ = true;
        // tmp_img_proc_line_det_flg_ = false;

        // img_proc_huddle_delta_x = img_procPtr->Get_delta_x();
        img_proc_contain_huddle_to_foot = img_procPtr->Get_contain_huddle_to_foot();

        ROS_ERROR(Str_HUDDLE2_SEQUENCE_1.c_str());
        // ROS_WARN("X diff : %d", img_proc_huddle_delta_x);
        ROS_WARN("Y diff : %d", img_proc_contain_huddle_to_foot);

        if (!Get_select_motion_on_flg())
        {
            // About huddle Y point
            if (!img_proc_contain_huddle_to_foot)
            {
                huddle_motion = Motion_Index::Forward_Halfstep;
                Set_motion_index_(huddle_motion);
                Set_select_motion_on_flg(true);
            }

            else if (img_proc_contain_huddle_to_foot)
            {
                contain_huddle_Y = true;
                ROS_WARN("Y POSITION IS OK!!!!!!!!!!!!!!!!!!!");
            }

            if (contain_huddle_Y)
            {
                huddle_motion = Motion_Index::InitPose;
                Set_motion_index_(huddle_motion);
                Set_select_motion_on_flg(true);
                contain_huddle_Y = false;

                tmp_huddle_seq++;
            }
        }
    }

    // 3 : Pose Control (Posture(Gradient))
    else if (tmp_huddle_seq == 3)
    {
        // Keeping Huddle mode
        // Set_huddle_det_flg_2d(true);
        // Set_line_det_flg(false);
        // tmp_img_proc_huddle_det_flg_2d_ = true;
        // tmp_img_proc_line_det_flg_ = false;

        huddle_posture = false;
        img_proc_huddle_angle = img_procPtr->Get_huddle_angle();
        ROS_ERROR("img_proc_huddle_angle : %lf", img_proc_huddle_angle);
        ROS_ERROR(Str_HUDDLE2_SEQUENCE_2.c_str());

        if (!Get_select_motion_on_flg())
        {
            huddle_motion = Motion_Index::Step_in_place;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);
        }

        if (!Get_turn_angle_on_flg())
        {
            if ((img_proc_huddle_angle > HUDDLE_TURN || img_proc_huddle_angle < -HUDDLE_TURN))
            {
                if (!huddle_posture)
                {
                    if (img_proc_huddle_angle >= 15)
                    {
                        huddle_actual_angle = HUDDLE_TURN;
                    }
                    else if (img_proc_huddle_angle <= -15)
                    {
                        huddle_actual_angle = -HUDDLE_TURN;
                    }
                    Set_turn_angle_(huddle_actual_angle);
                    Set_turn_angle_on_flg(true);
                }
                // huddle_posture = false;
            }
            else
            {
                huddle_posture = true;
            }
        }

        if (huddle_posture == true)
        {
            tmp_huddle_seq++;
            huddle_posture = false;
        }
    }

    // 4 : Motion : HUDDLE_JUMP
    else if (tmp_huddle_seq == 4)
    {
        // Keeping Huddle mode
        // Set_huddle_det_flg_2d(true);
        // Set_line_det_flg(false);
        // tmp_img_proc_huddle_det_flg_2d_ = true;
        // tmp_img_proc_line_det_flg_ = false;

        ROS_ERROR(Str_HUDDLE2_SEQUENCE_3.c_str());
        if (!Get_select_motion_on_flg())
        {
            huddle_motion = Motion_Index::Huddle_Jump;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);
        }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            tmp_huddle_seq++;
            to_be_line_mode++;
        }
        // ROS_ERROR("to_be_line_mode : %d", to_be_line_mode);
    }

    // 5 : Initializing
    else if (tmp_huddle_seq == 5)
    {
        ROS_ERROR(Str_HUDDLE2_SEQUENCE_4.c_str());
        // if (to_be_line_mode == 1)
        // {
        //     Set_huddle_det_stop_flg(true);
        // }

        // if (Get_huddle_det_stop_flg() == true)
        // {
        //     Set_huddle_det_flg_2d(false);
        //     Set_line_det_flg(true);
        //     huddle_seq_finish = true;
        // }

        if (!Get_select_motion_on_flg())
        {
            huddle_motion = Motion_Index::InitPose;
            Set_motion_index_(huddle_motion);
            Set_select_motion_on_flg(true);

            tmp_huddle_seq = 0;
        }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            tmp_huddle_seq = 0;

            contain_huddle_Y = false;
            huddle_posture = false;
            huddle_seq_finish = true;

            Set_huddle_det_flg_2d(false);
            Set_line_det_flg(true);

        }
        // Running_Info();
    }
}

void Move_Decision::CORNER_mode()
{
    // 0 : Approach to the Corner
    // --> Motion : Motion_Index::Forward_Halfstep (Until corner center) : About Y diff
    // --> Motion : Motion_Index::Left_Halfstep or Motion_Index::Right_Halfstep : About X diff

    // 1 : Pose Control (Posture(Gradient))
    // --> Motion : Motion_Index::Step_In_Place && Turn Angle(Gradient)

    // 2 : Motion : Step in place + Turn Angle 90(ㅓ) or -90(ㅜ)

    // 3 : Initializing

    corner_actual_angle = Get_turn_angle_();
    corner_ud_neck_angle = Get_UD_NeckAngle();
    corner_motion = Motion_Index::InitPose;

    // 0 : InitPose (Dummy)
    if (tmp_corner_seq == 0)
    {
        if (!Get_select_motion_on_flg())
        {
            corner_motion = Motion_Index::InitPose;
            Set_motion_index_(corner_motion);
            Set_select_motion_on_flg(true);
        }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            tmp_corner_seq++;
        }
    }

    // 0 : Approach to the Corner + Pose Control (Position)
    else if (tmp_corner_seq == 1)
    {
        // Initializing
        corner_seq_finish = false;
        Set_corner_det_stop_flg(false);

        huddle_seq_finish = false;
        Set_huddle_det_stop_flg(false);

        img_proc_corner_delta_x = img_procPtr->Get_delta_x();
        img_proc_contain_corner_to_foot = img_procPtr->Get_contain_corner_to_foot();

        ROS_ERROR(Str_CORNER_SEQUENCE_0.c_str());
        ROS_WARN("X diff : %d", img_proc_corner_delta_x);
        ROS_WARN("Y diff : %d", img_proc_contain_corner_to_foot);

        if (!Get_select_motion_on_flg())
        {
            // Corner center X point
            if (img_proc_corner_delta_x < 0)
            {
                corner_motion = Motion_Index::Right_Halfstep;
                Set_motion_index_(corner_motion);
                Set_select_motion_on_flg(true);
                contain_corner_X = false;
            }

            else if (img_proc_corner_delta_x > 0)
            {
                corner_motion = Motion_Index::Left_Halfstep;
                Set_motion_index_(corner_motion);
                Set_select_motion_on_flg(true);
                contain_corner_X = false;
            }

            // About Corner X point
            else if (img_proc_corner_delta_x == 0)
            {
                ROS_WARN("X diff : %d", img_proc_corner_delta_x);
                corner_motion = Motion_Index::InitPose;
                Set_motion_index_(corner_motion);
                Set_select_motion_on_flg(true);
                contain_corner_X = true;
                ROS_WARN("X POSITION IS OK!!!!!!!!!!!!!!!!!!!");
            }

            // About Corner Y point
            if (contain_corner_X && !img_proc_contain_corner_to_foot)
            {
                corner_motion = Motion_Index::Forward_Halfstep;
                Set_motion_index_(corner_motion);
                Set_select_motion_on_flg(true);
                Set_corner_det_flg_2d(false);
            }

            else if (img_proc_contain_corner_to_foot)
            {
                contain_corner_Y = true;
                ROS_WARN("Y POSITION IS OK!!!!!!!!!!!!!!!!!!!");
            }

            if (contain_corner_Y && contain_corner_X)
            {
                corner_motion = Motion_Index::InitPose;
                Set_motion_index_(corner_motion);
                Set_select_motion_on_flg(true);
                Set_corner_det_flg_2d(false);

                // Sequence++
                if (finish_past != Get_SM_req_finish())
                {
                    contain_corner_X = false;
                    contain_corner_Y = false;
                    req_finish_count++;
                    tmp_corner_seq++;
                }
            }
        }
    }

    // // 1 : Approach to the Corner + Pose Control (Position)
    // else if (tmp_corner_seq == 2)
    // {
    //     img_proc_corner_angle = img_procPtr->Get_corner_angle();
    //     ROS_ERROR("img_proc_corner_angle : %lf", img_proc_corner_angle);
    //     ROS_ERROR(Str_CORNER_SEQUENCE_1.c_str());

    //     if (!Get_select_motion_on_flg())
    //     {
    //         corner_motion = Motion_Index::Step_in_place;
    //         Set_motion_index_(corner_motion);
    //         Set_select_motion_on_flg(true);
    //     }

    //     if (!Get_turn_angle_on_flg())
    //     {
    //         if (img_proc_corner_angle > 10 || img_proc_corner_angle < -10)
    //         {
    //             if (img_proc_corner_angle >= 15)
    //             {
    //                 corner_actual_angle = CORNER_TURN;
    //             }
    //             else if (img_proc_corner_angle <= -15)
    //             {
    //                 corner_actual_angle = -CORNER_TURN;
    //             }
    //             Set_turn_angle_(corner_actual_angle);
    //             Set_turn_angle_on_flg(true);
    //         }
    //         else
    //         {
    //             corner_posture = true;
    //         }
    //     }

    //     if (corner_posture == true)
    //     {
    //         tmp_corner_seq++;
    //         corner_posture = false;
    //     }
    // }

    // 2 : Motion : Step in place + Turn Angle 90(ㅓ) or -90(ㅜ)
    else if (tmp_corner_seq == 2)
    {
        ROS_ERROR(Str_CORNER_SEQUENCE_2.c_str());

        if (tmp_turn90 == 0 && !Get_select_motion_on_flg())
        {
            corner_motion = Motion_Index::Step_in_place;
            Set_motion_index_(corner_motion);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                Set_turn_angle_(CORNER_90TURN);
                Set_turn_angle_on_flg(true);
            }

            if (finish_past != Get_SM_req_finish())
            {
                tmp_turn90++;
            }
        }

        else if (tmp_turn90 == 1 && !Get_select_motion_on_flg())
        {
            corner_motion = Motion_Index::Step_in_place;
            Set_motion_index_(corner_motion);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                Set_turn_angle_(CORNER_90TURN);
                Set_turn_angle_on_flg(true);
            }

            if (finish_past != Get_SM_req_finish())
            {
                tmp_turn90++;
            }
        }

        else if (tmp_turn90 == 2 && !Get_select_motion_on_flg())
        {
            corner_motion = Motion_Index::Step_in_place;
            Set_motion_index_(corner_motion);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                Set_turn_angle_(CORNER_90TURN);
                Set_turn_angle_on_flg(true);
            }

            if (finish_past != Get_SM_req_finish())
            {
                tmp_turn90++;
            }
        }

        else if (tmp_turn90 == 3 && !Get_select_motion_on_flg())
        {
            corner_motion = Motion_Index::Step_in_place;
            Set_motion_index_(corner_motion);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                Set_turn_angle_(CORNER_90TURN);
                Set_turn_angle_on_flg(true);
            }

            if (finish_past != Get_SM_req_finish())
            {
                tmp_turn90++;
            }
        }

        else if (tmp_turn90 == 4 && !Get_select_motion_on_flg())
        {
            corner_motion = Motion_Index::Step_in_place;
            Set_motion_index_(corner_motion);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                Set_turn_angle_(CORNER_90TURN);
                Set_turn_angle_on_flg(true);
            }

            if (finish_past != Get_SM_req_finish())
            {
                tmp_turn90++;
            }
        }

        else if (tmp_turn90 == 5 && !Get_select_motion_on_flg())
        {
            corner_motion = Motion_Index::Step_in_place;
            Set_motion_index_(corner_motion);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                Set_turn_angle_(CORNER_90TURN);
                Set_turn_angle_on_flg(true);
            }

            if (finish_past != Get_SM_req_finish())
            {
                tmp_turn90++;
            }
        }

        else if (tmp_turn90 == 6 && !Get_select_motion_on_flg())
        {
            corner_motion = Motion_Index::Step_in_place;
            Set_motion_index_(corner_motion);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                Set_turn_angle_(CORNER_90TURN);
                Set_turn_angle_on_flg(true);
            }

            if (finish_past != Get_SM_req_finish())
            {
                tmp_turn90++;
            }
        }

        else if (tmp_turn90 == 7)
        {
            tmp_turn90 = 0;
            tmp_corner_seq++;
        }
    }

    // 3 : Initializing
    else if (tmp_corner_seq == 3)
    {
        ROS_ERROR(Str_CORNER_SEQUENCE_3.c_str());
        // tmp_corner_shape = img_procPtr->Get_img_proc_corner_number();
        // tmp_corner_shape = 1;
        ROS_WARN("CORNER_SHAPE : %d", tmp_corner_shape);

        if (tmp_corner_shape == 1)
        {
            tmp_corner_seq = 0;
            // tmp_corner_shape = 0;
            Set_wall_det_flg(true);
            Set_corner_det_stop_flg(true);
            Set_corner_det_flg_2d(false);
            corner_seq_finish = true;
            wall_seq_start = true;
            // Running_Info();
        }

        else if (tmp_corner_shape == 2)
        {
            tmp_corner_seq = 0;
            // tmp_corner_shape = 0;
            Set_line_det_flg(true);
            Set_corner_det_stop_flg(true);
            Set_corner_det_flg_2d(false);
            corner_seq_finish = true;
            // Running_Info();
        }
    }
}

void Move_Decision::WALL_mode()
{

    // Case A (Distance >= 0.75)
    // Seq 0 : Pose Control 
    // Seq 1 : Position Control (Forward_2step) 
    // 1 : Start
    // -1 : IN wall
    // 10 : End

    // Case B (0.4 < Distance < 0.75)
    // Seq 0 : Pose Control
    // Seq 1 : Position Control (Forward_ 1step)
    // 2 : RIGHT Plane 
    // -2 : LEFT Plane 

    // Case C (Distance < 0.4)
    // 3 -> Left_2Step
    // -3 -> Right_2Step

    wall_motion = Get_motion_index_();
    img_proc_wall_angle = img_procPtr->Get_wall_angle();
    wall_status = Wall_Status(img_procPtr->Get_wall_distance());
    wall_distance = img_procPtr->Get_wall_distance();

    switch (wall_status)
    {
    case 1:
        // Case A
        // Start
        // Wall angle turn (Pose Control)
        if (!Get_select_motion_on_flg() && wall_number_seq_A == 0)
        {
            wall_motion = Motion_Index::Step_in_place;
            Set_motion_index_(wall_motion);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                if (img_proc_wall_angle > 15 || img_proc_wall_angle < -15)
                {
                    if (img_proc_wall_angle > 15)
                    {
                        wall_actual_angle = 5;
                    }
                    else if (img_proc_wall_angle < -15)
                    {
                        wall_actual_angle = -5;
                    }
                    Set_turn_angle_(wall_actual_angle);
                    Set_turn_angle_on_flg(true);
                }
                else
                {
                    wall_posture = true;
                }
            }

            if (wall_posture == true)
            {
                wall_number_seq_A++;
                wall_posture = false;
            }
        }

        // 처음 벽 인식 후 일정 거리 안까지 직진 (Position Control)
        else if (!Get_select_motion_on_flg() && wall_number_seq_A == 1)
        {
            wall_motion = Motion_Index::Forward_2step;
            Set_motion_index_(wall_motion);
            Set_select_motion_on_flg(true);

            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                wall_number_seq_A = 1;
            }
        }

        ROS_ERROR("FLAG 1 : START Straight");
        break;

    case 2:
        // Case B
        // 처음 벽 인식 후 일정 거리 안까지 직진 (Plane Decision)
        // RIGHT plane -> straight

        // Pose Control
        if (!Get_select_motion_on_flg() && wall_number_seq_B == 0)
        {
            wall_motion = Motion_Index::Step_in_place;
            Set_motion_index_(wall_motion);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                img_proc_wall_angle = img_procPtr->Get_wall_angle();
                if (img_proc_wall_angle > 15 || img_proc_wall_angle < -15)
                {
                    if (img_proc_wall_angle > 15)
                    {
                        wall_actual_angle = 5;
                    }
                    else if (img_proc_wall_angle < -15)
                    {
                        wall_actual_angle = -5;
                    }
                    Set_turn_angle_(wall_actual_angle);
                    Set_turn_angle_on_flg(true);
                }
                else
                {
                    wall_posture = true;
                }
            }

            if (wall_posture == true)
            {
                wall_number_seq_B++;
                wall_posture = false;
            }
        }

        // Position Control
        else if (!Get_select_motion_on_flg() && wall_number_seq_B == 1)
        {
            Set_motion_index_(Motion_Index::Forward_1step);
            Set_select_motion_on_flg(true);

            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                wall_number_seq_B = 1;
            }
        }

        ROS_ERROR("FLAG 2 : Straight");
        break;

    case 3:
        // Case C
        // 일정 거리 앞에서 정지 후 멀리 있는 벽이 보일 때 까지 좌보행
        
        if (!Get_select_motion_on_flg() && wall_number_seq_C == 0)
        {
            wall_motion = Motion_Index::Left_2step;
            Set_motion_index_(wall_motion);
            Set_select_motion_on_flg(true);

            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                wall_number_seq_C = 0;
            }
        }

        // else if (!Get_select_motion_on_flg() && wall_number_seq_C == 1)
        // {
        //     wall_motion = Motion_Index::InitPose;
        //     Set_motion_index_(wall_motion);
        //     Set_select_motion_on_flg(true);

        //     // Sequence++
        //     if (finish_past != Get_SM_req_finish())
        //     {
        //         wall_number_seq_C = 1;
        //     }
        // }


        ROS_ERROR("FLAG 3 : LEFT");
        break;

    case -2:
        // Case B
        // 멀리 있는 벽의 일정 거리 앞까지 직진  // LEFT plane -> straight
        
        // Pose Control
        if (!Get_select_motion_on_flg() && wall_number_seq_B == 0)
        {
            wall_motion = Motion_Index::Step_in_place;
            Set_motion_index_(wall_motion);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                img_proc_wall_angle = img_procPtr->Get_wall_angle();
                if (img_proc_wall_angle > 15 || img_proc_wall_angle < -15)
                {
                    if (img_proc_wall_angle > 15)
                    {
                        wall_actual_angle = 5;
                    }
                    else if (img_proc_wall_angle < -15)
                    {
                        wall_actual_angle = -5;
                    }
                    Set_turn_angle_(wall_actual_angle);
                    Set_turn_angle_on_flg(true);
                }
                else
                {
                    wall_posture = true;
                }
            }

            if (wall_posture == true)
            {
                wall_number_seq_B++;
                wall_posture = false;
            }
        }

        // Position Control
        else if (!Get_select_motion_on_flg() && wall_number_seq_B == 1)
        {
            Set_motion_index_(Motion_Index::Forward_1step);
            Set_select_motion_on_flg(true);

            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                wall_number_seq_B = 1;
            }
        }

        ROS_ERROR("FLAG -2 : Straight");
        break;

    case -3:
        // Case C
        // 일정 거리 앞에서 정지 후 멀리 있는 벽이 보일 때 까지 우보행

        if (!Get_select_motion_on_flg() && wall_number_seq_C == 0)
        {
            wall_motion = Motion_Index::Right_2step;
            Set_motion_index_(wall_motion);
            Set_select_motion_on_flg(true);

            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                wall_number_seq_C = 0;
            }
        }

        // else if (!Get_select_motion_on_flg() && wall_number_seq_C == 1)
        // {
        //     wall_motion = Motion_Index::InitPose;
        //     Set_motion_index_(wall_motion);
        //     Set_select_motion_on_flg(true);

        //     // Sequence++
        //     if (finish_past != Get_SM_req_finish())
        //     {
        //         wall_number_seq_C = 1;
        //     }
        // }

        ROS_ERROR("FLAG -3 : RIGHT");
        break;

    case 10:
        // Case A
        // 멀리 있는 벽의 일정 거리 앞까지 직진
        // Wall angle turn (Pose Control)
        
        if (!Get_select_motion_on_flg() && wall_number_seq_A == 0)
        {
            wall_motion = Motion_Index::Step_in_place;
            Set_motion_index_(wall_motion);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                if (img_proc_wall_angle > 15 || img_proc_wall_angle < -15)
                {
                    if (img_proc_wall_angle > 15)
                    {
                        wall_actual_angle = 5;
                    }
                    else if (img_proc_wall_angle < -15)
                    {
                        wall_actual_angle = -5;
                    }
                    Set_turn_angle_(wall_actual_angle);
                    Set_turn_angle_on_flg(true);
                }
                else
                {
                    wall_posture = true;
                }
            }

            if (wall_posture == true)
            {
                wall_number_seq_A++;
                wall_posture = false;
            }
        }

        else if (!Get_select_motion_on_flg() && wall_number_seq_A == 1)
        {
            wall_motion = Motion_Index::Forward_2step;
            Set_motion_index_(wall_motion);
            Set_select_motion_on_flg(true);

            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                wall_number_seq_A = 1;
                tmp_corner_shape = 2;
                wall_seq_start = false;

            }
        }

        ROS_ERROR("FLAG 10 : END Straight");
        break;

    case -1:
        // Case A
        // 멀리 있는 벽의 일정 거리 앞까지 직진
        // Wall angle turn (Pose Control)
        
        if (!Get_select_motion_on_flg() && wall_number_seq_A == 0)
        {
            wall_motion = Motion_Index::Step_in_place;
            Set_motion_index_(wall_motion);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                if (img_proc_wall_angle > 15 || img_proc_wall_angle < -15)
                {
                    if (img_proc_wall_angle > 15)
                    {
                        wall_actual_angle = 5;
                    }
                    else if (img_proc_wall_angle < -15)
                    {
                        wall_actual_angle = -5;
                    }
                    Set_turn_angle_(wall_actual_angle);
                    Set_turn_angle_on_flg(true);
                }
                else
                {
                    wall_posture = true;
                }
            }

            if (wall_posture == true)
            {
                wall_number_seq_A++;
                wall_posture = false;
            }
        }

        else if (!Get_select_motion_on_flg() && wall_number_seq_A == 1)
        {
            wall_motion = Motion_Index::Forward_2step;
            Set_motion_index_(wall_motion);
            Set_select_motion_on_flg(true);

            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                wall_number_seq_A = 1;
            }
        }

        ROS_ERROR("FLAG -1 : WALL Straight");

        break;

    default:
        Set_motion_index_(Motion_Index::InitPose);
        Set_UD_NeckAngle(10);
        Set_UD_Neck_on_flg(true);
        // wall_number_seq_A = 0;
        // wall_number_seq_B = 0;
        // wall_number_seq_C = 0;
        break;
    }

    Set_wall_det_flg(false);

    ROS_WARN("WALL_SEQ_A : %d", wall_number_seq_A);
    ROS_WARN("WALL_SEQ_B : %d", wall_number_seq_B);
    ROS_WARN("WALL_SEQ_C : %d", wall_number_seq_C);
    ROS_WARN("IMG_WALL_NUMBER_CASE : %d", wall_status);
}


/*
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
    corner_motion = Get_motion_index_();
    corner_ud_neck_angle = Get_UD_NeckAngle();

    // 0 : corner_shape dicision (From img_procPtr) (Depth)
    if (tmp_corner_seq == 0)
    {
        tmp_corner_shape = img_procPtr->Get_img_proc_corner_number();
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
        Set_corner_det_flg_2d(false);
    }

    // 1 : Motion : InitPose (For getting distance) (Depth)
    else if (tmp_corner_seq == 1)
    {
        if (!Get_select_motion_on_flg())
        {
            Set_motion_index_(Motion_Index::InitPose);
            Set_select_motion_on_flg(true);

            corner_distance = img_procPtr->Get_distance();
            corner_distance_save.push_back(corner_distance);

            if (corner_distance_save.size() == SPIN_RATE * 3)
            {
                corner_distance = accumulate(corner_distance_save.begin(), corner_distance_save.end(), 0.0) / corner_distance_save.size();
                corner_distance = std::floor(corner_distance * 1000.0) / 1000.0;
                corner_distance_save.clear();
            }
            Set_corner_det_flg_2d(false);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }

        if (!Get_UD_Neck_on_flg() )
        {
            corner_ud_neck_angle = 10;
            Set_UD_NeckAngle(corner_ud_neck_angle);
            Set_UD_Neck_on_flg(true);
        }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            req_finish_count++;
            finish_past = Get_SM_req_finish();
        }
        if (req_finish_count == 1)
        {
            req_finish_count = 0;
            tmp_corner_seq++;
        }
    }

    // 2 : Motion : Forward_Nstep (Far)
    else if (tmp_corner_seq == 2)
    {
        if (!Get_distance_on_flg())
        {
            Set_distance_(corner_distance);
            Set_distance_on_flg(true);
        }

        if (!Get_select_motion_on_flg())
        {
            corner_motion = Motion_Index::Forward_Nstep;
            Set_motion_index_(corner_motion);
            ROS_ERROR("corner_distance : %f", corner_distance);
            Set_select_motion_on_flg(true);

            if (!Get_UD_Neck_on_flg() )
            {
                huddle_ud_neck_angle = UD_CENTER;
                Set_UD_NeckAngle(huddle_ud_neck_angle);
                Set_UD_Neck_on_flg(true);
            }
            // Set_line_det_flg(true);

            Set_corner_det_flg_2d(false);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            req_finish_count++;
            finish_past = Get_SM_req_finish();
        }
        if (req_finish_count == 1)
        {
            req_finish_count = 0;
            tmp_corner_seq++;
        }
    }

    // 3 : Motion : InitPose (For getting distance) (Depth)
    else if (tmp_corner_seq == 3)
    {
        if (!Get_select_motion_on_flg())
        {
            Set_motion_index_(Motion_Index::InitPose);
            Set_select_motion_on_flg(true);

            corner_distance = img_procPtr->Get_distance();
            corner_distance_save.push_back(corner_distance);

            if (corner_distance_save.size() == SPIN_RATE * 3)
            {
                corner_distance = accumulate(corner_distance_save.begin(), corner_distance_save.end(), 0.0) / corner_distance_save.size();
                corner_distance = std::floor(corner_distance * 1000.0) / 1000.0;
                corner_distance_save.clear();
            }

            Set_corner_det_flg_2d(false);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }

        if (!Get_UD_Neck_on_flg() )
        {
            corner_ud_neck_angle = 40;
            Set_UD_NeckAngle(corner_ud_neck_angle);
            Set_UD_Neck_on_flg(true);
        }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            req_finish_count++;
            finish_past = Get_SM_req_finish();
        }
        if (req_finish_count == 1)
        {
            req_finish_count = 0;
            tmp_corner_seq++;
        }
    }

    // 4 : Motion : Forward_Nstep (Approach)
    else if (tmp_corner_seq == 4)
    {
        if (!Get_distance_on_flg())
        {
            Set_distance_(corner_distance);
            Set_distance_on_flg(true);
        }

        if (!Get_select_motion_on_flg())
        {
            corner_motion = Motion_Index::Forward_Nstep;
            Set_motion_index_(corner_motion);
            Set_select_motion_on_flg(true);
            Set_corner_det_flg_2d(false);

            if (!Get_UD_Neck_on_flg() )
            {
                huddle_ud_neck_angle = UD_CENTER;
                Set_UD_NeckAngle(huddle_ud_neck_angle);
                Set_UD_Neck_on_flg(true);
            }
            // Set_line_det_flg(true);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
        }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            req_finish_count++;
            finish_past = Get_SM_req_finish();
        }
        if (req_finish_count == 1)
        {
            req_finish_count = 0;
            tmp_corner_seq++;
        }
    }

    // 5 : Motion : Step in place
    else if (tmp_corner_seq == 5)
    {
        if (!Get_select_motion_on_flg())
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
            Set_corner_det_flg_2d(false);
        }

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            req_finish_count++;
            finish_past = Get_SM_req_finish();
        }
        if (req_finish_count == 1)
        {
            req_finish_count = 0;
            tmp_corner_seq++;
        }
    }

    // 6 : Motion : Turn Angle 90(ㅓ) or -90(ㅜ)
    else if (tmp_corner_seq == 6)
    {
        // Rotate 0 -> 30
        if (tmp_turn90 == 0)
        {
            if (!Get_select_motion_on_flg())
            {
                Set_motion_index_(Motion_Index::Step_in_place);
                Set_select_motion_on_flg(true);
                Set_corner_det_flg_2d(false);
            }

            if (!Get_turn_angle_on_flg())
            {
                corner_actual_angle = 30;
                Set_turn_angle_(corner_actual_angle);
                Set_turn_angle_on_flg(true);
            }

            if (finish_past != Get_TA_req_finish())
            {
                req_finish_count++;
                finish_past = Get_TA_req_finish();
            }

            if (req_finish_count == 1)
            {
                req_finish_count = 0;
                tmp_turn90++;
            }
        }

        // Rotate 30 -> 60
        if (tmp_turn90 == 1)
        {
            if (!Get_select_motion_on_flg())
            {
                Set_motion_index_(Motion_Index::Step_in_place);
                Set_select_motion_on_flg(true);
                Set_corner_det_flg_2d(false);
            }

            if (!Get_turn_angle_on_flg())
            {
                corner_actual_angle = 30;
                Set_turn_angle_(corner_actual_angle);
                Set_turn_angle_on_flg(true);
            }

            if (finish_past != Get_TA_req_finish())
            {
                req_finish_count++;
                finish_past = Get_TA_req_finish();
            }

            if (req_finish_count == 1)
            {
                req_finish_count = 0;
                tmp_turn90++;
            }
        }

        // Rotate 60 -> 90
        else if (tmp_turn90 == 2)
        {
            if (!Get_select_motion_on_flg())
            {
                Set_motion_index_(Motion_Index::Step_in_place);
                Set_select_motion_on_flg(true);
                Set_corner_det_flg_2d(false);
            }

            if (!Get_turn_angle_on_flg())
            {
                corner_actual_angle = 30;
                Set_turn_angle_(corner_actual_angle);
                Set_turn_angle_on_flg(true);
            }

            if (finish_past != Get_TA_req_finish())
            {
                req_finish_count++;
                finish_past = Get_TA_req_finish();
            }

            if (req_finish_count == 1)
            {
                req_finish_count = 0;
                tmp_turn90++;
            }
        }

        // Rotate 90->0
        else if (tmp_turn90 == 3)
        {
            Turn90 = true;

            if (!Get_select_motion_on_flg())
            {
                Set_motion_index_(Motion_Index::Step_in_place);
                Set_select_motion_on_flg(true);
                Set_corner_det_flg_2d(false);
            }

            if (Turn90)
            {
                if (!Get_turn_angle_on_flg())
                {
                    corner_actual_angle = 0;
                    Set_turn_angle_(corner_actual_angle);
                    Turn90 = false;
                }
            }

            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                req_finish_count++;
                finish_past = Get_SM_req_finish();
            }

            if (req_finish_count == 1)
            {
                req_finish_count = 0;
                tmp_corner_seq++;
            }
        }

        Set_corner_det_flg_2d(false);

        ROS_ERROR("tmp_turn90 : ############### %d ###############", tmp_turn90);
    }

    // 7 : Initializing
    else if (tmp_corner_seq == 7)
    {
        if (!Get_select_motion_on_flg())
        {
            Set_motion_index_(Motion_Index::InitPose);
            Set_select_motion_on_flg(true);
            Set_corner_det_flg_2d(false);

            tmp_turn90 = 0;
            tmp_corner_seq = 0;
            Set_corner_det_flg_ (false);
        }

        else if (!Get_SM_req_finish())
        {
            Set_motion_index_(Motion_Index::NONE);
            if (!Get_UD_Neck_on_flg() )
            {
                line_ud_neckangle = UD_CENTER;
                Set_UD_NeckAngle(line_ud_neckangle);
                Set_UD_Neck_on_flg(true);
            }
        }
    }

    if (Get_corner_det_stop_flg() == true)
    {
        Set_corner_det_flg_2d(false);
    }

    ROS_ERROR("CORNER_SEQ : %d", tmp_corner_seq);
    ROS_ERROR("req_finish_count : ############### %d ###############", req_finish_count);
}
*/

void Move_Decision::callbackThread()
{
    ros::NodeHandle nh(ros::this_node::getName());

    // Subscriber
    // IMU_sensor_x_subscriber_ = nh.subscribe("/Angle/x", 1000, &Move_Decision::IMUsensorCallback, this);
    IMU_sensor_y_subscriber_ = nh.subscribe("/Angle/y", 1000, &Move_Decision::IMUsensorCallback, this);
    // IMU_sensor_z_subscriber_ = nh.subscribe("/Angle/z", 1000, &Move_Decision::IMUsensorCallback, this);

    // Server
    SendMotion_server_ = nh.advertiseService("SendMotion", &Move_Decision::SendMotion, this);

    ros::Rate loop_rate(SPIN_RATE);
    startMode();
    while (nh.ok())
    {
        // ROS_INFO("-------------------------CALLBACKTHREAD----------------------------");
        // ROS_INFO("-------------------------------------------------------------------");
        Running_Mode_Decision();
        Running_Info();
        Motion_Info();
        // ROS_INFO("angle : %f", Get_turn_angle_());
        // ROS_INFO("RL_Neck : %f", Get_RL_NeckAngle());
        // ROS_INFO("UD_Neck : %f", Get_UD_NeckAngle());
        // ROS_INFO("Distance : %f", img_procPtr->Get_distance());
        // ROS_INFO("EMG : %s", Get_Emergency_() ? "true" : "false");
        // ROS_INFO("-------------------------------------------------------------------");
        // ROS_INFO("-------------------------CALLBACKTHREAD----------------------------");

        ros::spinOnce();
        loop_rate.sleep();
        // usleep(1000);
    }
}

//////////////////////////////////////////////////////////// Server Part ////////////////////////////////////////////////////////////////

bool Move_Decision::SendMotion(dynamixel_current_2port::SendMotion::Request &req, dynamixel_current_2port::SendMotion::Response &res)
{
    // Extract the unique ID included in the request
    int request_id = req.request_id;

    // Check if the request has already been processed
    if (isRequestProcessed(request_id) && !warning_printed)
    {
        if (warning_counter) // Check the counter
        {
            ROS_WARN("Duplicate service response prevented for request ID: %d", request_id);
            warning_printed = true; // Set the flag to true
            warning_counter++;      // Increase the counter
        }
        return false;
    }

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

    if (Get_SM_req_finish())
    {
        if (std::get<0>(res_SM) == Motion_Index::NONE)
        {
            std::get<0>(res_SM) = Get_motion_index_();
            Set_select_motion_on_flg(false);
        }

        res.select_motion = std::get<0>(res_SM);
        res.distance = std::get<1>(res_SM);
    }

    else if (!Get_SM_req_finish())
    {
        res.select_motion = Motion_Index::NONE;
        Set_select_motion_on_flg(false);
    }

    ROS_WARN("[MESSAGE] SM Request :   %s ", Get_SM_req_finish() ? "true" : "false");
    ROS_WARN("[MESSAGE] TA Request :   %s ", Get_TA_req_finish() ? "true" : "false");
    // ROS_WARN("[MESSAGE] RL Request :   %s ", Get_RL_req_finish() ? "true" : "false");
    // ROS_ERROR("[MESSAGE] EMG Request :   %s ", Get_EM_req_finish() ? "true" : "false");

    if (Get_TA_req_finish())
    {
        res.turn_angle = res_TA;
    }

    if (Get_UD_req_finish())
    {
        res.ud_neckangle = res_UD;
    }

    if (Get_RL_req_finish())
    {
        res.rl_neckangle = res_RL;
    }

    if (Get_EM_req_finish())
    {
        res.emergency = res_EM;
    }

    Send_Info(res.select_motion, res.turn_angle, res.ud_neckangle, res.rl_neckangle, res.emergency);

    res.success = true;
    recordProcessedRequest(request_id);

    return true;
}

std::tuple<int8_t, double> Move_Decision::playMotion()
{
    int8_t res_select_motion = 99;
    double res_distance = 0;
    // int8_t total = Get_TA_req_finish() + Get_UD_req_finish() + Get_RL_req_finish() + Get_EM_req_finish();

    if ((Get_stand_status_() == Stand_Status::Stand) && (Get_select_motion_on_flg() == true) /*&& total <=4*/)
    {
        switch (Get_motion_index_())
        {
        case Motion_Index::InitPose:
            res_select_motion = Motion_Index::InitPose;
            break;

        case Motion_Index::Forward_2step:
            res_select_motion = Motion_Index::Forward_2step;
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

        case Motion_Index::ForWard_fast4step:
            res_select_motion = Motion_Index::ForWard_fast4step;
            break;

        case Motion_Index::Forward_Nstep:
            res_select_motion = Motion_Index::Forward_Nstep;
            if (Get_distance_on_flg())
            {
                res_distance = Get_distance_();
                Set_distance_on_flg(false);
            }
            break;

        case Motion_Index::Huddle_Jump:
            res_select_motion = Motion_Index::Huddle_Jump;
            break;

        case Motion_Index::NONE:
            res_select_motion = Motion_Index::NONE;
            break;

        case Motion_Index::Forward_Halfstep:
            res_select_motion = Motion_Index::Forward_Halfstep;
            break;

        case Motion_Index::Right_Halfstep:
            res_select_motion = Motion_Index::Right_Halfstep;
            break;

        case Motion_Index::Left_Halfstep:
            res_select_motion = Motion_Index::Left_Halfstep;
            break;

        case Motion_Index::Back_Halfstep:
            res_select_motion = Motion_Index::Back_Halfstep;
            break;

        case Motion_Index::Forward_1step:
            res_select_motion = Motion_Index::Forward_1step;
            break;

        default:
            res_select_motion = Motion_Index::InitPose;
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

        default:
            res_select_motion = Motion_Index::InitPose;
        }
        Set_select_motion_on_flg(false);
    }

    // ROS_WARN("[MESSAGE] SM Request :   %s ", Get_SM_req_finish() ? "true" : "false");
    // ROS_INFO("#[MESSAGE] SM Motion :   %d#", res_select_motion);
    // ROS_INFO("#[MESSAGE] SM Distance : %f#", res_distance);

    return std::make_tuple(res_select_motion, res_distance);
}

void Move_Decision::Test_service()
{
    if (aaaa == 0)
    {
        ROS_ERROR("ccc : %d", ccc);
        ROS_ERROR("aaaa : %d", aaaa);
        abc++;

        if (ccc == 0 && !Get_select_motion_on_flg() /*  && Get_SM_req_finish() */)
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                Set_turn_angle_(10);
                Set_turn_angle_on_flg(true);
            }

            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                ccc++;
            }
        }
        if (ccc == 1 && !Get_select_motion_on_flg() /*  && Get_SM_req_finish() */)
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
            if (!Get_turn_angle_on_flg())
            {
                Set_turn_angle_(10);
                Set_turn_angle_on_flg(true);
            }
            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                ccc++;
            }
        }
        if (ccc == 2 && !Get_select_motion_on_flg() /*  && Get_SM_req_finish() */)
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
            if (!Get_turn_angle_on_flg())
            {
                Set_turn_angle_(10);
                Set_turn_angle_on_flg(true);
            }
            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                ccc = 0;
                aaaa++;
            }
        }

        ROS_ERROR("Loop NUMBER(%d)", abc);
    }

    else if (aaaa == 1 && !Get_select_motion_on_flg() /* && Get_SM_req_finish() */)
    {
        Set_motion_index_(Motion_Index::Forward_Halfstep);
        Set_select_motion_on_flg(true);

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            aaaa++;
        }
    }

    else if (aaaa == 2 && !Get_select_motion_on_flg() /*  && Get_SM_req_finish() */)
    {
        Set_motion_index_(Motion_Index::Left_Halfstep);
        Set_distance_on_flg(true);
        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            aaaa++;
        }
    }

    else if (aaaa == 3 && !Get_select_motion_on_flg() /* && Get_SM_req_finish() */)
    {
        Set_motion_index_(Motion_Index::Right_Halfstep);
        Set_select_motion_on_flg(true);
        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            aaaa++;
        }
    }

    else if (aaaa == 4 && !Get_select_motion_on_flg() /* && Get_SM_req_finish() */)
    {
        Set_motion_index_(Motion_Index::Right_2step);
        Set_select_motion_on_flg(true);
        if (finish_past != Get_SM_req_finish())
        {
            aaaa++;
        }
    }

    else if (aaaa == 5 && !Get_select_motion_on_flg() /*  && Get_SM_req_finish() */)
    {
        Set_motion_index_(Motion_Index::Huddle_Jump);
        Set_select_motion_on_flg(true);
        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            aaaa = 0;
        }
    }

    ROS_WARN("---------------------------------------");
    Motion_Info();
    ROS_WARN("---------------------------------------");
}

double Move_Decision::turn_angle()
{
    double res_turn_angle = 0;

    if ((Get_stand_status_() == Stand_Status::Stand) && Get_turn_angle_on_flg())
    {
        if (Get_motion_index_() == Motion_Index::Forward_2step || Get_motion_index_() == Motion_Index::Step_in_place || Get_motion_index_() == Motion_Index::Forward_Nstep)
        {
            res_turn_angle = this->Get_turn_angle_();
            if (res_turn_angle > TURN_MAX)
                res_turn_angle = TURN_MAX;
            else if (res_turn_angle < TURN_MIN)
                res_turn_angle = TURN_MIN;
            Set_turn_angle_on_flg(false);
        }
    }

    // ROS_WARN("[MESSAGE] TA Request :   %s ", Get_TA_req_finish() ? "true" : "false");
    // ROS_INFO("#[MESSAGE] TA Response : %f#", res_turn_angle);

    return res_turn_angle;
}

double Move_Decision::Move_UD_NeckAngle()
{

    double res_ud_neckangle = UD_CENTER;
    if (Get_running_mode_() == Running_Mode::WALL_MODE)
    {
        res_ud_neckangle = 10;
        Set_UD_Neck_on_flg(false);
    }

    else if ((Get_stand_status_() == Stand_Status::Stand) && Get_UD_Neck_on_flg())
    {
        res_ud_neckangle = this->Get_UD_NeckAngle();
        if (res_ud_neckangle > UD_MAX)
            res_ud_neckangle = UD_MAX;
        else if (res_ud_neckangle < UD_MIN)
            res_ud_neckangle = UD_MIN;
        Set_UD_Neck_on_flg(false);
    }

    else if (!Get_UD_req_finish())
    {
        if (Get_running_mode_() == Running_Mode::WALL_MODE)
        {
            res_ud_neckangle = 20;
        }
        else
        {
            res_ud_neckangle = this->Get_UD_NeckAngle();
            Set_UD_Neck_on_flg(false);
        }
    }

    // ROS_WARN("[MESSAGE] UD Request :   %s ", Get_UD_req_finish() ? "true" : "false");
    // ROS_INFO("#[MESSAGE] UD Response : %f#", res_ud_neckangle);

    return res_ud_neckangle;
}

double Move_Decision::Move_RL_NeckAngle()
{
    double res_rl_neckangle = 0;

    if ((Get_stand_status_() == Stand_Status::Stand) && Get_RL_Neck_on_flg())
    {
        // img_procssing
        res_rl_neckangle = this->Get_RL_NeckAngle();
        if (res_rl_neckangle > RL_MAX)
            res_rl_neckangle = RL_MAX;
        else if (res_rl_neckangle < RL_MIN)
            res_rl_neckangle = RL_MIN;
        Set_RL_Neck_on_flg(false);
    }

    // ROS_WARN("[MESSAGE] RL Request :   %s ", Get_RL_req_finish() ? "true" : "false");
    // ROS_INFO("#[MESSAGE] RL Response : %f#", res_rl_neckangle);

    return res_rl_neckangle;
}

bool Move_Decision::Emergency()
{
    bool res_emergency = false;

    if ((Get_stand_status_() == Stand_Status::Stand) && Get_emergency_on_flg())
    {
        // 1 : Stop
        // 0 : Keep Going (Option)
        res_emergency = Get_Emergency_();
        Set_emergency_on_flg(false);
    }

    // ROS_ERROR("[MESSAGE] EMG Request :   %s ", Get_EM_req_finish() ? "true" : "false");
    // ROS_ERROR("#[MESSAGE] EMG Response : %s#", res_emergency ? "true" : "false");

    return res_emergency;
}

///////////////////////////////////////// About Publish & Subscribe /////////////////////////////////////////

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
    else
        present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

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

void Move_Decision::startMode()
{
    Set_motion_index_(Motion_Index::InitPose);
}

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
    else /*if ((gradient < MARGIN_GRADIENT && (gradient > -MARGIN_GRADIENT)))*/
    {
        straightLine = true;
    }
}

void Move_Decision::Send_Motion_Info(int8_t res_motion)
{
    string tmp_motion;
    switch (res_motion)
    {
    case Motion_Index::InitPose:
        tmp_motion = Str_InitPose;
        break;

    case Motion_Index::Forward_2step:
        tmp_motion = Str_Forward_2step;
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

    case Motion_Index::ForWard_fast4step:
        tmp_motion = Str_ForWard_fast4step;
        break;

    case Motion_Index::Forward_Nstep:
        tmp_motion = Str_Forward_Nstep;
        break;

    case Motion_Index::Huddle_Jump:
        tmp_motion = Str_Huddle_Jump;
        break;

    case Motion_Index::Forward_Halfstep:
        tmp_motion = Str_Forward_Halfstep;
        break;

    case Motion_Index::Left_Halfstep:
        tmp_motion = Str_Left_Halfstep;
        break;

    case Motion_Index::Right_Halfstep:
        tmp_motion = Str_Right_Halfstep;
        break;

    case Motion_Index::Back_Halfstep:
        tmp_motion = Str_Back_Halfstep;
        break;

    case Motion_Index::FWD_UP:
        tmp_motion = Str_FWD_UP;
        break;

    case Motion_Index::BWD_UP:
        tmp_motion = Str_BWD_UP;
        break;

    case Motion_Index::Forward_1step:
        tmp_motion = Str_Forward_1step;
        break;

    case Motion_Index::NONE:
        tmp_motion = Str_NONE;
        break;
    }
    ROS_ERROR("SEND Motion_Index : %s", tmp_motion.c_str());
}

void Move_Decision::Send_Info(int8_t motion_, double turn_angle_, double ud, double rl, bool emg)
{
    ROS_INFO("------------------------- SEND ----------------------------");
    Send_Motion_Info(motion_);
    ROS_ERROR("#[MESSAGE] TA Response : %f#", turn_angle_);
    ROS_ERROR("#[MESSAGE] UD Response : %f#", ud);
    ROS_ERROR("#[MESSAGE] RL Response : %f#", rl);
    ROS_ERROR("#[MESSAGE] EMG Response : %s#", emg ? "true" : "false");
    ROS_INFO("------------------------- SEND ----------------------------");
}

void Move_Decision::Motion_Info()
{
    string tmp_motion;
    switch (Get_motion_index_())
    {
    case Motion_Index::InitPose:
        tmp_motion = Str_InitPose;
        break;

    case Motion_Index::Forward_2step:
        tmp_motion = Str_Forward_2step;
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

    case Motion_Index::ForWard_fast4step:
        tmp_motion = Str_ForWard_fast4step;
        break;

    case Motion_Index::Forward_Nstep:
        tmp_motion = Str_Forward_Nstep;
        break;

    case Motion_Index::Huddle_Jump:
        tmp_motion = Str_Huddle_Jump;
        break;

    case Motion_Index::Forward_Halfstep:
        tmp_motion = Str_Forward_Halfstep;
        break;

    case Motion_Index::Left_Halfstep:
        tmp_motion = Str_Left_Halfstep;
        break;

    case Motion_Index::Right_Halfstep:
        tmp_motion = Str_Right_Halfstep;
        break;

    case Motion_Index::Back_Halfstep:
        tmp_motion = Str_Back_Halfstep;
        break;

    case Motion_Index::FWD_UP:
        tmp_motion = Str_FWD_UP;
        break;

    case Motion_Index::BWD_UP:
        tmp_motion = Str_BWD_UP;
        break;

    case Motion_Index::Forward_1step:
        tmp_motion = Str_Forward_1step;
        break;

    case Motion_Index::Right_6step:
        tmp_motion = Right_6step;
        break;

    case Motion_Index::Left_6step:
        tmp_motion = Left_6step;
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
    ROS_INFO("------------------------- RUNNING ----------------------------");
    ROS_INFO("Running_Mode : %s", tmp_running.c_str());
    ROS_INFO("------------------------- RUNNING ----------------------------");
}

int8_t Move_Decision::Wall_Status(double distance_rect_)
{
    // Case A
    if (distance_rect_ >= 0.75)
    {
        wall_number_seq_C = 0;
        if (wall_status_ == 0) // start
        {
            wall_status_ = 1; // straight
        }
        else if (wall_status_ == -3) // End
        {
            wall_status_ = 10; // Straight to find corner
        }
        else if (wall_status_ == 3) // IN wall
        {
            wall_status_ = -1; // Straight to find right plane
        }
    }

    // Case B
    else if (distance_rect_ > 0.4 && distance_rect_ < 0.75)
    {
        // Plane Decision
        // LEFT Plane : 0
        // RIGHT Plane : 1

        wall_number_seq_A = 0;
        plane_mode = img_procPtr->Get_plane_mode();
        if (plane_mode == false)
        {
            wall_status_ = 2; // RIGHT plane -> straight
        }
        else if (plane_mode == true)
        {
            wall_status_ = -2; // LEFT plane -> straight
        }
    }

    // Case C
    else if (distance_rect_ <= 0.4)
    {
        wall_number_seq_B = 0;
        // Right plane
        if (wall_status_ == 2)
        {
            wall_status_ = 3; // LEFT_step
        }
        // Left plane
        else if (wall_status_ == -2)
        {
            wall_status_ = -3; // RIGHT_step
        }
    }
    return wall_status_;
}

// void Move_Diecison::CalculateQuotientAndRemainder(int dividend, int divisor, int &quotient, int &remainder)
// {
//     // Check if the divisor is not zero to avoid division by zero
//     if (divisor != 0)
//     {
//         quotient = dividend / divisor; // Calculate quotient
//         remainder = dividend % divisor; // Calculate remainder
//     }
//     else
//     {
//         // Handle division by zero error
//         std::cerr << "Error: Division by zero!" << std::endl;
//         quotient = 0; // Set quotient to zero (or you can choose to handle this case differently)
//         remainder = 0; // Set remainder to zero
//     }
// }

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

bool Move_Decision::Get_response_sent_() const
{
    std::lock_guard<std::mutex> lock(mtx_response_sent_);
    return response_sent_;
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

bool Move_Decision::Get_huddle_det_flg_2d() const
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_flg_2d);
    return huddle_det_flg_2d_;
}

bool Move_Decision::Get_huddle_det_flg_3d() const
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_flg_3d);
    return huddle_det_flg_3d_;
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

bool Move_Decision::Get_corner_det_flg_2d() const
{
    std::lock_guard<std::mutex> lock(mtx_corner_det_flg_2d);
    return corner_det_flg_2d_;
}

bool Move_Decision::Get_corner_det_flg_3d() const
{
    std::lock_guard<std::mutex> lock(mtx_corner_det_flg_3d);
    return corner_det_flg_3d_;
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

bool Move_Decision::Get_distance_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_distance_on_flg_);
    return distance_on_flg_;
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

    Set_select_motion_on_flg(true);
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
    std::lock_guard<std::mutex> lock(mtx_CallbackON_);
    this->CallbackON_ = CallbackON;
}

void Move_Decision::Set_response_sent_(bool response_sent)
{
    std::lock_guard<std::mutex> lock(mtx_response_sent_);
    this->response_sent_ = response_sent;
}

void Move_Decision::Set_goal_line_det_flg(bool goal_line_det_flg)
{
    std::lock_guard<std::mutex> lock(mtx_goal_line_det_flg);
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

void Move_Decision::Set_huddle_det_flg_2d(bool huddle_det_flg_2d)
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_flg_2d);
    this->huddle_det_flg_2d_ = huddle_det_flg_2d;
}

void Move_Decision::Set_huddle_det_flg_3d(bool huddle_det_flg_3d)
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_flg_3d);
    this->huddle_det_flg_3d_ = huddle_det_flg_3d;
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

void Move_Decision::Set_corner_det_flg_2d(bool corner_det_flg_2d)
{
    std::lock_guard<std::mutex> lock(mtx_corner_det_flg_2d);
    this->corner_det_flg_2d_ = corner_det_flg_2d;
}

void Move_Decision::Set_corner_det_flg_3d(bool corner_det_flg_3d)
{
    std::lock_guard<std::mutex> lock(mtx_corner_det_flg_3d);
    this->corner_det_flg_3d_ = corner_det_flg_3d;
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

void Move_Decision::Set_distance_on_flg(bool distance_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_distance_on_flg_);
    this->distance_on_flg_ = distance_on_flg;
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
