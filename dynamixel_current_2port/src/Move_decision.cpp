#include "Move_decision.hpp"

// Constructor
Move_Decision::Move_Decision()
    : FALL_FORWARD_LIMIT(60),
      FALL_BACK_LIMIT(-60),
      SPIN_RATE(1),
      stand_status_(Stand_Status::Stand),
      motion_index_(Motion_Index::InitPose),
      stop_fallen_check_(false),
      Emergency_(1),
      turn_angle_(0),
      gradient(0)
{
    // Init ROS
    ros::NodeHandle nh(ros::this_node::getName());

    boost::thread process_thread = boost::thread(boost::bind(&Move_Decision::processThread, this));
    boost::thread move_thread = boost::thread(boost::bind(&Move_Decision::MoveDecisionThread, this));
    boost::thread queue_thread = boost::thread(boost::bind(&Move_Decision::callbackThread, this));
}

Move_Decision::~Move_Decision()
{
}

// ********************************************** PROCESS THREAD************************************************** //

void Move_Decision::process()
{
    /////////////   DEBUG WINDOW    /////////////
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

    //// No-straight line - set gradient
    // Set_line_det_flg(true);
    // Set_gradient(30);

    //// no_line_det_flg
    // Set_no_line_det_flg(true);
    // aaaa++;
    // Set_delta_x(aaaa);

    //// wake up mode
    // Set_running_mode_(Running_Mode::WAKEUP_MODE);
    // Set_stand_status_(Stand_Status::Fallen_Forward);
    // Motion_Info();


    /////////////   DEBUG WINDOW    /////////////




    //////영상처리를 통해 line_det_flg(T/F) 판별필요

    
    ///////////////////////// LINE_MODE --- line_det_flg = true /////////////////////////
    // if (라인 인식 == true)
    // {
    //     Set_line_det_flg(true);
    // }

    // 영상처리를 통해 Gradient 값 가져오기
    // Gradient 추가(Line ~ center of frame )
    // Gradient -> Turn_angle_
    // Set_gradient(50);

    /////////////////////////NO_LINE_MODE --- no_line_det_flg = true /////////////////////////
    // else if (라인 인식 == false)
    // {
    //     Set_no_line_det_flg(true);
    // delta_x : Center of window.x - Center of last captured line.x

    //     delta_x = ;
    // }

    /////////////////////////NO_LINE_MODE --- no_line_det_flg = true /////////////////////////
    // else if (골 라인 인식 == true)
    // {
        
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
        //ProccessThread(gradient) = callbackThread(turn_angle)
        ROS_INFO("Gradient : %d", Get_gradient());
        ROS_INFO("delta_x : %f", delta_x);
        ROS_INFO("-------------------------PROCESSTHREAD----------------------------");
        ROS_INFO("\n");
        // relax to fit output rate
        loop_rate.sleep();
    }
}

// ********************************************** MoveDecision THREAD ************************************************** //

void Move_Decision::MoveDecisionThread()
{
    // set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    // node loop
    while (ros::ok())
    {
        // relax to fit output rate
        loop_rate.sleep();
    }
}

void Move_Decision::Running_Mode_Decision()
{
    if (running_mode_ == WAKEUP_MODE || stand_status_ == Fallen_Forward || stand_status_ == Fallen_Back)
    {
        running_mode_ = WAKEUP_MODE;
    }

    else if (running_mode_ != WAKEUP_MODE)
    {
        if (goal_line_det_flg && line_det_flg) // goal_line_detect_flg is true: goal_line detection mode
        {
            running_mode_ = GOAL_MODE;
        }
        else if (no_line_det_flg)
        {
            running_mode_ = NO_LINE_MODE;
        }
        else if (line_det_flg)
        {
            running_mode_ = LINE_MODE;
        }
        else if (huddle_det_flg)
        {
            running_mode_ = HUDDLE_MODE;
        }
        else if (wall_det_flg)
        {
            running_mode_ = WALL_MODE;
        }
        else if (stop_det_flg)
        {
            running_mode_ = STOP_MODE;
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
    }

    // running_mode_ = Running_Mode::LINE_MODE;
}

void Move_Decision::LINE_mode()
{
    // Set_motion_index_(Motion_Index::Right_2step);

    int8_t tmp_gradient = Get_gradient();
    StraightLineDecision(tmp_gradient, margin_gradient);

    // Straight Line
    if (straightLine == true)
    {
        Set_motion_index_(Motion_Index::Forward_4step);
    }

    // Non Straight Line
    if (straightLine == false)
    {
        Set_turn_angle_(tmp_gradient);
    }
}

void Move_Decision::NOLINE_mode()
{
    // Counter Clock wise(+) (Turn Angle sign)
    // delta_x > 0 : LEFT Window  ->  Left turn (-)
    // delta_x < 0 : RIGHT window ->  Right turn  (+)
    double tmp_delta_x = Get_delta_x();
    if (tmp_delta_x < 0) // Right
    {
        Actural_angle -= 1;
        if (Actural_angle < -Angle_ToFindLine)
        {
            Actural_angle = -Angle_ToFindLine;
        }
        Set_turn_angle_(Actural_angle);
        ROS_WARN("RIGHT TURN");
        // ROS_INFO("turn angle : %d", Get_turn_angle_());
    }
    else //LEFT
    {
        Actural_angle += 1;
        if (Actural_angle > Angle_ToFindLine)
        {
            Actural_angle = Angle_ToFindLine;
        }
        Set_turn_angle_(Actural_angle);
        ROS_WARN("LEFT_TURN");
    }
}

void Move_Decision::STOP_mode()
{
    Set_Emergency_(true);
}

void Move_Decision::WAKEUP_mode()
{
    int8_t tmp_stand_status = Get_stand_status_();
    if (tmp_stand_status == Stand_Status::Fallen_Back) Set_motion_index_(Motion_Index::FWD_UP);
    if (tmp_stand_status == Stand_Status::Fallen_Forward) Set_motion_index_(Motion_Index::BWD_UP);
    // Motion_Info();
}

void Move_Decision::GOAL_LINE_mode()
{
}

void Move_Decision::HUDDLE_mode()
{
}

void Move_Decision::WALL_mode()
{
}

// ********************************************** CALLBACK THREAD ************************************************** //

void Move_Decision::callbackThread()
{
    ros::NodeHandle nh(ros::this_node::getName());

    // Subscriber & Publisher
    Emergency_pub_ = nh.advertise<std_msgs::Bool>("Emergency", 0);
    imu_data_sub_ = nh.subscribe("imu", 1, &Move_Decision::imuDataCallback, this);

    // Server
    motion_index_server_ = nh.advertiseService("Select_Motion", &Move_Decision::playMotion, this);
    turn_angle_server_ = nh.advertiseService("Turn_Angle", &Move_Decision::turn_angle, this);

    ros::Rate loop_rate(1);
    while (nh.ok())
    {
        startMode();

        ROS_INFO("-------------------------CALLBACKTHREAD----------------------------");
        ROS_INFO("-------------------------------------------------------------------");
        Running_Mode_Decision();
        Running_Info();
        Motion_Info();
        ROS_INFO("angle : %d", Get_turn_angle_());
        ROS_INFO("-------------------------------------------------------------------");
        ROS_INFO("-------------------------CALLBACKTHREAD----------------------------");

        ros::spinOnce();
        loop_rate.sleep();
        // usleep(1000);
    }
}

///////////////////////////////////////// Server Part /////////////////////////////////////////
bool Move_Decision::playMotion(dynamixel_current_2port::Select_Motion::Request &req, dynamixel_current_2port::Select_Motion::Response &res)
{
    if ((req.finish == true) && (stand_status_ == Stand_Status::Stand))
    {
        switch (Get_motion_index_())
        {
        case Motion_Index::InitPose:
            res.select_motion = Motion_Index::InitPose;
            break;

        case Motion_Index::Forward_4step:
            res.select_motion = Motion_Index::Forward_4step;
            break;

        case Motion_Index::Left_2step:
            res.select_motion = Motion_Index::Left_2step;
            break;

        case Motion_Index::Step_in_place:
            res.select_motion = Motion_Index::Step_in_place;
            break;

        case Motion_Index::Right_2step:
            res.select_motion = Motion_Index::Right_2step;
            break;

        case Motion_Index::Back_4step:
            res.select_motion = Motion_Index::Back_4step;
            break;
        }
    }
    else if ((req.finish == true) && ((stand_status_ == Stand_Status::Fallen_Back)) || (stand_status_ == Stand_Status::Fallen_Forward))
    {
        switch (Get_motion_index_())
        {
        case Motion_Index::FWD_UP:
            res.select_motion = Motion_Index::FWD_UP;
            break;

        case Motion_Index::BWD_UP:
            res.select_motion = Motion_Index::BWD_UP;
            break;
        }
    }

    ROS_INFO("[MESSAGE] SM Request :   %d ", req.finish);
    ROS_INFO("#[MESSAGE] SM Response : %d#", res.select_motion);
    return true;
}

bool Move_Decision::turn_angle(dynamixel_current_2port::Turn_Angle::Request &req, dynamixel_current_2port::Turn_Angle::Response &res)
{
    if ((req.finish == true) && (stand_status_ == Stand_Status::Stand))
    {
        // img_procssing
        res.turn_angle = Get_turn_angle_();
    }

    ROS_INFO("[MESSAGE] TA Request :   %d ", req.finish);
    ROS_INFO("#[MESSAGE] TA Response : %d#", res.turn_angle);
    return true;
}

///////////////////////////////////////// About Publish & Subscribe /////////////////////////////////////////
void Move_Decision::startMode()
{
    bool send_emergency = Get_Emergency_();
    EmergencyPublish(send_emergency);
    Set_motion_index_(Motion_Index::InitPose);
}

// void Move_Decision::stopMode()
// {
//     playMotion(Motion_Index::Foward_4step);
// }

// Emergency Stop
// 0 : Stop
// 1 : Keep Going (Option)
void Move_Decision::EmergencyPublish(bool _emergency)
{
    std_msgs::Bool A;
    A.data = _emergency;

    Emergency_pub_.publish(A);
    // ROS_INFO("%d",Emergency_);
}

// check fallen states
void Move_Decision::imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (stop_fallen_check_ == true)
        return;

    Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::MatrixXd rpy_orientation = convertQuaternionToRPY(orientation);
    rpy_orientation *= (180 / M_PI);

    //   ROS_INFO_COND(DEBUG_PRINT, "Roll : %3.2f, Pitch : %2.2f", rpy_orientation.coeff(0, 0), rpy_orientation.coeff(1, 0));

    double pitch = rpy_orientation.coeff(1, 0);

    double alpha = 0.4;
    if (present_pitch_ == 0)
        present_pitch_ = pitch;
    else
        present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

    if (present_pitch_ > FALL_FORWARD_LIMIT)
        stand_status_ = Fallen_Forward;
    else if (present_pitch_ < FALL_BACK_LIMIT)
        stand_status_ = Fallen_Back;
    else
        stand_status_ = Stand;
}

// ********************************************** GETTERS ************************************************** //

bool Move_Decision::Get_Emergency_() const
{
    return Emergency_;
}

int8_t Move_Decision::Get_motion_index_() const
{
    return motion_index_;
}

int8_t Move_Decision::Get_stand_status_() const
{
    return stand_status_;
}

int8_t Move_Decision::Get_running_mode_() const
{
    return running_mode_;
}

int8_t Move_Decision::Get_turn_angle_() const
{
    return turn_angle_;
}

bool Move_Decision::Get_ProcessON() const
{
    return ProcessON_;
}

bool Move_Decision::Get_MoveDecisionON() const
{
    return MoveDecisionON_;
}

bool Move_Decision::Get_CallbackON() const
{
    return CallbackON_;
}

bool Move_Decision::Get_goal_line_det_flg() const
{
    return goal_line_det_flg;
}

bool Move_Decision::Get_line_det_flg() const
{
    return line_det_flg;
}

bool Move_Decision::Get_no_line_det_flg() const
{
    return no_line_det_flg;
}

bool Move_Decision::Get_huddle_det_flg() const
{
    return huddle_det_flg;
}

bool Move_Decision::Get_wall_det_flg() const
{
    return wall_det_flg;
}

bool Move_Decision::Get_stop_det_flg() const
{
    return stop_det_flg;
}

int8_t Move_Decision::Get_gradient() const
{
    return gradient;
}

double Move_Decision::Get_delta_x() const
{
    return delta_x;
}


// ********************************************** SETTERS ************************************************** //

void Move_Decision::Set_Emergency_(bool Emergency_)
{
    this->Emergency_ = Emergency_;
}

void Move_Decision::Set_motion_index_(int8_t motion_index_)
{
    this->motion_index_ = motion_index_;
}

void Move_Decision::Set_stand_status_(int8_t stand_status_)
{
    this->stand_status_ = stand_status_;
}

void Move_Decision::Set_running_mode_(int8_t running_mode_)
{
    this->running_mode_ = running_mode_;
}

void Move_Decision::Set_turn_angle_(int8_t turn_angle_)
{
    this->turn_angle_ = turn_angle_;
}

void Move_Decision::Set_ProcessON(bool ProcessON_)
{
    this->ProcessON_ = ProcessON_;
}

void Move_Decision::Set_MoveDecisionON(bool MoveDecisionON_)
{
    this->MoveDecisionON_ = MoveDecisionON_;
}

void Move_Decision::Set_CallbackON(bool CallbackON_)
{
    this->CallbackON_ = CallbackON_;
}

void Move_Decision::Set_goal_line_det_flg(bool goal_line_det_flg)
{
    this->goal_line_det_flg = goal_line_det_flg;
}

void Move_Decision::Set_line_det_flg(bool line_det_flg)
{
    this->line_det_flg = line_det_flg;
}

void Move_Decision::Set_no_line_det_flg(bool no_line_det_flg)
{
    this->no_line_det_flg = no_line_det_flg;
}

void Move_Decision::Set_huddle_det_flg(bool huddle_det_flg)
{
    this->huddle_det_flg = huddle_det_flg;
}

void Move_Decision::Set_wall_det_flg(bool wall_det_flg)
{
    this->wall_det_flg = wall_det_flg;
}

void Move_Decision::Set_stop_det_flg(bool stop_det_flg)
{
    this->stop_det_flg = stop_det_flg;
}

void Move_Decision::Set_gradient(int8_t gradient)
{
    this->gradient = gradient;
}

void Move_Decision::Set_delta_x(double delta_x)
{
    this->delta_x = delta_x;
}

// ********************************************** FUNCTION ************************************************** //

Eigen::Vector3d Move_Decision::convertRotationToRPY(const Eigen::Matrix3d &rotation)
{
    Eigen::Vector3d rpy; // = Eigen::MatrixXd::Zero(3,1);

    rpy.coeffRef(0, 0) = atan2(rotation.coeff(2, 1), rotation.coeff(2, 2));
    rpy.coeffRef(1, 0) = atan2(-rotation.coeff(2, 0), sqrt(pow(rotation.coeff(2, 1), 2) + pow(rotation.coeff(2, 2), 2)));
    rpy.coeffRef(2, 0) = atan2(rotation.coeff(1, 0), rotation.coeff(0, 0));

    return rpy;
}

Eigen::Vector3d Move_Decision::convertQuaternionToRPY(const Eigen::Quaterniond &quaternion)
{
    Eigen::Vector3d rpy = convertRotationToRPY(quaternion.toRotationMatrix());

    return rpy;
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

    case Motion_Index::FWD_UP:
        tmp_motion = Str_FWD_UP;
        break;

    case Motion_Index::BWD_UP:
        tmp_motion = Str_BWD_UP;
        break;
    }

    ROS_INFO("%s", tmp_motion.c_str());
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
    }
    ROS_INFO("%s", tmp_running.c_str());
}