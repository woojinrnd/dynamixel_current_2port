#include "Move_decision.hpp"

// Constructor
// Move_Decision::Move_Decision(Img_proc *img_procPtr)
//     : img_procPtr(img_procPtr),
//       FALL_FORWARD_LIMIT(60),
//       FALL_BACK_LIMIT(-60),
//       SPIN_RATE(100),
//       stand_status_(Stand_Status::Stand),
//       motion_index_(Motion_Index::InitPose),
//       stop_fallen_check_(false),
//       Emergency_(1),
//       turn_angle_(0),
//       gradient(0)
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
    boost::thread web_process_thread = boost::thread(boost::bind(&Move_Decision::webcam_thread, this));
    boost::thread depth_process_thread = boost::thread(boost::bind(&Move_Decision::realsense_thread, this));
    boost::thread move_thread = boost::thread(boost::bind(&Move_Decision::MoveDecisionThread, this));
    boost::thread queue_thread = boost::thread(boost::bind(&Move_Decision::callbackThread, this));
}

Move_Decision::~Move_Decision()
{
}

// ********************************************** 2D THREAD************************************************** //
void Move_Decision::on_trackbar(int, void *)
{
    // Function body if required.
}

void Move_Decision::create_threshold_trackbar_W(const std::string &window_name)
{
    cv::createTrackbar("Threshold_white", window_name, &threshold_value_white, max_value, on_trackbar);
}

void Move_Decision::create_threshold_trackbar_Y(const std::string &window_name)
{
    cv::createTrackbar("Threshold_yellow", window_name, &threshold_value_yellow, max_value, on_trackbar);
}

void Move_Decision::create_color_range_trackbar(const std::string &window_name)
{
    cv::createTrackbar("Hue Lower", window_name, &hue_lower, 179, on_trackbar);
    cv::createTrackbar("Hue Upper", window_name, &hue_upper, 179, on_trackbar);
    cv::createTrackbar("Saturation Lower", window_name, &saturation_lower, 255, on_trackbar);
    cv::createTrackbar("Saturation Upper", window_name, &saturation_upper, 255, on_trackbar);
    cv::createTrackbar("Value Lower", window_name, &value_lower, 255, on_trackbar);
    cv::createTrackbar("Value Upper", window_name, &value_upper, 255, on_trackbar);
}

cv::Mat Move_Decision::extract_color(const cv::Mat &input_frame, const cv::Scalar &lower_bound, const cv::Scalar &upper_bound)
{
    cv::Mat frame = input_frame.clone();
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, lower_bound, upper_bound, mask);

    cv::Mat color_extracted;
    cv::bitwise_and(frame, frame, color_extracted, mask);

    return frame;
}

cv::Mat Move_Decision::detect_color_areas(const cv::Mat &input_frame, const cv::Scalar &contour_color, int threshold_value)
{
    cv::Mat frame = input_frame.clone();
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    cv::threshold(gray, binary, threshold_value, max_value, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // cv::cvtColor(gray, frame, cv::COLOR_GRAY2BGR);

    std::vector<cv::Point> top_contour;
    double topmost_y = std::numeric_limits<double>::max();

    for (const auto &contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area > 1000)
        {
            cv::Moments m = cv::moments(contour);
            if (m.m00 == 0)
                continue;

            cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
            if (center.y < topmost_y)
            {
                topmost_y = center.y;
                top_contour = contour;
            }
            Set_img_proc_line_det(true);
        }
        else if (area < 0)
        {
            Set_img_proc_line_det(false);
        }
    }

    if (!top_contour.empty())
    {
        cv::RotatedRect min_area_rect = cv::minAreaRect(top_contour);
        cv::Point2f vertices[4];
        min_area_rect.points(vertices);
        for (int i = 0; i < 4; ++i)
            cv::line(frame, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 255), 3);

        float angle;
        if (min_area_rect.size.width < min_area_rect.size.height)
        {
            angle = min_area_rect.angle;
        }
        else
        {
            angle = 90 + min_area_rect.angle;
        }
        // std::cout << "Angle: " << angle << std::endl;
    }

    return frame;
}

void Move_Decision::webcam_thread()
{
    const int webcam_width = 640;
    const int webcam_height = 480;

    cv::VideoCapture cap(1);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, webcam_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, webcam_height);

    if (!cap.isOpened())
    {
        std::cerr << "Could not open the webcam\n";
        return;
    }

    const std::string window_name1 = "hsv Frame_white";
    const std::string window_name2 = "thresh Frame_white";
    const std::string window_name3 = "hsv Frame_yellow";
    const std::string window_name4 = "thresh Frame_yellow";
    // cv::namedWindow(window_name1);
    cv::namedWindow(window_name2);
    // cv::namedWindow(window_name3);
    // cv::namedWindow(window_name4);

    create_threshold_trackbar_W(window_name2);
    // create_color_range_trackbar(window_name1);
    create_threshold_trackbar_Y(window_name4);
    // create_color_range_trackbar(window_name3);

    cv::Mat frame, hsv_frame_white, hsv_frame_yellow, thresh_frame_white, thresh_frame_yellow, gray;

    while (true)
    {
        cap >> frame;
        if (frame.empty())
            break;

        hsv_frame_white = extract_color(frame, lower_bound_white, upper_bound_white);
        hsv_frame_yellow = extract_color(frame, lower_bound_yellow, upper_bound_yellow);
        thresh_frame_white = detect_color_areas(hsv_frame_white, green_color, threshold_value_white);
        thresh_frame_yellow = detect_color_areas(hsv_frame_yellow, blue_color, threshold_value_yellow);

        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // cv::imshow("origin", frame);
        // cv::imshow("gray", gray);
        cv::imshow("hsv Frame_white", hsv_frame_white);
        // cv::imshow("hsv Frame_yellow", hsv_frame_yellow);
        cv::imshow("thresh Frame_white", thresh_frame_white);
        // cv::imshow("thresh Frame_yellow", thresh_frame_yellow);
        if (cv::waitKey(1) == 27)
            break;
    }
}

// ********************************************** 3D THREAD************************************************** //

void Move_Decision::realsense_thread()
{
    const int realsense_width = 640;
    const int realsense_height = 480;

    rs2::colorizer color_map;
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, realsense_width, realsense_height, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, realsense_width, realsense_height, RS2_FORMAT_Z16, 30);

    try
    {
        pipe.start(cfg);
    }
    catch (const rs2::error &e)
    {
        std::cerr << "Failed to open the RealSense camera: " << e.what() << std::endl;
        return;
    }

    const auto window_name = "Realsense Depth Frame";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    const auto window_name_color = "Realsense Color Frame";
    cv::namedWindow(window_name_color, cv::WINDOW_AUTOSIZE);

    try
    {
        while (cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
        {
            rs2::frameset data = pipe.wait_for_frames();

            rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
            rs2::frame color = data.get_color_frame();

            const int w = depth.as<rs2::video_frame>().get_width();
            const int h = depth.as<rs2::video_frame>().get_height();

            cv::Mat depthMat(cv::Size(w, h), CV_16U, (void *)depth.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat colorMat(cv::Size(w, h), CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP);

            // cv::imshow(window_name, depthMat);
            // cv::imshow(window_name_color, colorMat);
        }
    }
    catch (const rs2::error &e)
    {
        std::cerr << "An error occurred during streaming: " << e.what() << std::endl;
    }
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
    // Set_stop_det_flg(true);

    // goal mode
    // Set_goal_line_det_flg(true);
    // Set_line_det_flg(true);

    // huddle mode
    // Set_huddle_det_flg(true);

    //////////////////////////////////////   DEBUG WINDOW    //////////////////////////////////////

    //////영상처리를 통해 line_det_flg(T/F) 판별필요

    ///////////////////////// LINE_MODE --- line_det_flg = true /////////////////////////
    tmp_img_proc_line_det_flg_ = Get_img_proc_line_det();

    if (!tmp_img_proc_line_det_flg_)
    {
        Set_line_det_flg(false);
        Set_no_line_det_flg(true);
        // ROS_ERROR("%d", Get_line_det_flg());
    }
    if (tmp_img_proc_line_det_flg_)
    {
        Set_line_det_flg(true);
        Set_no_line_det_flg(false);
    }
    // tmp_img_proc_line_det_flg_ = !tmp_img_proc_line_det_flg_;

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

    // else if (장애물과 로봇이 접촉해 정지가 필요할 때)
    // {
    // Set_stop_det_flg(true);
    // }

    /////////////////////////NO_LINE_MODE --- goal_line_det_flg = true /////////////////////////
    // else if (골 라인 인식 == true)
    // {
    // Set_goal_line_det_flg(true);
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
        ROS_INFO("Gradient : %f", Get_gradient());
        ROS_INFO("delta_x : %f", delta_x);
        ROS_INFO("img_proc_line_det_flg : %d", Get_img_proc_line_det());
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
        ROS_ERROR("STRAIGHT LINE");
    }

    // Non Straight Line
    if (straightLine == false)
    {
        // Counter Clock wise(+) (Turn Angle sign)
        // Gradient : Angle from center of window.x to center of line.x
        // LEFT TURN
        if (tmp_gradient >= margin_gradient * 5)
        {
            Actural_angle += 4;
            ROS_WARN("LEFT_TURN");
        }
        else if (tmp_gradient >= margin_gradient * 4)
        {
            Actural_angle += 3;
            ROS_WARN("LEFT_TURN");
        }
        else if (tmp_gradient >= margin_gradient * 3)
        {
            Actural_angle += 2;
            ROS_WARN("LEFT_TURN");
        }
        else if (tmp_gradient >= margin_gradient * 2)
        {
            Actural_angle += 2;
            ROS_WARN("LEFT_TURN");
        }
        else if (tmp_gradient > margin_gradient * 1)
        {
            Actural_angle += 2;
            ROS_WARN("LEFT_TURN");
        }

        // Right Turn
        if (tmp_gradient <= -margin_gradient * 5)
        {
            Actural_angle -= 4;
            ROS_WARN("RIGHT TURN");
        }
        else if (tmp_gradient <= -margin_gradient * 4)
        {
            Actural_angle -= 3;
            ROS_WARN("RIGHT TURN");
        }
        else if (tmp_gradient <= -margin_gradient * 3)
        {
            Actural_angle -= 2;
            ROS_WARN("RIGHT TURN");
        }
        else if (tmp_gradient <= -margin_gradient * 2)
        {
            Actural_angle -= 2;
            ROS_WARN("RIGHT TURN");
        }
        else if (tmp_gradient < -margin_gradient * 1)
        {
            Actural_angle -= 2;
            ROS_WARN("RIGHT TURN");
        }

        Set_turn_angle_(Actural_angle);
        ROS_ERROR("NO STRAIGHT LINE");
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
    else // LEFT
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
    Set_Emergency_(false);
}

void Move_Decision::WAKEUP_mode()
{
    int8_t tmp_stand_status = Get_stand_status_();
    if (tmp_stand_status == Stand_Status::Fallen_Back)
        Set_motion_index_(Motion_Index::FWD_UP);
    if (tmp_stand_status == Stand_Status::Fallen_Forward)
        Set_motion_index_(Motion_Index::BWD_UP);
    // Motion_Info();
}

void Move_Decision::GOAL_LINE_mode()
{
    // longer width 활용하고 싶음
    Set_motion_index_(Motion_Index::Forward_4step);
}

void Move_Decision::HUDDLE_mode()
{

    // 고개를 들고 허들의 거리값 받아와 걸음 수 계산
    // 걸음수 전달 후, LineMode로 진입
    // 허들을 다시 본다면 멈추고 다시 걸음 수 계산
    // LineMode

    double tmp_ud_neckangle = Get_UD_NeckAngle();
    tmp_ud_neckangle = 45;
    Set_UD_NeckAngle(tmp_ud_neckangle);

    Set_line_det_flg(true);
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
    UD_NeckAngle_server_ = nh.advertiseService("UD_NeckAngle", &Move_Decision::Move_UD_NeckAngle, this);
    RL_NeckAngle_server_ = nh.advertiseService("RL_NeckAngle", &Move_Decision::Move_RL_NeckAngle, this);

    ros::Rate loop_rate(SPIN_RATE);
    while (nh.ok())
    {
        startMode();

        ROS_INFO("-------------------------CALLBACKTHREAD----------------------------");
        ROS_INFO("-------------------------------------------------------------------");
        Running_Mode_Decision();
        Running_Info();
        Motion_Info();
        // ROS_INFO("angle : %f", Get_turn_angle_());
        ROS_INFO("RL_Neck : %f", Get_RL_NeckAngle());
        ROS_INFO("UD_Neck : %f", Get_UD_NeckAngle());
        ROS_INFO("img_proc_line_det_flg : %d", Get_img_proc_line_det());
        ROS_INFO("-------------------------------------------------------------------");
        ROS_INFO("-------------------------CALLBACKTHREAD----------------------------");

        ros::spinOnce();
        loop_rate.sleep();
        // usleep(1000);
    }
}

//////////////////////////////////////////////////////////// Server Part ////////////////////////////////////////////////////////////////
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
    ROS_INFO("#[MESSAGE] TA Response : %f#", res.turn_angle);
    return true;
}

bool Move_Decision::Move_UD_NeckAngle(dynamixel_current_2port::UD_NeckAngle::Request &req, dynamixel_current_2port::UD_NeckAngle::Response &res)
{
    if ((req.finish == true) && (stand_status_ == Stand_Status::Stand))
    {
        // img_procssing
        res.ud_neckangle = Get_UD_NeckAngle();
    }

    ROS_INFO("[MESSAGE] UD Request :   %d ", req.finish);
    ROS_INFO("#[MESSAGE] UD Response : %f#", res.ud_neckangle);
    return true;
}

bool Move_Decision::Move_RL_NeckAngle(dynamixel_current_2port::RL_NeckAngle::Request &req, dynamixel_current_2port::RL_NeckAngle::Response &res)
{
    if ((req.finish == true) && (stand_status_ == Stand_Status::Stand))
    {
        // img_procssing
        res.rl_neckangle = Get_RL_NeckAngle();
    }

    ROS_INFO("[MESSAGE] RL Request :   %d ", req.finish);
    ROS_INFO("#[MESSAGE] RL Response : %f#", res.rl_neckangle);
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

double Move_Decision::Get_turn_angle_() const
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

double Move_Decision::Get_gradient() const
{
    return gradient;
}

double Move_Decision::Get_delta_x() const
{
    return delta_x;
}

double Move_Decision::Get_UD_NeckAngle() const
{
    return UD_NeckAngle_;
}

double Move_Decision::Get_RL_NeckAngle() const
{
    return RL_NeckAngle_;
}

bool Move_Decision::Get_img_proc_line_det() const
{
    return img_proc_line_det_;
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

void Move_Decision::Set_turn_angle_(double turn_angle_)
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

void Move_Decision::Set_gradient(double gradient)
{
    this->gradient = gradient;
}

void Move_Decision::Set_delta_x(double delta_x)
{
    this->delta_x = delta_x;
}

void Move_Decision::Set_RL_NeckAngle(double RL_NeckAngle_)
{
    this->RL_NeckAngle_ = RL_NeckAngle_;
}

void Move_Decision::Set_UD_NeckAngle(double UD_NeckAngle_)
{
    this->UD_NeckAngle_ = UD_NeckAngle_;
}

void Move_Decision::Set_img_proc_line_det(bool img_proc_line_det_)
{
    this->img_proc_line_det_ = img_proc_line_det_;
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
    }
    ROS_INFO("Running_Mode : %s", tmp_running.c_str());
}