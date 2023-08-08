#include "img_proc.hpp"

// Constructor
Img_proc::Img_proc()
    : SPIN_RATE(30),
      img_proc_line_det_(false)
{
}

// ********************************************** 2D THREAD************************************************** //

void Img_proc::on_trackbar(int, void *)
{
    // Function body if required.
}

void Img_proc::create_threshold_trackbar_W(const std::string &window_name)
{
    cv::createTrackbar("Threshold_white", window_name, &threshold_value_white, max_value, on_trackbar);
}

void Img_proc::create_threshold_trackbar_Y(const std::string &window_name)
{
    cv::createTrackbar("Threshold_yellow", window_name, &threshold_value_yellow, max_value, on_trackbar);
}

void Img_proc::create_color_range_trackbar(const std::string &window_name)
{
    cv::createTrackbar("Hue Lower", window_name, &hue_lower, 179, on_trackbar);
    cv::createTrackbar("Hue Upper", window_name, &hue_upper, 179, on_trackbar);
    cv::createTrackbar("Saturation Lower", window_name, &saturation_lower, 255, on_trackbar);
    cv::createTrackbar("Saturation Upper", window_name, &saturation_upper, 255, on_trackbar);
    cv::createTrackbar("Value Lower", window_name, &value_lower, 255, on_trackbar);
    cv::createTrackbar("Value Upper", window_name, &value_upper, 255, on_trackbar);
}

cv::Mat Img_proc::extract_color(const cv::Mat &input_frame, const cv::Scalar &lower_bound, const cv::Scalar &upper_bound)
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

cv::Mat Img_proc::detect_color_areas(const cv::Mat &input_frame, const cv::Scalar &contour_color, int threshold_value)
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
            
            ROS_WARN("LINE_MODE ON");
        }
        else
        {
            Set_img_proc_line_det(false);
            ROS_ERROR("NO_LINE_MODE ON");
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

// void Img_proc::webcam_thread()
// {
//     cv::VideoCapture cap(webcam_id);
//     cap.set(cv::CAP_PROP_FRAME_WIDTH, webcam_width);
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, webcam_height);
//     cap.set(cv::CAP_PROP_FPS, webcam_fps);

//     if (!cap.isOpened())
//     {
//         std::cerr << "Could not open the webcam\n";
//         return;
//     }

//     const std::string window_name1 = "hsv Frame_white";
//     const std::string window_name2 = "thresh Frame_white";
//     // const std::string window_name3 = "hsv Frame_yellow";
//     // const std::string window_name4 = "thresh Frame_yellow";
//     cv::namedWindow(window_name1);
//     cv::namedWindow(window_name2);
//     // cv::namedWindow(window_name3);
//     // cv::namedWindow(window_name4);

//     create_color_range_trackbar(window_name1);
//     create_threshold_trackbar_W(window_name2);
//     // create_color_range_trackbar(window_name3);
//     // create_threshold_trackbar_Y(window_name4);

//     cv::Mat frame, hsv_frame_white, hsv_frame_yellow, thresh_frame_white, thresh_frame_yellow, gray;

//     // // set node loop rate
//     // ros::Rate loop_rate(SPIN_RATE);

//     while (ros::ok())
//     {
//         cap >> frame;
//         if (frame.empty())
//             break;

//         hsv_frame_white = extract_color(frame, lower_bound_white, upper_bound_white);
//         hsv_frame_yellow = extract_color(frame, lower_bound_yellow, upper_bound_yellow);
//         thresh_frame_white = detect_color_areas(hsv_frame_white, green_color, threshold_value_white);
//         thresh_frame_yellow = detect_color_areas(hsv_frame_yellow, blue_color, threshold_value_yellow);

//         cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

//         // cv::imshow("origin", frame);
//         // cv::imshow("gray", gray);
//         cv::imshow("hsv Frame_white", hsv_frame_white);
//         // cv::imshow("hsv Frame_yellow", hsv_frame_yellow);
//         cv::imshow("thresh Frame_white", thresh_frame_white);
//         // cv::imshow("thresh Frame_yellow", thresh_frame_yellow);
//         if (cv::waitKey(1) == 27)
//             break;
//         // loop_rate.sleep();
//     }
// }

// // ********************************************** 3D THREAD************************************************** //

void Img_proc::realsense_thread()
{
    rs2::colorizer color_map;
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, realsense_width, realsense_height, RS2_FORMAT_BGR8, realsense_fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, realsense_width, realsense_height, RS2_FORMAT_Z16, realsense_fps);

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
        while (ros::ok() && cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
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

// *************************************************************************  *****************************************************************************//

void Img_proc::RGB2HSV(const cv::Mat &rgb_image, cv::Mat &hsv_image)
{
    cv::cvtColor(rgb_image, hsv_image, cv::COLOR_BGR2HSV);
    morphologyEx(hsv_image, hsv_image, MORPH_OPEN, Mat(), Point(-1, -1), 1);
    morphologyEx(hsv_image, hsv_image, MORPH_ERODE, Mat(), Point(-1, -1), -5);
}

void Img_proc::RGB2LAB(const cv::Mat &rgb_image, cv::Mat &lab_image)
{
    cv::cvtColor(rgb_image, lab_image, cv::COLOR_BGR2Lab);
	morphologyEx(lab_image, lab_image, MORPH_OPEN, Mat(), Point(-1, -1), 1);
}

void Img_proc::extractAndDisplayObject()
{
    vcap >> Origin_img; // Read a frame from the camera

    if (Origin_img.empty())
    {
        std::cerr << "Empty frame received from the camera!" << std::endl;
        return;
    }

    cv::Mat hsv_image, lab_image;
    RGB2HSV(Origin_img, hsv_image);
    RGB2LAB(Origin_img, lab_image);

    // Create binary masks for the specified color ranges in HSV and LAB color spaces
    cv::Mat hsv_binary_mask, lab_binary_mask;

    // Get current HSV and LAB threshold values from trackbars
    int h_min = cv::getTrackbarPos("H min", "Threshold Adjustments");
    int h_max = cv::getTrackbarPos("H max", "Threshold Adjustments");
    int s_min = cv::getTrackbarPos("S min", "Threshold Adjustments");
    int s_max = cv::getTrackbarPos("S max", "Threshold Adjustments");
    int v_min = cv::getTrackbarPos("V min", "Threshold Adjustments");
    int v_max = cv::getTrackbarPos("V max", "Threshold Adjustments");
    int l_min = cv::getTrackbarPos("L min", "Threshold Adjustments");
    int l_max = cv::getTrackbarPos("L max", "Threshold Adjustments");
    int a_min = cv::getTrackbarPos("A min", "Threshold Adjustments");
    int a_max = cv::getTrackbarPos("A max", "Threshold Adjustments");
    int b_min = cv::getTrackbarPos("B min", "Threshold Adjustments");
    int b_max = cv::getTrackbarPos("B max", "Threshold Adjustments");

    cv::Scalar hsv_lower_thresh(h_min, s_min, v_min);
    cv::Scalar hsv_upper_thresh(h_max, s_max, v_max);
    cv::Scalar lab_lower_thresh(l_min, a_min, b_min);
    cv::Scalar lab_upper_thresh(l_max, a_max, b_max);

    cv::inRange(hsv_image, hsv_lower_thresh, hsv_upper_thresh, hsv_binary_mask);
    cv::inRange(lab_image, lab_lower_thresh, lab_upper_thresh, lab_binary_mask);

    // Combine binary masks from both color spaces
    cv::Mat final_binary_mask;
    Mat field;
    cv::bitwise_and(hsv_binary_mask, lab_binary_mask, final_binary_mask);

    // Apply morphological operations if needed (e.g., to remove noise)
    cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(final_binary_mask, final_binary_mask, cv::MORPH_OPEN, morph_kernel);

    field = final_binary_mask.clone();

    
    // Calculate object area
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> hull;
    std::vector<cv::Point> approxpoly;
    // cv::findContours(final_binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    cv::findContours(field, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    int _size = static_cast<int>(contours.size());
    int area_max = 0;
    int label_num = 0;

    if (_size > 0)
    {
        for (int i = 0; i < _size; i++)
        {
            int area = cvRound(cv::contourArea(contours[i]));
            area_max = std::max(area_max, area);
        }

        if (area_max >= 2000)
        {
            this->Set_img_proc_line_det(true);
            // ROS_WARN("img_proc_line_det : %d", Get_img_proc_line_det());
        }

        else if (500 < area_max < 2000)
        {
            this->Set_img_proc_line_det(false);
            // ROS_ERROR("img_proc_line_det : %d", Get_img_proc_line_det());
        }

        cv::convexHull(cv::Mat(contours[label_num]), hull, false);
        // cv::fillConvexPoly(Origin_img, hull, cv::Scalar(255), 8);
        cv::fillConvexPoly(field, hull, cv::Scalar(255), 8);

        cv::Scalar color(0, 0, 255); // Red color for text
        std::string area_text = "Area: " + std::to_string(area_max) + " pixels";

        // Draw the area information on the frame
        cv::putText(Origin_img, area_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, color, 2);
    }

    if(contours.empty())
    {
        this->Set_img_proc_no_line_det(true);
        // ROS_ERROR("img_proc_no_line_det : %d", Get_img_proc_no_line_det());
        // cout << "no area" << endl;
    }
    cv::morphologyEx(field, field, cv::MORPH_ERODE, cv::Mat(), cv::Point(-1, -1), 5);
    cv::bitwise_and(field, ~final_binary_mask, field);
    cv::morphologyEx(field, field, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2);

    // Show the original frame and the final binary mask
    cv::imshow("Original Frame", Origin_img);
    cv::imshow("Final Binary Mask", final_binary_mask);
}

void Img_proc::init()
{
    // vcap = VideoCapture(CAP_DSHOW + webcam_id);
    vcap = VideoCapture(webcam_id);
    vcap.set(cv::CAP_PROP_FRAME_WIDTH, webcam_width);
    vcap.set(cv::CAP_PROP_FRAME_HEIGHT, webcam_height);
    vcap.set(cv::CAP_PROP_FPS, webcam_fps);
    

    if (!vcap.isOpened())
    {
        std::cerr << "Could not open the webcam\n";
        return;
    }

    cv::namedWindow("Threshold Adjustments", cv::WINDOW_NORMAL);
    cv::createTrackbar("H min", "Threshold Adjustments", nullptr, 255);
    cv::createTrackbar("H max", "Threshold Adjustments", nullptr, 255);
    cv::createTrackbar("S min", "Threshold Adjustments", nullptr, 255);
    cv::createTrackbar("S max", "Threshold Adjustments", nullptr, 255);
    cv::createTrackbar("V min", "Threshold Adjustments", nullptr, 255);
    cv::createTrackbar("V max", "Threshold Adjustments", nullptr, 255);
    cv::createTrackbar("L min", "Threshold Adjustments", nullptr, 255);
    cv::createTrackbar("L max", "Threshold Adjustments", nullptr, 255);
    cv::createTrackbar("A min", "Threshold Adjustments", nullptr, 255);
    cv::createTrackbar("A max", "Threshold Adjustments", nullptr, 255);
    cv::createTrackbar("B min", "Threshold Adjustments", nullptr, 255);
    cv::createTrackbar("B max", "Threshold Adjustments", nullptr, 255);


    cv::setTrackbarPos("H min", "Threshold Adjustments", 77);
    cv::setTrackbarPos("S min", "Threshold Adjustments", 142);
    cv::setTrackbarPos("V min", "Threshold Adjustments", 54);
    cv::setTrackbarPos("L min", "Threshold Adjustments", 53);
    cv::setTrackbarPos("A min", "Threshold Adjustments", 77);
    cv::setTrackbarPos("B min", "Threshold Adjustments", 82);

    cv::setTrackbarPos("H max", "Threshold Adjustments", 235);
    cv::setTrackbarPos("S max", "Threshold Adjustments", 214);
    cv::setTrackbarPos("V max", "Threshold Adjustments", 156);
    cv::setTrackbarPos("L max", "Threshold Adjustments", 130);
    cv::setTrackbarPos("A max", "Threshold Adjustments", 143);
    cv::setTrackbarPos("B max", "Threshold Adjustments", 134);
}

void Img_proc::LINE_imgprocessing()
{
    try
    {
        /* code */
        double tmp_delta_x = 0;
        vector<vector<Point>> contours;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

void Img_proc::webcam_thread()
{
    init();
    // // set node loop rate
    // ros::Rate loop_rate(SPIN_RATE);

    while (cv::waitKey(1) != 27)
    {
        extractAndDisplayObject();
    }

    // Release the camera and close OpenCV windows
    vcap.release();
    cv::destroyAllWindows();
}



// ********************************************** GETTERS ************************************************** //

bool Img_proc::Get_img_proc_line_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_line_det_);
    return img_proc_line_det_;
}

bool Img_proc::Get_img_proc_no_line_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_no_line_det_);
    return img_proc_no_line_det_;
}

bool Img_proc::Get_img_proc_goal_line_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_goal_det_);
    return img_proc_goal_det_;
}

bool Img_proc::Get_img_proc_huddle_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_huddle_det_);
    return img_proc_huddle_det_;
}

bool Img_proc::Get_img_proc_wall_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_wall_det_);
    return img_proc_wall_det_;
}

bool Img_proc::Get_img_proc_stop_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_stop_det_);
    return img_proc_stop_det_;
}


// ********************************************** SETTERS ************************************************** //

void Img_proc::Set_img_proc_line_det(bool img_proc_line_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_line_det_);
    this->img_proc_line_det_ = img_proc_line_det;
}

void Img_proc::Set_img_proc_no_line_det(bool img_proc_no_line_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_no_line_det_);
    this->img_proc_no_line_det_ = img_proc_no_line_det;
}

void Img_proc::Set_img_proc_goal_line_det(bool img_proc_goal_line_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_goal_det_);
    this->img_proc_goal_det_ = img_proc_goal_line_det;
}

void Img_proc::Set_img_proc_huddle_det(bool img_proc_huddle_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_huddle_det_);
    this->img_proc_huddle_det_ = img_proc_huddle_det;
}

void Img_proc::Set_img_proc_wall_det(bool img_proc_wall_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_wall_det_);
    this->img_proc_wall_det_ = img_proc_wall_det;
}

void Img_proc::Set_img_proc_stop_det(bool img_proc_stop_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_stop_det_);
    this->img_proc_stop_det_ = img_proc_stop_det;
}
