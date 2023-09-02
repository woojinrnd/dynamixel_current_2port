#include "img_proc.hpp"

// Constructor
Img_proc::Img_proc()
    : SPIN_RATE(1),
      img_proc_line_det_(false),
      gradient_(0)
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

std::pair<cv::Mat, cv::Mat> Img_proc::extract_color(const cv::Mat &input_frame, const cv::Scalar &lower_bound, const cv::Scalar &upper_bound)
{
    cv::Mat frame = input_frame.clone();
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, lower_bound, upper_bound, mask);

    cv::Mat color_extracted;
    cv::bitwise_and(frame, frame, color_extracted, mask);

    return {color_extracted, frame};
}

std::tuple<cv::Mat, bool, int, int, bool, double> Img_proc::detect_Line_areas(const cv::Mat &input_frame, const cv::Mat &origin_frame, const cv::Scalar &contour_color, int threshold_value, bool check_disappearance, bool is_white_line)
{
    cv::Mat frame = input_frame.clone();
    cv::Mat ori_frame = origin_frame.clone();
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    cv::threshold(gray, binary, threshold_value, max_value, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // cv::cvtColor(gray, frame, cv::COLOR_GRAY2BGR);

    std::vector<cv::Point> top_contour;

    bool foundLargeContour = false;
    double topmost_y = std::numeric_limits<double>::max();
    double distance_huddle = 0;
    bool has_white_now = false;

    float angle = 0;

    bool &has_prev = is_white_line ? has_white_prev : has_yellow_prev;
    cv::Point &center_now = is_white_line ? center_now_white : center_now_yellow;

    for (const auto &contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area > 500)
        {
            cv::Moments m = cv::moments(contour);
            foundLargeContour = true;
            line_condition_count = 0;
            if (m.m00 == 0)
                continue;

            cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
            if (center.y < topmost_y)
            {
                topmost_y = center.y;
                top_contour = contour;
                center_now = center;
                distance_huddle = 480 - topmost_y;
            }
            has_white_now = true;

            // ROS_WARN("LINE_MODE ON");
        }
        else if (area < 500)
        {
            line_condition_count++;
            if (line_condition_count >= 3)
            {
                foundLargeContour = false;
                has_white_now = false;
            }
        }
    }
    cv::putText(ori_frame, "distance : " + std::to_string(distance_huddle), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, contour_color, 2);

    if (check_disappearance)
    {
        if (has_prev && !has_white_now && center_now.x < 320)
        {
            double tmp_delta_x = 1;
            delta_x_ = tmp_delta_x;
            tmp_delta_x = 0;
            std::cout << "Line area disappeared to the left\n";
        }
        else if (has_prev && !has_white_now && center_now.x > 320)
        {
            double tmp_delta_x = -1;
            delta_x_ = tmp_delta_x;
            tmp_delta_x = 0;
            std::cout << "Line area disappeared to the right\n";
        }
    }

    has_prev = has_white_now;

    if (!top_contour.empty())
    {
        cv::RotatedRect min_area_rect = cv::minAreaRect(top_contour);

        float width = min_area_rect.size.width;
        float height = min_area_rect.size.height;

        // std::cout << "Width: " << width << " Height: " << height << std::endl;

        cv::Point2f vertices[4];
        min_area_rect.points(vertices);
        for (int i = 0; i < 4; ++i)
            cv::line(ori_frame, vertices[i], vertices[(i + 1) % 4], contour_color, 3);

        if (min_area_rect.size.width < min_area_rect.size.height)
        {
            angle = -min_area_rect.angle;
        }

        else
        {
            angle = -min_area_rect.angle - 90;
        }

        if (is_white_line && min_area_rect.size.width * 1.5 > min_area_rect.size.height)
        {
            corner_condition_count++;
            if (corner_condition_count >= 3)
            {
                Corner = true;
            }
        }
        else
        {
            corner_condition_count = 0;
        }
        cv::putText(ori_frame, "Line angle : " + std::to_string(angle), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, contour_color, 2);

        cv::Point center_dot(570, 50);

        int length = 50;

        float radian_angle = (angle - 90) * (CV_PI / 180.0f);

        cv::Point end_point(center_dot.x + length * cos(radian_angle), center_dot.y + length * sin(radian_angle));

        cv::line(ori_frame, center_dot, end_point, contour_color, 3);
        // std::cout << "Angle: " << angle << std::endl;
    }
    return std::make_tuple(ori_frame, foundLargeContour, angle, distance_huddle, Corner, delta_x_);
}

void Img_proc::webcam_thread()
{
    // // // TEST
    // Set_img_proc_wall_det(true); 
    // Set_img_proc_corner_number(1);

    // CAMERA INIT
    cv::VideoCapture cap(webcam_id);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, webcam_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, webcam_height);
    cap.set(cv::CAP_PROP_FPS, webcam_fps);
    cap.set(CAP_PROP_AUTOFOCUS, false);
    cap.set(CAP_PROP_AUTO_EXPOSURE, false);

    cap.set(CAP_PROP_EXPOSURE, PROP_EXPOSURE);
    cap.set(CAP_PROP_GAIN, PROP_GAIN);
    cap.set(CAP_PROP_BRIGHTNESS, PROP_BRIGHTNESS);
    cap.set(CAP_PROP_CONTRAST, PROP_CONTRAST);
    cap.set(CAP_PROP_SATURATION, PROP_SATURATION);
    cap.set(CAP_PROP_TEMPERATURE, PROP_TEMPERATURE);

    if (!cap.isOpened())
    {
        std::cerr << "Could not open the webcam\n";
        return;
    }

    // const std::string window_name1 = "hsv Frame_white";
    const std::string window_name2 = "thresh Frame_white";
    // const std::string window_name3 = "hsv Frame_yellow";
    const std::string window_name4 = "thresh Frame_yellow";
    // cv::namedWindow(window_name1);
    cv::namedWindow(window_name2);
    // cv::namedWindow(window_name3);
    cv::namedWindow(window_name4);

    // create_color_range_trackbar(window_name1);
    create_threshold_trackbar_W(window_name2);
    // create_color_range_trackbar(window_name3);
    create_threshold_trackbar_Y(window_name4);

    cv::Mat frame, hsv_frame_white, hsv_frame_yellow, thresh_frame_white, thresh_frame_yellow, gray;

    while (ros::ok())
    {        
        cap >> frame;
        if (frame.empty())
            break;

        auto hsv_frame_white = extract_color(frame, lower_bound_white, upper_bound_white);
        auto hsv_frame_yellow = extract_color(frame, lower_bound_yellow, upper_bound_yellow);
        auto thresh_frame_white = detect_Line_areas(hsv_frame_white.first, frame, green_color, threshold_value_white, true, true);
        auto thresh_frame_yellow = detect_Line_areas(hsv_frame_yellow.first, frame, blue_color, threshold_value_yellow, false, false);
        bool WhiteContourDetected = std::get<1>(thresh_frame_white);
        double gradient = std::get<2>(thresh_frame_white);
        double tmp_delta_x = std::get<5>(thresh_frame_white);

        bool YellowContourDetected = std::get<1>(thresh_frame_yellow);
        bool Corner_mode = std::get<4>(thresh_frame_yellow);


        // this->Set_img_proc_huddle_det(YellowContourDetected);
        // this->Set_img_proc_corner_det(Corner_mode);


        //TEST
        // this->Set_img_proc_huddle_det(true);
        // this->Set_img_proc_corner_det(true);
        // this->Set_img_proc_corner_number(1);
        // this->Set_img_proc_wall_det(true);
        // ROS_WARN("%d",Get_img_proc_huddle_det());
        // this->Set_img_proc_corner_det(YellowContourDetected);
        // if (a == 0)
        // {
        //     this->Set_img_proc_corner_number(a);
        //     a = !a;
        // }
        // else if (a == 1)
        // {
        //     this->Set_img_proc_corner_number(a);
        //     a = !a;
        // }

        this->Set_img_proc_line_det(WhiteContourDetected);

        if (this->Get_img_proc_line_det() == true)
        {
            this->Set_img_proc_no_line_det(false);
            this->Set_gradient(gradient);
        }
        else if (this->Get_img_proc_line_det() == false)
        {
            this->Set_img_proc_no_line_det(true);
            this->Set_gradient(gradient);
            this->Set_delta_x(tmp_delta_x);
        }
        else
        {
            this->Set_gradient(0);
        }

        // cv::imshow("origin", frame);
        // cv::imshow("gray", gray);
        cv::imshow("hsv Frame_white", std::get<0>(thresh_frame_white));
        // cv::imshow("hsv Frame_yellow", hsv_frame_yellow);
        // cv::imshow("thresh Frame_white", thresh_frame_white);
        cv::imshow("hsv Frame_yellow", std::get<0>(thresh_frame_yellow));
        if (cv::waitKey(1) == 27)
            break;
        // loop_rate.sleep();
    }

    vcap.release();
    cv::destroyAllWindows();
}

// // ********************************************** 3D THREAD************************************************** //
 std::tuple<int, float, float> Img_proc::applyPCA(const rs2::depth_frame& depth, int x1, int y1, int x2, int y2, int x3, int y3){
     float z1 = depth.get_distance(x1, y1) * 20;
     float z2 = depth.get_distance(x2, y2) * 20;
     float z3 = depth.get_distance(x3, y3) * 20;

     float distance_rect = depth.get_distance(320, 240);
     float right_plane = depth.get_distance(600, 240);
     float left_plane = depth.get_distance(40, 240);

     bool left_plane_mode = false;
     bool right_plane_mode = false;

     if(right_plane - left_plane > 0.2){left_plane_mode = true;}
     else if(right_plane - left_plane < -0.2){right_plane_mode = true;}

     Eigen::Vector3f v1(x1 - x2, y1 - y2, z1 - z2);
     Eigen::Vector3f v2(x1 - x3, y1 - y3, z1 - z3);

     //std::cout << "v1: " << v1[0] << ", " << v1[1] << ", " << v1[2] << std::endl;
     //std::cout << "v2: " << v2[0] << ", " << v2[1] << ", " << v2[2] << std::endl;


     Eigen::Vector3f normal = v1.cross(v2);
     normal.normalize();

     // 카메라 벡터 정의
     Eigen::Vector3f camera_vector(0, 0, -1);

     // 법선 벡터와 카메라 벡터 사이의 각도 계산
     float dot_product = normal.dot(camera_vector);
     float normal_magnitude = normal.norm();
     float camera_magnitude = camera_vector.norm();
     float cos_theta = dot_product / (normal_magnitude * camera_magnitude);
     float angle_degrees = std::acos(cos_theta) * 180.0 / M_PI;
     float pitch = atan2(normal[1], normal[2]) * 180.0 / M_PI;
     float yaw = atan2(normal[0], sqrt(normal[1] * normal[1] + normal[2] * normal[2])) * 180.0 / M_PI;// 라디안을 도로 변환

     //std::cout << "Angle between normal vector and camera vector: " << angle_degrees << " degrees" << std::endl;
     //std::cout << "normal: " << normal[0] << ", " << normal[1] << ", " << normal[2] << std::endl;
    //  cout << yaw << endl;
    int8_t tmp_img_proc_wall_number = Get_img_proc_wall_number();

     if (distance_rect > 0.6)
     {
        if (tmp_img_proc_wall_number == 0)
        {
            tmp_img_proc_wall_number = 1;
        }
        else if (tmp_img_proc_wall_number == -3)
        {
            tmp_img_proc_wall_number = 10;
        }
     }
     else if (distance_rect > 0.3 && distance_rect < 0.6 && tmp_img_proc_wall_number == 1)
     {
        if (right_plane_mode)
        {
            tmp_img_proc_wall_number = 2;
        }
        else if (left_plane_mode)
        {
            tmp_img_proc_wall_number = -2;
        }
     }
     else if (distance_rect < 0.3)
     {
        if (tmp_img_proc_wall_number == 2)
        {
            tmp_img_proc_wall_number = 3;
        }
        else if (tmp_img_proc_wall_number == -2)
        {
            tmp_img_proc_wall_number = -3;
        }
     }

     return std::make_tuple(tmp_img_proc_wall_number, yaw, distance_rect);
 }

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
            rs2::depth_frame depth_frame = data.get_depth_frame();

            float depth_scale = pipe.get_active_profile().get_device().first<rs2::depth_sensor>().get_depth_scale();
            rs2_intrinsics intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

            const int w = depth.as<rs2::video_frame>().get_width();
            const int h = depth.as<rs2::video_frame>().get_height();

            cv::Mat colorMat(cv::Size(w, h), CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depthMat(cv::Size(w, h), CV_8UC3, (void *)depth.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_dist(cv::Size(w, h), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            Eigen::Vector3f normal_vector;

            // Wall mode
            auto pca = applyPCA(depth_frame, 300, 200, 320, 260, 340, 200);

            int8_t wall_number_ = std::get<0>(pca);
            double angle_ = std::get<1>(pca);
            double distance_ = std::get<2>(pca);

            if (Get_img_proc_wall_det() == true)
            {
                this->Set_img_proc_wall_number(wall_number_);
                this->Set_gradient(angle_);
                this->Set_distance(distance_);
            }

            cv::imshow(window_name, depthMat);
            cv::imshow(window_name_color, colorMat);

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

void Img_proc::saveParameters(const std::string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open configuration file for writing." << std::endl;
        return;
    }

    fs << "HSVParams"
       << "{"
       << "h_min" << static_cast<int>(h_min) << "h_max" << static_cast<int>(h_max)
       << "s_min" << static_cast<int>(s_min) << "s_max" << static_cast<int>(s_max)
       << "v_min" << static_cast<int>(v_min) << "v_max" << static_cast<int>(v_max) << "}";

    fs << "LABParams"
       << "{"
       << "l_min" << static_cast<int>(l_min) << "l_max" << static_cast<int>(l_max)
       << "a_min" << static_cast<int>(a_min) << "a_max" << static_cast<int>(a_max)
       << "b_min" << static_cast<int>(b_min) << "b_max" << static_cast<int>(b_max) << "}";

    fs.release();
}

void Img_proc::loadParameters(const std::string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open configuration file for reading." << std::endl;
        return;
    }

    cv::FileNode hsvParams = fs["HSVParams"];
    h_min = static_cast<int>(hsvParams["h_min"]);
    h_max = static_cast<int>(hsvParams["h_max"]);
    s_min = static_cast<int>(hsvParams["s_min"]);
    s_max = static_cast<int>(hsvParams["s_max"]);
    v_min = static_cast<int>(hsvParams["v_min"]);
    v_max = static_cast<int>(hsvParams["v_max"]);

    cv::FileNode labParams = fs["LABParams"];
    l_min = static_cast<int>(labParams["l_min"]);
    l_max = static_cast<int>(labParams["l_max"]);
    a_min = static_cast<int>(labParams["a_min"]);
    a_max = static_cast<int>(labParams["a_max"]);
    b_min = static_cast<int>(labParams["b_min"]);
    b_max = static_cast<int>(labParams["b_max"]);

    fs.release();
}

void Img_proc::running_process()
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
    // cv::Mat final_binary_mask;
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

    if (contours.empty())
    {
        this->Set_img_proc_no_line_det(true);
        // ROS_ERROR("img_proc_no_line_det : %d", Get_img_proc_no_line_det());
        // cout << "no area" << endl;
    }
    cv::morphologyEx(field, field, cv::MORPH_ERODE, cv::Mat(), cv::Point(-1, -1), 5);
    cv::bitwise_and(field, ~final_binary_mask, field);
    cv::morphologyEx(field, field, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2);

    this->contours_ = contours;
    LINE_imgprocessing();
    GOAL_LINE_recognition();

    // Show the original frame and the final binary mask
    cv::imshow("Original Frame", Origin_img);
    cv::imshow("Final Binary Mask", final_binary_mask);
}

void Img_proc::LINE_imgprocessing()
{
    double tmp_delta_x = 0;

    Mat img_contour_tmp;
    if (!final_binary_mask.empty())
    {
        img_contour_tmp = final_binary_mask.clone();
        // imshow("ddd", img_contour_tmp);
    }
    else
    {
        // Handle the case when final_binary_mask is empty
        ROS_WARN("final_binary_mask is empty");
    }

    //    Parameter Setting Start    //
    /*    setting border line    */

    /*** ROI SETTING ***/

    /*** RR LINE SETTING ***/
    float curvature = RR_LINE_CURVATURE;
    float y_tip_point = Y_VERTEX;
    for (int i = 0; i < IMG_W; ++i)
    {
        float x = i;
        float y = curvature * (x - IMG_W / 2) * (x - IMG_W / 2) + y_tip_point;
        circle(Origin_img, Point(int(x), int(y)), 2, Scalar(255, 0, 0), 2);
    }

    for (int i = 0; i < IMG_H; ++i) // i = y-axis , j = x-axis
    {
        for (int j = 0; j < IMG_W; ++j)
        {
            //// delete top area
            // if (i > BOTTOM_BORDER_LINE || i < TOP_BORDER_LINE)
            //{
            //	img_contour_tmp.at<char>(i, j) = 0;
            //	if (roi_line_flg == true)
            //	{
            //		Origin_img.at<Vec3b>(i, j)[0] = Origin_img.at<Vec3b>(i, j)[0] * 0.5;
            //		Origin_img.at<Vec3b>(i, j)[1] = Origin_img.at<Vec3b>(i, j)[1] * 0.5;
            //		Origin_img.at<Vec3b>(i, j)[2] = Origin_img.at<Vec3b>(i, j)[2] * 0.5;
            //	}
            // }

            // delete curvature line upper area
            if (i < curvature * (j - IMG_W / 2) * (j - IMG_W / 2) + y_tip_point)
            {
                img_contour_tmp.at<char>(i, j) = 0;
                if (roi_line_flg == true)
                {
                    Origin_img.at<Vec3b>(i, j)[0] = Origin_img.at<Vec3b>(i, j)[0] * 0.5;
                    Origin_img.at<Vec3b>(i, j)[1] = Origin_img.at<Vec3b>(i, j)[1] * 0.5;
                    Origin_img.at<Vec3b>(i, j)[2] = Origin_img.at<Vec3b>(i, j)[2] * 0.5;
                }
            }

            else if (i > IMG_H - 10)
            {
                img_contour_tmp.at<char>(i, j) = 0;
                if (roi_line_flg == true)
                {
                    Origin_img.at<Vec3b>(i, j)[0] = Origin_img.at<Vec3b>(i, j)[0] * 0.5;
                    Origin_img.at<Vec3b>(i, j)[1] = Origin_img.at<Vec3b>(i, j)[1] * 0.5;
                    Origin_img.at<Vec3b>(i, j)[2] = Origin_img.at<Vec3b>(i, j)[2] * 0.5;
                }
            }

            
            // delete center-bottom curvature line lower area
            else if (i > 0.015 * (j - IMG_W / 2) * (j - IMG_W / 2) + (IMG_H - CIRCLE_RADIUS))
            {
                img_contour_tmp.at<char>(i, j) = 0;
                if (roi_line_flg == true)
                {
                    Origin_img.at<Vec3b>(i, j)[0] = Origin_img.at<Vec3b>(i, j)[0] * 0.5;
                    Origin_img.at<Vec3b>(i, j)[1] = Origin_img.at<Vec3b>(i, j)[1] * 0.5;
                    Origin_img.at<Vec3b>(i, j)[2] = Origin_img.at<Vec3b>(i, j)[2] * 0.5;
                }
            }

      
            //  delete both side edge area
            else if (j < LEFT_EDGE_BORDER_LINE || j > RIGHT_EDGE_BORDER_LINE)
            {
                img_contour_tmp.at<char>(i, j) = 0;
                if (roi_line_flg == true)
                {
                    Origin_img.at<Vec3b>(i, j)[0] = Origin_img.at<Vec3b>(i, j)[0] * 0.5;
                    Origin_img.at<Vec3b>(i, j)[1] = Origin_img.at<Vec3b>(i, j)[1] * 0.5;
                    Origin_img.at<Vec3b>(i, j)[2] = Origin_img.at<Vec3b>(i, j)[2] * 0.5;
                }
            }
        }
    }
    //    Parameter Setting Ends    //

    //    Image Processing Start    //

    // findContours(img_contour_tmp, this->contours_, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(final_binary_mask, this->contours_, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    int _size = (int)this->contours_.size();

    vector<Moments> _moment(_size);
    vector<Point2f> centerpoints(_size);

    int line_count = 0;
    float x_points[5], y_points[5];

    if (_size > 0)
    {
        for (int i = 0; i < _size; i++)
        {
            if (contourArea(this->contours_[i]) > 50)
            {
                drawContours(Origin_img, this->contours_, i, Scalar(0, 255, 0), 2);

                _moment[i] = moments(this->contours_[i], false);
                centerpoints[i] = Point2f(_moment[i].m10 / _moment[i].m00, _moment[i].m01 / _moment[i].m00);

                if (line_count < 5)
                {
                    x_points[line_count] = centerpoints[i].x;
                    y_points[line_count] = centerpoints[i].y;
                    line_count++;
                }
            }
        }
    }

    string str = to_string(line_count) + " Dot";
    putText(Origin_img, str, Point(10, 30), 2, 0.8, CV_RGB(0, 255, 0), 2);

    if (line_count > 0)
    {
        float min_distance = (IMG_W / 2 * IMG_W / 2) + (TOP_BORDER_LINE - IMG_H) * (TOP_BORDER_LINE - IMG_H);
        int min_distance_index = 0;

        for (int i = 0; i < line_count; i++)
        {
            float tmp_distance = (x_points[i] - IMG_W / 2) * (x_points[i] - IMG_W / 2) + (y_points[i] - IMG_H) * (y_points[i] - IMG_H);
            if (tmp_distance < min_distance)
            {
                min_distance = tmp_distance;
                min_distance_index = i;
            }
        }
        tmp_point_target = Point(x_points[min_distance_index], y_points[min_distance_index]);
        if (abs(tmp_point_target.x - point_target.x) > NOISE_DELETE_DELTA_X)
        {
            tmp_point_target.x = point_target.x;
            tmp_point_target.y = point_target.y;
        }
    }

    if (line_count == 0 || tmp_point_target.x == point_target.x)
    {
        // when no dot found, move slope to center
        if (point_target.x > IMG_W / 2)
        {
            tmp_point_target = Point(point_target.x - 1, point_target.y);
            Set_img_proc_no_line_det(true);
        }
        else if (point_target.x < IMG_W / 2)
        {
            tmp_point_target = Point(point_target.x + 1, point_target.y);
            Set_img_proc_no_line_det(true);
        }
        else
        {
            tmp_point_target = Point(IMG_W / 2, point_target.y);
            // Set_img_proc_no_line_det(false);
        } // == point_target.x = IMG_W/2
    }

    float dydx = (tmp_point_target.y - IMG_H) / (tmp_point_target.x - IMG_W / 2 + 0.0001);
    // ROS_INFO("%f", dydx);
    for (int i = IMG_H; i > 0; i--)
    {
        int y = i;
        int x = 1 / dydx * (y - IMG_H) + IMG_W / 2;
        circle(Origin_img, Point(x, y), 2, Scalar(0, 255, 255), -1);

        if (x < 2)
        {
            tmp_delta_x = IMG_W / 2 - 0;
            break;
        }
        else if (x > IMG_W - 2)
        {
            tmp_delta_x = IMG_W / 2 - IMG_W;
            break;
        }
        else if (abs(curvature * (x - IMG_W / 2) * (x - IMG_W / 2) + y_tip_point - y) < 2)
        {
            circle(Origin_img, Point(x, y), 4, Scalar(0, 255, 255), -1);
            tmp_delta_x = IMG_W / 2 - x;
            break;
        }
    }
    //////////////////////////////////////////////////
    // float dydx = (tmp_point_target.y - IMG_H) / (tmp_point_target.x - IMG_W/2 + 0.0001);
    double base_y = Origin_img.rows - 1; // Bottom of the screen
    double center_x = tmp_point_target.x;
    double center_y = tmp_point_target.y;
    double dx = center_x - (IMG_W / 2);
    double dy = base_y - center_y;
    double angle_rad = std::atan2(dy, dx);          //[rad]
    double angle_deg = angle_rad * (180.0 / CV_PI); // [deg]

    if (center_x < (IMG_W / 2))
        angle_deg -= 90;
    else
        angle_deg = angle_deg - 90;

    // Draw a line connecting the center point at the bottom of the screen and the center of the object
    // std::string strangle_deg = "Angle : " + std::to_string(angle_deg) + " Deg";
    // cv::putText(Origin_img, strangle_deg, cv::Point(IMG_W, IMG_H), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2);

    // ROS_WARN("center x : %f", center_x);
    // ROS_WARN("center y : %f", center_y);
    // ROS_WARN("Angle_deg : %f" , angle_deg);

    //////////////////////////////////////////////////

    circle(Origin_img, tmp_point_target, 4, Scalar(0, 0, 255), -1);
    point_target = tmp_point_target;

    delta_x_list[2] = delta_x_list[1];
    delta_x_list[1] = delta_x_list[0];
    delta_x_list[0] = tmp_delta_x;
    // delta_x = (delta_x_list[2] + delta_x_list[1] + delta_x_list[0]) / 3;
    delta_x_ = tmp_delta_x;

    Set_img_proc_line_det(true);
    Set_gradient(angle_deg);
    Set_delta_x(delta_x_);

    //    image processing ends    //
}

void Img_proc::GOAL_LINE_recognition()
{
    vector<vector<Point>> contours;
    contours = this->contours_;
    Mat img_contour_tmp;
    if (!final_binary_mask.empty())
    {
        img_contour_tmp = final_binary_mask.clone();
    }
    else
    {
        // Handle the case when final_binary_mask is empty
        ROS_WARN("final_binary_mask is empty");
    }
    Point2f corner[4];

    findContours(img_contour_tmp, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    int _size = (int)contours.size();

    vector<Moments> _moment(_size);
    vector<Point2f> centerpoints(_size);
    float x_points[5], y_points[5];

    if (_size > 0)
    {
        for (int i = 0; i < _size; i++)
        {
            ////////////////////////////////////
            ///    1.Draw the minAreaRect    ///
            ////////////////////////////////////

            if (contourArea(contours[i]) > CONTOUR_AREA)
            {
                RotatedRect rect = minAreaRect(contours[i]);

                _moment[i] = moments(contours[i], false);
                centerpoints[i] = Point2f(_moment[i].m10 / _moment[i].m00, _moment[i].m01 / _moment[i].m00);

                if (_size < 1)
                {
                    x_points[_size] = centerpoints[i].x;
                }

                rect.points(corner);

                vector<Point> boundingContour;
                vector<vector<Point>> boxContours;

                for (unsigned int j = 0; j < 4; j++)
                {
                    boundingContour.push_back(corner[j]);
                }

                boxContours.push_back(boundingContour);
                drawContours(Origin_img, boxContours, 0, Scalar(0, 255, 0), 2);

                for (unsigned int k = 0; k < 4; k++)
                {
                    circle(Origin_img, Point2f(corner[k]), 4, Scalar(255, 0, 0), -1);
                }

                /////////////////////////////////////

                double height = rect.size.height; // get height
                double width = rect.size.width;
                int area = height * width;
                string area_str = to_string((int)area);
                double angle = rect.angle;

                string height_str = to_string((int)height);
                string width_str = to_string((int)width);

                int min_goalSize = 50;
                double fake_height;

                // putText(Origin_img2, "H" + height_str + "W" + width_str + "A" + area_str, Point(corner[3]), 2, 0.5, CV_RGB(0, 255, 0), 1);

                /////////////////////////////////////////////////
                ///    2. Set height is larger than width     ///
                /////////////////////////////////////////////////
                if (height >= width)
                {
                    height = height;
                }
                else if (width > height)
                {
                    fake_height = height;
                    height = width;
                    width = fake_height;
                }

                ////////////////////////////////////////////////

                //////////////////////////////////////////////////
                ///    3. Determine the shape of the line      ///
                //////////////////////////////////////////////////
                if ((height > min_goalSize) && (width > min_goalSize))
                {
                    putText(Origin_img, "GOAL LINE", Point(100, 60), 2, 0.8, CV_RGB(255, 0, 0), 2);
                    Set_img_proc_goal_line_det(true);
                }
                else
                {
                    putText(Origin_img, "Driving LINE", Point(100, 30), 2, 0.8, CV_RGB(0, 255, 0), 2);
                    Set_img_proc_goal_line_det(false);
                }
                //////////////////////////////////////////////////
            }
        }
    }
}

void Img_proc::init()
{
    // // set node loop rate
    // ros::Rate loop_rate(SPIN_RATE);
    // // vcap = VideoCapture(CAP_DSHOW + webcam_id);
    // vcap = VideoCapture(webcam_id);
    // vcap.set(cv::CAP_PROP_FRAME_WIDTH, webcam_width);
    // vcap.set(cv::CAP_PROP_FRAME_HEIGHT, webcam_height);
    // vcap.set(cv::CAP_PROP_FPS, webcam_fps);

    // if (!vcap.isOpened())
    // {
    //     std::cerr << "Could not open the webcam\n";
    //     return;
    // }
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

bool Img_proc::Get_img_proc_corner_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_corner_det_);
    return img_proc_corner_det_;
}

int8_t Img_proc::Get_img_proc_corner_number() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_corner_number_);
    return img_proc_corner_number_;
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

int8_t Img_proc::Get_img_proc_wall_number() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_wall_number_);
    return img_proc_wall_number_;
}

bool Img_proc::Get_img_proc_stop_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_stop_det_);
    return img_proc_stop_det_;
}

double Img_proc::Get_gradient() const
{
    std::lock_guard<std::mutex> lock(mtx_gradient);
    return gradient_;
}

double Img_proc::Get_delta_x() const
{
    std::lock_guard<std::mutex> lock(mtx_delta_x);
    return delta_x_;
}

double Img_proc::Get_wall_angle() const
{
    std::lock_guard<std::mutex> lock(mtx_wall_angle);
    return wall_angle_;
}

double Img_proc::Get_distance() const
{
    std::lock_guard<std::mutex> lock(mtx_distance);
    return distance_;
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

void Img_proc::Set_img_proc_corner_det(bool img_proc_corner_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_corner_det_);
    this->img_proc_corner_det_ = img_proc_corner_det;
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

void Img_proc::Set_img_proc_corner_number(int8_t img_proc_corner_number)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_corner_number_);
    this->img_proc_corner_number_ = img_proc_corner_number;
}

void Img_proc::Set_img_proc_wall_number(int8_t img_proc_wall_number)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_wall_number_);
    this->img_proc_wall_number_ = img_proc_wall_number;
}

void Img_proc::Set_img_proc_stop_det(bool img_proc_stop_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_stop_det_);
    this->img_proc_stop_det_ = img_proc_stop_det;
}

void Img_proc::Set_gradient(double gradient)
{
    std::lock_guard<std::mutex> lock(mtx_gradient);
    this->gradient_ = gradient;
}

void Img_proc::Set_delta_x(double delta_x)
{
    std::lock_guard<std::mutex> lock(mtx_delta_x);
    this->delta_x_ = delta_x;
}

void Img_proc::Set_wall_angle(double wall_angle)
{
    std::lock_guard<std::mutex> lock(mtx_wall_angle);
    this->wall_angle_ = wall_angle;
}

void Img_proc::Set_distance(double distance)
{
    std::lock_guard<std::mutex> lock(mtx_distance);
    this->distance_ = distance;
}