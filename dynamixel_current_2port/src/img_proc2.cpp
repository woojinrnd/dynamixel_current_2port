#include "img_proc.hpp"

// Constructor
Img_proc::Img_proc()
    : SPIN_RATE(100),
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

void Img_proc::create_threshold_trackbar_B(const std::string &window_name)
{
    cv::createTrackbar("Threshold_blue", window_name, &threshold_value_blue, max_value, on_trackbar);
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

std::tuple<cv::Mat, cv::Mat> Img_proc::ROI_Line(const cv::Mat &input_frame, const cv::Mat &ori_frame)
{
    cv::Mat draw_frame = ori_frame.clone();
    cv::Mat mask = cv::Mat::zeros(input_frame.size(), CV_8UC1);

    cv::rectangle(mask, cv::Point(100, 480), cv::Point(540, 0), cv::Scalar(255), -1);

    cv::Mat circleMask = cv::Mat::zeros(input_frame.size(), CV_8UC1);

    cv::circle(circleMask, cv::Point(320, 500), 500, cv::Scalar(255), -1);
    cv::bitwise_and(mask, circleMask, mask);

    //cv::line(draw_frame, cv::Point(100, 480), cv::Point(100, 50), white_color, 3);
    //cv::line(draw_frame, cv::Point(540, 480), cv::Point(540, 50), white_color, 3);

    int circle_center_x = 320;
    int circle_center_y = 500;
    int radius = 500;

    for (int x = 100; x <= 540; x++)
    {

        double y_positive = circle_center_y + std::sqrt(radius * radius - (x - circle_center_x) * (x - circle_center_x));
        double y_negative = circle_center_y - std::sqrt(radius * radius - (x - circle_center_x) * (x - circle_center_x));

        //cv::circle(draw_frame, cv::Point(x, y_positive), 1, white_color, 2);
       //cv::circle(draw_frame, cv::Point(x, y_negative), 1, white_color, 2);
    }

    cv::Mat roi;
    ori_frame.copyTo(roi, mask);

    return {roi, draw_frame};
}

cv::Mat Img_proc::ROI_Circle(const cv::Mat &input_frame)
{
    int bigRadius = 500;
    int smallRadius = 500;

    cv::Mat mask = cv::Mat::zeros(input_frame.rows, input_frame.cols, CV_8U);

    // 바깥쪽 원
    cv::ellipse(mask, cv::Point(320, 500), cv::Size(bigRadius, bigRadius), 0, 0, 360, 255, -1);
    // 안쪽 원
    // cv::ellipse(mask, cv::Point(320, 850), cv::Size(smallRadius, smallRadius), 0, 0, 360, 0, -1);
    cv::Mat roi;
    input_frame.copyTo(roi, mask);

    // cv::circle(roi, cv::Point(320, 500), 500, cv::Scalar(0, 0, 255), 2);
    //  cv::circle(roi, cv::Point(320, 850), 500, cv::Scalar(0, 0, 255), 2);

    // cv::imshow("ROI", roi);

    return roi;
}

cv::Mat Img_proc::ROI_Rectangle(const cv::Mat &input_frame, int y_start, int y_end, int x_start, int x_end)
{
    cv::Mat mask = cv::Mat::zeros(input_frame.rows, input_frame.cols, CV_8U);

    cv::rectangle(mask, cv::Point(x_start, y_start), cv::Point(x_end, y_end), 255, -1);

    cv::Mat roi;
    input_frame.copyTo(roi, mask);

    cv::rectangle(roi, cv::Point(x_start, y_start), cv::Point(x_end, y_end), cv::Scalar(0, 0, 255), 2);

    return roi;
}

std::tuple<cv::Mat, cv::Mat> Img_proc::extract_color(const cv::Mat &input_frame, const cv::Scalar &lower_bound, const cv::Scalar &upper_bound)
{
    cv::Mat frame = input_frame.clone();
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, lower_bound, upper_bound, mask);

    cv::Mat color_extracted;
    cv::bitwise_and(frame, frame, color_extracted, mask);

    std::vector<std::vector<cv::Point>> contours;

    return {color_extracted, frame};
}

std::tuple<cv::Mat, bool, int, int, bool, int8_t, cv::Point, cv::Point, cv::Point, int, int, cv::Point, int, std::vector<cv::Point>, int> Img_proc::detect_Line_areas(const cv::Mat &input_frame, const cv::Mat &origin_frame, const cv::Scalar &contour_color, int threshold_value, bool is_yellow_line, bool is_white_line)
{
    cv::Mat frame = input_frame.clone();
    cv::Mat ori_frame = origin_frame.clone();
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    cv::threshold(gray, binary, threshold_value, max_value, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> top_contour;

    bool foundLargeContour = false;
    double topmost_y = std::numeric_limits<double>::max();
    double distance_huddle = 0;
    bool has_white_now = false;
    bool has_yellow_now = false;

    float SM_angle = 0;
    float Rnd_angle = 0;
    float huddle_angle = 0;
    float Line_Angle = 0;
    float corner_angle = 0;
    int line_area = 0;
    int huddle_area = 0;
    int blue_area = 0;

    cv::Point corner_center = cv::Point(0, 0);
    cv::Point huddle_center = cv::Point(0, 0);

    cv::Point top_center, bottom_center, left_center, right_center;

    bool &has_prev = is_white_line ? has_white_prev : has_yellow_prev;
    cv::Point &center_now = is_white_line ? center_now_white : center_now_yellow;

    for (const auto &contour : contours)
    {
        /// LINE
        if (is_white_line)
        {
            line_area = cv::contourArea(contour);
        }
        else if (is_yellow_line)
        {
            huddle_area = cv::contourArea(contour);
        }

        if (line_area > LINE_AREA || huddle_area > HUDDLE_AREA || (!is_white_line && !is_yellow_line))
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
        }
        else if (line_area < LINE_AREA)
        {
            line_condition_count++;
            if (line_condition_count >= 15)
            {
                foundLargeContour = false;
                has_white_now = false;
            }
        }
    }

    if (!top_contour.empty())
    {
        top_contour_area = cv::contourArea(top_contour);
        cv::line(ori_frame, center_now, cv::Point(320, 480), contour_color, 2);

        float deltaY = center_now.y - 480;
        float deltaX = center_now.x - 320;

        float radians = atan2(deltaY, deltaX);

        float adjustedAngle = radians * (180.0 / CV_PI);

        Rnd_angle = -90 - adjustedAngle;
    }

    if (is_white_line)
    {
        if (has_prev && !has_white_now && center_now.x < 320)
        {
            int8_t tmp_delta_x = 1;
            delta_x_ = tmp_delta_x;
            tmp_delta_x = 0;
            std::cout << "Line area disappeared to the left\n";
        }
        else if (has_prev && !has_white_now && center_now.x > 320)
        {
            int8_t tmp_delta_x = -1;
            delta_x_ = tmp_delta_x;
            tmp_delta_x = 0;
            std::cout << "Line area disappeared to the right\n";
        }
    }

    has_prev = has_white_now;

    if (!top_contour.empty())
    {
        std::vector<cv::Point> approx;
        double epsilon = 0.02 * cv::arcLength(top_contour, true);
        cv::approxPolyDP(top_contour, approx, epsilon, true);

        int numVertices = approx.size(); // 근사화된 컨투어의 꼭지점 수를 얻음

        cv::RotatedRect min_area_rect = cv::minAreaRect(top_contour);

        float width = min_area_rect.size.width;
        float height = min_area_rect.size.height;

        float long_len = 0;
        float short_len = 0;

        if (width > height)
        {
            long_len = width;
            short_len = height;
        }
        else if (width < height)
        {
            long_len = height;
            short_len = width;
        }

        // std::cout << "Width: " << width << " Height: " << height << std::endl;

        cv::Point2f vertices[4];
        min_area_rect.points(vertices);

        for (int i = 0; i < 4; ++i)
            cv::line(ori_frame, vertices[i], vertices[(i + 1) % 4], contour_color, 3);

        cv::Rect bbox = min_area_rect.boundingRect();
        bbox &= cv::Rect(0, 0, ori_frame.cols, ori_frame.rows);
        cv::Mat cropped = ori_frame(bbox);

        int croppedWidth = cropped.cols;
        int croppedHeight = cropped.rows;

        if (is_white_line)
        {
            // Line angle
            if (short_len * 1.2 < long_len && numVertices > 3 && numVertices < 5)
            {
                if (min_area_rect.size.width < min_area_rect.size.height)
                {
                    SM_angle = -min_area_rect.angle;
                }
                else
                {
                    SM_angle = -min_area_rect.angle + 90;
                }
                cv::imshow("white binary", binary);
                cv::moveWindow("white binary", 700, 540);

                corner_count = 0;
                Corner = false;
                // cout << "white area" << line_area << endl;
            }

            // Corner angle
            else if (numVertices >= 6 && numVertices <= 9)
            {

                cv::polylines(ori_frame, approx, true, cv::Scalar(0, 255, 255), 2);

                std::sort(approx.begin(), approx.end(), [](const cv::Point &a, const cv::Point &b)
                          { return a.y < b.y; });

                cv::Point topmost1 = approx[0];
                cv::Point topmost2 = approx[1];

                cv::Point bottommost1 = approx[approx.size() - 1];
                cv::Point bottommost2 = approx[approx.size() - 2];

                top_center = (topmost1 + topmost2) / 2;
                bottom_center = (bottommost1 + bottommost2) / 2;

                std::sort(approx.begin(), approx.end(), [](const cv::Point &a, const cv::Point &b)
                          { return a.x < b.x; });

                cv::Point leftmost1 = approx[0];
                cv::Point leftmost2 = approx[1];

                cv::Point rightmost1 = approx[approx.size() - 1];
                cv::Point rightmost2 = approx[approx.size() - 2];

                left_center = (leftmost1 + leftmost2) / 2;
                right_center = (rightmost1 + rightmost2) / 2;

                if (croppedWidth > croppedHeight)
                {
                    double raw_dy = right_center.y - left_center.y;
                    double raw_dx = right_center.x - left_center.x;

                    double raw_angle_rad = std::atan2(raw_dy, raw_dx);
                    double raw_angle_deg = raw_angle_rad * (180.0 / CV_PI);

                    corner_angle = -raw_angle_deg;
                    corner_center = (left_center + right_center) / 2;

                    cv::line(ori_frame, left_center, right_center, red_color, 3);
                    corner_count++;

                    if (corner_count > 15)
                    {
                        Corner = true; // corner shape : ㅜ
                    }

                    else
                    {
                        Corner = false;
                    }
                }
                else if (croppedWidth < croppedHeight)
                {
                    double col_dy = bottom_center.y - top_center.y;
                    double col_dx = bottom_center.x - top_center.x;

                    double col_angle_rad = std::atan2(col_dy, col_dx);
                    double col_angle_deg = col_angle_rad * (180.0 / CV_PI);

                    corner_angle = 90 - col_angle_deg;
                    corner_center = (top_center + bottom_center) / 2;

                    cv::line(ori_frame, top_center, bottom_center, red_color, 3);
                    corner_count++;

                    if (corner_count > 15)
                    {
                        Corner = true; // corner shape : ㅓ
                    }

                    else
                    {
                        Corner = false;
                    }
                }
                cv::circle(ori_frame, corner_center, 10, contour_color, -1);
            }
        }

        else if (is_yellow_line)
        {
            if (short_len * 1.5 < long_len)
            {
                if (min_area_rect.size.width < min_area_rect.size.height)
                {
                    huddle_angle = -min_area_rect.angle + 90;
                }
                else
                {
                    huddle_angle = -min_area_rect.angle;
                }
                huddle_center = min_area_rect.center;
                // cout << top_contour_area << endl;
                cv::circle(ori_frame, huddle_center, 2, contour_color, -1, 8);
                //cv::imshow("yellow binary", binary);
                //cv::moveWindow("yellow binary", 1000, 540);
                // cout << "yellow area" << huddle_area << endl;
            }
        }

        if (Rnd_angle > 0)
        {
            Line_Angle = (SM_angle + Rnd_angle) * 0.5;
        }
        else if (Rnd_angle <= 0)
        {
            Line_Angle = (SM_angle + (3 * Rnd_angle));
        }

        //--------------------------------------------------------------------- frame interface -------------------------------------------------------------------------

        if (has_white_now)
        {
            cv::putText(ori_frame, "MODE : " + Str_LINE_MODE, cv::Point(webcam_width * 0.5 + 50, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, contour_color, 2);
        }
        if (!has_white_now)
        {
            cv::putText(ori_frame, "MODE : " + Str_NO_LINE_MODE, cv::Point(webcam_width * 0.5 + 50, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, contour_color, 2);
        }

        cv::putText(ori_frame, "Rnd_Angle : " + std::to_string(Rnd_angle), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2);
        cv::putText(ori_frame, "SM_angle : " + std::to_string(SM_angle), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2);
        cv::putText(ori_frame, "Line angle : " + std::to_string(SM_angle + Rnd_angle), cv::Point(10, 75), cv::FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2);
        cv::putText(ori_frame, "Vertice : " + std::to_string(numVertices), cv::Point(10, 300), cv::FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2);
        cv::putText(ori_frame, "Corner Angle : " + std::to_string(corner_angle), cv::Point(10, 125), cv::FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2);
        cv::putText(ori_frame, "Area : " + std::to_string(top_contour_area), cv::Point(10, 455), cv::FONT_HERSHEY_SIMPLEX, 0.7, contour_color, 2);
        cv::putText(ori_frame, "Hurdle Angle: " + std::to_string(huddle_angle), cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.7, contour_color, 2);

        topmost_point = *std::min_element(top_contour.begin(), top_contour.end(),
                                          [](const cv::Point &a, const cv::Point &b)
                                          {
                                              return a.y < b.y;
                                          });

        bottommost_point = *std::max_element(top_contour.begin(), top_contour.end(),
                                             [](const cv::Point &a, const cv::Point &b)
                                             {
                                                 return a.y < b.y;
                                             });

        // cout << corner_count << endl;
        // cout << line_area << "    " << huddle_area << endl;
        // cv::putText(ori_frame, "point : " + std::to_string(topmost_point.y), cv::Point(10, 175), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar{0, 255, 0}, 2);
        // cv::putText(ori_frame, "point : " + std::to_string(bottommost_point.y), cv::Point(100, 175), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar{0, 255, 0}, 2);

        // cv::circle(ori_frame, topmost_point, 5, cv::Scalar(0, 0, 255), -1);
        // cv::circle(ori_frame, bottommost_point, 5, cv::Scalar(0, 255, 255), -1);
    }
    return std::make_tuple(ori_frame, foundLargeContour, Line_Angle, distance_huddle, Corner, delta_x_, topmost_point, bottommost_point, corner_center, huddle_angle, line_area, huddle_center, corner_angle, top_contour, huddle_area);
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

    // Threshold Trackbar (White / Yellow / Blue)
    const std::string window_name_thresh_white = "thresh Frame_white";
    const std::string window_name_thresh_blue = "thresh Frame_blue";

    cv::namedWindow(window_name_thresh_white);
    cv::namedWindow(window_name_thresh_blue);

    create_threshold_trackbar_W(window_name_thresh_white);
    create_threshold_trackbar_B(window_name_thresh_blue);

    const std::string window_name_thresh_yellow = "thresh Frame_yellow";
    cv::namedWindow(window_name_thresh_yellow);
    create_threshold_trackbar_Y(window_name_thresh_yellow);

    cv::Mat frame, hsv_frame_white, hsv_frame_yellow, thresh_frame_white, thresh_frame_yellow, gray;

    int White_count = 0;
    int Yellow_count = 0;
    int noline_count = 0;

    while (ros::ok())
    {
        cap >> frame;
        if (frame.empty())
            break;

        cv::Mat Line_frame = frame.clone();

        auto Roi_Line = ROI_Line(Line_frame, frame);

        cv::Mat Roi_huddle = ROI_Rectangle(frame, 0, 480, IMG_W / 2 - 100, IMG_W / 2 + 100);

        // HSV
        auto hsv_frame_white = extract_color(std::get<0>(Roi_Line), lower_bound_white, upper_bound_white);
        auto hsv_frame_yellow = extract_color(Roi_huddle, lower_bound_yellow, upper_bound_yellow);
        auto hsv_frame_blue = extract_color(frame, lower_bound_blue, upper_bound_blue);


        // Draw
        auto thresh_frame_yellow = detect_Line_areas(std::get<0>(hsv_frame_yellow), frame, yellow_color, threshold_value_yellow, true, false);
        auto thresh_frame_blue = detect_Line_areas(std::get<0>(hsv_frame_blue), frame, blue_color, threshold_value_blue, false, false);
        auto thresh_frame_white = detect_Line_areas(std::get<0>(hsv_frame_white), frame, white_color, threshold_value_white, false, true);

        // Filled Area
        int WhiteColorDetected = 0;
        int YellowColorDetected = 0;


        WhiteColorDetected = std::get<10>(thresh_frame_white);
        YellowColorDetected = std::get<14>(thresh_frame_yellow);

        if (YellowColorDetected > HUDDLE_AREA)
        {
            Yellow_count++;
            if(Yellow_count > 1)
            {
                noline_count = 0;
                // this->Set_img_proc_line_det(false);
                this->Set_img_proc_huddle_det_2d(true);

                double gradient = std::get<2>(thresh_frame_yellow);
                this->Set_gradient(gradient);

                double huddle_angle_ = std::get<9>(thresh_frame_yellow);
                Set_huddle_angle(huddle_angle_);

                cv::Point huddle_center = std::get<11>(thresh_frame_yellow);
                // cout << "center_huddle.x : " << huddle_center.x << endl;
                // cout << "center_huddle.y : " << huddle_center.y << endl;
                cv::circle(std::get<0>(thresh_frame_yellow), huddle_center, 2, CV_RGB(0, 255, 255), -1);

                std::vector<cv::Point> top_yellow_contour = std::get<13>(thresh_frame_yellow);

                cv::Point foot_top_point = std::get<6>(thresh_frame_blue);
                cv::Point huddle_bottom_point = std::get<7>(thresh_frame_yellow);
                cv::circle(std::get<0>(thresh_frame_yellow), huddle_bottom_point, 5, cv::Scalar(0, 0, 255), -1);

                int minDistance = std::numeric_limits<int>::max();
                cv::Point2f closestPoint;

                for (const auto &point : top_yellow_contour)
                {
                    int distance = static_cast<int>(cv::norm(foot_top_point - point));
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        closestPoint = point;
                    }
                }

                cv::line(std::get<0>(thresh_frame_yellow), foot_top_point, closestPoint, cv::Scalar(0, 0, 255), 3);
                cv::putText(std::get<0>(thresh_frame_yellow), "Distance: " + std::to_string(minDistance), cv::Point(10, 175), cv::FONT_HERSHEY_SIMPLEX, 0.7, yellow_color, 2);

                int _foot_huddle_distance = std::abs(foot_top_point.y - huddle_bottom_point.y);
                Set_foot_huddle_distance(_foot_huddle_distance);
                // cout << foot_huddle_distance << endl;

                if (_foot_huddle_distance < HUDDLE_Y_MARGIN)
                {
                    Set_contain_huddle_to_foot(true);
                }

                else
                {
                    Set_contain_huddle_to_foot(false);
                }
                White_count = 0;

                cv::imshow("hsv Frame_yellow", std::get<0>(thresh_frame_yellow));
                cv::moveWindow("hsv Frame_yellow", 700, 0);
            }
        }
        else if(YellowColorDetected < HUDDLE_AREA)
        {
            this->Set_img_proc_huddle_det_2d(false);
        }

        if (WhiteColorDetected > LINE_AREA)
        {
            White_count++;
            if(White_count > 30)
            {
                noline_count = 0;
                this->Set_img_proc_line_det(true);
                this->Set_img_proc_no_line_det(false);
                bool WhiteContourDetected = std::get<1>(thresh_frame_white);
                double gradient = std::get<2>(thresh_frame_white);
                double corner_angle = std::get<12>(thresh_frame_white);
                double tmp_delta_x = std::get<5>(thresh_frame_white);

                // corner mode
                bool Corner_mode = std::get<4>(thresh_frame_white);
                // ROS_ERROR("Corner_mode : %d", Corner_mode);
                if (Corner_mode)
                {
                    Set_img_proc_corner_det_2d(true);
                    Set_corner_angle(corner_angle);
                    cv::Point foot_top_point = std::get<6>(thresh_frame_blue);
                    cv::Point corner_bottom_point = std::get<7>(thresh_frame_white);
                    int foot_corner_distance = std::abs(foot_top_point.y - corner_bottom_point.y);
                    cv::Point corner_center = std::get<8>(thresh_frame_white);

                    // Corner is RIGHT side of robot -> Need to move RIGHT side
                    if (corner_center.x > IMG_W / 2 + CORNER_X_MARGIN)
                    {
                        Set_delta_x(-10); // (-) Right side // 10 : dummy variable
                    }

                    // Corner is LEFT side of robot -> Need to move LEFT side
                    else if (corner_center.x < IMG_W / 2 - CORNER_X_MARGIN)
                    {
                        Set_delta_x(10); // (+) Right side // 10 : dummy variable
                    }

                    else if (IMG_W / 2 - CORNER_X_MARGIN <= corner_center.x <= IMG_W / 2 + CORNER_X_MARGIN)
                    {
                        Set_delta_x(0);
                    }

                    // cout << foot_corner_distance << endl;
                    // cout << Get_delta_x() << endl;
                    // cout << gradient << endl;

                    if (foot_corner_distance < CORNER_Y_MARGIN)
                    {
                        Set_contain_corner_to_foot(true);
                    }

                    else
                    {
                        Set_contain_corner_to_foot(false);
                    }
                }

                else
                {
                    Set_img_proc_corner_det_2d(false);
                }

                if (this->Get_img_proc_line_det() == true)
                {
                    this->Set_img_proc_no_line_det(false);
                    this->Set_gradient(gradient);
                }
                else
                {
                    this->Set_gradient(0);
                }
                Yellow_count = 0;

                cv::imshow("hsv Frame_blue", std::get<0>(thresh_frame_blue));
                cv::imshow("hsv Frame_white", std::get<0>(thresh_frame_white));

                cv::moveWindow("hsv Frame_white", 0, 0);
                cv::moveWindow("hsv Frame_blue", 0, 540);
            }
        }
        
        else if(WhiteColorDetected < LINE_AREA && YellowColorDetected < HUDDLE_AREA)
        {
            noline_count++;
            if(noline_count > 15)
            {
                double gradient = std::get<2>(thresh_frame_white);
                double tmp_delta_x = std::get<5>(thresh_frame_white);
                
                this->Set_img_proc_no_line_det(true);
                this->Set_img_proc_line_det(false);
                if (this->Get_img_proc_line_det() == false)
                {
                    this->Set_gradient(gradient);
                    this->Set_delta_x(tmp_delta_x);
                }
            }
        }

        // Line mode / Corner mode

        if (cv::waitKey(1) == 27)
            break;
        // loop_rate.sleep();
    }

    vcap.release();
    cv::destroyAllWindows();
}

// // ********************************************** 3D THREAD************************************************** //

Eigen::Vector3f Img_proc::calculateNormalVector(const std::vector<Eigen::Vector3f>& points) {
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto& point : points) {
        centroid += point;
    }
    centroid /= points.size();

    Eigen::MatrixXf centeredPoints(points.size(), 3);
    for (size_t i = 0; i < points.size(); ++i) {
        centeredPoints.row(i) = points[i] - centroid;
    }

    Eigen::Matrix3f covarianceMatrix = centeredPoints.transpose() * centeredPoints / points.size();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covarianceMatrix);
    Eigen::Vector3f normal = eigensolver.eigenvectors().col(0);

    return normal;
}

// 카메라와 평면 사이 각도 추출 -----------------------------------------------------------------------------------------

float Img_proc::calculateYRotationBetweenVectors(const Eigen::Vector3f& vec1, const Eigen::Vector3f& vec2) {
    // Project the vectors onto the XZ plane
    Eigen::Vector3f vec1_xz = vec1.normalized();
    vec1_xz.y() = 0;

    Eigen::Vector3f vec2_xz = vec2.normalized();
    vec2_xz.y() = 0;

    float dot_product = vec1_xz.dot(vec2_xz);
    float cross_product = vec1.x() * vec2.z() - vec1.z() * vec2.x();
    float vec1_xz_magnitude = vec1_xz.norm();
    float vec2_xz_magnitude = vec2_xz.norm();
    float cos_theta = dot_product / (vec1_xz_magnitude * vec2_xz_magnitude);
    float angle = std::acos(cos_theta) * 180.0 / M_PI;

    // Determine the angle's sign based on the cross product's sign
    if (cross_product < 0) {
        angle = angle;
    }
    if (angle > 90){
       angle = angle - 180;
       angle = angle;
    }


    return angle;
}

// PCA 평면과의 거리 추출 ------------------------------------------------------------------------------------------------

float Img_proc::calculateDistanceFromPlaneToCamera(const Eigen::Vector3f& normal, const Eigen::Vector3f& centroid) {
    Eigen::Vector3f camera_position(0, 0, 0);
    float distance = normal.dot(camera_position - centroid);
    return abs(distance);
}

std::tuple<bool, float, float> Img_proc::applyPCA(cv::Mat &colorMat, cv::Mat depthMat, float depth_scale, cv::Mat depth_dist, rs2::depth_frame depth_frame, rs2_intrinsics intr, int startX, int startY, int endX, int endY)
{
    cv::Mat gray, wall_binary;
    cv::cvtColor(depthMat, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, wall_binary, 240, 255, cv::THRESH_BINARY);

    cv::Mat wall_plane_det_img = ROI_Rectangle(wall_binary, 150, 350, 0, 848);

    imshow("plane_detect", wall_binary);

    float real_distance = 0;

    std::vector<Eigen::Vector3f> points;

    int plane_rect_x1 = 350, plane_rect_y1 = 250, plane_rect_x2 = 498, plane_rect_y2 = 350;

    for (int y = plane_rect_y1; y < plane_rect_y2; ++y) {
        for (int x = plane_rect_x1; x < plane_rect_x2; ++x) {
            float depth = depth_dist.at<uint16_t>(y, x) * depth_scale;

            // Set a depth threshold
            if (depth > 0.2 && depth < 2.0) {
                float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
                float point[3];
                rs2_deproject_pixel_to_point(point, &intr, pixel, depth);

                points.emplace_back(Eigen::Vector3f(point[0], point[1], point[2]));
            }
        }
    }

    // PCA 영역안 평면 거리 ------------------------------------------------------------------------------------------------

    Eigen::Vector3f normal = calculateNormalVector(points);
    //std::cout << "Normal vector: " << normal.transpose() << std::endl;

    Eigen::Vector3f camera_z_axis(0, 0, -1);
    float angle = calculateYRotationBetweenVectors(normal, camera_z_axis);
    //std::cout << "Angle between normal vector and camera Z-axis: " << angle << " degrees" << std::endl;

    std::vector<Eigen::Vector3f> points_rect1;
    for (int y = 150; y < 320; ++y) {
        for (int x = 200; x < 400; ++x) {
            float depth = depth_dist.at<uint16_t>(y, x) * depth_scale;
            if (depth > 0.2 && depth < 2.0) {
                float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
                float point[3];
                rs2_deproject_pixel_to_point(point, &intr, pixel, depth);
                points_rect1.emplace_back(Eigen::Vector3f(point[0], point[1], point[2]));
            }
        }
    }
    Eigen::Vector3f normal_rect1 = calculateNormalVector(points_rect1);
    Eigen::Vector3f centroid_rect1 = Eigen::Vector3f::Zero();
    for (const auto& point : points_rect1) {
        centroid_rect1 += point;
    }
    centroid_rect1 /= points_rect1.size();

    float distance_rect1 = calculateDistanceFromPlaneToCamera(normal_rect1, centroid_rect1);
    float distance_right_wall = depth_frame.get_distance(828, 240);
    float distance_left_wall = depth_frame.get_distance(20, 240);
    float distance_wall_center = depth_frame.get_distance(424, 240);

    if(distance_rect1 > 0.5 && distance_rect1 < 0.7)
    {
        int center_x = wall_plane_det_img.cols / 2;
        int left_white_pixels = 0;
        int right_white_pixels = 0;

        for (int y = 0; y < wall_plane_det_img.rows; y++)
        {
            for (int x = 0; x < center_x; x++)
            {
                if (wall_plane_det_img.at<uchar>(y, x) == 255)  // 왼쪽 영역 흰색 픽셀 확인
                    left_white_pixels++;
            }

            for (int x = center_x; x < wall_plane_det_img.cols; x++)
            {
                if (wall_plane_det_img.at<uchar>(y, x) == 255)  // 오른쪽 영역 흰색 픽셀 확인
                    right_white_pixels++;
            }
        }

        if (right_white_pixels * 1.5 < left_white_pixels)
        {
            if(consecutive_changes >= CHANGE_THRESHOLD)
            {
                plane_direction = true;
                consecutive_changes = 0;
            }
            else
            {
                consecutive_changes++;
            }
        }
        else if(right_white_pixels > left_white_pixels * 1.5)
        {
            if(consecutive_changes >= CHANGE_THRESHOLD)
            {
                plane_direction = false;
                consecutive_changes = 0;
            }
            else
            {
                consecutive_changes++;
            }
        }

        prev_plane_direction = plane_direction;

    }

//    if(distance_rect1 > (distance_right_wall * 2))
//    {
//        real_distance = distance_right_wall;
//    }
//    else if(distance_rect1 > (distance_left_wall * 2))
//    {
//        real_distance = distance_left_wall;
//    }
//    else
//    {
//        real_distance = distance_rect1;
//    }

    real_distance = distance_wall_center;

    if (abs(real_distance - previous_real_distance) > DISTANCE_THRESHOLD) {
        change_counter++;
        if (change_counter < FRAME_THRESHOLD) {
            // 연속된 변화가 FRAME_THRESHOLD보다 작으면 이전 값을 유지
            real_distance = previous_real_distance;
        } else {
            // FRAME_THRESHOLD 이상의 연속된 변화가 있으면 값을 업데이트
            previous_real_distance = real_distance;
            change_counter = 0;
        }
    }
    else
    {
        // 임계값 이내의 변화가 감지되면 카운터 초기화 및 이전 값을 업데이트
        previous_real_distance = real_distance;
        change_counter = 0;
    }

    if (std::isnan(angle)) {
        angle = previous_angle;
    } else {
        previous_angle = angle;
    }

    // real_distance 값이 nan인 경우 이전 값을 사용
    if (std::isnan(real_distance)) {
        real_distance = previous_real_distance;
    } else {
        previous_real_distance = real_distance;
    }

    cv::putText(colorMat, "distance : " + std::to_string(real_distance), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar{0, 255, 0}, 2);
    cv::putText(colorMat, "Angle : " + std::to_string(-angle), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar{0, 255, 0}, 2);
    cv::putText(colorMat, "plane : " + std::to_string(plane_direction), cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar{0, 255, 0}, 2);

    cv::circle(colorMat, cv::Point(828, 240), 5, red_color, -1);
    cv::circle(colorMat, cv::Point(20, 240), 5, red_color, -1);

    cv::rectangle(colorMat, {plane_rect_x1, plane_rect_y1}, {plane_rect_x2, plane_rect_y2}, cv::Scalar(0, 0, 255), 2);


    return std::make_tuple(plane_direction, -angle, real_distance);
}

double Img_proc::Distance_Point(const rs2::depth_frame &depth, cv::Point center)
{
    double Distance_Point = depth.get_distance(center.x, center.y);

    double Real_distance = Distance_Point * Distance_Point - Robot_Height_Cam * Robot_Height_Cam;

    Real_distance = std::sqrt(Real_distance);

    return Real_distance;
}

void Img_proc::realsense_thread()
{
    rs2::colorizer color_map;
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, realsense_width, realsense_height, RS2_FORMAT_BGR8, realsense_color_fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, realsense_width, realsense_height, RS2_FORMAT_Z16, realsense_depth_fps);

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

    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::spatial_filter spatial;
    rs2::temporal_filter temporal;
    rs2::hole_filling_filter hole_filling;

    try
    {
        while (ros::ok() && cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
        {
            rs2::frameset data = pipe.wait_for_frames();
            data = align_to.process(data);

            rs2::depth_frame depth_frame = data.get_depth_frame();

            // depth_frame = spatial.process(depth_frame);
            // depth_frame = temporal.process(depth_frame);
            depth_frame = hole_filling.process(depth_frame);

            rs2::frame depth = depth_frame;
            rs2::frame color = data.get_color_frame();

            color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);

            float depth_scale = pipe.get_active_profile().get_device().first<rs2::depth_sensor>().get_depth_scale();
            rs2_intrinsics intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

            const int w = depth.as<rs2::video_frame>().get_width();
            const int h = depth.as<rs2::video_frame>().get_height();

            cv::Mat colorMat(cv::Size(w, h), CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depthMat(cv::Size(w, h), CV_8UC3, (void *)depth.apply_filter(color_map).get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_dist(cv::Size(w, h), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            Eigen::Vector3f normal_vector;

            // Wall mode
            auto pca = applyPCA(colorMat, depthMat, depth_scale, depth_dist, depth_frame, intrinsics, 300, 200, 350, 400);

            bool plane_direction = std::get<0>(pca);
            double angle_ = std::get<1>(pca);
            double distance_ = std::get<2>(pca);

            // this->Set_img_proc_wall_number(plane_direction);
            this->Set_plane_mode(plane_direction);
            this->Set_wall_angle(angle_);
            this->Set_wall_distance(distance_);

            cv::imshow(window_name, depthMat);
            cv::imshow(window_name_color, colorMat);

        }
    }

    catch (const rs2::error &e)
    {
        std::cerr << "An error occurred during streaming: " << e.what() << std::endl;
    }
}

// *******************************************************************REFERENCE*****************************************************************************//

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
            this->Set_img_proc_no_line_det(false);
            // ROS_WARN("img_proc_line_det : %d", Get_img_proc_line_det());
        }

        else if (500 < area_max < 2000)
        {
            this->Set_img_proc_line_det(false);
            this->Set_img_proc_no_line_det(true);
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
        this->Set_img_proc_line_det(false);
        // ROS_ERROR("img_proc_no_line_det : %d", Get_img_proc_no_line_det());
        // cout << "no area" << endl;
    }
    cv::morphologyEx(field, field, cv::MORPH_ERODE, cv::Mat(), cv::Point(-1, -1), 5);
    cv::bitwise_and(field, ~final_binary_mask, field);
    cv::morphologyEx(field, field, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2);

    this->contours_ = contours;
    LINE_imgprocessing();
    // GOAL_LINE_recognition();

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

    if (Get_img_proc_line_det())
    {
        string str = to_string(line_count) + " Dot";
        putText(Origin_img, str, Point(10, 30), 2, 0.8, CV_RGB(0, 255, 0), 2);
    }

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
            // this->Set_img_proc_no_line_det(true);
            // this->Set_img_proc_line_det(false);
        }
        else if (point_target.x < IMG_W / 2)
        {
            tmp_point_target = Point(point_target.x + 1, point_target.y);
            // this->Set_img_proc_no_line_det(true);
            // this->Set_img_proc_line_det(false);
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
    std::string strangle_deg = "Angle : " + std::to_string(angle_deg) + " Deg";
    cv::putText(Origin_img, strangle_deg, cv::Point(IMG_W, IMG_H), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2);

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

    // Set_img_proc_line_det(true);
    Set_gradient(angle_deg);

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
    // set node loop rate
    ros::Rate loop_rate(SPIN_RATE);
    // vcap = VideoCapture(CAP_DSHOW + webcam_id);
    vcap = VideoCapture(webcam_id);
    vcap.set(cv::CAP_PROP_FRAME_WIDTH, IMG_W);
    vcap.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_H);
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
    cv::setTrackbarPos("H max", "Threshold Adjustments", 235);

    cv::setTrackbarPos("S min", "Threshold Adjustments", 131);
    cv::setTrackbarPos("S max", "Threshold Adjustments", 214);

    cv::setTrackbarPos("V min", "Threshold Adjustments", 60);
    cv::setTrackbarPos("V max", "Threshold Adjustments", 156);

    cv::setTrackbarPos("L min", "Threshold Adjustments", 16);
    cv::setTrackbarPos("L max", "Threshold Adjustments", 151);

    cv::setTrackbarPos("A min", "Threshold Adjustments", 115);
    cv::setTrackbarPos("A max", "Threshold Adjustments", 177);

    cv::setTrackbarPos("B min", "Threshold Adjustments", 66);
    cv::setTrackbarPos("B max", "Threshold Adjustments", 173);
}

double Img_proc::Calc_angle(double _x, Point _pt)
{
    // Calculate the vertical and horizontal differences
    double _dy = (double)_pt.y - 119.5;
    double _dx = (double)_pt.x - 159.5;

    // Normalize the differences based on the zoom factor
    _dy /= 100 * 0.01;
    _dx /= 100 * 0.01;

    // Undo the normalization by adding the initial values
    _dy += (IMG_H / 2 - 1);
    _dx += (IMG_W / 2 - 1);

    // Normalize the horizontal difference '_x'
    _x -= (IMG_W / 2 - 1);
    _x /= 100 * 0.01;
    _x += (IMG_W / 2 - 1);

    // Adjust the vertical position
    _dy = (IMG_H - 1) - _dy;

    // Calculate the angle in degrees using the arctan function
    double _rad2deg = 180.0 / M_PI;
    double _degree = atan(_dx / _dy) * _rad2deg * 0.5;

    return _degree;
}

void Img_proc::webcam_thread2()
{
    init();
    // // set node loop rate
    // ros::Rate loop_rate(SPIN_RATE);

    while (cv::waitKey(1) != 27)
    {
        running_process();
        // LINE_imgprocessing();
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

bool Img_proc::Get_img_proc_corner_det_3d() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_corner_det_3d);
    return img_proc_corner_det_3d_;
}

bool Img_proc::Get_img_proc_corner_det_2d() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_corner_det_2d);
    return img_proc_corner_det_2d_;
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

bool Img_proc::Get_img_proc_huddle_det_3d() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_huddle_det_3d);
    return img_proc_huddle_det_3d_;
}

bool Img_proc::Get_img_proc_huddle_det_2d() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_huddle_det_2d);
    return img_proc_huddle_det_2d_;
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

double Img_proc::Get_wall_distance() const
{
    std::lock_guard<std::mutex> lock(mtx_wall_distance_);
    return wall_distance_;
}

bool Img_proc::Get_plane_mode() const
{
    std::lock_guard<std::mutex> lock(mtx_plane_mode_);
    return plane_mode_;
}

double Img_proc::Get_huddle_distance() const
{
    std::lock_guard<std::mutex> lock(mtx_huddle_distance_);
    return huddle_distance_;
}

bool Img_proc::Get_contain_huddle_to_foot() const
{
    std::lock_guard<std::mutex> lock(mtx_contain_huddle_to_foot);
    return contain_huddle_to_foot_;
}

int Img_proc::Get_foot_huddle_distance() const
{
    std::lock_guard<std::mutex> lock(mtx_foot_huddle_distance_);
    return foot_huddle_distance_;
}

bool Img_proc::Get_contain_corner_to_foot() const
{
    std::lock_guard<std::mutex> lock(mtx_contain_corner_to_foot);
    return contain_corner_to_foot_;
}

double Img_proc::Get_huddle_angle() const
{
    std::lock_guard<std::mutex> lock(mtx_huddle_angle_);
    return huddle_angle_;
}

double Img_proc::Get_corner_angle() const
{
    std::lock_guard<std::mutex> lock(mtx_corner_angle_);
    return corner_angle_;
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

void Img_proc::Set_img_proc_corner_det_3d(bool img_proc_corner_det_3d)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_corner_det_3d);
    this->img_proc_corner_det_3d_ = img_proc_corner_det_3d;
}

void Img_proc::Set_img_proc_corner_det_2d(bool img_proc_corner_det_2d)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_corner_det_2d);
    this->img_proc_corner_det_2d_ = img_proc_corner_det_2d;
}

void Img_proc::Set_img_proc_goal_line_det(bool img_proc_goal_line_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_goal_det_);
    this->img_proc_goal_det_ = img_proc_goal_line_det;
}

void Img_proc::Set_img_proc_huddle_det_3d(bool img_proc_huddle_det_3d)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_huddle_det_3d);
    this->img_proc_huddle_det_3d_ = img_proc_huddle_det_3d;
}

void Img_proc::Set_img_proc_huddle_det_2d(bool img_proc_huddle_det_2d)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_huddle_det_2d);
    this->img_proc_huddle_det_2d_ = img_proc_huddle_det_2d;
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

void Img_proc::Set_wall_distance(double wall_distance)
{
    std::lock_guard<std::mutex> lock(mtx_wall_distance_);
    this->wall_distance_ = wall_distance;
}

void Img_proc::Set_plane_mode(bool plane_mode)
{
    std::lock_guard<std::mutex> lock(mtx_plane_mode_);
    this->plane_mode_ = plane_mode;
}

void Img_proc::Set_huddle_distance(double huddle_distance)
{
    std::lock_guard<std::mutex> lock(mtx_huddle_distance_);
    this->huddle_distance_ = huddle_distance;
}

void Img_proc::Set_contain_huddle_to_foot(bool contain_huddle_to_foot)
{
    std::lock_guard<std::mutex> lock(mtx_contain_huddle_to_foot);
    this->contain_huddle_to_foot_ = contain_huddle_to_foot;
}

void Img_proc::Set_foot_huddle_distance(int foot_huddle_distance)
{
    std::lock_guard<std::mutex> lock(mtx_foot_huddle_distance_);
    this->foot_huddle_distance_ = foot_huddle_distance;
}

void Img_proc::Set_contain_corner_to_foot(bool contain_corner_to_foot)
{
    std::lock_guard<std::mutex> lock(mtx_contain_corner_to_foot);
    this->contain_corner_to_foot_ = contain_corner_to_foot;
}

void Img_proc::Set_huddle_angle(double huddle_angle)
{
    std::lock_guard<std::mutex> lock(mtx_huddle_angle_);
    this->huddle_angle_ = huddle_angle;
}

void Img_proc::Set_corner_angle(double corner_angle)
{
    std::lock_guard<std::mutex> lock(mtx_corner_angle_);
    this->corner_angle_ = corner_angle;
}