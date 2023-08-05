#include "img_proc.hpp"
// #include "Move_decision.hpp"


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

    return color_extracted;
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
        std::cout << "Angle: " << angle << std::endl;
    }

    return frame;
}

// void Img_proc::webcam_thread() {
//     const int webcam_width = 640;
//     const int webcam_height = 480;

//     cv::VideoCapture cap(2);
//     cap.set(cv::CAP_PROP_FRAME_WIDTH, webcam_width);
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, webcam_height);

//     if (!cap.isOpened())
//     {
//         std::cerr << "Could not open the webcam\n";
//         return;
//     }

//     const std::string window_name1 = "hsv Frame_white";
//     const std::string window_name2 = "thresh Frame_white";
//     const std::string window_name3 = "hsv Frame_yellow";
//     const std::string window_name4 = "thresh Frame_yellow";
//     cv::namedWindow(window_name1);
//     cv::namedWindow(window_name2);
//     cv::namedWindow(window_name3);
//     cv::namedWindow(window_name4);

//     create_threshold_trackbar_W(window_name2);
//     //create_color_range_trackbar(window_name1);
//     create_threshold_trackbar_Y(window_name4);
//     //create_color_range_trackbar(window_name3);

//     cv::Mat frame, hsv_frame_white, hsv_frame_yellow, thresh_frame_white, thresh_frame_yellow;

//     while (true)
//     {
//         cap >> frame;
//         if (frame.empty())
//             break;

//         hsv_frame_white = extract_color(frame, lower_bound_white, upper_bound_white);
//         hsv_frame_yellow = extract_color(frame, lower_bound_yellow, upper_bound_yellow);
//         thresh_frame_white = detect_color_areas(hsv_frame_white, green_color, threshold_value_white);
//         thresh_frame_yellow = detect_color_areas(hsv_frame_yellow, blue_color, threshold_value_yellow);

//         cv::imshow("hsv Frame_white", hsv_frame_white);
//         cv::imshow("hsv Frame_yellow", hsv_frame_yellow);
//         cv::imshow("thresh Frame_white", thresh_frame_white);
//         cv::imshow("thresh Frame_yellow", thresh_frame_yellow);
//         if (cv::waitKey(1) == 27)
//             break;
//     }
// }

// void Img_proc::realsense_thread() {
//     const int realsense_width = 640;
//     const int realsense_height = 480;

//     rs2::colorizer color_map;
//     rs2::pipeline pipe;
//     rs2::config cfg;
//     cfg.enable_stream(RS2_STREAM_COLOR, realsense_width, realsense_height, RS2_FORMAT_BGR8, 30);
//     cfg.enable_stream(RS2_STREAM_DEPTH, realsense_width, realsense_height, RS2_FORMAT_Z16, 30);

//     try {
//         pipe.start(cfg);
//     }
//     catch (const rs2::error& e) {
//         std::cerr << "Failed to open the RealSense camera: " << e.what() << std::endl;
//         return;
//     }

//     const auto window_name = "Realsense Depth Frame";
//     cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

//     const auto window_name_color = "Realsense Color Frame";
//     cv::namedWindow(window_name_color, cv::WINDOW_AUTOSIZE);

//     try {
//         while (cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
//         {
//             rs2::frameset data = pipe.wait_for_frames();

//             rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
//             rs2::frame color = data.get_color_frame();

//             const int w = depth.as<rs2::video_frame>().get_width();
//             const int h = depth.as<rs2::video_frame>().get_height();

//             cv::Mat depthMat(cv::Size(w, h), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
//             cv::Mat colorMat(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

//             cv::imshow(window_name, depthMat);
//             cv::imshow(window_name_color, colorMat);
//         }
//     }
//     catch (const rs2::error& e) {
//         std::cerr << "An error occurred during streaming: " << e.what() << std::endl;
//     }
// }
