#include "IMG_PROC.hpp"

// Constructor
Img_proc::Img_proc()
    : SPIN_RATE(30)
{
}

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

// ********************************************** 2D THREAD************************************************** //

void Img_proc::webcam_thread()
{
    // init();
    cv::VideoCapture cap(1);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, IMG_W);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_H);
    cap.set(cv::CAP_PROP_FPS, 60);
    while (ros::ok())
    {
        cv::Mat frame;
        cap >> frame;
        cv::imshow("Origin_img", frame);
        cam_1_btn_flg = true;
        img_processing();
    }
}

int Img_proc::img_processing()
{
    if (cam_1_finish)
        return 0;
    int64 elapsed = 0;
    int index = 0;
    char buf[256];
    Mat frameMat;

    if (load_st_flg)
    {
        if (vcap_1.isOpened())
        {
            vcap_1.release();
        }
        // if (vcap_2.isOpened())
        //{
        //	vcap_2.release();
        // }

        String dir = "recording/1_cam_";
        if (load_img_num_down_flg)
        {
            load_img_num--;
            load_img_num_down_flg = false;
        }
        else if (load_img_num_up_flg)
        {
            load_img_num++;
            load_img_num_up_flg = false;
        }

        dir += std::to_string((long double)(load_img_num));
        dir += ".bmp";
        if (load_img_play_flg)
        {
            load_img_num++;
        }

        Origin_img = imread(dir);
        Draw_img = imread(dir);

        if (Origin_img.data)
        {
            cv::resize(Origin_img, Origin_img, cv::Size(IMG_W, IMG_H));
            cv::resize(Draw_img, Draw_img, cv::Size(IMG_W, IMG_H));
            Depth_img = Mat::zeros(Size(IMG_W, IMG_H), CV_16UC1);
        }
        String().swap(dir);
        cam_1_btn_flg = true;
        if (get_hsv_flg && !capture_end_flg)
        {
            cv::resize(Origin_img, Capture_img, Size(IMG_W, IMG_H));
            capture_end_flg = true;
        }
    }
    else
    {
        if (!cam_1_btn_flg)
        {
            if (vcap_1.isOpened()) // && vcap_2.isOpened())
            {
                while (!vcap_1.read(cam_img_1))
                    ;

                // while (!vcap_2.read(cam_img_2));

                if (cam_img_1.data)
                {
                    int tmp_img_width = cam_img_1.cols;
                    if (tmp_img_width == IMG_W)
                    {
                        Origin_img = cam_img_1.clone();
                        Draw_img = cam_img_1.clone();
                        // Origin_img_2 = cam_img_2.clone();
                        Depth_img = Mat::zeros(Size(IMG_W, IMG_H), CV_16UC1);
                    }
                    else
                    {
                        cv::resize(cam_img_1, Origin_img, cv::Size(IMG_W, IMG_H));
                        cv::resize(cam_img_1, Draw_img, cv::Size(IMG_W, IMG_H));
                        // cv::resize(cam_img_2, Origin_img_2, cv::Size(IMG_W, IMG_H));
                        Depth_img = Mat::zeros(Size(IMG_W, IMG_H), CV_16UC1);
                    }
                    if (get_hsv_flg && !capture_end_flg)
                    {
                        resize(Origin_img, Capture_img, Size(IMG_W, IMG_H));
                        // resize(Origin_img_2, Capture_img_2, Size(IMG_W, IMG_H));
                        capture_end_flg = true;
                    }
                }
                else
                {
                    Mat tmp_draw_img(IMG_W, IMG_H, CV_8UC1);
                    tmp_draw_img = Scalar(0);

                    string str;
                    str = "no image";
                    putText(tmp_draw_img, str, Point(2, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar::all(255), 2);
                    img_view(tmp_draw_img, 0);
                    img_view(tmp_draw_img, 1);
                    img_view(tmp_draw_img, 2);
                    string().swap(str);
                    tmp_draw_img.release();
                    return 0;
                }
            }
        }
        else
        {
            cam_1_btn_flg = false;

            vcap_1.release();
            vcap_1 = VideoCapture(cv::CAP_DSHOW + cam_1_idx);

            ////vcap_1 = VideoCapture(cv::CAP_DSHOW + "run.mp4");
            vcap_1.set(CAP_PROP_FPS, 60);
            vcap_1.set(CAP_PROP_FRAME_WIDTH, 320);
            vcap_1.set(CAP_PROP_FRAME_HEIGHT, 240);
            vcap_1.set(CAP_PROP_ZOOM, cam_1_zoom);

            if (!vcap_1.isOpened())
            {
                ROS_ERROR("CAM_1_ERROR");
            }

            /*vcap_2 = VideoCapture(cv::CAP_DSHOW + cam_2_idx);
            vcap_2.set(CAP_PROP_FPS, 60);
            vcap_2.set(CAP_PROP_FRAME_WIDTH, 320);
            vcap_2.set(CAP_PROP_FRAME_HEIGHT, 240);
            vcap_2.set(CAP_PROP_ZOOM, 100);

            if (!vcap_2.isOpened())
            {
                AfxMessageBox("CAM_2_ERROR!");
            }*/
        }
    }
    if (!Origin_img.data)
    {
        Mat tmp_draw_img(IMG_W, IMG_H, CV_8UC1);
        tmp_draw_img = Scalar(0);

        string str;
        str = "no image";
        putText(tmp_draw_img, str, Point(2, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar::all(255), 2);
        img_view(tmp_draw_img, 0);
        img_view(tmp_draw_img, 1);
        img_view(tmp_draw_img, 2);

        string().swap(str);
        tmp_draw_img.release();
        return 0;
    }

    if (rec_st_flg)
    {
        rec_img_num++;
    }

    ///////////////////////////////////////////////////////////////
    ////////////////////////// start //////////////////////////////
    ///////////////////////////////////////////////////////////////

    Origin_img2 = Origin_img.clone(); // original 이미지를 복사해서 img2로 만듦
    // Origin_img = imread("Data/Realimg.jpg"); // 이거 활성화시키면 이미지가 picture box에 들어간다. 이 문장 앞에서는 캠 영상에 대해 나열, 마지막에 이미지 파일 불러오니 앞에 것들을 의미 없어짐
    // resize(Origin_img, Origin_img, Size(IMG_W, IMG_H));
    // Origin_img2 = imread("Data/Realimg2.jpg"); // 이거 활성화시키면 이미지가 picture box에 들어간다. 이 문장 앞에서는 캠 영상에 대해 나열, 마지막에 이미지 파일 불러오니 앞에 것들을 의미 없어짐
    // VideoCapture vcap(cv::CAP_DSHOW + "Data/RunVideo.mp4");

    resize(Origin_img2, Origin_img2, Size(IMG_W, IMG_H));

    RGBtoLAB();
    RGBtoHSV();

    FIELD_imgprocessing();

    LINE_imgprocessing();
    // GOAL_LINE_recognition();

    // DRAW_OBJ();	//write information in picture

    // if (Move_flg)
    //	det_ratio_all = ((double)pass_pix_num / (double)total_chk_pix_num) * 100.f;

    img_view(Origin_img, 0);  // 원본 영상 출력 (case 0)
    img_view(Origin_img2, 3); // (1,1)에 출력되는 영상(case 3)
    // img_view(&vcap, 3);
    if (hsv_mode == 1)
    {
        img_view(Field.img_field, 1);
    }
    else if (hsv_mode == 0)
    {
        img_view(Field.img_hsv, 1);
    }
    img_view(Field.img_lab, 2);

    /////////////////////////////////////////////////////////////
    ////////////////////////// end //////////////////////////////
    /////////////////////////////////////////////////////////////

    return 1;
}

// ********************************************** GETTERS ************************************************** //

bool Img_proc::Get_img_proc_line_det() const
{
    return img_proc_line_det_;
}

// ********************************************** SETTERS ************************************************** //

void Img_proc::Set_img_proc_line_det(bool img_proc_line_det_)
{
    img_proc_line_det_ = img_proc_line_det_;
}

// ************************************************************************************************************ //
void Img_proc::img_view(const cv::Mat &img, int mode)
{
    cv::Mat mat_temp = img.clone(); // Clone the input image to prevent modifications

    switch (mode)
    {
    case 0:
    default:
        cv::imshow("Image Mode 0", mat_temp);
        break;
    case 1:
        // Modify mat_temp as needed for mode 1
        cv::imshow("Image Mode 1", mat_temp);
        break;
    case 2:
        // Modify mat_temp as needed for mode 2
        cv::imshow("Image Mode 2", mat_temp);
        break;
    case 3:
        // Modify mat_temp as needed for mode 3
        cv::imshow("Image Mode 3", mat_temp);
        break;
    }

    cv::waitKey(0); // Wait for a key press to close the displayed image
}

void Img_proc::COLOR_SET(int COLOR_mode)
{
    switch (COLOR_mode)
    {
    case COLOR1:
        H_Range = Field.H_Range;
        H_max = Field.H_max;
        H_min = Field.H_min;
        S_Range = Field.S_Range;
        S_max = Field.S_max;
        S_min = Field.S_min;
        V_Range = Field.V_Range;
        V_max = Field.V_max;
        V_min = Field.V_min;
        H_val = Field.H_val;
        S_val = Field.S_val;
        V_val = Field.V_val;

        L_max = Field.L_max;
        L_min = Field.L_min;
        A_max = Field.A_max;
        A_min = Field.A_min;
        B_max = Field.B_max;
        B_min = Field.B_min;
        break;

    case COLOR2:
        H_Range = Line.H_Range;
        H_max = Line.H_max;
        H_min = Line.H_min;
        S_Range = Line.S_Range;
        S_max = Line.S_max;
        S_min = Line.S_min;
        V_Range = Line.V_Range;
        V_max = Line.V_max;
        V_min = Line.V_min;
        H_val = Line.H_val;
        S_val = Line.S_val;
        V_val = Line.V_val;

        L_max = Line.L_max;
        L_min = Line.L_min;
        A_max = Line.A_max;
        A_min = Line.A_min;
        B_max = Line.B_max;
        B_min = Line.B_min;
        break;

    /*case COLOR3:
        H_Range	= M_Field.H_Range;	H_max = M_Field.H_max;	H_min = M_Field.H_min;
        S_Range	= M_Field.S_Range;	S_max = M_Field.S_max;	S_min = M_Field.S_min;
        V_Range	= M_Field.V_Range;	V_max = M_Field.V_max;	V_min = M_Field.V_min;
        break;
    case COLOR4:
        H_Range = M_c_INF2.H_Range;	H_max = M_c_INF2.H_max;	H_min = M_c_INF2.H_min;
        S_Range = M_c_INF2.S_Range;	S_max = M_c_INF2.S_max;	S_min = M_c_INF2.S_min;
        V_Range = M_c_INF2.V_Range;	V_max = M_c_INF2.V_max;	V_min = M_c_INF2.V_min;
        break;*/
    default:
        H_Range = Field.H_Range;
        H_max = Field.H_max;
        H_min = Field.H_min;
        S_Range = Field.S_Range;
        S_max = Field.S_max;
        S_min = Field.S_min;
        V_Range = Field.V_Range;
        V_max = Field.V_max;
        V_min = Field.V_min;
        H_val = Field.H_val;
        S_val = Field.S_val;
        V_val = Field.V_val;

        L_max = Field.L_max;
        L_min = Field.L_min;
        A_max = Field.A_max;
        A_min = Field.A_min;
        B_max = Field.B_max;
        B_min = Field.B_min;
    }
}

// void Img_proc::DRAW_OBJ()
// {
//     string str;

//     if (RR_Real < 0) // right
//     {
//         str = format(("RR %d RT"), RR_Real);
//     }
//     else if (RR_val > 0) // left
//     {
//         str = format(("RR %d LF"), RR_Real);
//     }
//     else
//     {
//         str = format(("RR %d "), RR_Real);
//     }
//     putText(Origin_img, str, Point(5, 220), 2, 0.6, CV_RGB(0, 255, 0));

//     str = format(("RV %d"), RV_Real);
//     putText(Origin_img, str, Point(5, 200), 2, 0.6, CV_RGB(0, 255, 0));

//     str = format(("TL : %4d , %4d"), TL_value_X, TL_value_Y); // tilt sensor value
//     putText(Origin_img, str, Point(5, 235), 2, 0.6, CV_RGB(0, 255, 0));

//     str = format(("SV : %d"), RV_save_val);
//     putText(Origin_img, str, Point(180, 200), 2, 0.6, CV_RGB(0, 255, 0));

//     str = format(("dX : %.0f"), DX);
//     putText(Origin_img, str, Point(180, 220), 2, 0.6, CV_RGB(0, 255, 0));

//     str = format(("UDneck : %d"), UD_NeckAngle);
//     putText(Origin_img, str, Point(180, 235), 2, 0.6, CV_RGB(0, 255, 0));

//     // line(Origin_img, Point(0, Line_y), Point(IMG_W - 1, Line_y), Scalar(200, 0, 0), 2);
//     // line(Origin_img_2, Point(0, Line_y), Point(IMG_W - 1, Line_y), Scalar(200, 0, 0), 2);
//     // line(Origin_img,Point(IMG_W / 2, 0),Point(IMG_W / 2 , IMG_H -1),Scalar(200,0,0),2);
//     // circle(Origin_img, Point(cvRound(intercept_x), Line_y), 3, CV_RGB(255, 0, 0), 2);
//     // circle(Origin_img_2, Point(cvRound(intercept_x_2), Line_y), 3, CV_RGB(255, 0, 0), 2);

//     if (Move_Mode == WAKEUP_MODE)
//     {
//         line(Origin_img, Point(TL_line_x_min, 0), Point(TL_line_x_min, 239), CV_RGB(0, 220, 0), 2);
//         line(Origin_img, Point(TL_line_x_max, 0), Point(TL_line_x_max, 239), CV_RGB(0, 220, 0), 2);
//     }

//     string().swap(str);
// }

void Img_proc::RGBtoLAB()
{
    Mat lab;
    cv::cvtColor(Origin_img, lab, COLOR_BGR2Lab);

    if (Field.A_max == Field.A_min)
    {
        inRange(lab, Scalar(Field.L_min, Field.A_min, Field.B_min), Scalar(Field.L_max, Field.A_min, Field.B_max), Field.img_lab);
    }
    else if (Field.A_max > Field.A_min)
    {
        inRange(lab, Scalar(Field.L_min, Field.A_min, Field.B_min), Scalar(Field.L_max, Field.A_max, Field.B_max), Field.img_lab);
    }
    else if (Field.A_max < Field.A_min)
    {
        Mat tmp_F1, tmp_F2;
        inRange(lab, Scalar(0, 0, 0), Scalar(Field.L_max, Field.A_max, Field.B_max), tmp_F1);
        inRange(lab, Scalar(Field.L_min, Field.A_min, Field.B_min), Scalar(255, 255, 255), tmp_F2);
        Field.img_lab = tmp_F1 | tmp_F2;
    }

    cv::morphologyEx(Field.img_lab, Field.img_lab, MORPH_OPEN, Mat(), Point(-1, -1), 1);
}

void Img_proc::RGBtoHSV()
{
    Mat hsv;
    cv::cvtColor(Origin_img, hsv, COLOR_BGR2HSV);

    // if (mouse_flg)
    // {
    //     mouse_flg = false;
    //     p_dial->Dial_val_set(H_val, S_val, V_val);
    //     H_dial_view(&p_dial->Dial_image);
    //     p_dial->return_value(&H_val, &H_max, &H_min, &S_val, &S_max, &S_min, &V_val, &V_max, &V_min);
    // MINMAX_SET();
    // }

    if (Field.H_max == Field.H_min && Field.H_Range == 0)
    {
        inRange(hsv, Scalar(Field.H_min, Field.S_min, Field.V_min), Scalar(Field.H_min, Field.S_max, Field.V_max), Field.img_hsv);
    }
    else if (Field.H_max > Field.H_min)
    {
        inRange(hsv, Scalar(Field.H_min, Field.S_min, Field.V_min), Scalar(Field.H_max, Field.S_max, Field.V_max), Field.img_hsv);
    }
    else
    {
        Mat tmp_F1, tmp_F2;
        inRange(hsv, Scalar(0, Field.S_min, Field.V_min), Scalar(Field.H_max, Field.S_max, Field.V_max), tmp_F1);
        inRange(hsv, Scalar(Field.H_min, Field.S_min, Field.V_min), Scalar(180, Field.S_max, Field.V_max), tmp_F2);
        Field.img_hsv = tmp_F1 | tmp_F2;
    }

    cv::morphologyEx(Field.img_hsv, Field.img_hsv, MORPH_OPEN, Mat(), Point(-1, -1), 1);
    cv::morphologyEx(Field.img_hsv, Field.img_hsv, MORPH_ERODE, Mat(), Point(-1, -1), -5);
}

void Img_proc::FIELD_imgprocessing() // find field contour area and fill
{
    // seperate 'bitwise image' and 'field image' for next bitwise operation.
    Mat field;
    Mat bitwise;
    bitwise_and(Field.img_hsv, Field.img_lab, bitwise);

    cv::morphologyEx(bitwise, bitwise, MORPH_OPEN, Mat(), Point(-1, -1), 2);
    field = bitwise.clone();

    // for (int i = 100; i < 180; ++i) // i = y-axis, j = x-axis, draw rectangle for straight line detection
    //{
    //	for (int j = 120; j < 200; ++j)
    //	{
    //		field.at<char>(i, j) = 255;
    //	}
    // }

    for (int i = 0; i < 240; ++i) // i = y-axis, j = x-axis, draw rectangle for straight line detection
    {
        for (int j = 0; j < 320; ++j)
        {
            if (i > 240 - 5)
            {
                if (j > 40 && j < 280)
                {
                    field.at<char>(i, j) = 255;
                }
            }
        }
    }

    // morphologyEx(field, field, MORPH_CLOSE, Mat(), Point(-1, -1), 3);
    // morphologyEx(bitwise, field, MORPH_ERODE, Mat(), Point(-1, -1), 2);

    vector<vector<Point>> contours;
    vector<Point> hull;
    vector<Point> approxpoly;
    cv::findContours(field, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    int _size = (int)contours.size();
    int area_max = 0;
    int label_num = 0;
    if (_size > 0)
    {
        for (int i = 0; i < _size; i++)
        {
            int area = cvRound(contourArea(contours[i]) * 100.0);
            if (area > area_max)
            {
                area_max = area;
                label_num = i;
            }
        }

        //*** WARNING algorithm changed !! WARNING ***//

        // approxPolyDP(Mat(contours[label_num]), approxpoly, arcLength(contours[label_num], true)*0.02, true);
        // fillConvexPoly(field, approxpoly, Scalar(255), 8);
        convexHull(Mat(contours[label_num]), hull, false);
        fillConvexPoly(field, hull, Scalar(255), 8);

        //***   WARNING WARNING WARNING WARNING   ***//
        // drawContours(field, Mat(contours[label_num]), -1, Scalar(255), -1);
    }
    cv::morphologyEx(field, field, MORPH_ERODE, Mat(), Point(-1, -1), 5);
    bitwise_and(field, ~bitwise, field);
    cv::morphologyEx(field, Field.img_field, MORPH_OPEN, Mat(), Point(-1, -1), 2);
    // morphologyEx(field, Field.img_field, MORPH_ERODE, Mat(), Point(-1, -1), 5);
    // morphologyEx(img_field, img_field, MORPH_DILATE, Mat(), Point(-1, -1), 2);
}
void Img_proc::MINMAX_SET()
{
    if (H_max > 180)
        H_max -= 180;
    if (H_min < 0)
        H_min += 180;
    if (S_max > 255)
        S_max = 255;
    if (S_min < 0)
        S_min = 0;
    if (V_max > 255)
        V_max = 255;
    if (V_min < 0)
        V_min = 0;
    if (L_max > 255)
        L_max = 255;
    if (L_min < 0)
        L_min = 0;
    if (A_max > 255)
        A_max = 255;
    if (A_min < 0)
        A_min = 0;
    if (B_max > 255)
        B_max = 255;
    if (B_min < 0)
        B_min = 0;

    switch (COLOR_MODE)
    {
    case COLOR1:
        Field.H_max = H_max;
        Field.S_max = S_max;
        Field.V_max = V_max;
        Field.H_min = H_min;
        Field.S_min = S_min;
        Field.V_min = V_min;
        Field.H_val = H_val;
        Field.S_val = S_val;
        Field.V_val = V_val;

        Field.H_Range = H_Range;
        Field.S_Range = S_Range;
        Field.V_Range = V_Range;

        Field.L_max = L_max;
        Field.L_min = L_min;
        Field.A_max = A_max;
        Field.A_min = A_min;
        Field.B_max = B_max;
        Field.B_min = B_min;
        break;

    case COLOR2:
        Line.H_max = H_max;
        Line.S_max = S_max;
        Line.V_max = V_max;
        Line.H_min = H_min;
        Line.S_min = S_min;
        Line.V_min = V_min;
        Line.H_val = H_val;
        Line.S_val = S_val;
        Line.V_val = V_val;

        Line.H_Range = H_Range;
        Line.S_Range = S_Range;
        Line.V_Range = V_Range;

        Line.L_max = L_max;
        Line.L_min = L_min;
        Line.A_max = A_max;
        Line.A_min = A_min;
        Line.B_max = B_max;
        Line.B_min = B_min;
        break;

        /*case COLOR3:
            M_Field.H_max = H_max;		M_Field.S_max = S_max;		M_Field.V_max = V_max;
            M_Field.H_min = H_min;		M_Field.S_min = S_min;		M_Field.V_min = V_min;
            M_Field.H_Range = H_Range;	M_Field.S_Range = S_Range;	M_Field.V_Range = V_Range;
            M_Field.H_Range2 = H_Range2;
            M_Field.H_val = H_val;		M_Field.S_val = S_val;		M_Field.V_val = V_val;
        case COLOR4:
            M_c_INF2.H_max = H_max;		M_c_INF2.S_max = S_max;		M_c_INF2.V_max = V_max;
            M_c_INF2.H_min = H_min;		M_c_INF2.S_min = S_min;		M_c_INF2.V_min = V_min;
            M_c_INF2.H_Range = H_Range;	M_c_INF2.S_Range = S_Range;	M_c_INF2.V_Range = V_Range;
            M_c_INF2.H_Range2 = H_Range2;
            M_c_INF2.H_val = H_val;		M_c_INF2.S_val = S_val;		M_c_INF2.V_val = V_val;*/

    default:
        Field.H_max = H_max;
        Field.S_max = S_max;
        Field.V_max = V_max;
        Field.H_min = H_min;
        Field.S_min = S_min;
        Field.V_min = V_min;
        Field.H_val = H_val;
        Field.S_val = S_val;
        Field.V_val = V_val;

        Field.H_Range = H_Range;
        Field.S_Range = S_Range;
        Field.V_Range = V_Range;

        Field.L_max = L_max;
        Field.L_min = L_min;
        Field.A_max = A_max;
        Field.A_min = A_min;
        Field.B_max = B_max;
        Field.B_min = B_min;
    }
}

void Img_proc::LINE_imgprocessing()
{
    try
    {
        double tmp_delta_x = 0;
        // int REFERNCE_Y;

        vector<vector<Point>> contours;
        Mat img_contour_tmp;
        img_contour_tmp = Field.img_field.clone();

        //    Parameter Setting Start    //
        /*    setting border line    */

        /*** ROI SETTING ***/
        /*** RR LINE SETTING ***/

        float curvature = RR_LINE_CURVATURE;
        float y_tip_point = Y_VERTEX;

        // for (int i = 0; i < IMG_W; ++i) // ���׶� �Ķ��� �� �߱�
        //{
        //	float x = i;
        //	float y = curvature * (x - 160) * (x - 160) + y_tip_point;
        //	circle(Origin_img, Point(int(x), int(y)), 2, Scalar(255, 0, 0), 2);
        // }

        for (int i = 0; i < IMG_H; ++i) // i = y-axis , j = x-axis
        {
            for (int j = 0; j < IMG_W; ++j)
            {
                //// delete top area & bottom line
                if (i > BOTTOM_BORDER_LINE || i < TOP_BORDER_LINE)
                {
                    img_contour_tmp.at<char>(i, j) = 0;
                    if (roi_line_flg == true)
                    {
                        Origin_img.at<Vec3b>(i, j)[0] = Origin_img.at<Vec3b>(i, j)[0] * 0.5;
                        Origin_img.at<Vec3b>(i, j)[1] = Origin_img.at<Vec3b>(i, j)[1] * 0.5;
                        Origin_img.at<Vec3b>(i, j)[2] = Origin_img.at<Vec3b>(i, j)[2] * 0.5;
                    }
                }

                // //delete curvature line upper area
                if (i < curvature * (j - 160) * (j - 160) + y_tip_point)
                {
                    img_contour_tmp.at<char>(i, j) = 0;
                    if (roi_line_flg == true)
                    {
                        Origin_img.at<Vec3b>(i, j)[0] = Origin_img.at<Vec3b>(i, j)[0] * 0.5;
                        Origin_img.at<Vec3b>(i, j)[1] = Origin_img.at<Vec3b>(i, j)[1] * 0.5;
                        Origin_img.at<Vec3b>(i, j)[2] = Origin_img.at<Vec3b>(i, j)[2] * 0.5;
                    }
                }
                // delete both side edge area
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

        findContours(img_contour_tmp, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

        int _size = (int)contours.size();
        int line_count = contours.capacity(); // ���༱�� ��� �޶��� ������ �����۵��ϴ��� Ȯ���غ��� ��
        string str = to_string(line_count) + " Dot";
        putText(Origin_img, str, Point(10, 30), 2, 0.8, CV_RGB(0, 255, 0), 2);
        // int line_count = 0;

        vector<Moments> _moment(_size);
        vector<Point2f> centerpoints(_size);

        vector<pair<float, float>> edge_center_b; // �غ� �Ϻ� �߾���, ��ǥ ������ ���� pair<float, float> �̿�
        vector<pair<float, float>> edge_center_t; // �غ� ��� �߾���
        Point2f corner[4];                        // �ڳ� ��ǥ ���� �迭 ���� �迭�� �����ϸ� ���� ��Ȳ�� ���� �߻� ����->���ͷ� �ٲ�(���� �ٲ� ���� �ƴϴ�.)

        vector<float> vector_y; // y���� reference_y �񱳸� ���� ���༱�� y���� ��� ������ ����
        vector<float> vector_x;
        vector<float> vector_moment_x;
        vector<float> vector_moment_y;

        vector<float> vect_box_area;
        vector<float> width_size;
        vector<float> height_size;

        float min_distance_y = 0;

        // float target_x;

        float center_x_b[9999]; // center_x_bottom
        float center_y_b[9999];
        float center_x_t[9999]; // center_x_top
        float center_y_t[9999];

        /*** Draw Contour ***/
        if (_size > 0)
        {
            for (int i = 0; i < _size; i++)
            {
                // CONNECT METHOD.1 /*** START: base_edge_point to base_edge_point connect method ***/

                //	if (contourArea(contours[i]) > CONTOUR_AREA) // ������ contourarea_num �̻��� ������ ó��->(���� ���� ������ �ֵ��� ������ ó������ ����)
                //	{
                //		RotatedRect rect = minAreaRect(contours[i]);
                //		rect.points(corner);
                //		vector<Point> boundingContour;
                //		vector<vector<Point>> boxContours;
                //		for (unsigned int j = 0; j < 4; j++)
                //		{
                //			boundingContour.push_back(corner[j]);//�ڳ��� 4���� push_back���� boundingContour�� �־��ش�.
                //		}
                //		boxContours.push_back(boundingContour);//boundingContour�� �ٽ� boxContours�� �־ ������ �׸� �� �ְ� ������ ������
                //		drawContours(Origin_img, boxContours, 0, Scalar(0, 255, 0), 2); // 0���� ���� �� �۵�
                //		for (unsigned int k = 0; k < 4; k++)
                //		{
                //			circle(Origin_img, Point2f(corner[k]), 4, Scalar(255, 0, 0), -1);//4�� ���� �׷�
                //		}
                //		double height = rect.size.height;//get height
                //		double width = rect.size.width;
                //		int area = height*width;
                //		string area_str = to_string(area);
                //		double angle = rect.angle;
                //
                //		if (line_count > 0)
                //		{
                //			if (width <= height) // (y=m*x(������)���� m�� ������ ��)
                //			{
                //			////////////// base_edge_point to base_edge_point connect method /////////////
                //				center_x_b[i] = (corner[3].x - corner[0].x) / 2 + corner[0].x;//�Ʒ��κ� �߾��� ������ ��
                //				center_y_b[i] = (corner[0].y - corner[3].y) / 2 + corner[3].y;
                //				//edge_center_b.push_back(make_pair(center_x_b[i], center_y_b[i]));
                //				center_x_t[i] = (corner[2].x - corner[1].x) / 2 + corner[1].x;//�� �κ� �߾��� ������ ��
                //				center_y_t[i] = (corner[1].y - corner[2].y) / 2 + corner[2].y;
                //				//edge_center_t.push_back(make_pair(center_x_t[i], center_y_t[i]));
                //			}
                //			else if (width > height)//(y=m*x(������)���� m�� ����� ��)
                //			{
                //				center_x_b[i] = (corner[0].x - corner[1].x) / 2 + corner[1].x;
                //				center_y_b[i] = (corner[0].y - corner[1].y) / 2 + corner[1].y;
                //				//edge_center_b.push_back(make_pair(center_x_b[i], center_y_b[i]));
                //				center_x_t[i] = (corner[3].x - corner[2].x) / 2 + corner[2].x;
                //				center_y_t[i] = (corner[3].y - corner[2].y) / 2 + corner[2].y;
                //				//edge_center_t.push_back(make_pair(center_x_t[i], center_y_t[i]));
                //			}
                //			vector_y.push_back(center_y_b[i]);//vector_y�� bottom�� y�� �־�
                //			vector_y.push_back(center_y_t[i]);//vector_y�� top�� y�� �־�
                //			vector_x.push_back(center_x_b[i]);
                //			vector_x.push_back(center_x_t[i]);
                //		}
                //		//for (int i = 0; i < line_count; i++)
                //		//{
                //		//	string st_x_b = to_string((int)center_x_b[i]);
                //		//	string st_y_b = to_string((int)center_y_b[i]);
                //		//	string st_x_t = to_string((int)center_x_t[i]);
                //		//	string st_y_t = to_string((int)center_y_t[i]);
                //		//	string num = to_string(i);
                //		//}
                //	}
                //	else if (MIN_CONTOUR_AREA < contourArea(contours[i]) < CONTOUR_AREA)
                //	{
                //		_moment[i] = moments(contours[i], false);
                //		centerpoints[i] = Point2f(_moment[i].m10 / _moment[i].m00, _moment[i].m01 / _moment[i].m00);
                //		vector_y.push_back(centerpoints[i].y);//vector_y�� ������ ���� ���༱�� y��ǥ�� �־�
                //		vector_y.push_back(centerpoints[i].y + 0.0001f);//vector_y�� ������ ���� ���༱�� y��ǥ�� �־�: ������ ���� ���༱�� �νĵǾ 2���� push_back �ϱ� ����
                //		vector_x.push_back(centerpoints[i].x);
                //		vector_x.push_back(centerpoints[i].x + 0.0001f);
                //		circle(Origin_img, Point(centerpoints[i].x, centerpoints[i].y), 4, Scalar(0, 0, 255), -1);//�ǵ����̸� edge_center_t[i].first �� ���� ���� center_x_b[i] �� ����
                //		string num = to_string((int)i);
                //		string st_c_x = to_string((int)centerpoints[i].x);
                //		string st_c_y = to_string((int)centerpoints[i].y);
                //	}
                //}

                // /*** END: base_edge_point to base_edge_point connect method ***/

                // CONNECT METHOD.2 /*** START: moment_point to moment_point connect method ***/

                if (contourArea(contours[i]) > CONTOUR_AREA) // ������ contourarea_num �̻��� ������ ó��->(���� ���� ������ �ֵ��� ������ ó������ ����)
                {
                    RotatedRect rect = minAreaRect(contours[i]);
                    rect.points(corner);
                    vector<Point> boundingContour;
                    vector<vector<Point>> boxContours;
                    for (unsigned int j = 0; j < 4; j++)
                    {
                        boundingContour.push_back(corner[j]); // �ڳ��� 4���� push_back���� boundingContour�� �־��ش�.
                    }
                    boxContours.push_back(boundingContour);                         // boundingContour�� �ٽ� boxContours�� �־ ������ �׸� �� �ְ� ������ ������
                    drawContours(Origin_img, boxContours, 0, Scalar(0, 255, 0), 2); // 0���� ���� �� �۵�
                    for (unsigned int k = 0; k < 4; k++)
                    {
                        circle(Origin_img, Point2f(corner[k]), 4, Scalar(255, 0, 0), -1); // 4�� ���� �׷�
                    }

                    double height = rect.size.height; // get height
                    double width = rect.size.width;
                    int area = height * width;
                    string area_str = to_string(area);
                    double angle = rect.angle;

                    // if (line_count > 0)
                    //{
                    //	if (width <= height) // (y=m*x(������)���� m�� ������ ��)
                    //	{
                    //		////////////// base_edge_point to base_edge_point connect method /////////////
                    //		center_x_b[i] = (corner[3].x - corner[0].x) / 2 + corner[0].x;//�Ʒ��κ� �߾��� ������ ��
                    //		center_y_b[i] = (corner[0].y - corner[3].y) / 2 + corner[3].y;
                    //		//edge_center_b.push_back(make_pair(center_x_b[i], center_y_b[i]));
                    //		center_x_t[i] = (corner[2].x - corner[1].x) / 2 + corner[1].x;//�� �κ� �߾��� ������ ��
                    //		center_y_t[i] = (corner[1].y - corner[2].y) / 2 + corner[2].y;
                    //		//edge_center_t.push_back(make_pair(center_x_t[i], center_y_t[i]));
                    //	}
                    //	else if (width > height)//(y=m*x(������)���� m�� ����� ��)
                    //	{
                    //		center_x_b[i] = (corner[0].x - corner[1].x) / 2 + corner[1].x;
                    //		center_y_b[i] = (corner[0].y - corner[1].y) / 2 + corner[1].y;
                    //		//edge_center_b.push_back(make_pair(center_x_b[i], center_y_b[i]));
                    //		center_x_t[i] = (corner[3].x - corner[2].x) / 2 + corner[2].x;
                    //		center_y_t[i] = (corner[3].y - corner[2].y) / 2 + corner[2].y;
                    //		//edge_center_t.push_back(make_pair(center_x_t[i], center_y_t[i]));
                    //	}
                    //	_moment[i] = moments(contours[i], false);
                    //	centerpoints[i] = Point2f(_moment[i].m10 / _moment[i].m00, _moment[i].m01 / _moment[i].m00);
                    //	vector_x.push_back(centerpoints[i].x);
                    //	vector_y.push_back(centerpoints[i].y);
                    //	vector_y.push_back(center_y_t[i]);
                    //	vector_x.push_back(center_x_t[i]);
                    //	vector_moment_x.push_back(centerpoints[i].x);
                    //	vector_moment_y.push_back(centerpoints[i].y);
                    // }
                    if (line_count > 0)
                    {
                        if (width <= height) // (y=m*x(������)���� m�� ������ ��)
                        {
                            center_x_b[i] = (corner[3].x - corner[0].x) / 2 + corner[0].x; // �Ʒ��κ� �߾��� ������ ��
                            center_y_b[i] = (corner[0].y - corner[3].y) / 2 + corner[3].y;
                            center_x_t[i] = (corner[2].x - corner[1].x) / 2 + corner[1].x; // �� �κ� �߾��� ������ ��
                            center_y_t[i] = (corner[1].y - corner[2].y) / 2 + corner[2].y;
                        }
                        else if (width > height) //(y=m*x(������)���� m�� ����� ��)
                        {
                            center_x_b[i] = (corner[0].x - corner[1].x) / 2 + corner[1].x;
                            center_y_b[i] = (corner[0].y - corner[1].y) / 2 + corner[1].y;
                            center_x_t[i] = (corner[3].x - corner[2].x) / 2 + corner[2].x;
                            center_y_t[i] = (corner[3].y - corner[2].y) / 2 + corner[2].y;
                        }
                        vector_x.push_back(center_x_b[i]);
                        vector_y.push_back(center_y_b[i]); // vector_y�� bottom�� y�� �־�
                        vector_x.push_back(center_x_t[i]);
                        vector_y.push_back(center_y_t[i]); // vector_y�� top�� y�� �־�
                    }
                }

                else if (MIN_CONTOUR_AREA < contourArea(contours[i]) < CONTOUR_AREA)
                {
                    _moment[i] = moments(contours[i], false);
                    centerpoints[i] = Point2f(_moment[i].m10 / _moment[i].m00, _moment[i].m01 / _moment[i].m00);
                    vector_y.push_back(centerpoints[i].y);          // vector_y�� ������ ���� ���༱�� y��ǥ�� �־�
                    vector_y.push_back(centerpoints[i].y + 0.0001); // vector_y�� ������ ���� ���༱�� y��ǥ�� �־�: ������ ���� ���༱�� �νĵǾ 2���� push_back �ϱ� ����
                    vector_x.push_back(centerpoints[i].x);
                    vector_x.push_back(centerpoints[i].x + 0.0001);
                    circle(Origin_img, Point(centerpoints[i].x, centerpoints[i].y), 4, Scalar(0, 0, 255), -1); // �ǵ����̸� edge_center_t[i].first �� ���� ���� center_x_b[i] �� ����
                }

                // /*** END: moment_point to moment_point connect method ***/
            }
        }

        /***  START: Determination of driving direction  (by gradient) ***/

        bool straightLine = true;
        double gradient = -(center_y_t[line_count - 1] - center_y_b[0]) / (center_x_t[line_count - 1] - center_x_b[0]); // Determination of the driving line direction using the inclination between the first center point and the last center point
        int grd_straight = 2.2f;
        // int max_RV_value = GetDlgItemInt(IDC_EDIT_RV_val);
        // int min_RV_value = 100; //120
        int Up_ref_y = 90;
        int Dowm_ref_y = 220;

        if ((gradient > grd_straight) || (gradient < -grd_straight)) // judged to be a straight line. If it exists between the slopes, it is a straight line.
        {
            straightLine = true;
        }
        else /*if ((gradient < grd_straight) && (gradient > -grd_straight))*/
        {
            straightLine = false;
        }

        if (straightLine == true) // straight line
        {
            // RV_save_val++;
            if ((gradient > grd_straight * 2) || (gradient < -(grd_straight * 2)))
            {
                // RV_save_val += 2;
            }
            else if ((gradient > grd_straight) || (gradient < -(grd_straight)))
            {
                // RV_save_val += 1.25f;
            }
            // if (RV_save_val > max_RV_value)
            // {
            // 	RV_save_val = max_RV_value;
            // }
        }
        if (straightLine == false) // No straight line
        {
            // RV_save_val--;
            // if ((gradient > grd_straight*2) || (gradient < -(grd_straight*2)))
            //{
            //	RV_save_val-=1;
            // }
            if ((gradient > 1.7f) || (gradient < -1.7f)) // Stay current
            {
                // RV_save_val;
            }
            else if ((gradient > 0.6f) || (gradient < -0.6f)) // angle: 30 degree
            {
                // RV_save_val -= 3;
            }
            else
            {
                // RV_save_val -= 6;
            }
            // if (RV_save_val < min_RV_value)
            // {
            // 	RV_save_val = min_RV_value;
            // }
        }

        /***  END: Determination of driving direction  (by gradient)***/

        /*** START: change the reference line(by RV_save_val) ***/
        //// setting REFERNCE_Y_RV_save_val graph

        // if ((min_RV_value < RV_save_val) && (RV_save_val < max_RV_value))
        // {
        // 	//REFERNCE_Y = -RV_save_val+300;//IDC_EDIT_RV_val=220
        // 	//REFERNCE_Y = (int)-(200 * RV_save_val) / max_RV_value + 280;//IDC_EDIT_RV_val=280
        // 	REFERNCE_Y = (int)(-100)*(RV_save_val - min_RV_value) / (max_RV_value - min_RV_value) + 180;
        // 	if (REFERNCE_Y < Up_ref_y)
        // 	{
        // 		REFERNCE_Y = Up_ref_y;
        // 	}
        // 	else if (REFERNCE_Y > Dowm_ref_y)
        // 	{
        // 		REFERNCE_Y = Dowm_ref_y;//180
        // 	}
        // }
        // else if (RV_save_val > max_RV_value)
        // {
        // 	REFERNCE_Y = Up_ref_y;
        // }
        // else if (RV_save_val < min_RV_value/*max_RV_value*0.5*/)
        // {
        // 	REFERNCE_Y = Dowm_ref_y;
        // }
        /*** END: change the reference line(by RV_save_val) ***/

        vector<float> distance_y_i;
        vector<float> distance_y_f;
        vector<float> distance_y_plus;
        vector<float> distance_y_i_f;
        int mdi = 0; // min_distance_index

        // /*** START: Draw dx point method (edge center point to edge center point) ***/

        // if (line_count >0)
        //{
        //	for (int i = 0; i < line_count * 2 - 1; i++) // ���ӵ� 2�� ����� ������ ����=line_count * 2 - 1
        //	{
        //		distance_y_i.push_back(abs(vector_y[i] - REFERNCE_Y));
        //		distance_y_f.push_back(abs(vector_y[i + 1] - REFERNCE_Y));
        //		distance_y_plus.push_back(distance_y_i[i] + distance_y_f[i]);//���밪 ���� ���ӵ� �߾����� y�� ��
        //		distance_y_i_f.push_back(vector_y[i] - vector_y[i + 1]);
        //	}
        //	float min_distance;//distance_y_plus[0]���� �ʱ�ȭ
        //	if (line_count > 0) { min_distance = distance_y_plus[0]; } // initialization
        //	for (int i = 0; i < line_count * 2 - 1; i++)
        //	{
        //		if (min_distance >= distance_y_plus[i])
        //		{
        //			min_distance = distance_y_plus[i];
        //			mdi = i;
        //		}
        //		target_x = (REFERNCE_Y - vector_y[mdi])*(vector_x[mdi + 1] - vector_x[mdi]) / (vector_y[mdi + 1] - vector_y[mdi]) + vector_x[mdi];
        //		if (distance_y_i_f[mdi] < 0.001f) { target_x = vector_x[mdi]; }//������ ���� �� target_x�� moment�߽��� x��ǥ
        //		Point_vector_x_y_save.x = target_x;
        //	}
        // }

        // /*** END: Draw dx point method (edge center point to edge center point) ***/

        // /*** START: Draw dx point method (moment point to moment point) ***/

        // if (line_count == 1)
        //{
        //	for (int i = 0; i < line_count * 2 - 1; i++)
        //	{
        //		distance_y_i.push_back(abs(vector_y[i] - REFERNCE_Y));
        //		distance_y_f.push_back(abs(vector_y[i + 1] - REFERNCE_Y));
        //		distance_y_plus.push_back(distance_y_i[i] + distance_y_f[i]);
        //		distance_y_i_f.push_back(vector_y[i] - vector_y[i + 1]);
        //	}
        //	float min_distance;
        //	if (line_count > 0) { min_distance = distance_y_plus[0]; } // initialization
        //	for (int i = 0; i < line_count * 2 - 1; i++)
        //	{
        //		if (min_distance >= distance_y_plus[i])
        //		{
        //			min_distance = distance_y_plus[i];
        //			mdi = i;
        //		}
        //		target_x = (REFERNCE_Y - vector_y[mdi])*(vector_x[mdi + 1] - vector_x[mdi]) / (vector_y[mdi + 1] - vector_y[mdi]) + vector_x[mdi];
        //		if (distance_y_i_f[mdi] < 0.001f) { target_x = vector_x[mdi]; }
        //		Point_vector_x_y_save.x = target_x;
        //	}
        // }
        // if (line_count > 1)
        //{
        //	for (int i = 0; i < line_count - 1; i++)
        //	{
        //		distance_y_i.push_back(abs(vector_moment_y[i] - REFERNCE_Y));
        //		distance_y_f.push_back(abs(vector_moment_y[i + 1] - REFERNCE_Y));
        //		distance_y_plus.push_back(distance_y_i[i] + distance_y_f[i]);
        //		distance_y_i_f.push_back(vector_moment_y[i] - vector_moment_y[i + 1]);
        //	}
        //	float min_distance;
        //	if (line_count > 0) { min_distance = distance_y_plus[0]; }
        //	for (int i = 0; i < line_count - 1; i++)
        //	{
        //		if (min_distance >= distance_y_plus[i])
        //		{
        //			min_distance = distance_y_plus[i];
        //			mdi = i;
        //		}
        //		target_x = (REFERNCE_Y - vector_moment_y[mdi])*(vector_moment_x[mdi + 1] - vector_moment_x[mdi]) / (vector_moment_y[mdi + 1] - vector_moment_y[mdi]) + vector_moment_x[mdi];
        //		if (distance_y_i_f[mdi] < 0.001f) { target_x = vector_moment_x[mdi]; }
        //		Point_vector_x_y_save.x = target_x;
        //	}
        // }
        /*** END: draw dx point method (moment point to moment point) ***/

        ///-------------------------------------------------------------------------------------////
        // /*** START: Draw dx point method (first edge center point to last edge center point) ***/
        ///-------------------------------------------------------------------------------------////
        if (line_count > 0)
        {
            for (int i = 0; i < line_count * 2 - 1; i++) // ���ӵ� 2�� ����� ������ ����=line_count * 2 - 1
            {
                distance_y_i.push_back(abs(vector_y[i] - REFERNCE_Y));
                distance_y_f.push_back(abs(vector_y[i + 1] - REFERNCE_Y));
                distance_y_plus.push_back(distance_y_i[i] + distance_y_f[i]); // ���밪 ���� ���ӵ� �߾����� y�� ��
                distance_y_i_f.push_back(vector_y[i] - vector_y[i + 1]);
            }
            float min_distance; // distance_y_plus[0]���� �ʱ�ȭ
            if (line_count > 0)
            {
                min_distance = distance_y_plus[0];
            } // initialization
            for (int i = 0; i < line_count * 2 - 1; i++)
            {
                if (min_distance >= distance_y_plus[i])
                {
                    min_distance = distance_y_plus[i];
                    mdi = i;
                }
                target_x = (REFERNCE_Y - vector_y.front()) * (vector_x.back() - vector_x.front()) / (vector_y.back() - vector_y.front()) + vector_x.front();
            }
            if (line_count == 1)
            {
                if (distance_y_i_f[mdi] < 0.001f)
                {
                    target_x = vector_x[mdi];
                }
            }
            Point_vector_x_y_save.x = target_x;

            no_line_det_flg = false;
        }

        ///-------------------------------------------------------------------------------------////
        // /*** END: Draw dx point method (first edge center point to last edge center point) ***/
        ///-------------------------------------------------------------------------------------////

        /*** Draw the reference line (yellow line)  ***/
        line(Origin_img, Point(0, REFERNCE_Y), Point(320, REFERNCE_Y), Scalar(0, 255, 255), 2, 8, 0);
        line(Origin_img, Point(160, 240), Point(160, 0), Scalar(0, 255, 255), 2, 8, 0);

        dx = -(target_x - REFERNCE_X);
        //           dx is larger than +- 160               //
        if (dx > 160 + 0.0001f)
        {
            dx = 160;
        }
        if (dx < -160 - 0.0001f)
        {
            dx = -160;
        }
        // string st_dx = to_string((int)dx);
        // string st_target_x = to_string((int)target_x);
        circle(Origin_img, Point(target_x, REFERNCE_Y), 4, Scalar(255, 51, 255), -1);
        // circle(Origin_img2, Point(160, 120), 4, Scalar(255, 051, 255), -1); // Origin_img2 draw Test

        Point_vector_x_y_save.y = REFERNCE_Y;
        Point_tmp_point_target_save.y = Point_vector_x_y_save.y;
        Point_tmp_point_target_save.x = dx;

        DX = dx;

        /**************           NO LINE -> using before target_x               *****************/

        if (line_count == 0 /*|| Point_tmp_point_target_save.x == Point_vector_x_y_save.x*/)
        {

            no_line_det_flg = true;
        }

        line_det_flg = true;
    }

    catch (Exception e)
    {
        line_det_flg = false;
    }
}

void Img_proc::init()
{
    Color_inf.reserve(c_img_num + 2);
    for (int i = 0; i < c_img_num + 2; i++)
    {
        Color_inf.push_back(COLOR());
    }

    cam_1_btn_flg = true;
    cam_1_finish = false;
    img_thread_finish_flg = false;
    cam_1_idx = 1; // 캠 바꾸기 0: 노트북 캠, 1: 외부 캠
    load_img_num = 1;
    load_st_flg = false;
    load_img_num_down_flg = false;
    load_img_num_up_flg = false;
    load_img_play_flg = false;

    if (!load_st_flg)
    {
        vcap_1 = VideoCapture(cv::CAP_DSHOW + cam_1_idx); // 캠 영상 띄우기
        // vcap_1 = VideoCapture("runVideo.mp4");//영상 띄우기
        // vcap_1 = VideoCapture("run2.mp4");
        // vcap_1 = VideoCapture("run3_rotation.mp4");
        vcap_1.set(CAP_PROP_FPS, 60);
        vcap_1.set(CAP_PROP_FRAME_WIDTH, 320);
        vcap_1.set(CAP_PROP_FRAME_HEIGHT, 240);
        vcap_1.set(CAP_PROP_ZOOM, cam_1_zoom);
    }

    rec_st_flg = false;
    rec_img_num = 0;

    /*
    @ HSV
    */
    ROI_flg = false;
    get_hsv_flg = false;
    mouse_flg = false;
    capture_end_flg = false;

    COLOR_MODE = COLOR1;
    COLOR_SET(COLOR1);

    Capture_img.create(Size(IMG_W, IMG_H), CV_8UC3);
    // Capture_img_2.create(Size(IMG_W, IMG_H), CV_8UC3);
    Origin_img = ZEROS_3;
    // Origin_img2 = Origin_img.clone();
    Draw_img = ZEROS_3;

    /*
    @ Line Determine
    */

    delta_x = 0;
    DX = 0;
    delta_x_list[0] = 0.f;
    delta_x_list[1] = 0.f;
    delta_x_list[2] = 0.f;
    // delta_x_list[3] = 0.f;
    // delta_x_list[4] = 0.f;
    // tmp_i = 0;
    point_target = Point(160, 240);
    tmp_point_target = Point(160, 120);

    // pix_num_chk.reserve(500);

    line_det_flg = false;
    goal_line_detect_flg = false;
    no_line_det_flg = false;
    roi_line_flg = true;
}