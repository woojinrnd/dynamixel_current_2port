#include <opencv2/opencv.hpp>

// Line Processing
// delta_x :화면의 중앙.x - 마지막으로 포착한 라인의 중심점.x
// delta_x > 0 : LEFT
// delta_x < 0 : RIGHT
// static double delta_x = 0;
static cv::Point point_target;
static cv::Point tmp_point_target;

