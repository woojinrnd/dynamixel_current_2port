#include <iostream>
#include <time.h>
#include "sensor.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Sensor_node");
    ros::Time::init();
    ros::Rate loop_rate(400);
    ros::NodeHandle nh;

    Sensor sensor;

    // Timer
    struct timespec start, end;
    double run_time;
    clock_gettime(CLOCK_REALTIME, &start); // Wall-clock time

    // callback.Write_Arm_Theta();
    // callback.MotionMaker();

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    clock_gettime(CLOCK_REALTIME, &end);                                                         // Wall-clock time
    run_time = (end.tv_sec - start.tv_sec) * 1000.0 + (end.tv_nsec - start.tv_nsec) / 1000000.0; // 단위는 ms

    return 0;
}