#include <iostream>
#include <time.h>
#include "dynamixel.hpp"
#include "callback.hpp"
#include "dynamixel_controller.hpp"
#include "Walkingpattern_generator.hpp"
#include "sensor.hpp"

#include "dynamixel_current_2port/Select_Motion.h"
#include "dynamixel_current_2port/Turn_Angle.h"

Dxl dxl;
Dxl_Controller dxl_ctrl;
Motions motion;
// Callback callback;

FILE *imu_accel;
FILE *imu_gyro;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "daynmixel_current_2port");
    ros::Time::init();
    ros::Rate loop_rate(1);
    ros::NodeHandle nh;
    Sensor sensor;
    Callback callback;


    // ros::AsyncSpinner spinner(0); // Multi-threaded spinning
    // spinner.start(); // Multi-threaded spinning


    // IMU
    //  imu_accel = fopen("/home/woojin/imu_Accel_0613_(1).dat", "w");
    //  imu_gyro = fopen("/home/woojin/imu_gyro1_0613_(1).dat", "w");

    ros::Publisher joint_state_publisher_; ///< Publishes joint states from reads
    ros::Subscriber joint_state_subscriber_; ///< Gets joint states for writes
    ros::Subscriber FSR_L_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR_L
    ros::Subscriber FSR_R_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR_R
    ros::Subscriber IMU_sensor_subscriber_; ///< Gets IMU Sensor data from XSENSE mti_driver_node
    ros::Subscriber Emergency_subscriber_; ///< Emergency Subscribe

    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("KWJ_joint_states", 100);
    joint_state_subscriber_ = nh.subscribe("KWJ_desired_joint_states", 1000, &Callback::JointStatesCallback, &callback);
    FSR_L_sensor_subscriber_ = nh.subscribe("FSR_L", 1000, &Callback::FSRsensorCallback, &callback);
    FSR_R_sensor_subscriber_ = nh.subscribe("FSR_R", 1000, &Callback::FSRsensorCallback, &callback);
    IMU_sensor_subscriber_ = nh.subscribe("/imu/data", 1000, &Callback::IMUsensorCallback, &callback);
    Emergency_subscriber_ = nh.subscribe("/Move_decision/Emergency", 1000, &Callback::Emergencycallback, &callback);


    // //Client (재민이형 코드에 들어감)
    // ros::ServiceClient client_SM = nh.serviceClient<dynamixel_current_2port::Select_Motion>("/Move_decision/Select_Motion");
    // ros::ServiceClient client_TA = nh.serviceClient<dynamixel_current_2port::Turn_Angle>("/Move_decision/Turn_Angle");



    ros::Subscriber Motion_Selector_; ///< Gets Motion number from motion_decision
    // Motion_Selector_ = nh.subscribe("/Move_decision/Select_Motion", 1000, &Callback::SelectMotion, &callback);

    // ros::waitForShutdown(); // Multi-threaded spinning

    callback.srv_SM.request.finish = 1;


    struct timespec start, end;
    double run_time;
    clock_gettime(CLOCK_REALTIME, &start); // Wall-clock time

    while (ros::ok())
    {

        if (callback.client_SM.call(callback.srv_SM))
        {
            ROS_INFO("#[MESSAGE] SM Request : %d#",  callback.srv_SM.request.finish);
            ROS_INFO("[MESSAGE] SM Response : %d", callback.srv_SM.response.select_motion);
        }
        else
        {
            ROS_INFO("Response Wait...");
        }
        callback.SelectMotion();
        



        ros::spinOnce();
        loop_rate.sleep();

        // About joint msg
        //         sensor_msgs::JointState msg;
        //         msg.header.stamp = ros::Time::now();
        // // , "j3", "j4", "j5", "j6", "j7", "j8", "j9", "j10", "j11", "j12"}
        //         std::vector<std::string> joint_name = {"j1", "j2", "j3"};

        //         for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
        //         {
        //             msg.name.push_back(joint_name.at(i));
        //             // dxl.syncReadTheta();
        //             // msg.position.push_back(dxl.th_[i]);
        //         }
        //         joint_state_publisher_.publish(msg);

        // About FSR
        //  dxl.FSR_flag();
        //  dxl.syncWriteTheta();
        //  std::cout << callback.fsr_value << std::endl;
        

        // About IMU
        // dxl.Quaternino2RPY();
        
        //  for (int i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
        //  {
        //      A[i] = callback.RPY[i];
        //  }
        //  dxl.SetThetaRef(A);
        //  dxl.syncWriteTheta();


        // file write
        //  fprintf(imu_accel, "%d %.lf %.lf %.lf\n",t, callback.Accel(0),callback.Accel(1),callback.Accel(2));
        //  fprintf(imu_gyro, "%d %.lf %.lf %.lf\n",t, callback.Gyro(0),callback.Gyro(1),callback.Gyro(2));

    }

    
    clock_gettime(CLOCK_REALTIME, &end); // Wall-clock time
    run_time = (end.tv_sec - start.tv_sec) * 1000.0 + (end.tv_nsec - start.tv_nsec) / 1000000.0; // 단위는 ms

    cout << run_time << endl;
    // ROS_INFO("daynmixel_current_2port!");
    dxl.~Dxl();
    return 0;
}
