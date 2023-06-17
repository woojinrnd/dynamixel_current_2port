#include <iostream>
#include "dynamixel.hpp"
#include "callback.hpp"
#include "dynamixel_controller.hpp"
#include "Walkingpattern_generator.hpp"
#include "sensor.hpp"


Dxl dxl;
Callback callback;
Dxl_Controller dxl_ctrl;
Motions motion;

FILE *imu_accel;
FILE *imu_gyro;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "daynmixel_current_2port");
    ros::Time::init();
    ros::Rate loop_rate(500);
    ros::NodeHandle nh;
    Sensor sensor; 

    ros::AsyncSpinner spinner(0); // Multi-threaded spinning
    spinner.start(); // Multi-threaded spinning

    // IMU
    //  imu_accel = fopen("/home/woojin/imu_Accel_0613_(1).dat", "w");
    //  imu_gyro = fopen("/home/woojin/imu_gyro1_0613_(1).dat", "w");

    ros::Publisher joint_state_publisher_; ///< Publishes joint states from reads
    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("KWJ_joint_states", 100);

    ros::Subscriber joint_state_subscriber_; ///< Gets joint states for writes
    joint_state_subscriber_ = nh.subscribe("KWJ_desired_joint_states", 1000, &Callback::JointStatesCallback, &callback);

    ros::Subscriber FSR_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR
    FSR_sensor_subscriber_ = nh.subscribe("FSR", 1000, &Callback::FSRsensorCallback, &callback);

    ros::Subscriber IMU_sensor_subscriber_; ///< Gets IMU Sensor data from XSENSE mti_driver_node
    IMU_sensor_subscriber_ = nh.subscribe("/imu/data", 1000, &Callback::IMUsensorCallback, &callback);


    // ros::waitForShutdown(); // Multi-threaded spinning

    VectorXd A(NUMBER_OF_DYNAMIXELS);
    for (int i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        A[i] = 90 * DEG2RAD;
    }
    dxl.SetThetaRef(A);
    dxl.syncWriteTheta();

    int t = 0;

    while (ros::ok())
    {
        // About motion
        t += 1;

        // for (int i = 0; i < 6; i++)
        // {
        //     A[i] = RL_motion1(t, i);
        // }

        // for (int i=6;i<12;i++)
        // {
        //     A[i] = LL_Motion1(t, i);
        // }

        // if (t >= 923)
        //     t = 0;

        // dxl.SetThetaRef(A);
        // dxl.syncWriteTheta();

        // for (int i = 0; i < NUMBER_OF_DYNAMIXELS; i ++)
        // cout << "A[" << i << "] : " << A[i] << endl;

        dxl.SetThetaRef(A);
        dxl.syncWriteTheta();

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
        //  dxl.Quaternino2RPY();
        //  for (int i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
        //  {
        //      A[i] = callback.RPY[i];
        //  }
        //  dxl.SetThetaRef(A);
        //  dxl.syncWriteTheta();


        // file write
        //  fprintf(imu_accel, "%d %.lf %.lf %.lf\n",t, callback.Accel(0),callback.Accel(1),callback.Accel(2));
        //  fprintf(imu_gyro, "%d %.lf %.lf %.lf\n",t, callback.Gyro(0),callback.Gyro(1),callback.Gyro(2));

        sensor.Publish_Accel_Origin();
        sensor.Publish_Gyro_Origin();
        sensor.Publish_Accel_HPF();
        sensor.Publish_Gyro_LPF();
        sensor.Publish_Velocity_HPF_Integral();
        sensor.Publish_Velocity_Integral();

        ros::spinOnce();
        loop_rate.sleep();
    }

    // ROS_INFO("daynmixel_current_2port!");
    dxl.~Dxl();
    return 0;
}
