#include <iostream>
#include "dynamixel.hpp"
#include "callback.hpp"
#include "dynamixel_controller.hpp"
#include "Walkingpattern_generator.hpp"

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
    // ros::AsyncSpinner spinner(0); // Multi-threaded spinning
    // spinner.start(); // Multi-threaded spinning

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

    // Origin Gyro
    ros::Publisher IMU_Gryo_x_publisher_; ///< Publishes Imu/gyro.x from reads
    IMU_Gryo_x_publisher_ = nh.advertise<std_msgs::Float32>("/Gyro/x", 100);

    ros::Publisher IMU_Gryo_y_publisher_; ///< Publishes Imu/gyro.y from reads
    IMU_Gryo_y_publisher_ = nh.advertise<std_msgs::Float32>("/Gyro/y", 100);

    ros::Publisher IMU_Gryo_z_publisher_; ///< Publishes Imu/gyro.z from reads
    IMU_Gryo_z_publisher_ = nh.advertise<std_msgs::Float32>("/Gyro/z", 100);

    // Filterd Gyro (LPF)
    ros::Publisher IMU_Gryo_filted_x_publisher_; ///< Publishes Imu/gyro.x from reads
    IMU_Gryo_filted_x_publisher_ = nh.advertise<std_msgs::Float32>("/filtered/Gyro/x", 100);

    ros::Publisher IMU_Gryo_filted_y_publisher_; ///< Publishes Imu/gyro.y from reads
    IMU_Gryo_filted_y_publisher_ = nh.advertise<std_msgs::Float32>("/filtered/Gyro/y", 100);

    ros::Publisher IMU_Gryo_filted_z_publisher_; ///< Publishes Imu/gyro.z from reads
    IMU_Gryo_filted_z_publisher_ = nh.advertise<std_msgs::Float32>("/filtered/Gyro/z", 100);

    // Origin Accel
    ros::Publisher IMU_Accel_x_publisher_; ///< Publishes Imu/accel.x from reads
    IMU_Accel_x_publisher_ = nh.advertise<std_msgs::Float32>("/Accel/x", 100);

    ros::Publisher IMU_Accel_y_publisher_; ///< Publishes Imu/accel.y from reads
    IMU_Accel_y_publisher_ = nh.advertise<std_msgs::Float32>("/Accel/y", 100);

    ros::Publisher IMU_Accel_z_publisher_; ///< Publishes Imu/accel.z from reads
    IMU_Accel_z_publisher_ = nh.advertise<std_msgs::Float32>("/Accel/z", 100);

    // Filterd Accel (HPF)
    ros::Publisher IMU_Accel_filted_x_publisher_; ///< Publishes Imu/accel.x from reads
    IMU_Accel_filted_x_publisher_ = nh.advertise<std_msgs::Float32>("/filtered/Accel/x", 100);

    ros::Publisher IMU_Accel_filted_y_publisher_; ///< Publishes Imu/accel.y from reads
    IMU_Accel_filted_y_publisher_ = nh.advertise<std_msgs::Float32>("/filtered/Accel/y", 100);

    ros::Publisher IMU_Accel_filted_z_publisher_; ///< Publishes Imu/accel.z from reads
    IMU_Accel_filted_z_publisher_ = nh.advertise<std_msgs::Float32>("/filtered/Accel/z", 100);

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

        ///////////// IMU Origin ///////////////
        // Gyro x y z
        geometry_msgs::Vector3 gyro;

        gyro.x = callback.Gyro(0);
        gyro.y = callback.Gyro(1);
        gyro.z = callback.Gyro(2);

        std_msgs::Float32 wx;
        std_msgs::Float32 wy;
        std_msgs::Float32 wz;

        wx.data = gyro.x;
        wy.data = gyro.y;
        wz.data = gyro.z;

        IMU_Gryo_x_publisher_.publish(wx);
        IMU_Gryo_y_publisher_.publish(wy);
        IMU_Gryo_z_publisher_.publish(wz);

        // Accel x y z
        geometry_msgs::Vector3 accel;

        accel.x = callback.Accel(0);
        accel.y = callback.Accel(1);
        accel.z = callback.Accel(2);

        std_msgs::Float32 ax;
        std_msgs::Float32 ay;
        std_msgs::Float32 az;

        ax.data = accel.x;
        ay.data = accel.y;
        az.data = accel.z;

        IMU_Accel_x_publisher_.publish(ax);
        IMU_Accel_y_publisher_.publish(ay);
        IMU_Accel_z_publisher_.publish(az);

        /////////////// IMU Filtering ////////////////
        std_msgs::Float32 wx_f;
        std_msgs::Float32 wy_f;
        std_msgs::Float32 wz_f;

        // Gyro Low_Pass_Filter
        float lpf_y_pre_x = 0;
        float lpf_y_pre_y = 0;
        float lpf_y_pre_z = 0;

        float Ts = 0.002;
        float tau = 0.5;

        // Apply the low-pass filter
        float lpf_y_k_x = dxl.LPF(gyro.x, lpf_y_k_x, Ts, tau);
        // Update the previous filtered value for the next iteration
        lpf_y_pre_x = lpf_y_k_x;
        // Publish_msg
        wx_f.data = lpf_y_k_x;
        IMU_Gryo_filted_x_publisher_.publish(wx_f);

        // Apply the low-pass filter
        float lpf_y_k_y = dxl.LPF(gyro.y, lpf_y_k_y, Ts, tau);
        // Update the previous filtered value for the next iteration
        lpf_y_pre_y = lpf_y_k_y;
        // publish_msg
        wy_f.data = lpf_y_k_y;
        IMU_Gryo_filted_y_publisher_.publish(wy_f);

        // Apply the low-pass filter
        float lpf_y_k_z = dxl.LPF(gyro.z, lpf_y_k_z, Ts, tau);
        // Update the previous filtered value for the next iteration
        lpf_y_pre_z = lpf_y_k_z;
        // publish_msg
        wz_f.data = lpf_y_k_z;
        IMU_Gryo_filted_z_publisher_.publish(wz_f);

        std_msgs::Float32 ax_f;
        std_msgs::Float32 ay_f;
        std_msgs::Float32 az_f;

        float hpf_x_pre_x = 0; // 이전 input (초기값 = 0)
        float hpf_x_pre_y = 0; // 이전 input (초기값 = 0)
        float hpf_x_pre_z = 0; // 이전 input (초기값 = 0)

        float hpf_y_pre_x = 0; // 이전 output (초기값 = 0)
        float hpf_y_pre_y = 0; // 이전 output (초기값 = 0)
        float hpf_y_pre_z = 0; // 이전 output (초기값 = 0)

        // Apply the High-pass filter
        float acc_f_x = dxl.HPF(accel.x, hpf_x_pre_x, hpf_y_pre_x, Ts, tau);
        // Update the previous input and filtered values for the next iteration
        hpf_x_pre_x = accel.x;
        hpf_y_pre_x = acc_f_x;
        // publish_msg
        ax_f.data = acc_f_x;
        IMU_Accel_filted_x_publisher_.publish(ax_f);

        // Apply the High-pass filter
        float acc_f_y = dxl.HPF(accel.y, hpf_x_pre_y, hpf_y_pre_y, Ts, tau);
        // Update the previous input and filtered values for the next iteration
        hpf_x_pre_y = accel.y;
        hpf_y_pre_y = acc_f_y;
        // publish_msg
        ay_f.data = acc_f_y;
        IMU_Accel_filted_y_publisher_.publish(ay_f);

        // Apply the High-pass filter
        float acc_f_z = dxl.HPF(accel.z, hpf_x_pre_z, hpf_y_pre_z, Ts, tau);
        // Update the previous input and filtered values for the next iteration
        hpf_x_pre_z = accel.z;
        hpf_y_pre_z = acc_f_z;
        // publish_msg
        az_f.data = acc_f_z;
        IMU_Accel_filted_z_publisher_.publish(az_f);

        // file write
        //  fprintf(imu_accel, "%d %.lf %.lf %.lf\n",t, callback.Accel(0),callback.Accel(1),callback.Accel(2));
        //  fprintf(imu_gyro, "%d %.lf %.lf %.lf\n",t, callback.Gyro(0),callback.Gyro(1),callback.Gyro(2));

        ros::spinOnce();
        loop_rate.sleep();
    }

    // ROS_INFO("daynmixel_current_2port!");
    dxl.~Dxl();
    return 0;
}
