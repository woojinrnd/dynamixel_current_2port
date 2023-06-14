#ifndef SENSOR_H
#define SENSOR_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include "callback.hpp"
#include "dynamixel.hpp"

extern Callback callback;

class Sensor
{
private:
    ros::NodeHandle nh_;
    // Origin Gyro
    ros::Publisher IMU_Gryo_x_publisher_; ///< Publishes Imu/gyro.x from reads

    ros::Publisher IMU_Gryo_y_publisher_; ///< Publishes Imu/gyro.y from reads

    ros::Publisher IMU_Gryo_z_publisher_; ///< Publishes Imu/gyro.z from reads

    // Sampling time, tau value
    float Ts = 0.002; // 1/ 500
    float tau_LPF = 0.5;
    float tau_HPF = 0.5;
    float tau_HPF_Integral = 0.5;

    ///////////// IMU Origin ///////////////
    // Gyro x y z
    geometry_msgs::Vector3 gyro;

    std_msgs::Float32 wx;
    std_msgs::Float32 wy;
    std_msgs::Float32 wz;

    IMU_Gryo_x_publisher_.publish(wx);
    IMU_Gryo_y_publisher_.publish(wy);
    IMU_Gryo_z_publisher_.publish(wz);

    // Accel x y z
    geometry_msgs::Vector3 accel;

    std_msgs::Float32 ax;
    std_msgs::Float32 ay;
    std_msgs::Float32 az;

    IMU_Accel_x_publisher_.publish(ax);
    IMU_Accel_y_publisher_.publish(ay);
    IMU_Accel_z_publisher_.publish(az);

public:
    Sensor()
    {
        nh_ = ros::NodeHandle();
        IMU_Gryo_x_publisher_ = nh_.advertise<std_msgs::Float32>("/Gyro/x", 100);
        IMU_Gryo_y_publisher_ = nh_.advertise<std_msgs::Float32>("/Gyro/y", 100);
        IMU_Gryo_z_publisher_ = nh_.advertise<std_msgs::Float32>("/Gyro/z", 100);

        ///////////// IMU Origin ///////////////
        gyro.x = callback.Gyro(0);
        gyro.y = callback.Gyro(1);
        gyro.z = callback.Gyro(2);
        wx.data = gyro.x;
        wy.data = gyro.y;
        wz.data = gyro.z;

        accel.x = callback.Accel(0);
        accel.y = callback.Accel(1);
        accel.z = callback.Accel(2);

        ax.data = accel.x;
        ay.data = accel.y;
        az.data = accel.z;
    }

    virtual float LPF(float x_k, float y_pre, float Ts, float tau_LPF);
    virtual float HPF(float x_k, float x_pre, float y_pre, float Ts, float tau_HPF);
    virtual float HPF_Integral(float x_k, float y_pre, float Ts, float tau_HPF_Integral);
    virtual float Integral(float x_k, float y_pre, float Ts);
};

#endif