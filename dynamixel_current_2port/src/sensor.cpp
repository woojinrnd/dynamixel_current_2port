#include "sensor.hpp"

Sensor::Sensor(Motions *motionPtr, Callback *callbackPtr)
    : motionPtr(motionPtr),
      callbackPtr(callbackPtr),
      SPIN_RATE(100)
{
    nh_ = ros::NodeHandle();
    ////////////////Origin////////////////

    ros::NodeHandle nh(ros::this_node::getName());
    boost::thread queue_thread = boost::thread(boost::bind(&Sensor::callbackThead, this));
}

Sensor::~Sensor()
{
}

// ********************************************** PUBLISHER ************************************************** //
// **********************************************  TRHEAD ************************************************** //

void Sensor::callbackThead()
{
    ros::NodeHandle nh(ros::this_node::getName());
    IMU_Gryo_x_publisher_ = nh_.advertise<std_msgs::Float32>("/Gyro/x", 100);
    IMU_Gryo_y_publisher_ = nh_.advertise<std_msgs::Float32>("/Gyro/y", 100);
    IMU_Gryo_z_publisher_ = nh_.advertise<std_msgs::Float32>("/Gyro/z", 100);

    IMU_Accel_x_publisher_ = nh_.advertise<std_msgs::Float32>("/Accel/x", 100);
    IMU_Accel_y_publisher_ = nh_.advertise<std_msgs::Float32>("/Accel/y", 100);
    IMU_Accel_z_publisher_ = nh_.advertise<std_msgs::Float32>("/Accel/z", 100);

    // /////////////Filterd Gyro (LPF)////////////////
    IMU_Gryo_filtered_x_publisher_ = nh_.advertise<std_msgs::Float32>("/filtered/Gyro/x", 100);
    IMU_Gryo_filtered_y_publisher_ = nh_.advertise<std_msgs::Float32>("/filtered/Gyro/y", 100);
    IMU_Gryo_filtered_z_publisher_ = nh_.advertise<std_msgs::Float32>("/filtered/Gyro/z", 100);

    //////////////// Filterd Accel (Integral)/////////////////
    IMU_Velocity_x_publisher_ = nh_.advertise<std_msgs::Float32>("/Velocity/x", 100);
    IMU_Velocity_y_publisher_ = nh_.advertise<std_msgs::Float32>("/Velocity/y", 100);
    IMU_Velocity_z_publisher_ = nh_.advertise<std_msgs::Float32>("/Velocity/z", 100);

    //////////////// Filterd Accel (HPF)/////////////////
    IMU_Accel_filtered_x_publisher_ = nh_.advertise<std_msgs::Float32>("/filtered/Accel/x", 100);
    IMU_Accel_filtered_y_publisher_ = nh_.advertise<std_msgs::Float32>("/filtered/Accel/y", 100);
    IMU_Accel_filtered_z_publisher_ = nh_.advertise<std_msgs::Float32>("/filtered/Accel/z", 100);

    /////////////// Filterd Accel (HPF_Integral) ///////////////////
    IMU_Velocity_filtered_x_publisher_ = nh_.advertise<std_msgs::Float32>("/filtered/Velocity/x", 100);
    IMU_Velocity_filtered_y_publisher_ = nh_.advertise<std_msgs::Float32>("/filtered/Velocity/y", 100);
    IMU_Velocity_filtered_z_publisher_ = nh_.advertise<std_msgs::Float32>("/filtered/Velocity/z", 100);

    /////////////// Filterd Accel (HPF_Integral) ///////////////////
    IMU_Velocity_Complementary_x_publisher_ = nh_.advertise<std_msgs::Float32>("/filtered/Velocity_Complementary/x", 100);
    IMU_Velocity_Complementary_y_publisher_ = nh_.advertise<std_msgs::Float32>("/filtered/Velocity_Complementary/y", 100);
    IMU_Velocity_Complementary_z_publisher_ = nh_.advertise<std_msgs::Float32>("/filtered/Velocity_Complementary/z", 100);

    ros::Rate loop_rate(SPIN_RATE);
    while(nh.ok())
    {   
        Publish_Velocity_Complementary();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// Low Pass Filter
// x_k     input value
// y_pre   previous filtered value
// Ts      sampling time
// tau     time constant
// y_k     output
float Sensor::LPF(float x_k, float y_pre, float Ts, float tau_LPF)
{
    float y_k;
    y_k = (tau_LPF * y_pre + Ts * x_k) / (Ts + tau_LPF);
    return y_k;
}

// Low Pass Filter
// x_k     input value
// y_pre   previous filtered value
// Ts      sampling time
// tau     time constant
// y_k     output
// float Sensor::LPF(const float& x_k, const float& y_pre, const float Ts, const float tau_LPF)
// {
//     return (tau_LPF * y_pre + Ts * x_k) / (Ts + tau_LPF);
// }

// High Pass Filter
// x_k     input value
// x_pre   previous input value
// y_pre   previous filtered value
// Ts      sampling time
// tau     time constant
// y_k     output
float Sensor::HPF(float x_k, float x_pre, float y_pre, float Ts, float tau_HPF)
{
    static float y_k;
    y_k = (tau_HPF / (tau_HPF + Ts) * y_pre) + (tau_HPF / (tau_HPF + Ts)) * (x_k - x_pre);
    return y_k;
}

// High Pass Filter
// x_k     input value
// x_pre   previous input value
// y_pre   previous filtered value
// Ts      sampling time
// tau     time constant
// y_k     output
// float Sensor::HPF(const float& x_k, const float& x_pre, const float& y_pre, const float Ts, const float tau_HPF)
// {
//     return (tau_HPF / (tau_HPF + Ts) * y_pre) + (tau_HPF / (tau_HPF + Ts)) * (x_k - x_pre);
// }

// High Pass Filter + Integral
// x_k     input value
// y_pre   previous filtered value
// Ts      sampling time
// tau     time constant
// y_k     output
float Sensor::HPF_Integral(float x_k, float y_pre, float Ts, float tau_HPF_Integral)
{
    static float y_k;
    y_k = y_pre * (1 - tau_HPF_Integral * Ts) + (x_k * Ts);
    return y_k;
}

// Integral
// x_k     input value
// y_pre   previous filtered value
// Ts      sampling time
// y_k     output
float Sensor::Integral(float x_k, float y_pre, float Ts)
{
    static float y_k;
    y_k = y_pre + x_k * Ts;
    return y_k;
}

// Complementary filter (Linear velocity = alpha*angular_velocity*L + (1-alpha)*HPF_Integral(Linear_accel))
// alpha : Weight
// gyro : Angular Velocity
// HPF_Int : Linear Velocity
float Sensor::Complementary(float gyro, float HPF_Int, float alpha)
{
    float y_k;
    y_k = alpha * gyro * L + (1 - alpha) * HPF_Int;
    return y_k;
}

///////////////////////////////////////// Publish //////////////////////////////////////////

///////////// IMU Origin ///////////////
void Sensor::Publish_Gyro_Origin()
{
    // create message object
    // Gyro x y z
    std_msgs::Float32 wx;
    std_msgs::Float32 wy;
    std_msgs::Float32 wz;

    gyro.x = callbackPtr->Gyro(0);
    gyro.y = callbackPtr->Gyro(1);
    gyro.z = callbackPtr->Gyro(2);
    wx.data = gyro.x;
    wy.data = gyro.y;
    wz.data = gyro.z;

    // publish the message
    IMU_Gryo_x_publisher_.publish(wx);
    IMU_Gryo_y_publisher_.publish(wy);
    IMU_Gryo_z_publisher_.publish(wz);
}

void Sensor::Publish_Accel_Origin()
{
    // Accel x y z
    std_msgs::Float32 ax;
    std_msgs::Float32 ay;
    std_msgs::Float32 az;

    accel.x = callbackPtr->Accel(0);
    accel.y = callbackPtr->Accel(1);
    accel.z = callbackPtr->Accel(2);
    ax.data = accel.x;
    ay.data = accel.y;
    az.data = accel.z;

    IMU_Accel_x_publisher_.publish(ax);
    IMU_Accel_y_publisher_.publish(ay);
    IMU_Accel_z_publisher_.publish(az);
}

/////////////// IMU Filtering ////////////////

///////////////Gyro_LPF//////////////////////
// wx_f : message object
// lpf_y_pre_* : previous output
// lpf_y_k_* : output
void Sensor::Publish_Gyro_LPF()
{
    // Create message objects
    std_msgs::Float32 wx_f;
    std_msgs::Float32 wy_f;
    std_msgs::Float32 wz_f;

    // Apply the low-pass filter and update the previous filtered values
    float lpf_y_k_x = LPF(gyro.x, lpf_y_pre_x, Ts, tau_LPF);
    float lpf_y_k_y = LPF(gyro.y, lpf_y_pre_y, Ts, tau_LPF);
    float lpf_y_k_z = LPF(gyro.z, lpf_y_pre_z, Ts, tau_LPF);

    lpf_y_pre_x = lpf_y_k_x;
    lpf_y_pre_y = lpf_y_k_y;
    lpf_y_pre_z = lpf_y_k_z;

    // Publish the filtered values
    wx_f.data = lpf_y_k_x;
    wy_f.data = lpf_y_k_y;
    wz_f.data = lpf_y_k_z;

    IMU_Gryo_filtered_x_publisher_.publish(wx_f);
    IMU_Gryo_filtered_y_publisher_.publish(wy_f);
    IMU_Gryo_filtered_z_publisher_.publish(wz_f);
}

//////////////////HPF//////////////////////
void Sensor::Publish_Accel_HPF()
{
    std_msgs::Float32 ax_f;
    std_msgs::Float32 ay_f;
    std_msgs::Float32 az_f;

    float acc_f_x = HPF(accel.x, hpf_x_pre_x, hpf_y_pre_x, Ts, tau_HPF);
    float acc_f_y = HPF(accel.y, hpf_x_pre_y, hpf_y_pre_y, Ts, tau_HPF);
    float acc_f_z = HPF(accel.z, hpf_x_pre_z, hpf_y_pre_z, Ts, tau_HPF);

    // Update the previous input and filtered values for the next iteration
    hpf_x_pre_x = accel.x;
    hpf_y_pre_x = acc_f_x;

    hpf_x_pre_y = accel.y;
    hpf_y_pre_y = acc_f_y;

    hpf_x_pre_z = accel.z;
    hpf_y_pre_z = acc_f_z;

    // publish_msg
    ax_f.data = acc_f_x;
    ay_f.data = acc_f_y;
    az_f.data = acc_f_z;

    IMU_Accel_filtered_x_publisher_.publish(ax_f);
    IMU_Accel_filtered_y_publisher_.publish(ay_f);
    IMU_Accel_filtered_z_publisher_.publish(az_f);
}

///////////////Integral//////////////////////
void Sensor::Publish_Velocity_Integral()
{
    std_msgs::Float32 ax_i;
    std_msgs::Float32 ay_i;
    std_msgs::Float32 az_i;

    // Apply Integral filter
    float acc_i_x = Integral(accel.x, hpf_yi_pre_x, Ts);
    float acc_i_y = Integral(accel.y, hpf_yi_pre_y, Ts);
    float acc_i_z = Integral(accel.z, hpf_yi_pre_z, Ts);

    // Update the previous input and filtered values for the next iteration
    hpf_yi_pre_x = acc_i_x;
    hpf_yi_pre_y = acc_i_y;
    hpf_yi_pre_z = acc_i_z;

    // publish_msg
    ax_i.data = acc_i_x; // [cm/s]
    ay_i.data = acc_i_y; // [cm/s]
    az_i.data = acc_i_z; // [cm/s]

    IMU_Velocity_x_publisher_.publish(ax_i);
    IMU_Velocity_y_publisher_.publish(ay_i);
    IMU_Velocity_z_publisher_.publish(az_i);
}

///////////////HPF_Integral//////////////////////
void Sensor::Publish_Velocity_HPF_Integral()
{
    std_msgs::Float32 ax_f;
    std_msgs::Float32 ay_f;
    std_msgs::Float32 az_f;

    // Apply the High-pass + Integral filter
    float acc_f_x = HPF_Integral(accel.x, hpf_y_pre_x, Ts, tau_HPF_Integral);
    float acc_f_y = HPF_Integral(accel.y, hpf_y_pre_y, Ts, tau_HPF_Integral);
    float acc_f_z = HPF_Integral(accel.z, hpf_y_pre_y, Ts, tau_HPF_Integral);

    // Update the previous input and filtered values for the next iteration
    hpf_y_pre_x = acc_f_x;
    hpf_y_pre_y = acc_f_y;
    hpf_y_pre_z = acc_f_z;

    // publish_msg
    ax_f.data = acc_f_x; // [cm/s]
    ay_f.data = acc_f_y; // [cm/s]
    az_f.data = acc_f_z; // [cm/s]

    IMU_Velocity_filtered_x_publisher_.publish(ax_f);
    IMU_Velocity_filtered_y_publisher_.publish(ay_f);
    IMU_Velocity_filtered_z_publisher_.publish(az_f);
}

///////////////////Complementary Filter/////////////////
void Sensor::Publish_Velocity_Complementary()
{
    std_msgs::Float32 ax_c;
    std_msgs::Float32 ay_c;
    std_msgs::Float32 az_c;

    gyro.x = callbackPtr->Gyro(0);
    gyro.y = callbackPtr->Gyro(1);
    gyro.z = callbackPtr->Gyro(2);

    float HPF_Intx = HPF_Integral(accel.x, hpf_y_pre_x, Ts, tau_HPF_Integral);
    float HPF_Inty = HPF_Integral(accel.y, hpf_y_pre_y, Ts, tau_HPF_Integral);
    float HPF_Intz = HPF_Integral(accel.z, hpf_y_pre_z, Ts, tau_HPF_Integral);

    // Update the previous input and filtered values for the next iteration
    hpf_y_pre_x = HPF_Intx;
    hpf_y_pre_y = HPF_Inty;
    hpf_y_pre_z = HPF_Intz;

    float Complementaryx = Complementary(gyro.y, HPF_Intx, alpha);
    float Complementaryy = Complementary(gyro.z, HPF_Inty, alpha);
    float Complementaryz = Complementary(gyro.x, HPF_Intz, alpha);

    ax_c.data = Complementaryx;
    ay_c.data = Complementaryy;
    az_c.data = Complementaryz;

    IMU_Velocity_Complementary_x_publisher_.publish(ax_c);
    IMU_Velocity_Complementary_y_publisher_.publish(ay_c);
    IMU_Velocity_Complementary_z_publisher_.publish(az_c);
}

//////////////////////////////FUNCTION//////////////////////////
// argument : motion
//  MatrixXd Sensor::GetCapturePoint()
//  {

// }

// MatrixXd Reference_CP_CM(MatrixXd &_motion)
// {
//     uint8_t a = callbackPtr->mode;
//     if (a == 0) return //CP_CM
// }
