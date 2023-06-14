#include "sensor.hpp"

Sensor::Sensor() {}

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


// High Pass Filter + Integral
// x_k     input value
// y_pre   previous filtered value
// Ts      sampling time
// tau     time constant
// y_k     output
float Sensor::HPF_Integral(float x_k, float y_pre, float Ts, float tau_HPF_Integral)
{
    static float y_k;
    y_k = y_pre*(1 - tau_HPF_Integral*Ts) + (x_k*Ts);
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
    y_k = y_pre + x_k*Ts;
    return y_k;
}


//////////////////Publish/////////////////////////

