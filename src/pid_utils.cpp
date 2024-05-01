#include "pid_utils.hpp"

namespace vel_pid_ros2
{
    PID::PID(float p_gain_, float i_gain_, float d_gain_):p_gain(p_gain_),d_gain(d_gain_),i_gain(i_gain_){}

    float PID::calc_pid(float target, float now, float delta_time)
    {
        proporsal = target - now;
        integral += proporsal * delta_time;
        differential = (proporsal - previous_proporsal) / delta_time;
        previous_proporsal = proporsal;

        return proporsal * p_gain + integral * i_gain + differential * d_gain;
    }

    void PID::set_gain(float p_gain_, float i_gain_, float d_gain_)
    {
        p_gain = p_gain_;
        i_gain = i_gain_;
        d_gain = d_gain_;
    }
}