#ifndef PID_UTILS_HPP_
#define PID_UTILS_HPP_

namespace vel_pid_ros2
{
    struct PID
    {
        public:
        PID(float p_gain_=0.1, float i_gain_=0.1, float d_gain_=0.1);
        void set_gain(float p_gain_, float i_gain_, float d_gain_);
        float calc_pid(float target, float now, float delta_time);

        private:
        float p_gain;
        float i_gain;
        float d_gain;

        float proporsal{0.0};
        float differential{0.0};
        float integral{0.0};
        float previous_proporsal{0.0};
    };
}

#endif