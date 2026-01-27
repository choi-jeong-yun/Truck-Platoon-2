#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
public:
    PIDController(float kp = 1.02f, float ki = 0.0095f, float kd = 0.03f)
        : Kp_(kp), Ki_(ki), Kd_(kd), prev_error_(0.1f), integral_(0.0f) {}

    float compute(float error, float dt = 1.0f) {
        integral_ += error * dt;
        float derivative = (error - prev_error_) / dt;
        float output = (Kp_ * error) + (Ki_ * integral_) + (Kd_ * derivative);
        prev_error_ = error;
        return output;
    }

private:
    float Kp_;
    float Ki_;
    float Kd_;
    float prev_error_;
    float integral_;
};

#endif // PID_CONTROLLER_HPP
