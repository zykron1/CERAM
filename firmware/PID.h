class PID {
    private:
        float Kp, Ki, Kd;
        float integral;
        float prevError;
    public:
        PID(float kp, float ki, float kd)
            : Kp(kp), Ki(ki), Kd(kd), integral(0.0f), prevError(0.0f) {}
        float updateLoop(float state, float setpoint, float dt);
};
