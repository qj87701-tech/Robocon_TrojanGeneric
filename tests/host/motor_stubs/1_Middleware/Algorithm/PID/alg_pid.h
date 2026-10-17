#ifndef TESTS_HOST_MOTOR_STUBS_ALG_PID_H
#define TESTS_HOST_MOTOR_STUBS_ALG_PID_H

class Class_PID
{
public:
    void Set_Integral_Error(const float &) {}
    void Set_Target(const float &) {}
    void Set_Now(const float &) {}
    void TIM_Calculate_PeriodElapsedCallback() {}
    float Get_Out() const { return 0.0f; }
};

#endif
