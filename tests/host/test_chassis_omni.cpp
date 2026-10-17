#include <math.h>

#include "test_assert.h"
#include "3_Module/Chassis/Omni/mod_chassis_omni.h"

namespace
{
void Assert_Near(const float value, const float expected)
{
    assert(fabsf(value - expected) < 1.0e-5f);
}

void Assert_All_Wheels_Zero(const Class_Chassis_Omni &chassis)
{
    Assert_Near(chassis.Get_Wheel_Omega_RF(), 0.0f);
    Assert_Near(chassis.Get_Wheel_Omega_LF(), 0.0f);
    Assert_Near(chassis.Get_Wheel_Omega_RB(), 0.0f);
    Assert_Near(chassis.Get_Wheel_Omega_LB(), 0.0f);
}

void Test_Init_And_Zero_Input_Clear_All_Wheels()
{
    Class_Chassis_Omni chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Calculate();

    Assert_All_Wheels_Zero(chassis);
}

void Test_Pure_X_Uses_USTC_Sqrt2_Over_2_Coefficients()
{
    Class_Chassis_Omni chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_X(1.0f);
    chassis.Calculate();

    Assert_Near(chassis.Get_Wheel_Omega_RF(), -7.07106781f);
    Assert_Near(chassis.Get_Wheel_Omega_LF(), -7.07106781f);
    Assert_Near(chassis.Get_Wheel_Omega_RB(), 7.07106781f);
    Assert_Near(chassis.Get_Wheel_Omega_LB(), 7.07106781f);
}

void Test_Pure_Y_Uses_USTC_Sqrt2_Over_2_Coefficients()
{
    Class_Chassis_Omni chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_Y(1.0f);
    chassis.Calculate();

    Assert_Near(chassis.Get_Wheel_Omega_RF(), 7.07106781f);
    Assert_Near(chassis.Get_Wheel_Omega_LF(), -7.07106781f);
    Assert_Near(chassis.Get_Wheel_Omega_RB(), 7.07106781f);
    Assert_Near(chassis.Get_Wheel_Omega_LB(), -7.07106781f);
}

void Test_Pure_W_Gives_Equal_Positive_Wheel_Speeds()
{
    Class_Chassis_Omni chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_W(2.0f);
    chassis.Calculate();

    Assert_Near(chassis.Get_Wheel_Omega_RF(), 10.0f);
    Assert_Near(chassis.Get_Wheel_Omega_LF(), 10.0f);
    Assert_Near(chassis.Get_Wheel_Omega_RB(), 10.0f);
    Assert_Near(chassis.Get_Wheel_Omega_LB(), 10.0f);
}

void Test_Combined_Velocity_Matches_All_Four_Formulas()
{
    Class_Chassis_Omni chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_X(0.6f);
    chassis.Set_Speed_Y(-0.2f);
    chassis.Set_Speed_W(1.5f);
    chassis.Calculate();

    Assert_Near(chassis.Get_Wheel_Omega_RF(), 1.84314575f);
    Assert_Near(chassis.Get_Wheel_Omega_LF(), 4.67157288f);
    Assert_Near(chassis.Get_Wheel_Omega_RB(), 10.32842712f);
    Assert_Near(chassis.Get_Wheel_Omega_LB(), 13.15685425f);
}

void Test_Zero_And_Negative_Wheel_Radius_Clear_All_Wheels()
{
    Class_Chassis_Omni chassis;
    chassis.Init(0.0f, 0.5f);
    chassis.Set_Speed_X(1.0f);
    chassis.Set_Speed_Y(1.0f);
    chassis.Set_Speed_W(1.0f);
    chassis.Calculate();
    Assert_All_Wheels_Zero(chassis);

    chassis.Init(-0.1f, 0.5f);
    chassis.Set_Speed_X(1.0f);
    chassis.Calculate();
    Assert_All_Wheels_Zero(chassis);
}

void Test_Reinit_Clears_Previous_Input_And_Output()
{
    Class_Chassis_Omni chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_X(1.0f);
    chassis.Calculate();
    assert(chassis.Get_Wheel_Omega_RF() != 0.0f);

    chassis.Init(0.1f, 0.5f);
    chassis.Calculate();
    Assert_All_Wheels_Zero(chassis);
}
}

int main()
{
    Test_Init_And_Zero_Input_Clear_All_Wheels();
    Test_Pure_X_Uses_USTC_Sqrt2_Over_2_Coefficients();
    Test_Pure_Y_Uses_USTC_Sqrt2_Over_2_Coefficients();
    Test_Pure_W_Gives_Equal_Positive_Wheel_Speeds();
    Test_Combined_Velocity_Matches_All_Four_Formulas();
    Test_Zero_And_Negative_Wheel_Radius_Clear_All_Wheels();
    Test_Reinit_Clears_Previous_Input_And_Output();
    return 0;
}
