#include <math.h>
#include <stdint.h>

#include "test_assert.h"
#include "3_Module/Chassis/Swerve/mod_chassis_swerve.h"

namespace
{
#define TEST_PI 3.14159265358979323846f
#define TEST_SQRT_2_DIV_2 0.70710678118f

void Assert_Near(const float value, const float expected)
{
    assert(fabsf(value - expected) < 1.0e-5f);
}

void Assert_All_Wheels(const Class_Chassis_Swerve &chassis,
                       const float target_angle, const float wheel_omega)
{
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_RF), target_angle);
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), target_angle);
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_RB), target_angle);
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LB), target_angle);

    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_RF), wheel_omega);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_LF), wheel_omega);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_RB), wheel_omega);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_LB), wheel_omega);
}

void Test_Forward_Translation_Points_All_Wheels_Forward()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_Y(1.0f);
    chassis.Calculate();

    Assert_All_Wheels(chassis, 0.0f, 10.0f);
}

void Test_Right_Translation_Points_All_Wheels_Right()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_X(1.0f);
    chassis.Calculate();

    Assert_All_Wheels(chassis, TEST_PI / 2.0f, 10.0f);
}

void Test_Pure_Rotation_Calculates_Tangent_Wheel_Vectors()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_W(2.0f);
    chassis.Calculate();

    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_RF), -TEST_PI / 4.0f);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_RF), 10.0f);
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), TEST_PI / 4.0f);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_LF), -10.0f);
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_RB), TEST_PI / 4.0f);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_RB), 10.0f);
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LB), -TEST_PI / 4.0f);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_LB), -10.0f);
}

void Test_Combined_Motion_Can_Stop_Only_One_Wheel()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_X(1.0f);
    chassis.Calculate();
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_RF), TEST_PI / 2.0f);

    float speed = 0.5f * TEST_SQRT_2_DIV_2;
    chassis.Set_Speed_X(speed);
    chassis.Set_Speed_Y(-speed);
    chassis.Set_Speed_W(1.0f);
    chassis.Calculate();

    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_RF), 0.0f);
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_RF), TEST_PI / 2.0f);
    assert(chassis.Get_Wheel_Omega(Swerve_Wheel_LF) != 0.0f);
    assert(chassis.Get_Wheel_Omega(Swerve_Wheel_RB) != 0.0f);
    assert(chassis.Get_Wheel_Omega(Swerve_Wheel_LB) != 0.0f);
}

void Test_Speed_Below_Dead_Zone_Holds_Last_Target()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_X(1.0f);
    chassis.Calculate();

    chassis.Set_Speed_X(0.0005f);
    chassis.Calculate();

    Assert_All_Wheels(chassis, TEST_PI / 2.0f, 0.0f);
}

void Test_First_Zero_Command_Holds_Current_Steer_Angle()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.07f, 0.45f);
    chassis.Set_Now_Angle(Swerve_Wheel_LF, 0.60f);
    chassis.Set_Speed_X(0.0f);
    chassis.Set_Speed_Y(0.0f);
    chassis.Set_Speed_W(0.0f);
    chassis.Calculate();

    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), 0.60f);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_LF), 0.0f);
}

void Test_Zero_Command_Tracks_Late_Feedback_Until_First_Motion()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.07f, 0.45f);
    chassis.Set_Now_Angle(Swerve_Wheel_LF, 0.0f);
    chassis.Calculate();
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), 0.0f);

    chassis.Set_Now_Angle(Swerve_Wheel_LF, 0.60f);
    chassis.Calculate();
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), 0.60f);

    chassis.Set_Speed_X(1.0f);
    chassis.Calculate();
    float motion_target = chassis.Get_Target_Angle(Swerve_Wheel_LF);
    chassis.Set_Speed_X(0.0f);
    chassis.Set_Now_Angle(Swerve_Wheel_LF, 0.20f);
    chassis.Calculate();
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), motion_target);
}

void Test_Low_Speed_Command_Uses_Start_Stop_Hysteresis()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Now_Angle(Swerve_Wheel_LF, 0.25f);

    // 尚未进入运动状态时，0.08m/s的小幅输入不应改变舵向。
    chassis.Set_Speed_X(0.08f);
    chassis.Calculate();
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), 0.25f);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_LF), 0.0f);

    // 超过0.10m/s后开始解算舵向和轮速。
    chassis.Set_Speed_X(0.11f);
    chassis.Calculate();
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), TEST_PI / 2.0f);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_LF), 1.10f);

    // 已进入运动状态后，回落到0.05~0.10m/s之间仍保持连续控制。
    chassis.Set_Speed_X(0.08f);
    chassis.Calculate();
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), TEST_PI / 2.0f);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_LF), 0.80f);

    // 低于0.05m/s后退出运动状态并保持最后舵向。
    chassis.Set_Speed_X(0.04f);
    chassis.Calculate();
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), TEST_PI / 2.0f);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_LF), 0.0f);
}

void Test_Zero_And_Negative_Wheel_Radius_Clear_Speeds_And_Hold_Angles()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.0f, 0.5f);
    chassis.Set_Speed_X(1.0f);
    chassis.Calculate();
    Assert_All_Wheels(chassis, 0.0f, 0.0f);

    chassis.Init(-0.1f, 0.5f);
    chassis.Set_Speed_Y(1.0f);
    chassis.Calculate();
    Assert_All_Wheels(chassis, 0.0f, 0.0f);
}

void Test_Reinit_Clears_All_Wheel_State()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_X(1.0f);
    chassis.Calculate();
    assert(chassis.Get_Wheel_Omega(Swerve_Wheel_RF) != 0.0f);

    chassis.Init(0.1f, 0.5f);
    chassis.Calculate();
    Assert_All_Wheels(chassis, 0.0f, 0.0f);
}

void Test_Invalid_Wheel_Enum_Is_Ignored_And_Reads_Zero()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    Enum_Swerve_Wheel invalid_wheel = static_cast<Enum_Swerve_Wheel>(255U);

    chassis.Set_Now_Angle(invalid_wheel, 1.0f);
    Assert_Near(chassis.Get_Target_Angle(invalid_wheel), 0.0f);
    Assert_Near(chassis.Get_Wheel_Omega(invalid_wheel), 0.0f);
}

void Test_Error_Greater_Than_Positive_Half_Pi_Reverses_Wheel()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_X(TEST_SQRT_2_DIV_2);
    chassis.Set_Speed_Y(-TEST_SQRT_2_DIV_2);
    chassis.Calculate();

    Assert_All_Wheels(chassis, -TEST_PI / 4.0f, -10.0f);
}

void Test_Error_Less_Than_Negative_Half_Pi_Reverses_Wheel()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_X(-TEST_SQRT_2_DIV_2);
    chassis.Set_Speed_Y(-TEST_SQRT_2_DIV_2);
    chassis.Calculate();

    Assert_All_Wheels(chassis, TEST_PI / 4.0f, -10.0f);
}

void Test_Error_Exactly_Positive_Or_Negative_Half_Pi_Does_Not_Reverse()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_X(1.0f);
    chassis.Calculate();
    Assert_All_Wheels(chassis, TEST_PI / 2.0f, 10.0f);

    chassis.Init(0.1f, 0.5f);
    chassis.Set_Speed_X(-1.0f);
    chassis.Calculate();
    Assert_All_Wheels(chassis, -TEST_PI / 2.0f, 10.0f);
}

void Test_Ninety_Degree_Optimization_Uses_Hysteresis()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(1.0f, 0.0f);
    chassis.Set_Now_Angle(Swerve_Wheel_LF, 0.0f);

    chassis.Set_Speed_X(sinf(96.0f * TEST_PI / 180.0f));
    chassis.Set_Speed_Y(cosf(96.0f * TEST_PI / 180.0f));
    chassis.Calculate();
    assert(chassis.Get_Wheel_Omega(Swerve_Wheel_LF) < 0.0f);

    chassis.Set_Speed_X(sinf(89.0f * TEST_PI / 180.0f));
    chassis.Set_Speed_Y(cosf(89.0f * TEST_PI / 180.0f));
    chassis.Calculate();
    assert(chassis.Get_Wheel_Omega(Swerve_Wheel_LF) < 0.0f);

    chassis.Set_Speed_X(sinf(84.0f * TEST_PI / 180.0f));
    chassis.Set_Speed_Y(cosf(84.0f * TEST_PI / 180.0f));
    chassis.Calculate();
    assert(chassis.Get_Wheel_Omega(Swerve_Wheel_LF) > 0.0f);
}

void Test_Target_Remains_Continuous_Across_Positive_Pi_Boundary()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Now_Angle(Swerve_Wheel_RF, 3.13f);
    chassis.Set_Speed_X(sinf(-3.13f));
    chassis.Set_Speed_Y(cosf(-3.13f));
    chassis.Calculate();

    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_RF), 3.15318531f);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_RF), 10.0f);
}

void Test_Target_Uses_Nearest_Equivalent_Angle_Beyond_One_Revolution()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f, 0.0f);
    chassis.Set_Now_Angle(Swerve_Wheel_LF, 4.0f * TEST_PI + 0.10f);
    chassis.Set_Speed_Y(1.0f);
    chassis.Calculate();

    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), 4.0f * TEST_PI);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_LF), 10.0f);
}

void Test_Repeated_Calculate_Does_Not_Accumulate_Target_Error()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Now_Angle(Swerve_Wheel_RF, 0.2f);
    chassis.Set_Speed_X(0.6f);
    chassis.Set_Speed_Y(0.8f);
    chassis.Calculate();
    float first_target = chassis.Get_Target_Angle(Swerve_Wheel_RF);

    chassis.Calculate();
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_RF), first_target);
}

void Test_Negative_Angle_Limit_Uses_Equivalent_Reversed_Target()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Now_Angle(Swerve_Wheel_LF, -12.42f);
    chassis.Set_Speed_Y(1.0f);
    chassis.Calculate();

    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), -3.0f * TEST_PI);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_LF), -10.0f);
}

void Test_Positive_Angle_Limit_Uses_Equivalent_Reversed_Target()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f);
    chassis.Set_Now_Angle(Swerve_Wheel_LF, 12.42f);
    chassis.Set_Speed_Y(1.0f);
    chassis.Calculate();

    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), 3.0f * TEST_PI);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_LF), -10.0f);
}

void Test_Reset_Wheel_Uses_Current_Angle_As_Held_Target()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f, 245.0f);
    chassis.Set_Now_Angle(Swerve_Wheel_LF, 6.2f);
    chassis.Reset_Wheel(Swerve_Wheel_LF, 6.2f);

    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), 6.2f);
    Assert_Near(chassis.Get_Wheel_Omega(Swerve_Wheel_LF), 0.0f);
    chassis.Calculate();
    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), 6.2f);
}

void Test_Reset_Wheel_Locks_Upper_Computer_Zero_After_Feedback_Moves()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f, 245.0f);
    chassis.Set_Now_Angle(Swerve_Wheel_LF, 0.03f);
    chassis.Reset_Wheel(Swerve_Wheel_LF, 0.0f);

    chassis.Set_Now_Angle(Swerve_Wheel_LF, 0.2f);
    chassis.Calculate();

    Assert_Near(chassis.Get_Target_Angle(Swerve_Wheel_LF), 0.0f);
}

void Test_Reset_Wheel_Clears_Previous_Reversed_Choice()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(1.0f, 0.0f, 245.0f);
    chassis.Set_Now_Angle(Swerve_Wheel_LF, 0.0f);
    chassis.Set_Speed_X(sinf(96.0f * TEST_PI / 180.0f));
    chassis.Set_Speed_Y(cosf(96.0f * TEST_PI / 180.0f));
    chassis.Calculate();
    assert(chassis.Get_Wheel_Omega(Swerve_Wheel_LF) < 0.0f);

    chassis.Reset_Wheel(Swerve_Wheel_LF, 0.0f);
    chassis.Set_Speed_X(sinf(89.0f * TEST_PI / 180.0f));
    chassis.Set_Speed_Y(cosf(89.0f * TEST_PI / 180.0f));
    chassis.Calculate();
    assert(chassis.Get_Wheel_Omega(Swerve_Wheel_LF) > 0.0f);
}

void Test_Nearest_Target_Stays_Inside_245_Radian_Soft_Limit()
{
    Class_Chassis_Swerve chassis;
    chassis.Init(0.1f, 0.5f, 245.0f);
    chassis.Set_Now_Angle(Swerve_Wheel_LF, 244.9f);
    chassis.Set_Speed_X(1.0f);
    chassis.Calculate();

    assert(fabsf(chassis.Get_Target_Angle(Swerve_Wheel_LF)) <= 245.0f);
}
}

int main()
{
    Test_Forward_Translation_Points_All_Wheels_Forward();
    Test_Right_Translation_Points_All_Wheels_Right();
    Test_Pure_Rotation_Calculates_Tangent_Wheel_Vectors();
    Test_Combined_Motion_Can_Stop_Only_One_Wheel();
    Test_Speed_Below_Dead_Zone_Holds_Last_Target();
    Test_First_Zero_Command_Holds_Current_Steer_Angle();
    Test_Zero_Command_Tracks_Late_Feedback_Until_First_Motion();
    Test_Low_Speed_Command_Uses_Start_Stop_Hysteresis();
    Test_Zero_And_Negative_Wheel_Radius_Clear_Speeds_And_Hold_Angles();
    Test_Reinit_Clears_All_Wheel_State();
    Test_Invalid_Wheel_Enum_Is_Ignored_And_Reads_Zero();
    Test_Error_Greater_Than_Positive_Half_Pi_Reverses_Wheel();
    Test_Error_Less_Than_Negative_Half_Pi_Reverses_Wheel();
    Test_Error_Exactly_Positive_Or_Negative_Half_Pi_Does_Not_Reverse();
    Test_Ninety_Degree_Optimization_Uses_Hysteresis();
    Test_Target_Remains_Continuous_Across_Positive_Pi_Boundary();
    Test_Target_Uses_Nearest_Equivalent_Angle_Beyond_One_Revolution();
    Test_Repeated_Calculate_Does_Not_Accumulate_Target_Error();
    Test_Negative_Angle_Limit_Uses_Equivalent_Reversed_Target();
    Test_Positive_Angle_Limit_Uses_Equivalent_Reversed_Target();
    Test_Reset_Wheel_Uses_Current_Angle_As_Held_Target();
    Test_Reset_Wheel_Locks_Upper_Computer_Zero_After_Feedback_Moves();
    Test_Reset_Wheel_Clears_Previous_Reversed_Choice();
    Test_Nearest_Target_Stays_Inside_245_Radian_Soft_Limit();
    return 0;
}
