#include "test_assert.h"
#include "4_Task/tsk_motor_init/motor_init.h"
#include "fdcan.h"

FDCAN_HandleTypeDef hfdcan1{FDCAN1};
FDCAN_HandleTypeDef hfdcan3{FDCAN3};
int fake_sequence = 0;

namespace
{
void Assert_Swerve_Motor(const Class_Motor_DM_Normal &motor, const std::uint8_t master_id, const std::uint8_t motor_id)
{
    const Fake_Motor_State &state = motor.Get_Fake_State();
    assert(state.can_handler == &hfdcan1);
    assert(state.init_rx_id == master_id);
    assert(state.init_tx_id == motor_id);
    assert(state.method == Motor_DM_Control_Method_NORMAL_MIT);
    assert(state.angle_max == 251.3f);
    assert(state.omega_max == 45.0f);
    assert(state.torque_max == 10.0f);
    assert(state.control_angle == 0.0f);
    assert(state.control_omega == 0.0f);
    assert(state.control_torque == 0.0f);
    assert(state.k_p == 7.0f);
    assert(state.k_d == 0.20f);
    assert(!state.motor_enable);
    assert(state.enter_count == 0);
}

void Assert_VESC_Motor(const Class_Motor_VESC &motor, const std::uint8_t vesc_id)
{
    const Fake_VESC_State &state = motor.Get_Fake_State();
    assert(state.can_handler == &hfdcan3);
    assert(state.vesc_id == vesc_id);
    assert(state.pole_pairs == 21U);
    assert(state.gearbox_rate == 1.0f);
    assert(state.erpm_max == 30240);
    assert(state.current_max == 5.0f);
    assert(state.current_brake_max == 5.0f);
    assert(state.control_method == Motor_VESC_Control_Method_RPM);
    assert(state.status == Motor_VESC_Status_DISABLE);
    assert(!state.motor_enable);
    assert(state.control_erpm == 0);
    assert(state.control_omega == 0.0f);
}

void Test_J8006_And_Four_DM6220_Initialization_Contract()
{
    fake_sequence = 0;
    Motor_Init();

    const Fake_Motor_State &j8006 = DM_J8006.Get_Fake_State();
    assert(DM_J8006_MASTER_ID == 0x55U);
    assert(j8006.can_handler == &hfdcan1);
    assert(j8006.init_rx_id == 0x55U);
    assert(j8006.init_tx_id == 0x05U);
    assert(j8006.method == Motor_DM_Control_Method_NORMAL_OMEGA);
    assert(j8006.angle_max == 12.5f);
    assert(j8006.omega_max == 45.0f);
    assert(j8006.torque_max == 20.0f);
    assert(j8006.control_omega == 0.0f);
    assert(!j8006.motor_enable);
    assert(j8006.enter_count == 0);

    assert(DM_CHASSIS_LF_MASTER_ID == 0x11U);
    assert(DM_CHASSIS_RF_MASTER_ID == 0x22U);
    assert(DM_CHASSIS_LB_MASTER_ID == 0x33U);
    assert(DM_CHASSIS_RB_MASTER_ID == 0x44U);
    Assert_Swerve_Motor(DM_Chassis_LF, 0x11U, 0x01U);
    Assert_Swerve_Motor(DM_Chassis_RF, 0x22U, 0x02U);
    Assert_Swerve_Motor(DM_Chassis_LB, 0x33U, 0x03U);
    Assert_Swerve_Motor(DM_Chassis_RB, 0x44U, 0x04U);

    Assert_VESC_Motor(VESC_Chassis_LF, 1U);
    Assert_VESC_Motor(VESC_Chassis_RF, 2U);
    Assert_VESC_Motor(VESC_Chassis_LB, 3U);
    Assert_VESC_Motor(VESC_Chassis_RB, 4U);
}

}

int main()
{
    Test_J8006_And_Four_DM6220_Initialization_Contract();
    return 0;
}
