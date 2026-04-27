#ifndef ARM_HARDWARE_INTERFACE_HPP
#define ARM_HARDWARE_INTERFACE_HPP
#include "hardware_interface/system_interface.hpp"
#include "hitrobot/hitbot_interface.h"
#include "hitrobot/ControlBeanEx.h"

namespace arm_hardware_interface {
class ArmHardwareInterface : public hardware_interface::SystemInterface {
public:
    //ros的生命节点覆写
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    //system_interface覆写
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
private:
    int robot_id;
    int ret;
    int IO_COUNT;
    ControlBeanEx *robot;
    void console_pause();

    void Sleep(int ms);
    //------------------2，IO方面的测试---------------------------
    int test_io();
    int test_initial();
};
} // namespace arm_hardware_interface

#endif //  ARM_HARDWARE_INTERFACE_HPP