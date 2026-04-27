#include "my_robot_hardware/arm_hardware_interface.hpp"
namespace arm_hardware_interface
{
    // 以下是私有方法定义
    void ArmHardwareInterface::console_pause(){
    #ifdef WINDOWS
        system("pause");
    #endif // WINDOWS
    }
    void ArmHardwareInterface::Sleep(int ms)
    {
    #ifndef WINDOWS
        usleep(ms * 1000);

    #endif
    }
    int ArmHardwareInterface::test_initial()
    {
        net_port_initial();
        if (card_number_connect(robot_id))
        {
            robot = get_robot(robot_id);
            ret = robot->initial(1, 210);
            robot->unlock_position();
            if (ret != 1)
            {
                std::cout << "initial ret = " << ret << std::endl;
                console_pause();
                return 0;
            }
        }
        else
        {
            std::cout << "robot isnot connected" << std::endl;
            console_pause();
        }
        return 1;
    }
    int ArmHardwareInterface::test_io(){
        for (int i = 0; i < IO_COUNT; i++)
        {
            robot->set_digital_out(i, 0);
        }
        Sleep(200);
        for (int i = 0; i < IO_COUNT; i++)
        {
            if (robot->get_digital_in(i) == 1)
                std::cout << "io_" << i << " test_1 ok" << std::endl;
            else
            {
                std::cout << "io_" << i << " test_1 ng" << std::endl;
                console_pause();
                return 0;
            }
        }
        for (int i = 0; i < IO_COUNT; i++)
            robot->set_digital_out(i, 1);
        Sleep(200);
        for (int i = 0; i < IO_COUNT; i++)
        {
            if (robot->get_digital_in(i) == 0)
                std::cout << "io_" << i << " test_0 ok" << std::endl;
            else
            {
                std::cout << "io_" << i << " test_0 ng" << std::endl;
                console_pause();
                return 0;
            }
        }
        return 1;
    }

    //以下为公有方法定义
    hardware_interface::CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        if(hardware_interface::SystemInterface::on_init(hardware_info)!= hardware_interface::CallbackReturn::SUCCESS){
            return hardware_interface::CallbackReturn::ERROR;
        }
        info_ = hardware_info;

        ret = 0;
        IO_COUNT = 11;
        robot_id = 3;
        return hardware_interface::CallbackReturn::SUCCESS;
    } 
    hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state){
        (void)previous_state; // 一个方法，可以让函数内不使用参数编译器也不会给警告
        if (test_initial() == 1 && test_io() == 1)
            return hardware_interface::CallbackReturn::SUCCESS;
        else
        return hardware_interface::CallbackReturn::ERROR;
    }
    hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state; // 一个方法，可以让函数内不使用参数编译器也不会给警告
        robot->get_scara_param();
        set_state("joint1/position", robot->z);
        set_state("joint2/position", robot->goal_angle1);
        set_state("joint3/position", robot->goal_angle2);
        set_state("joint_tool_gripper/position", robot->rotation);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;//一个方法，可以让函数内不使用参数编译器也不会给警告
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::return_type ArmHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void) time;
        (void) period;
        robot->get_scara_param();
        set_state("joint1/position", robot->z);
        set_state("joint2/position", robot->goal_angle1);
        set_state("joint3/position", robot->goal_angle2);
        set_state("joint_tool_gripper/position", robot->rotation);
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;
        float z = get_command("joint1/command");
        float gangle1 = get_command("joint2/command");
        float gangle2 = get_command("joint3/command");
        float grotation = get_command("joint_tool_gripper/command");
        float gspeed = 10;
        robot->new_movej_angle(gangle1, gangle2, z, grotation, gspeed, 1);
        return hardware_interface::return_type::OK;
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_hardware_interface::ArmHardwareInterface, hardware_interface::SystemInterface) // 一个子类一个父类