#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
int main(int argc, const char** argv) {
    rclcpp::init(argc, argv);
    auto node =std::make_shared<rclcpp::Node>("test_moveit");//创建智能指针来指向这个节点
    rclcpp::executors::SingleThreadedExecutor exec;//创建rclcpp的执行器
    exec.add_node(node);//在这个执行器中加入之前创建的ros节点
    auto spinner = std::thread([&exec]()
                               { exec.spin(); });//创建新的线程来让这个ros节点一直执行下去
    auto arm = moveit::planning_interface::MoveGroupInterface(node,"arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);//设置最大速度和加速度

    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success1){
        arm.execute(plan1);
    }
    rclcpp::shutdown();
    spinner.join();
    return 0;
}