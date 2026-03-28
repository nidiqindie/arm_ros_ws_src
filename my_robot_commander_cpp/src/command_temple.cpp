#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
using Movegroupinterface=moveit::planning_interface::MoveGroupInterface;
class Command{
public:
    Command(std::shared_ptr<rclcpp::Node> node){
    node_=node;
    arm_=std::make_shared<Movegroupinterface>(node_,"arm");
    arm_->setMaxAccelerationScalingFactor(0.5);
    arm_->setMaxVelocityScalingFactor(0.5);
    gripper_=std::make_shared<Movegroupinterface>(node_,"gripper");
    gripper_->setMaxVelocityScalingFactor(0.5);
    gripper_->setMaxAccelerationScalingFactor(0.5);
    }
    void NameTarget(std::string name){
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }
    void PoseTarget(geometry_msgs::msg::Pose pose){
        arm_->setStartStateToCurrentState();
        arm_->setPoseTarget(pose);
        planAndExecute(arm_);
    }
    void JointTarget(std::vector<double> joint){
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joint);
        planAndExecute(arm_);
    }
    void CartesianPath(std::vector<geometry_msgs::msg::Pose> waypoints){
        arm_->setStartStateToCurrentState();
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
        if (fraction == 1.0)
        {
            arm_->execute(trajectory);
        }
    }
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<Movegroupinterface> arm_;
    std::shared_ptr<Movegroupinterface> gripper_;
    void planAndExecute(const std::shared_ptr<Movegroupinterface> &group){
        Movegroupinterface::Plan plan;
        bool success = (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            group->execute(plan);
        }
    }
};
int main(int argc, const char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("command_temple");
    rclcpp::executors::SingleThreadedExecutor exec; // 创建rclcpp的执行器
    exec.add_node(node);                            // 在这个执行器中加入之前创建的ros节点
    auto spinner = std::thread([&exec]()
                               { exec.spin(); }); // 创建新的线程来让这个ros节点一直执行下去
    Command command(node);
    command.NameTarget("home");
    rclcpp::shutdown();
    spinner.join();
    return 0;
}