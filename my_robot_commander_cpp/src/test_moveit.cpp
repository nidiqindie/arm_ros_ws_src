#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
int main(int argc, const char** argv) {
    //初始化ros2和moveit
    rclcpp::init(argc, argv);
    auto node =std::make_shared<rclcpp::Node>("test_moveit");//创建智能指针来指向这个节点
    rclcpp::executors::SingleThreadedExecutor exec;//创建rclcpp的执行器
    exec.add_node(node);//在这个执行器中加入之前创建的ros节点
    auto spinner = std::thread([&exec]()
                               { exec.spin(); });//创建新的线程来让这个ros节点一直执行下去
    auto arm = moveit::planning_interface::MoveGroupInterface(node,"arm");
    auto gripper=moveit::planning_interface::MoveGroupInterface(node,"gripper");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);//设置最大速度和加速度
    gripper.setMaxVelocityScalingFactor(0.5);
    gripper.setMaxAccelerationScalingFactor(0.5);


    //姿态规划
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success1){
        arm.execute(plan1);
    }


    //joint规划
    // std::vector<double> joint_group_positions={1.5,2.0,3.0,1.1};
    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joint_group_positions);
    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success2){
    //     arm.execute(plan2);
    // }



    //末端移动规划,这里给的xyz是根据你设置的baselink为基准，末端爪子跑到的位置
    // tf2::Quaternion q;
    // q.setRPY(0,0,0);
    // geometry_msgs::msg::PoseStamped target_pose;
    // target_pose.header.frame_id = "base_link";
    // target_pose.pose.position.x = 0.5;
    // target_pose.pose.position.y = 0.0;
    // target_pose.pose.position.z = 0.5;
    // target_pose.pose.orientation.x = q.getX();
    // target_pose.pose.orientation.y = q.getY();
    // target_pose.pose.orientation.z = q.getZ();
    // target_pose.pose.orientation.w = q.getW();
    // arm.setStartStateToCurrentState();
    // arm.setPoseTarget(target_pose);
    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (success2)
    // {
    //     arm.execute(plan2);
    // }


    //笛卡尔移动
    // std::vector<geometry_msgs::msg::Pose> waypoints;
    // geometry_msgs::msg::Pose pose1=arm.getCurrentPose().pose;
    // pose1.position.z += 0.2;
    // waypoints.push_back(pose1);
    // geometry_msgs::msg::Pose pose2=pose1;
    // pose2.position.y += 0.2;
    // waypoints.push_back(pose2);//可以连着多加几个点进去的
    // moveit_msgs::msg::RobotTrajectory trajectory;
    // double fraction = arm.computeCartesianPath(waypoints,0.01,trajectory);
    // if (fraction==1.0)
    // {
    //     arm.execute(trajectory);
    // }
    
    rclcpp::shutdown();
    spinner.join();
    return 0;
}