#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("ros_joints_plan",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    
    using moveit::planning_interface::MoveGroupInterface;
    auto group = MoveGroupInterface(node, "manipulator");
    
    //设置随机目标
    //group.setRandomTarget();
    // 设置机器人各个关节的目标位置 : shoulder_joint  upperArm_joint foreArm_joint wrist1_joint wrist2_joint wrist3_joint
    std::vector<double> target_joint1_ = {0.017602809239,0.01766356,0.017664451,0.017610403,0.01751058,0.017652615};
    std::vector<double> target_joint2_ = {-1.4255541656607345,0.01763318509638841,-1.566302598422464,-1.566302598422464,1.495946113010705,0.017520049091267947};
    
    
    //设置关节速度0-1
    group.setMaxVelocityScalingFactor(0.5);  
    while(rclcpp::ok()){
        
         group.setJointValueTarget(target_joint1_);
    
         group.move(); //plan and execute
         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
         group.setJointValueTarget(target_joint2_);
         group.move();
         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    }
    rclcpp::shutdown();
    return 0;
}
