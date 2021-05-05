#include <rclcpp/rclcpp.hpp>
#include <shipsim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MimicNode : public rclcpp::Node
{
public:
  MimicNode() : rclcpp::Node("ship_mimic")
  {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("output/cmd_vel", 1);
    pose_sub_ = this->create_subscription<shipsim::msg::Pose>(
        "input/pose", 1, std::bind(&MimicNode::poseCallback, this, std::placeholders::_1));
  }

private:
  void poseCallback(const shipsim::msg::Pose::SharedPtr pose)
  {
    geometry_msgs::msg::Twist twist;
    twist.angular.z = pose->angular_velocity;
    twist.linear.x = pose->linear_velocity;
    twist_pub_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<shipsim::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MimicNode>());
}
