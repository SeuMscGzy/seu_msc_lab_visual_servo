#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

// BEGIN_TUTORIAL
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
std_msgs::msg::Float64MultiArray joint_jog_msg;
double joint1_vel = 0;
double joint2_vel = 0;
double joint3_vel = 0;
double joint4_vel = 0;
double joint5_vel = 0;
double joint6_vel = 0;


void arrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  joint1_vel = msg->data[0];
  joint2_vel = msg->data[1];
  joint3_vel = msg->data[2];
  joint4_vel = msg->data[3];
  joint5_vel = msg->data[4];
  joint6_vel = msg->data[5];
}

void publish_joint_jog()
{
  joint_jog_msg.data[0] = joint1_vel;
  joint_jog_msg.data[1] = joint2_vel;
  joint_jog_msg.data[2] = joint3_vel;
  joint_jog_msg.data[3] = joint4_vel;
  joint_jog_msg.data[4] = joint5_vel;
  joint_jog_msg.data[5] = joint6_vel;
  joint_cmd_pub_->publish(joint_jog_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  // This is false for now until we fix the QoS settings in moveit to enable intra process comms
  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);

  // Create the publisher
  joint_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 10);
  joint_jog_msg.data =  std::vector<double>(6, 0.0);
  // Create the subscriber to receive the array message
  auto array_sub = node_->create_subscription<std_msgs::msg::Float64MultiArray>("simulink_mpc_result", 10, arrayCallback);
  
  auto timer_ = node_->create_wall_timer(2ms, std::bind(&publish_joint_jog));

  rclcpp::spin(node_);

  rclcpp::shutdown();
  return 0;
}
