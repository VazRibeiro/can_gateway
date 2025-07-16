#ifndef CAN_GATEWAY_NODE_HPP_
#define CAN_GATEWAY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <can_msgs/msg/frame.hpp>
#include <thread>
#include <atomic>
#include <linux/can.h> 
#include <sys/epoll.h>

namespace can_gateway
{
class CanGatewayNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CanGatewayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
  void io_thread_func();
  bool open_can();
  void close_can();
  void publish_frame(const can_msgs::msg::Frame & frame);
  can_msgs::msg::Frame to_ros_msg(const struct can_frame & frame);

  // parameters
  std::string iface_name_;

  // ROS i/o
  rclcpp_lifecycle::LifecyclePublisher<can_msgs::msg::Frame>::SharedPtr pub_raw_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_tx_;

  int can_fd_ {-1};
  int epfd_ {-1};

  std::thread io_thread_;
  std::atomic_bool running_ {false};
};
} // namespace can_gateway

#endif // CAN_GATEWAY_NODE_HPP_