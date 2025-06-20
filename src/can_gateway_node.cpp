#include "can_gateway/can_gateway_node.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstring>

namespace can_gateway
{
CanGatewayNode::CanGatewayNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("can_gateway", options)
{
  declare_parameter<std::string>("interface", "can0");
  declare_parameter<int>("rx_rate_hz", 10000);  // poll interval
}

bool CanGatewayNode::open_can()
{
  iface_name_ = get_parameter("interface").as_string();
  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0)
  {
    RCLCPP_ERROR(get_logger(), "Failed to open CAN socket: %s", strerror(errno));
    return false;
  }
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, iface_name_.c_str(), IFNAMSIZ);
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
  {
    RCLCPP_ERROR(get_logger(), "CAN interface %s not found", iface_name_.c_str());
    close(s);
    return false;
  }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(get_logger(), "bind() failed: %s", strerror(errno));
    close(s);
    return false;
  }
  // set non‑blocking
  fcntl(s, F_SETFL, O_NONBLOCK);
  can_fd_ = s;
  return true;
}

void CanGatewayNode::close_can()
{
  if (can_fd_ >= 0)
  {
    close(can_fd_);
    can_fd_ = -1;
  }
}

void CanGatewayNode::read_loop()
{
  const int poll_timeout_ms = 1000 / rx_rate_hz_;
  struct pollfd fds { can_fd_, POLLIN, 0 };
  while (running_ && rclcpp::ok())
  {
    int ret = poll(&fds, 1, poll_timeout_ms);
    if (ret > 0 && (fds.revents & POLLIN))
    {
      struct can_frame frame{};
      int nbytes = read(can_fd_, &frame, sizeof(frame));
      if (nbytes == sizeof(frame))
      {
        can_msgs::msg::Frame ros_msg;
        ros_msg.id = frame.can_id & CAN_EFF_MASK;
        ros_msg.dlc = frame.can_dlc;
        ros_msg.is_error = frame.can_id & CAN_ERR_FLAG;
        ros_msg.is_rtr = frame.can_id & CAN_RTR_FLAG;
        ros_msg.is_extended = frame.can_id & CAN_EFF_FLAG;
        std::copy(frame.data, frame.data + frame.can_dlc, ros_msg.data.begin());
        publish_frame(ros_msg);
      }
    }
  }
}

void CanGatewayNode::publish_frame(const can_msgs::msg::Frame & frame)
{
  if (pub_raw_->is_activated())
  {
    pub_raw_->publish(frame);
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CanGatewayNode::on_configure(const rclcpp_lifecycle::State &)
{
  rx_rate_hz_ = get_parameter("rx_rate_hz").as_int();
  pub_raw_ = create_publisher<can_msgs::msg::Frame>("/can/raw", rclcpp::SensorDataQoS());
  sub_tx_ = create_subscription<can_msgs::msg::Frame>(
    "/can/tx", rclcpp::SensorDataQoS(),
    [this](const can_msgs::msg::Frame::SharedPtr msg)
    {
      if (can_fd_ < 0) { return; }
      struct can_frame frame{};
      frame.can_id = msg->id;
      if (msg->is_extended) frame.can_id |= CAN_EFF_FLAG;
      if (msg->is_rtr)      frame.can_id |= CAN_RTR_FLAG;
      if (msg->is_error)    frame.can_id |= CAN_ERR_FLAG;
      frame.can_dlc = msg->dlc;
      std::copy(msg->data.begin(), msg->data.begin() + msg->dlc, frame.data);
      write(can_fd_, &frame, sizeof(frame));
    });

  if (!open_can())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CanGatewayNode::on_activate(const rclcpp_lifecycle::State &)
{
  pub_raw_->on_activate();
  running_ = true;
  io_thread_ = std::thread(&CanGatewayNode::read_loop, this);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CanGatewayNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  running_ = false;
  if (io_thread_.joinable()) io_thread_.join();
  pub_raw_->on_deactivate();
  close_can();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CanGatewayNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  running_ = false;
  if (io_thread_.joinable()) io_thread_.join();
  close_can();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace can_gateway

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(can_gateway::CanGatewayNode)