#include "can_gateway/can_gateway_node.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

namespace can_gateway
{
CanGatewayNode::CanGatewayNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("can_gateway", options)
{
  declare_parameter<std::string>("interface", "can0");
}

bool CanGatewayNode::open_can()
{
  iface_name_ = get_parameter("interface").as_string();
  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open socket: %s", strerror(errno));
    return false;
  }
  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, iface_name_.c_str(), IFNAMSIZ);
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(get_logger(), "Interface %s not found", iface_name_.c_str());
    close(s);
    return false;
  }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(get_logger(), "bind failed: %s", strerror(errno));
    close(s);
    return false;
  }
  fcntl(s, F_SETFL, O_NONBLOCK);
  can_fd_ = s;

  // epoll setup
  epfd_ = epoll_create1(0);
  if (epfd_ < 0) {
    RCLCPP_ERROR(get_logger(), "epoll_create1 failed: %s", strerror(errno));
    close(can_fd_);
    return false;
  }
  struct epoll_event ev{};
  ev.events = EPOLLIN;
  ev.data.fd = can_fd_;
  if (epoll_ctl(epfd_, EPOLL_CTL_ADD, can_fd_, &ev) < 0) {
    RCLCPP_ERROR(get_logger(), "epoll_ctl failed: %s", strerror(errno));
    close_can();
    return false;
  }
  return true;
}

void CanGatewayNode::close_can()
{
  if (epfd_ >= 0) { close(epfd_); epfd_ = -1; }
  if (can_fd_ >= 0) { close(can_fd_); can_fd_ = -1; }
}

can_msgs::msg::Frame CanGatewayNode::to_ros_msg(const struct can_frame & frame)
{
  can_msgs::msg::Frame msg;
  msg.id = frame.can_id & CAN_EFF_MASK;
  msg.dlc = frame.can_dlc;
  msg.is_error = frame.can_id & CAN_ERR_FLAG;
  msg.is_rtr   = frame.can_id & CAN_RTR_FLAG;
  msg.is_extended = frame.can_id & CAN_EFF_FLAG;
  std::copy(frame.data, frame.data + frame.can_dlc, msg.data.begin());
  return msg;
}

void CanGatewayNode::publish_frame(const can_msgs::msg::Frame & frame)
{
  if (pub_raw_->is_activated()) {
    pub_raw_->publish(frame);
  }
}

void CanGatewayNode::io_thread_func()
{
  constexpr int MAX_EVENTS = 8;
  struct epoll_event events[MAX_EVENTS];

  while (running_ && rclcpp::ok()) {
    int n = epoll_wait(epfd_, events, MAX_EVENTS, -1);  // block until data
    if (n < 0) {
      if (errno == EINTR) continue;
      RCLCPP_ERROR(get_logger(), "epoll_wait error: %s", strerror(errno));
      break;
    }
    for (int i = 0; i < n; ++i) {
      if (events[i].data.fd == can_fd_) {
        struct can_frame frames[32];
        struct mmsghdr msgs[32]{};
        struct iovec iovecs[32]{};
        for (int k = 0; k < 32; ++k) {
          iovecs[k].iov_base = &frames[k];
          iovecs[k].iov_len = sizeof(struct can_frame);
          msgs[k].msg_hdr.msg_iov = &iovecs[k];
          msgs[k].msg_hdr.msg_iovlen = 1;
        }
        int received = recvmmsg(can_fd_, msgs, 32, 0, nullptr);
        if (received < 0) {
          if (errno != EAGAIN) {
            RCLCPP_WARN(get_logger(), "recvmmsg error: %s", strerror(errno));
          }
          continue;
        }
        for (int r = 0; r < received; ++r) {
          publish_frame(to_ros_msg(frames[r]));
        }
      }
    }
  }
}

// ---------- lifecycle callbacks ----------

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn CanGatewayNode::on_configure(const rclcpp_lifecycle::State &)
{
  pub_raw_ = create_publisher<can_msgs::msg::Frame>("/can/raw", rclcpp::SensorDataQoS());
  sub_tx_ = create_subscription<can_msgs::msg::Frame>(
    "/can/tx", rclcpp::SensorDataQoS(),
    [this](const can_msgs::msg::Frame::SharedPtr msg){
      if (can_fd_ < 0) return;
      struct can_frame frame{};
      frame.can_id = msg->id;
      if (msg->is_extended) frame.can_id |= CAN_EFF_FLAG;
      if (msg->is_rtr) frame.can_id |= CAN_RTR_FLAG;
      if (msg->is_error) frame.can_id |= CAN_ERR_FLAG;
      frame.can_dlc = msg->dlc;
      std::copy(msg->data.begin(), msg->data.begin() + msg->dlc, frame.data);
      write(can_fd_, &frame, sizeof(frame));
    });

  if (!open_can()) {
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CanGatewayNode::on_activate(const rclcpp_lifecycle::State &)
{
  pub_raw_->on_activate();
  running_ = true;
  io_thread_ = std::thread(&CanGatewayNode::io_thread_func, this);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CanGatewayNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  running_ = false;
  if (io_thread_.joinable()) io_thread_.join();
  pub_raw_->on_deactivate();
  close_can();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CanGatewayNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  running_ = false;
  if (io_thread_.joinable()) io_thread_.join();
  close_can();
  return CallbackReturn::SUCCESS;
}

} // namespace can_gateway

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(can_gateway::CanGatewayNode)