
#include <string>
#include <cstdlib>
#include <vector>
#include <functional>

#include <ros/ros.h>
#include <async_comm/serial.h>
#include "otomo_msgs/Joystick.h"

namespace otomo_serial
{

static constexpr uint32_t BAUD_RATE = 38400; // match the one from bluetooth for now

class SerialNode
{

public:
  SerialNode(ros::NodeHandle& nh);
  ~SerialNode() = default;
  void joystick_cb(const otomo_msgs::Joystick::ConstPtr& joystick_ros);
  void serial_cb(const uint8_t* buf, size_t len);

private:
  std::vector<uint8_t> serial_buf_;
  ros::Subscriber joystick_sub_;
  async_comm::Serial* serial_{nullptr};
};

SerialNode::SerialNode(ros::NodeHandle& nh)
{
  std::string joy_topic("/manual/joystick");
  joystick_sub_ = nh.subscribe<otomo_msgs::Joystick>(joy_topic, 10, &SerialNode::joystick_cb, this);
  serial_ = new async_comm::Serial("/dev/ttyUSB0", BAUD_RATE);

  std::function<void(const uint8_t*, size_t)> serial_call =
    std::bind(&SerialNode::serial_cb, this, std::placeholders::_1, std::placeholders::_2);
  serial_->register_receive_callback(serial_call);

  // check result of this!
  if (!serial_->init())
  {
    ROS_FATAL("Could not initialize serial port!!!");
  }
}

void SerialNode::joystick_cb(const otomo_msgs::Joystick::ConstPtr& joystick_ros)
{
  // convert and send

  // encode
  // send
  uint8_t send_buf[10] = {0};
  serial_->send_bytes(send_buf, sizeof(send_buf));
}

void SerialNode::serial_cb(const uint8_t* buf, size_t len)
{
  // append
  // for (size_t i = 0; i < len; i++)
  // {
  //   serial_buf_.push_back(buf[i]);
  // }

  // parse msg
  // publish
}

}  // otomo_serial

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "serial_node");

  ros::NodeHandle nh;

  ros::spin();

  return 0;
}