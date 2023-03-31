
#include <string>
#include <cstdlib>
#include <vector>
#include <functional>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <async_comm/serial.h>
#include "otomo-serial/kiss_tnc.h"
#include "otomo_msgs/Joystick.h"
#include "otomo_msgs/otomo.pb.h"
#include "std_msgs/Bool.h"

namespace otomo_serial
{

static constexpr uint32_t BT_BAUD_RATE = 38400; // match the one from bluetooth for now
static constexpr uint32_t USB_BAUD_RATE = 115200;

class SerialNode
{

public:
  SerialNode(ros::NodeHandle& nh);
  ~SerialNode() = default;
  void joystickCallback(const otomo_msgs::Joystick::ConstPtr& joystick_ros);
  void fanCallback(const std_msgs::Bool::ConstPtr& fan_on);
  void serialCallback(const uint8_t* buf, size_t len);

private:
  ros::Subscriber joystick_sub_;
  ros::Subscriber fan_sub_;
  async_comm::Serial* serial_{nullptr};
  KissInputStream recv_buf_;
};

SerialNode::SerialNode(ros::NodeHandle& nh)
{
  std::string joy_topic("/manual/joystick");
  joystick_sub_ = nh.subscribe<otomo_msgs::Joystick>(joy_topic, 10, &SerialNode::joystickCallback, this);

  fan_sub_ = nh.subscribe<std_msgs::Bool>("/manual/fan_on", 1, &SerialNode::fanCallback, this);

  serial_ = new async_comm::Serial("/dev/ttyACM1", USB_BAUD_RATE);

  std::function<void(const uint8_t*, size_t)> serial_call =
    std::bind(&SerialNode::serialCallback, this, std::placeholders::_1, std::placeholders::_2);
  serial_->register_receive_callback(serial_call);

  // check result of this!
  if (!serial_->init())
  {
    ROS_FATAL("Could not initialize serial port!!!");
  }
  else
  {
    ROS_WARN("Serial initialized!");
  }
}

void SerialNode::joystickCallback(const otomo_msgs::Joystick::ConstPtr& joystick_ros)
{
  otomo::TopMsg msg;
  otomo::Joystick * joy = new otomo::Joystick();
  joy->set_heading(joystick_ros->heading);
  joy->set_speed(joystick_ros->speed);
  msg.set_allocated_joystick(joy);

  std::string out_string;
  if (!msg.SerializeToString(&out_string))
  {
    ROS_ERROR("could not write joystick message to string");
  }

  const char* out_c = out_string.c_str();
  uint8_t len = static_cast<uint8_t>(strlen(out_c));

  KissOutputStream out_kiss;
  for (uint8_t i = 0; i < len; i++)
  {
    out_kiss.addByte(out_c[i]);
  }

  auto buf = out_kiss.getBuffer();

  serial_->send_bytes((uint8_t *)&buf[0], buf.size());
}

void SerialNode::fanCallback(const std_msgs::Bool::ConstPtr& fan_on)
{
  otomo::TopMsg msg;
  otomo::FanControl * fan = new otomo::FanControl();
  fan->set_on(fan_on->data);
  msg.set_allocated_fan(fan);

  std::string out_string;
  if (!msg.SerializeToString(&out_string))
  {
    ROS_ERROR("could not serialize fan message to string");
  }

  const char* out_c = out_string.c_str();
  uint8_t len = static_cast<uint8_t>(strlen(out_c));

  KissOutputStream out_kiss;
  for (uint8_t i = 0; i < len; i++)
  {
    out_kiss.addByte(out_c[i]);
  }

  auto buf = out_kiss.getBuffer();

  serial_->send_bytes((uint8_t *)&buf[0], buf.size());
}

void SerialNode::serialCallback(const uint8_t* buf, size_t len)
{
  // TEST: Joystick is 12 bytes long
  if (buf == NULL || len == 0)
  {
    ROS_ERROR("Received empty buf!");
    return;
  }

  for (size_t i = 0; i < len; i++)
  {
    int ret = recv_buf_.addByte(buf[i]);
    if (ret != 0)
    {
      ROS_ERROR("Receive buffer error: %d", ret);
      recv_buf_.init();
    }
    else if (recv_buf_.isReady())
    {
      uint8_t port;
      std::vector<uint8_t> in_proto(recv_buf_.getBuffer(ret, port));
      recv_buf_.init();

      otomo::TopMsg msg;
      if (!msg.ParseFromArray((const void *)&in_proto[0], in_proto.size()))
      {
        ROS_ERROR("Could not deserialize proto msg from mcu!, 0x%x, %ld", in_proto.front(), in_proto.size());
      }
      else
      {
        std::string joy_yes = msg.has_joystick() ? "yes" : "no";
        ROS_WARN("joystick? %s", joy_yes.c_str());
      }
    }
  }
}

}  // otomo_serial

int main(int argc, char* argv[])
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ros::init(argc, argv, "serial_node");

  ros::NodeHandle nh;

  otomo_serial::SerialNode sn(nh);

  ros::spin();

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}