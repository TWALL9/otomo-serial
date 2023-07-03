
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
#include "std_msgs/Float32.h"

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

  ros::Publisher left_motor_pub_;
  ros::Publisher right_motor_pub_;
  ros::Publisher fan_state_pub_;
  async_comm::Serial* serial_{nullptr};
  KissInputStream recv_buf_;

  bool encodeMessage(KissOutputStream& out, otomo::TopMsg& msg);
};

SerialNode::SerialNode(ros::NodeHandle& nh)
{
  std::string joy_topic("/manual/joystick");
  joystick_sub_ = nh.subscribe<otomo_msgs::Joystick>(joy_topic, 10, &SerialNode::joystickCallback, this);

  fan_sub_ = nh.subscribe<std_msgs::Bool>("/manual/fan_on", 1, &SerialNode::fanCallback, this);

  left_motor_pub_ = nh.advertise<std_msgs::Float32>("/motors/left/speed", 5);
  right_motor_pub_ = nh.advertise<std_msgs::Float32>("/motors/right/speed", 5);
  fan_state_pub_ = nh.advertise<std_msgs::Bool>("/motors/fan/active", 1, true);

  std_msgs::Bool fan_msg;
  fan_msg.data = false;
  fan_state_pub_.publish(fan_msg);

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

  KissOutputStream out_kiss;

  if (!encodeMessage(out_kiss, msg))
  {
    ROS_ERROR("could not write joystick message to string");
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

  KissOutputStream out_kiss;

  if (!encodeMessage(out_kiss, msg))
  {
    ROS_ERROR("could not serialize fan message to string");
  }

  auto buf = out_kiss.getBuffer();

  serial_->send_bytes((uint8_t *)&buf[0], buf.size());
}

bool SerialNode::encodeMessage(KissOutputStream& out_kiss, otomo::TopMsg& msg)
{
  std::string out_string;
  if (!msg.SerializeToString(&out_string))
  {
    return false;
  }

  const char* out_c = out_string.c_str();
  size_t len = out_string.size();

  for (uint8_t i = 0; i < len; i++)
  {
    out_kiss.addByte(out_c[i]);
  }

  return true;
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
      else if (msg.has_joystick())
      {
        ROS_WARN("joystick? yes");
      }
      else if (msg.has_state())
      {
        ROS_INFO_THROTTLE(0.5, "Got robot state!");
        const auto& state = msg.state();

        std_msgs::Float32 msg;
        msg.data = state.left_motor().angular_velocity();
        left_motor_pub_.publish(msg);

        msg.data = state.right_motor().angular_velocity();
        right_motor_pub_.publish(msg);

        std_msgs::Bool fan_msg;
        msg.data = state.fan_on();
        fan_state_pub_.publish(fan_msg);
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