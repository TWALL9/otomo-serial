
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

namespace otomo_serial
{

static constexpr uint32_t BAUD_RATE = 38400; // match the one from bluetooth for now
static constexpr uint8_t SYNC[4] = {'S', 'Y', 'N', 'C'};

class SerialNode
{

public:
  SerialNode(ros::NodeHandle& nh);
  ~SerialNode() = default;
  void joystick_cb(const otomo_msgs::Joystick::ConstPtr& joystick_ros);
  void serial_cb(const uint8_t* buf, size_t len);

private:
  std::vector<uint8_t> recv_buf_;
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
  else
  {
    ROS_WARN("Serial initialized!");
  }
}

void SerialNode::joystick_cb(const otomo_msgs::Joystick::ConstPtr& joystick_ros)
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

  std::vector<uint8_t> buf(SYNC, SYNC + len + 1);
  buf.push_back(len);
  for (uint8_t i = 0; i < len; i++)
  {
    buf.push_back(static_cast<uint8_t>(out_c[i]));
  }

  serial_->send_bytes((uint8_t *)&buf[0], buf.size());
}

void SerialNode::serial_cb(const uint8_t* buf, size_t len)
{
  // TEST: Joystick is 12 bytes long
  if (buf == NULL)
  {
    ROS_ERROR("Received empty buf!");
    return;
  }
  for (size_t i = 0; i < len; i++)
  {
    recv_buf_.push_back(buf[i]);
  }

  bool first, second, third, fourth, len_is_next = false;
  uint8_t msg_len = false;
  uint8_t idx = 0;
  for (std::vector<uint8_t>::iterator it = recv_buf_.begin(); it != recv_buf_.end(); ++it)
  {
    switch (*it)
    {
      case(SYNC[0]):
        first = true;
        break;
      case(SYNC[1]):
        second = true;
        break;
      case(SYNC[2]):
        third = true;
        break;
      case(SYNC[3]):
        fourth = true;
        break;
      default:
        break;
    }

    idx++;

    if (len_is_next)
    {
      len_is_next = false;
      msg_len = *it;
      break;
    }

    if (first && second && third && fourth)
    {
      len_is_next = true;
    }
  }

  if (msg_len != 0)
  {
    otomo::TopMsg msg;
    if (!msg.ParseFromArray((const void *)&recv_buf_[idx], static_cast<int>(msg_len)))
    {
      ROS_ERROR("could not deserialize!");
    }
    else
    {
      ROS_WARN_THROTTLE(0.25, "joystick? %d", msg.has_joystick());
    }
  }
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
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ros::init(argc, argv, "serial_node");

  ros::NodeHandle nh;

  otomo_serial::SerialNode sn(nh);

  ros::spin();

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}