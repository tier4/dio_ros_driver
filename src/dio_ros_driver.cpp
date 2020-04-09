#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <cstring>

#include "dio_ros_driver/dio_user_handler.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dio_ros_driver");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // set param
  // threshold to detect DI pressed.
  double pressing_period_threshold;
  pnh.param<double>("pressing_period_threshold",
                    pressing_period_threshold, 1.0);
  // chipname
  std::string chip_name;
  pnh.param<std::string>("chip_name",
                         chip_name, "gpiochip0");
  // line offset
  int line_offset;
  pnh.param<int>("line_offset",
                 line_offset, (int)0);
  
  // set publisher
  // startbutton is directed to Autoware.
  ros::Publisher button_pub =
    nh.advertise<std_msgs::Bool>("startbutton", 10);
  // startbutton_raw is a topic for debugging.
  ros::Publisher button_raw_pub =
    pnh.advertise<std_msgs::Bool>("startbutton_raw", 10);

  if (!init_di(chip_name.c_str(), line_offset))
    {
      std::cerr << "init_di cannot <-- ERROR\n" << std::endl;
      return -1;
    }

  // handling button pushed.
  constexpr double sleep_sec = 0.02;
  ros::Rate loop_rate(1.0 / sleep_sec);
  uint32_t pressed_count = 0;
  std_msgs::Bool msg;
  while (ros::ok())
  {
    const bool is_pressed =
      static_cast<bool>(read_di_line());
    pressed_count = (is_pressed) ? pressed_count + 1 : 0;
    const double pressing_period = pressed_count * sleep_sec;

    msg.data = is_pressed;

    // publish topics of button(DI) pushed.
    button_raw_pub.publish(msg);
    if (pressing_period > pressing_period_threshold)
    {
      button_pub.publish(msg);
    }
    loop_rate.sleep();
  }

  return 0;
}

