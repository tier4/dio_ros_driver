#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <iostream>

#include "dio_ros_driver/dio_user_handler.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dio_ros_driver");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // set param for threshold to detec DI pressed.
  double pressing_period_threshold;
  pnh.param<double>(
    "pressing_period_threshold", pressing_period_threshold, 1.0);

  // set publisher
  // startbutton is directed to Autoware.
  ros::Publisher button_pub =
    nh.advertise<std_msgs::Bool>("startbutton", 10);
  // startbutton_raw is a topic for debugging.
  ros::Publisher button_raw_pub =
    pnh.advertise<std_msgs::Bool>("startbutton_raw", 10);

  if (!init_di())
    {
      std::cerr << "init_di cannot <-- ERROR\n" << std::endl;
      return -1;
    }

  // target di line number.
  constexpr uint32_t target_di_line_number = 0;
  // handling button pushed.
  constexpr double sleep_sec = 0.02;
  ros::Rate loop_rate(1.0 / sleep_sec);
  uint32_t pressed_count = 0;
  std_msgs::Bool msg;
  while (ros::ok())
  {
    const bool is_pressed =
      static_cast<bool>(read_di_line(target_di_line_number));
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

