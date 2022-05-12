
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#define _USE_MATH_DEFINES

#include "std_msgs/msg/string.hpp"

#include "ros_msgs/msg/robot_pose.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class echoListener
{
public:
  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  explicit echoListener(rclcpp::Clock::SharedPtr clock)
  : buffer_(clock)
  {
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
  }

  ~echoListener()
  {
  }
};


int main(int argc, char ** argv)
{

  // Initialize ROS
  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);


  double rate_hz;
  // Allow 2 or 3 command line arguments
  if (args.size() == 3) {
    rate_hz = 1.0;
  } else if (args.size() == 4) {
    size_t pos;
    try {
      rate_hz = std::stof(args[3], &pos);
    } catch (const std::invalid_argument &) {
      // If the user provided an argument that wasn't a number (like 'foo'), stof() will throw.
      fprintf(
        stderr, "Failed to convert rate argument '%s' to a floating-point number\n",
        args[3].c_str());
      return 2;
    }

    // If the user provide an floating-point argument with junk on the end (like '1.0foo'), the pos
    // argument will show it didn't convert the whole argument.
    if (pos != args[3].length()) {
      fprintf(
        stderr, "Failed to convert rate argument '%s' to a floating-point number\n",
        args[3].c_str());
      return 3;
    }
  } else {
    printf("Usage: tf2_echo source_frame target_frame [echo_rate]\n\n");
    printf("This will echo the transform from the coordinate frame of the source_frame\n");
    printf("to the coordinate frame of the target_frame. \n");
    printf("Note: This is the transform to get data from target_frame into the source_frame.\n");
    printf("Default echo rate is 1 if echo_rate is not given.\n");
    return 1;
  }
  // TODO(tfoote): restore parameter option
  // // read rate parameter
  // ros::NodeHandle p_nh("~");
  // p_nh.param("rate", rate_hz, 1.0);
  rclcpp::Rate rate(rate_hz);

  // TODO(tfoote): restore anonymous??
  // ros::init_options::AnonymousName);

  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("robotpose");

  rclcpp::Clock::SharedPtr clock = nh->get_clock();
  // Instantiate a local listener
  echoListener echoListener(clock);

  std::string source_frameid = args[1];
  std::string target_frameid = args[2];

  static rclcpp::Publisher<ros_msgs::msg::RobotPose>::SharedPtr robot_pose_pub_;
  robot_pose_pub_ = nh->create_publisher<ros_msgs::msg::RobotPose>("robotpose", 1);

  // Wait for the first transforms to become avaiable.
  std::string warning_msg;
  while (rclcpp::ok() && !echoListener.buffer_.canTransform(
      source_frameid, target_frameid, tf2::TimePoint(), &warning_msg))
  {
    RCLCPP_INFO_THROTTLE(
      nh->get_logger(), *clock, 1000, "Waiting for transform %s ->  %s: %s",
      source_frameid.c_str(), target_frameid.c_str(), warning_msg.c_str());
    rate.sleep();
  }

  // Nothing needs to be done except wait for a quit
  // The callbacks within the listener class will take care of everything
  while (rclcpp::ok()) {
    try {
      geometry_msgs::msg::TransformStamped echo_transform;
      echo_transform = echoListener.buffer_.lookupTransform(
        source_frameid, target_frameid,
        tf2::TimePoint());
      std::cout.precision(3);
      std::cout.setf(std::ios::fixed, std::ios::floatfield);
      // std::cout << "At time " << echo_transform.header.stamp.sec << "." <<
      //   echo_transform.header.stamp.nanosec << std::endl;
      double yaw, pitch, roll;
      // echo_transform.getBasis().getRPY(roll, pitch, yaw);
      // tf::Quaternion q = echo_transform.getRotation();
      // tf::Vector3 v = echo_transform.getOrigin();
      auto translation = echo_transform.transform.translation;
      //auto rotation = echo_transform.transform.rotation;
      // std::cout << "- Translation: [" << translation.x << ", " << translation.y << ", " <<
      // //   translation.z << "]" << std::endl;
      // std::cout << "- Rotation: in Quaternion [" << rotation.x << ", " << rotation.y << ", " <<
      //   rotation.z << ", " << rotation.w << "]" << std::endl;




      tf2::Quaternion q_orig;

      // Get the original orientation of 'commanded_pose'
      tf2::convert(echo_transform.transform.rotation , q_orig);

      tf2::Matrix3x3 m(q_orig);
      m.getRPY(roll, pitch, yaw);
    


      // std::cout << "x [" << translation.x  << "] , y [" << translation.y << "] , theta [" <<
      //   yaw*180.0/M_PI  << "]" << std::endl;

      
      // Publisher
      auto publisher_data = ros_msgs::msg::RobotPose();
      publisher_data.x = translation.x ;
      publisher_data.y = translation.y ;
      publisher_data.theta = yaw*180.0/M_PI  ;
      robot_pose_pub_->publish(publisher_data);

      // TODO(tfoote): restory rpy
      // << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" <<
      // std::endl
      // << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI <<
      // ", " << yaw*180.0/M_PI << "]" << std::endl;
    } catch (const tf2::TransformException & ex) {
      std::cout << "Failure at " << clock->now().seconds() << std::endl;
      std::cout << "Exception thrown:" << ex.what() << std::endl;
      std::cout << "The current list of frames is:" << std::endl;
      std::cout << echoListener.buffer_.allFramesAsString() << std::endl;
    }
    rate.sleep();
  }

  return 0;
}