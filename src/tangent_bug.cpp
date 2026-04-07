#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class TangentBug : public rclcpp::Node
{
public:
  TangentBug()
  : Node("tangent_bug")
  {
    // publishes and subscribers
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&TangentBug::laserCallback, this, std::placeholders::_1)
    );

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/odom",
      rclcpp::SensorDataQoS(),
      std::bind(&TangentBug::laserCallback, this, std::placeholders::_1)
    );

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10
    );

    // timer for the control loop
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TangentBug::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Tangent bug node started.");
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = msg;
    // TODO: change so it converts to (x,y) points and stores them directly
  }

  void odomCallback(const nav_msgs::msg::Odometry msg)
  {
    // store x, y and yaw
  }

  void timerCallback()
  {
    if (!last_scan_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for laser scan...");
      return;
    }

    auto twist = geometry_msgs::msg::Twist();

    // TODO: implement Tangent Bug logic here using last_scan_
    // 
    // check if line to goal intercepts known obstacle
    //   if it doesn't, send velocity in the goal direction and return
    // get discontinuity points and calculate
    // calculate the heuristic to determine the goal point
    // compare d_reach to d_followed
    //   if d_reach <= d_followed, store it as d_followed and follow behavior 1
    //   if d_reach > d_followed, follow behavior 2
    // behavior 1 (move to goal):
    //   send velocity towards goal point
    // behavior 2 (boundary follow):
    //   send velocity towards the next discontinuity point 

    // before publishing, process velocity (feedback linearization and safe distance)
    cmd_vel_pub_->publish(twist);
  }

  // variables
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr                                 control_timer_;

  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TangentBug>());
  rclcpp::shutdown();
  return 0;
}
