#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <eigen3/Eigen/Dense>

#include <cmath>
#include <vector>

class ParametricCurve : public rclcpp::Node
{
  public:
    ParametricCurve()
      : Node("parametric_curve")
    {
      // publishers and subscribers
      odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom",
          10,
          std::bind(&ParametricCurve::odomCallback, this, std::placeholders::_1)
          );

      cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
          "/cmd_vel",
          10
          );

      // timer for the control loop
      control_timer = this->create_wall_timer(
          std::chrono::milliseconds(LOOP_DT_MS),
          std::bind(&ParametricCurve::controlLoop, this)
          );

      RCLCPP_INFO(this->get_logger(), "Parametric curve node started.");
    }

  private:

    // ---------------------- Callbacks -------------------------

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      // get position
      robot_pos.x() = msg->pose.pose.position.x;
      robot_pos.y() = msg->pose.pose.position.y;

      // get quaternion and extract yaw
      double x = msg->pose.pose.orientation.x;
      double y = msg->pose.pose.orientation.y;
      double z = msg->pose.pose.orientation.z;
      double w = msg->pose.pose.orientation.w;

      double siny_cosp = 2.0 * (w * z + x * y);
      double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
      robot_yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    void controlLoop()
    {
      // TODO: Implement parametric curve control logic
    }

    // ------------------ Utility Functions ---------------------

    void sendVelocity(Eigen::Vector2d vel)
    {
      double v_x = vel.x();
      double v_y = vel.y();

      // feedback linearization
      double v = (v_x * std::cos(robot_yaw)) + (v_y * std::sin(robot_yaw));
      double w = (-v_x * std::sin(robot_yaw) + v_y * std::cos(robot_yaw)) / D;

      // ros2 msg
      geometry_msgs::msg::Twist vel_twist;
      vel_twist.linear.x = v;
      vel_twist.angular.z = w;

      cmd_vel_pub->publish(vel_twist);
    }

    Eigen::Vector2d getLamniscate(double t)
    {
      double a = 1.0;

      Eigen::Vector2d result;
      result.x() = a*sqrt(2)*cos(t)/(sin(t)*sin(t) + 1);
      result.y() = a*sqrt(2)*cos(t)*sin(t)/(sin(t)*sin(t) + 1);

      return result;
    }

    // --------------------- Variables --------------------------

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub;
    rclcpp::TimerBase::SharedPtr                                 control_timer;

    // robot and goal
    Eigen::Vector2d goal;
    Eigen::Vector2d robot_pos;
    double          robot_yaw;

    // consts
    const double D = 0.05;
    const int    LOOP_DT_MS = 100;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParametricCurve>());
  rclcpp::shutdown();
  return 0;
}
