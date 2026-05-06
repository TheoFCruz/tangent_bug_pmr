#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "pmr_tp1/visualizer.hpp"

#include <eigen3/Eigen/Dense>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

class ParametricCurve : public rclcpp::Node
{
public:
  ParametricCurve() : Node("parametric_curve"), visualizer(this)
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

    start_srv = this->create_service<std_srvs::srv::SetBool>(
      "/parametric_curve/start",
      std::bind(
        &ParametricCurve::startCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

    publishPath();

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
    if (!active) return;

    // save start time if loop in first iteration
    if (is_first_iteration) {
      t_start = this->now();
      is_first_iteration = false;
    }

    rclcpp::Time t_current = this->now();
    double t = (t_current - t_start).seconds();

    // calculate tracking point P
    Eigen::Vector2d tracking_pos;
    tracking_pos.x() = robot_pos.x() + D * std::cos(robot_yaw);
    tracking_pos.y() = robot_pos.y() + D * std::sin(robot_yaw);

    Eigen::Vector2d followed_point = getLamniscate(t);
    visualizer.publishPoint(
      "/parametric_curve/followed_point",
      followed_point,
      "odom",
      0,
      0.0,
      1.0,
      1.0
    );

    // get position error
    Eigen::Vector2d error = followed_point - tracking_pos;

    // estimate feedforward velocity
    double dt = (double)LOOP_DT_MS / 1000.0;
    Eigen::Vector2d d_pos = getLamniscate(t + dt) - getLamniscate(t - dt); 
    Eigen::Vector2d ff_vel = d_pos / (2.0 * dt);

    // get result velocity command
    Eigen::Vector2d result_vel = VEL_GAIN * error + ff_vel;
    sendVelocity(result_vel);
  }

  void startCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    active = request->data;

    if (active)
    {
      is_first_iteration = true;
      response->message = "Parametric curve started.";
    }
    else
    {
      sendVelocity(Eigen::Vector2d::Zero());
      response->message = "Parametric curve stopped.";
    }

    response->success = true;
  }

  // ------------------ Utility Functions ---------------------

  void publishPath() {
    double dt = (double)LOOP_DT_MS / 1000.0;
    int num_samples = (int)std::ceil(T / dt);

    std::vector<Eigen::Vector2d> path_points;
    path_points.reserve(num_samples + 1);

    for (int i = 0; i <= num_samples; ++i) {
      double t = i * dt;
      if (t > T) {
        t = T;
      }

      Eigen::Vector2d point = getLamniscate(t);
      path_points.push_back(point);
    }

    visualizer.publishPath(path_points, "odom");
  }

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
    double theta = TRAJECTORY_FREQ * t;

    Eigen::Vector2d result;
    result.x() = A*sqrt(2)*cos(theta)/(sin(theta)*sin(theta) + 1);
    result.y() = A*sqrt(2)*cos(theta)*sin(theta)/(sin(theta)*sin(theta) + 1);

    return result;
  }

  // --------------------- Variables --------------------------

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr           start_srv;
  rclcpp::TimerBase::SharedPtr                                 control_timer;

  Visualizer                                                    visualizer;

  // robot and goal
  Eigen::Vector2d goal;
  Eigen::Vector2d robot_pos;
  double          robot_yaw;

  // time tracking
  rclcpp::Time t_start;
  bool         is_first_iteration = true;
  bool         active = false;

  // consts
  const double D = 0.05;
  const double VEL_GAIN = 1.0;
  const int    LOOP_DT_MS = 100;
  const double PI = 3.14159265358979323846;
  const double A = 3.0;
  const double TRAJECTORY_FREQ = 0.1;
  const double T = 2.0 * PI / TRAJECTORY_FREQ;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParametricCurve>());
  rclcpp::shutdown();
  return 0;
}
