#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <eigen3/Eigen/Dense>

#include <cmath>
#include <vector>

class TangentBug : public rclcpp::Node
{
public:
  TangentBug()
  : Node("tangent_bug")
  {
    // publishes and subscribers
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&TangentBug::laserCallback, this, std::placeholders::_1)
    );

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&TangentBug::odomCallback, this, std::placeholders::_1)
    );

    goal_sub = this->create_subscription<geometry_msgs::msg::Point>(
      "/goal",
      10,
      std::bind(&TangentBug::goalCallback, this, std::placeholders::_1)
    );

    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10
    );

    // timer for the control loop
    control_timer = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TangentBug::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Tangent bug node started.");
  }

private:

  // ---------------------- Callbacks -------------------------
  
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // clear old points
    laser_points.clear();

    // used for robot -> map reference
    Eigen::Rotation2Dd r_yaw = Eigen::Rotation2Dd(robot_yaw); 

    double current_angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
      // get points relative to the robot
      double r = msg->ranges[i];

      if (std::isfinite(r))
      {
        Eigen::Vector2d new_point(0,0);
        new_point.x() = r * std::cos(current_angle);
        new_point.y() = r * std::sin(current_angle);

        // transform the points to the map using the robot pose
        new_point = r_yaw * new_point + robot_pos;

        laser_points.push_back(new_point);
      }

      // incrementa o angulo
      current_angle += msg->angle_increment;
    }
  }

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

  void goalCallback(geometry_msgs::msg::Point::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: (%.2lf, %.2lf)", msg->x, msg->y);
    goal = Eigen::Vector2d(msg->x, msg->y);
    goal_received = true;
  }

  void controlLoop()
  {
    if (!goal_received) return;

    // TODO: implement Tangent Bug logic here using last_scan_
    // 
    // check if line to goal intercepts known obstacle
    //   if it doesn't, send velocity in the goal direction and return
    
    if (isGoalClear())
    {
      Eigen::Vector2d vel = goal - robot_pos;
      vel = vel.normalized() * SPEED;
      sendVelocity(vel);
    } 

    // get discontinuity points 
    // calculate the heuristic to determine the best discontinuity point
    // compare d_reach to d_followed
    //   if d_reach <= d_followed, store it as d_followed and follow behavior 1
    //   if d_reach > d_followed, follow behavior 2
    // behavior 1 (move to goal):
    //   send velocity towards best discontinuity point
    // behavior 2 (boundary follow):
    //   send velocity towards the next discontinuity point 
    
    // check if goal reached
    if ((robot_pos - goal).norm() <= TOLERANCE)
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping control loop");
      goal_received = false;
    }
  }

  // ----------------- Utility Functions ----------------------
  
  bool isGoalClear()
  {
    Eigen::Vector2d goal_dir = (goal - robot_pos).normalized();
    double goal_dist = (goal-robot_pos).norm();

    Eigen::Vector2d robot_to_obst = {0,0};
    for (const auto& point : laser_points)
    {
      robot_to_obst = point - robot_pos;
      double projection = robot_to_obst.dot(goal_dir); // obstacle dist in the dir of the goal
      
      // skip obstacles behind robot and after goal
      if (projection <= 0 || projection > goal_dist) continue; 

      double normal_dist = (robot_to_obst - projection*goal_dir).norm();
      if (normal_dist <= SAFE_RADIUS) return false;
    }
    return true;
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

  // --------------------- Variables --------------------------
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr   goal_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub;
  rclcpp::TimerBase::SharedPtr                                 control_timer;

  // laser points vector
  std::vector<Eigen::Vector2d> laser_points; 

  // robot and goal variables
  Eigen::Vector2d goal;
  Eigen::Vector2d robot_pos;
  double          robot_yaw;

  // flags
  bool goal_received = false;

  // consts
  const double SPEED = 0.5;
  const double SAFE_RADIUS = 0.25;
  const double D = 0.1;
  const double TOLERANCE = 0.1;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TangentBug>());
  rclcpp::shutdown();
  return 0;
}
