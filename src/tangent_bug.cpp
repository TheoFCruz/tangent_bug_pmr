#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

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

    discontinuities_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "/discontinuities",
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

    // get closest obstacle point
    double min_dist = 1000;

    double current_angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
      // get points relative to the robot
      double r = msg->ranges[i];

      if (!std::isfinite(r)) r = 10;

      Eigen::Vector2d new_point(0,0);
      new_point.x() = r * std::cos(current_angle);
      new_point.y() = r * std::sin(current_angle);

      // transform the points to the map using the robot pose
      new_point = r_yaw * new_point + robot_pos;

      laser_points.push_back(new_point);

      if (r < min_dist)
      {
        closest_point = new_point;
        min_dist = r;
      }

      // next angle
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

    // check if goal was reached
    if (goal_received && (robot_pos - goal).norm() <= TOLERANCE)
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping control loop");
      sendVelocity(Eigen::Vector2d::Zero());
      goal_received = false;
    }
  }

  void goalCallback(geometry_msgs::msg::Point::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: (%.2lf, %.2lf)", msg->x, msg->y);
    goal = Eigen::Vector2d(msg->x, msg->y);
    goal_received = true;

    // reset state machine for the new goal
    current_state = State::MOTION_TO_GOAL;
    last_heuristic = {1e9, 1e9};
    d_followed = 1e9;
    d_reach = 1e9;
    check_unreachable = false;
  }

  void controlLoop()
  {
    if (!goal_received) return;

    // get discontinuity points 
    std::vector<Eigen::Vector2d> discontinuities = getDiscontinuities();
    publishDiscontinuities(discontinuities);

    // calculate the heuristic to determine the best discontinuity point
    Eigen::Vector2d result_point = calculateHeuristic(discontinuities);

    switch (current_state) {

      case State::MOTION_TO_GOAL:
        // if path to goal is clear, send direct velocity
        if (isGoalClear())
        {
          double dist_to_goal = (goal - robot_pos).norm();
          if (dist_to_goal < d_followed) d_followed = dist_to_goal;

          Eigen::Vector2d vel = goal - robot_pos;
          vel = vel.normalized() * SPEED;
          sendVelocity(vel);
          return;
        } 

        sendVelocity((result_point - robot_pos).normalized()*SPEED);

        d_reach = (goal - result_point).norm();
        if (d_reach > d_followed + HYSTERESIS)
        {
          // local minimum detected
          RCLCPP_INFO(this->get_logger(), "Local minimum detected. Switching to boundary following.");

          // get M point
          M_point = getMPoint();

          // set d_followed to d(M, goal)
          d_followed = (goal - M_point).norm();

          // Initialize last_heuristic for continuity
          last_heuristic = result_point;

          // change state
          current_state = State::BOUNDARY_FOLLOWING;
        }
        else
        {
          d_followed = d_reach;
        }
        break;

      case State::BOUNDARY_FOLLOWING:
        {
          // get discontinuity closest to the one it was following
          Eigen::Vector2d disc = result_point;
          double min_dist = 1e9;
          for (const auto& point : discontinuities)
          {
            double dist = (point - last_heuristic).norm();
            if (dist < min_dist)
            {
              disc = point;
              min_dist = dist;
            }
          }

          // get d_reach
          d_reach = (goal - disc).norm();

          // follow it
          sendVelocity((disc - robot_pos).normalized()*SPEED);

          // update last heuristic
          last_heuristic = disc;

          // check if d_reach < d_followed - HYSTERESIS
          if (d_reach < d_followed - HYSTERESIS)
          {
            RCLCPP_INFO(this->get_logger(), "Condition met. Going back to motion to goal.");
            current_state = State::MOTION_TO_GOAL;
            d_followed = d_reach;
            check_unreachable = false;
            break;
          }

          // check for unreachable goal
          // TODO: Add a more robust check to account for sensor noise 
          Eigen::Vector2d current_best = getMPoint();
          if (check_unreachable)
          {
            if ((current_best - M_point).norm() < GOAL_UNREACHABLE_MIN)
            {
              RCLCPP_INFO(this->get_logger(), "Made a full turn. Goal is unreachable");
              sendVelocity(Eigen::Vector2d::Zero());
              current_state = State::MOTION_TO_GOAL;
              goal_received = false;
            }
          }
          else if ((current_best - M_point).norm() > GOAL_UNREACHABLE_TH)
          {
            RCLCPP_INFO(this->get_logger(), "Checking for unreachable goal.");
            check_unreachable = true;
          }
        }
        break;
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
      // skip "infinite" points
      if ((point - robot_pos).norm() >= 5) continue;

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
    vel = getSafeVelocity(vel);

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

  Eigen::Vector2d getSafeVelocity(Eigen::Vector2d desired_vel)
  {
    if (laser_points.empty()) return desired_vel; 

    double range = (closest_point - robot_pos).norm();
    Eigen::Vector2d n = (closest_point - robot_pos).normalized();
    double projection = desired_vel.dot(n);

    // HACK: AI black magic that must be revised
    
    // Radial correction velocity (P-controller)
    // If range < SAFE_RADIUS, this is negative (away from obstacle)
    // If range > SAFE_RADIUS, this is positive (towards obstacle)
    double dist_gain = 0.5; 
    double v_radial_corr = - dist_gain * (SAFE_RADIUS - range);

    // We take the minimum (most 'away') of the desired and correction components.
    // This ensures we always move away if too close, and never move in faster than
    // the soft boundary allows when approaching the safe radius.
    double v_radial = std::min(projection, v_radial_corr);

    // Tangent component of the desired velocity
    Eigen::Vector2d tangent_vel = desired_vel - projection * n;
    Eigen::Vector2d result_vel = tangent_vel + v_radial * n;

    // Handle the case where the resulting velocity is nearly zero
    if (result_vel.norm() < 1e-6) {
      result_vel = Eigen::Vector2d(-n.y(), n.x());
    }

    return result_vel.normalized() * SPEED;
  }

  std::vector<Eigen::Vector2d> getDiscontinuities()
  {
    // returns instantly if there are 0 or 1 point to avoid size_t underflow
    if (laser_points.size() < 2) return laser_points;

    // compares the distance between one point and the next
    const double THRESHOLD = 0.6;
    const double MAX_DISCONTINUITY_RANGE = 7.0;
    std::vector<Eigen::Vector2d> discontinuities;
    for (size_t i = 0; i < laser_points.size() - 1; i++)
    {
      if ((laser_points[i] - laser_points[i+1]).norm() >= THRESHOLD)
      {
        discontinuities.push_back(laser_points[i]);
        discontinuities.push_back(laser_points[i+1]);
      }
    }

    // compare first and last points
    if ((laser_points[0] - laser_points[laser_points.size() - 1]).norm() >= THRESHOLD)
    {
      discontinuities.push_back(laser_points[0]);
      discontinuities.push_back(laser_points[laser_points.size() - 1]);
    }

    for (size_t i = 0; i < discontinuities.size();)
    {
      if ((discontinuities[i] - robot_pos).norm() > MAX_DISCONTINUITY_RANGE)
      {
        discontinuities.erase(discontinuities.begin() + i);
      }
      else i++;
    }

    return discontinuities;
  }

  void publishDiscontinuities(const std::vector<Eigen::Vector2d> &discontinuities)
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "tangent_bug";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.12;
    marker.scale.y = 0.12;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (const auto& discontinuity : discontinuities)
    {
      geometry_msgs::msg::Point point;
      point.x = discontinuity.x();
      point.y = discontinuity.y();
      point.z = 0.0;
      marker.points.push_back(point);
    }

    discontinuities_pub->publish(marker);
  }

  Eigen::Vector2d getMPoint()
  {
    if (laser_points.empty()) return robot_pos;

    double d_min = 1e9;
    Eigen::Vector2d M = laser_points[0];
    for (const auto& point : laser_points)
    {
      // skip "infinite" points
      if ((point - robot_pos).norm() >= 5) continue;

      double d = (goal - point).norm();
      if (d < d_min)
      {
        d_min = d;
        M = point;
      }
    }
    return M;
  }

  Eigen::Vector2d calculateHeuristic(std::vector<Eigen::Vector2d> &discontinuities)
  {
    if (discontinuities.empty()) return goal;

    Eigen::Vector2d best = discontinuities[0];
    double min_cost = 1000;

    for (const auto& point : discontinuities)
    {
      double cost = (goal - point).norm() + (point - robot_pos).norm();

      if (cost < min_cost)
      {
        best = point;
        min_cost = cost;
      }
    }

    return best;
  }

  // states
  enum class State {
    MOTION_TO_GOAL,
    BOUNDARY_FOLLOWING
  };

  // --------------------- Variables --------------------------

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr   goal_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr discontinuities_pub;
  rclcpp::TimerBase::SharedPtr                                 control_timer;

  // laser points vector
  std::vector<Eigen::Vector2d> laser_points; 
  Eigen::Vector2d              closest_point;

  // robot and goal
  Eigen::Vector2d goal;
  Eigen::Vector2d robot_pos;
  double          robot_yaw;
  bool            goal_received = false;

  // state machine variables
  Eigen::Vector2d  last_heuristic;
  Eigen::Vector2d  M_point;
  State            current_state = State::MOTION_TO_GOAL;
  double           d_followed;
  double           d_reach;
  bool             check_unreachable = false;

  // consts
  const double SPEED = 0.5;
  const double SAFE_RADIUS = 0.4;
  const double D = 0.05;
  const double TOLERANCE = 0.05;
  const double HYSTERESIS = 0.05;
  const double GOAL_UNREACHABLE_TH= 0.5;
  const double GOAL_UNREACHABLE_MIN= 0.05;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TangentBug>());
  rclcpp::shutdown();
  return 0;
}
