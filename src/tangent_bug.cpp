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

    // get closest obstacle point
    double min_dist = 1000;

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

        if (r < min_dist)
        {
          closest_point = new_point;
          min_dist = r;
        }
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
    boundary_direction = 1;
    last_heuristic = 1e9;
    d_reach = 1e9;
    d_followed = 1e9;
    check_unreachable = false;
    out_of_min_dist = false;
  }

  void controlLoop()
  {
    if (!goal_received) return;

    // if path to goal is clear, send direct velocity
    if (isGoalClear())
    {
      Eigen::Vector2d vel = goal - robot_pos;
      vel = vel.normalized() * SPEED;
      sendVelocity(vel);
      return;
    } 

    // get discontinuity points 
    std::vector<Eigen::Vector2d> discontinuities = getDiscontinuities();

    // calculate the heuristic to determine the best discontinuity point
    HeuristicResult res = calculateHeuristic(discontinuities);
    Eigen::Vector2d result_point = res.point;
    double result_cost = res.cost;

    switch (current_state) {

      case State::MOTION_TO_GOAL:
        sendVelocity((result_point - robot_pos).normalized()*SPEED);

        if (result_cost > last_heuristic + 0.05)
        {
          // found local minimum
          RCLCPP_INFO(this->get_logger(), "Found local minimum, switching to boundary following mode.");
          current_state = State::BOUNDARY_FOLLOWING;
          d_reach = last_heuristic;
          d_followed = result_cost;

          // determine optimal boundary direction
          Eigen::Vector2d to_obstacle = closest_point - robot_pos;
          Eigen::Vector2d tangent_ccw(-to_obstacle.y(), to_obstacle.x());
          Eigen::Vector2d to_best_point = result_point - robot_pos;

          if (tangent_ccw.dot(to_best_point) >= 0) {
            boundary_direction = 1;
          } else {
            boundary_direction = -1;
          }

          // start checking for unreachable goal
          if (!check_unreachable)
          {
            unreachable_start = robot_pos;
            check_unreachable = true;
            out_of_min_dist = false;
          }

        } else {
          last_heuristic = result_cost;
        }
        break;

      case State::BOUNDARY_FOLLOWING:
        {
          if (laser_points.empty()) break;

          // check if goal is unreachable
          if (!out_of_min_dist &&(robot_pos - unreachable_start).norm() > GOAL_UNREACHABLE_MIN_DIST)
          { 
            out_of_min_dist = true;
            RCLCPP_INFO(this->get_logger(), "Checking if goal is unreachable.");
          }
          if (out_of_min_dist)
          {
            if ((robot_pos - unreachable_start).norm() <= GOAL_UNREACHABLE_TH)
            {
              RCLCPP_INFO(this->get_logger(), "Completed cycle, goal seems unreachable.");
              current_state = State::MOTION_TO_GOAL;
              sendVelocity(Eigen::Vector2d::Zero());
              check_unreachable = false;
              goal_received = false;
            }
          }

          // tangent direction based on chosen boundary direction
          Eigen::Vector2d to_obstacle = closest_point - robot_pos;
          Eigen::Vector2d tangent(-to_obstacle.y(), to_obstacle.x());
          tangent.normalize();
          tangent = boundary_direction*tangent;

          // distance correction to maintain safe radius
          double dist_error = to_obstacle.norm() - SAFE_RADIUS;
          Eigen::Vector2d correction = to_obstacle.normalized() * dist_error * 2.0;

          // final boundary following velocity
          Eigen::Vector2d boundary_vel = (tangent + correction).normalized() * SPEED;
          sendVelocity(boundary_vel);

          // check leave condition
          if (result_cost < d_reach)
          {
            RCLCPP_INFO(this->get_logger(), "Leave condition met! Returning to Motion-to-Goal.");
            current_state = State::MOTION_TO_GOAL;
            last_heuristic = result_cost; 
          }
          else
        {
            // update d_followed
            d_followed = result_cost;
          }
          break;
        }
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
    // return if no obstacles
    if (laser_points.empty()) return desired_vel; 

    // return if not too close
    double range = (closest_point - robot_pos).norm();
    if (range > SAFE_RADIUS) return desired_vel;

    // return if velocity isn't going towards obstacle
    if (desired_vel.dot(closest_point - robot_pos) <= 0) return desired_vel;

    // remove normal component
    Eigen::Vector2d n = (closest_point - robot_pos).normalized();
    double projection = desired_vel.dot(n);
    Eigen::Vector2d result_vel = (desired_vel - projection*n).normalized()*SPEED;

    return result_vel;
  }

  std::vector<Eigen::Vector2d> getDiscontinuities()
  {
    // returns instantly if there are 0 or 1 point to avoid size_t underflow
    if (laser_points.size() < 2) return laser_points;

    // compares the distance between one point and the next
    const double THRESHOLD = 0.3;
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

    return discontinuities;
  }

  struct HeuristicResult {
    Eigen::Vector2d point;
    double cost;
  };

  HeuristicResult calculateHeuristic(std::vector<Eigen::Vector2d> &discontinuities)
  {
    if (discontinuities.empty()) return {goal, (goal - robot_pos).norm()};

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

    return {best, min_cost};
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
  rclcpp::TimerBase::SharedPtr                                 control_timer;

  // laser points vector
  std::vector<Eigen::Vector2d> laser_points; 
  Eigen::Vector2d              closest_point;

  // robot and goal
  Eigen::Vector2d goal;
  Eigen::Vector2d robot_pos;
  double          robot_yaw;
  bool goal_received = false;

  // state machine variables
  State   current_state = State::MOTION_TO_GOAL;
  int     boundary_direction = 1;
  double  d_reach;
  double  d_followed;
  double  last_heuristic;

  // detect unreachable goal variables
  Eigen::Vector2d unreachable_start;
  bool            check_unreachable = false;
  bool            out_of_min_dist = false;  

  // consts
  const double SPEED = 0.5;
  const double SAFE_RADIUS = 0.6;
  const double D = 0.05;
  const double TOLERANCE = 0.05;
  const double GOAL_UNREACHABLE_TH= 0.3;
  const double GOAL_UNREACHABLE_MIN_DIST= 0.5;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TangentBug>());
  rclcpp::shutdown();
  return 0;
}
