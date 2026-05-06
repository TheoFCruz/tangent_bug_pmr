#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "pmr_tp1/visualizer.hpp"

#include <eigen3/Eigen/Dense>

#include <cmath>
#include <string>
#include <vector>

class PathWithPotential : public rclcpp::Node
{
public:
  PathWithPotential()
  : Node("path_with_potential"), visualizer(this)
  {
    // node parameters
    robot_id = this->declare_parameter<int>("robot_id", 1);
    num_robots = this->declare_parameter<int>("num_robots", 1);

    if (robot_id < 1)
    {
      RCLCPP_WARN(this->get_logger(), "Invalid robot_id %d. Using robot_id=1.", robot_id);
      robot_id = 1;
    }

    if (num_robots < 1)
    {
      RCLCPP_WARN(this->get_logger(), "Invalid num_robots %d. Using num_robots=1.", num_robots);
      num_robots = 1;
    }

    // publishers and subscribers
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      rclcpp::SensorDataQoS(),
      std::bind(&PathWithPotential::laserCallback, this, std::placeholders::_1)
    );

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10,
      std::bind(&PathWithPotential::odomCallback, this, std::placeholders::_1)
    );

    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10
    );

    createOtherRobotSubscriptions();

    if (robot_id == 1)
    {
      publishPath();
    }

    // timer for the control loop
    control_timer = this->create_wall_timer(
      std::chrono::milliseconds(LOOP_DT_MS),
      std::bind(&PathWithPotential::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Path with potential node started.");
    RCLCPP_INFO(
      this->get_logger(),
      "robot_id=%d num_robots=%d namespace=%s",
      robot_id,
      num_robots,
      this->get_namespace()
    );
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

      Eigen::Vector2d new_point(0, 0);
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
    odom_received = true;
  }

  void otherOdomCallback(size_t index, const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    other_robot_positions[index] = Eigen::Vector2d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y
    );
    other_robot_odom_received[index] = true;
  }

  void controlLoop()
  {
    if (!odom_received) return;

    Eigen::Vector2d trajectory_vel = calculateTrajectoryVelocity();
    Eigen::Vector2d repulsive_vel = calculateRepulsiveVelocity();

    sendVelocity(trajectory_vel + repulsive_vel);
  }

  // ------------------ Utility Functions ---------------------

  void createOtherRobotSubscriptions()
  {
    other_robot_positions.clear();
    other_robot_odom_received.clear();
    other_odom_subs.clear();

    for (int id = 1; id <= num_robots; ++id)
    {
      if (id == robot_id) continue;

      std::string topic = "/robot_" + std::to_string(id) + "/odom";
      size_t index = other_robot_positions.size();
      other_robot_positions.push_back(Eigen::Vector2d::Zero());
      other_robot_odom_received.push_back(false);

      other_odom_subs.push_back(
        this->create_subscription<nav_msgs::msg::Odometry>(
          topic,
          10,
          [this, index](const nav_msgs::msg::Odometry::SharedPtr msg) {otherOdomCallback(index, msg);}
        )
      );

      RCLCPP_INFO(this->get_logger(), "Subscribing to robot_%d odom on %s", id, topic.c_str());
    }
  }

  Eigen::Vector2d calculateTrajectoryVelocity()
  {
    if (is_first_iteration)
    {
      t_start = this->now();
      is_first_iteration = false;
    }

    // reference time based on ID
    rclcpp::Time t_current = this->now();
    double t = (t_current - t_start).seconds();
    double reference_t = t - (robot_id - 1) * ROBOT_DELTA_T;

    // adjust reference to controlled point
    Eigen::Vector2d tracking_pos;
    tracking_pos.x() = robot_pos.x() + D * std::cos(robot_yaw);
    tracking_pos.y() = robot_pos.y() + D * std::sin(robot_yaw);

    // get reference point
    Eigen::Vector2d followed_point = getLemniscate(reference_t);
    visualizer.publishPoint(
      "path_with_potential/followed_point",
      followed_point,
      "odom",
      robot_id,
      0.0,
      1.0,
      1.0
    );

    Eigen::Vector2d error = followed_point - tracking_pos;

    // feedforward velocity
    double dt = (double)LOOP_DT_MS / 1000.0;
    Eigen::Vector2d d_pos = getLemniscate(reference_t + dt) - getLemniscate(reference_t - dt);
    Eigen::Vector2d ff_vel = d_pos / (2.0 * dt);

    return VEL_GAIN * error + ff_vel;
  }

  Eigen::Vector2d calculateRepulsiveVelocity()
  {
    return calculateObstacleRepulsion() + calculateRobotRepulsion();
  }

  Eigen::Vector2d calculateObstacleRepulsion()
  {
    return Eigen::Vector2d::Zero();
  }

  Eigen::Vector2d calculateRobotRepulsion()
  {
    Eigen::Vector2d repulsive_vel = Eigen::Vector2d::Zero();

    for (size_t i = 0; i < other_robot_positions.size(); ++i)
    {
      if (!other_robot_odom_received[i]) continue;

      Eigen::Vector2d diff = robot_pos - other_robot_positions[i];
      double dist = diff.norm();

      if (dist < 1e-6 || dist >= ROBOT_EFFECTIVE_RADIUS) continue;

      repulsive_vel += ROBOT_REPULSION_GAIN *
        (1.0 / dist - 1.0 / ROBOT_EFFECTIVE_RADIUS) *
        diff / (dist * dist * dist);
    }

    if (repulsive_vel.norm() > 1e-6)
    {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Robot repulsion velocity: x=%.3f y=%.3f",
        repulsive_vel.x(),
        repulsive_vel.y()
      );
    }

    return repulsive_vel;
  }

  Eigen::Vector2d getLemniscate(double t)
  {
    double theta = TRAJECTORY_FREQ * t;

    Eigen::Vector2d result;
    result.x() = A * std::sqrt(2) * std::cos(theta) /
      (std::sin(theta) * std::sin(theta) + 1);
    result.y() = A * std::sqrt(2) * std::cos(theta) * std::sin(theta) /
      (std::sin(theta) * std::sin(theta) + 1);

    return result;
  }

  void publishPath()
  {
    double dt = (double)LOOP_DT_MS / 1000.0;
    int num_samples = (int)std::ceil(T / dt);

    std::vector<Eigen::Vector2d> path_points;
    path_points.reserve(num_samples + 1);

    for (int i = 0; i <= num_samples; ++i)
    {
      double t = i * dt;
      if (t > T)
      {
        t = T;
      }

      path_points.push_back(getLemniscate(t));
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

  // --------------------- Variables --------------------------

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr          laser_sub;       
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr              odom_sub;        
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr               cmd_vel_pub;     
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> other_odom_subs; 
  rclcpp::TimerBase::SharedPtr                                          control_timer;   

  Visualizer                                                            visualizer;

  // laser points vector
  std::vector<Eigen::Vector2d> laser_points;
  Eigen::Vector2d              closest_point;

  // robot pose
  Eigen::Vector2d robot_pos;
  double          robot_yaw;
  bool            odom_received = false;

  // trajectory tracking
  rclcpp::Time t_start;
  bool         is_first_iteration = true;

  // multi-robot
  int robot_id;
  int num_robots;
  std::vector<Eigen::Vector2d> other_robot_positions;
  std::vector<bool> other_robot_odom_received;

  // consts
  const double D = 0.05;
  const double VEL_GAIN = 1.5;
  const int    LOOP_DT_MS = 100;
  const double PI = 3.14159265358979323846;
  const double A = 3.0;
  const double TRAJECTORY_FREQ = 0.1;
  const double T = 2.0 * PI / TRAJECTORY_FREQ;
  const double ROBOT_DELTA_T = 4.0;
  const double ROBOT_EFFECTIVE_RADIUS = 0.8;
  const double ROBOT_REPULSION_GAIN = 0.5;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathWithPotential>());
  rclcpp::shutdown();
  return 0;
}
