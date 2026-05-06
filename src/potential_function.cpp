#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <eigen3/Eigen/Dense>

class PotentialFunction : public rclcpp::Node
{
public:
  PotentialFunction()
  : Node("potential_function")
  {
    // publishers and subscribers
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&PotentialFunction::laserCallback, this, std::placeholders::_1)
    );

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&PotentialFunction::odomCallback, this, std::placeholders::_1)
    );

    goal_sub = this->create_subscription<geometry_msgs::msg::Point>(
      "/goal",
      10,
      std::bind(&PotentialFunction::goalCallback, this, std::placeholders::_1)
    );

    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10
    );

    // timer for the control loop
    control_timer = this->create_wall_timer(
      std::chrono::milliseconds(LOOP_DT_MS),
      std::bind(&PotentialFunction::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Potential function node started.");
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
  }

  void controlLoop()
  {
    if (!goal_received) return; 

    // Forca atrativa
    Eigen::Vector2d q_diff = robot_pos - goal;
    double dist_to_goal = q_diff.norm();
    Eigen::Vector2d grad_U_att;

    if (dist_to_goal <= D_ESTRELA) {
        // Quadratico: robo esta perto
        grad_U_att = ZETA * q_diff;
    } else {
        // Conico: robo esta longe
        grad_U_att = (D_ESTRELA * ZETA * q_diff) / dist_to_goal;
    }

    Eigen::Vector2d f_att = -grad_U_att;

    // Forca repulsiva
    Eigen::Vector2d f_rep(0, 0);
    double dist = (robot_pos - closest_point).norm();

    if (dist < Q_ESTRELA) {
        Eigen::Vector2d rep_dir = (robot_pos - closest_point).normalized();
        f_rep = ETA * (1.0/dist - 1.0/Q_ESTRELA) * (1.0/(dist * dist)) * rep_dir;
    }
    
    Eigen::Vector2d f_total = f_att + f_rep;
    if (f_total.norm() > MAX_VEL) f_total = f_total.normalized() * MAX_VEL;
    sendVelocity(f_total);
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
  bool            goal_received = false; 

  // consts
  const int    LOOP_DT_MS = 100;
  const double TOLERANCE = 0.05;
  const double D = 0.05;
  const double ZETA = 12;
  const double ETA = 1.3;
  const double MAX_VEL = 1; 
  const double D_ESTRELA = 1.5;
  const double Q_ESTRELA = 1.4;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotentialFunction>());
  rclcpp::shutdown();
  return 0;
}
