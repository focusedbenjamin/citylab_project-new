#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <ratio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    // Create callback groups for handling subscriptions and timers
    scan_sub_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    RCLCPP_INFO(this->get_logger(), "Initialized Callback Groups !");

    // Create a subscription to laser scan data with specified callback group
    rclcpp::SubscriptionOptions scan_sub_options_;
    scan_sub_options_.callback_group = scan_sub_group_;
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Patrol::sensorCallback, this, _1),
        scan_sub_options_);
    RCLCPP_INFO(this->get_logger(), "Initialized LaserScan Subscriber !");

    // Create a publisher for publishing velocity commands
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 5);
    RCLCPP_INFO(this->get_logger(), "Initialized Twist Publisher !");

    // Create a timer for controlling the robot's movement
    control_timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::controlCallback, this), timer_group_);
    RCLCPP_INFO(this->get_logger(), "Initialized Control Timer !");

    // Initialize the velocity command message
    twist_msg_.linear.x = 0.0;
    twist_msg_.angular.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Initialized Patrol Node !");
  }

  ~Patrol() {
    // Destructor
    RCLCPP_INFO(this->get_logger(), "Destructor Called");
  }

private:
  void sensorCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (scan_init_done_) {
      // this part always repeats after initialization
      // sensor callbacks must be as short as possible
      // do not overload sensor callbacks with heavy logic
      // save the instantaneous laserscan data in a class variable
      scan_msg_ = *msg;
    } else {
      // get the required information from laser scanner
      // this part needs to be done only once
      // automatically detects laser scanner properties
      ranges_size_ = msg->ranges.size();
      right_index_ = static_cast<int>(0.250 * ranges_size_);
      front_index_ = static_cast<int>(0.500 * ranges_size_);
      left_index_ = static_cast<int>(0.750 * ranges_size_);
      // Frontal 180 degrees of scan rays are divided into
      // three segments of 60 degrees each
      segment_size_ = static_cast<int>((left_index_ - right_index_) / 3.0);
      right_index_min_ = right_index_;
      right_index_max_ = right_index_ + segment_size_;
      front_index_min_ = front_index_ - static_cast<int>(segment_size_ / 2.0);
      front_index_max_ = front_index_ + static_cast<int>(segment_size_ / 2.0);
      left_index_min_ = left_index_ - segment_size_;
      left_index_max_ = left_index_;
      // set the boolean flag to true after initialization
      scan_init_done_ = true;
      RCLCPP_INFO(this->get_logger(), "Laser Initialization Done !");
    }
  }

  void controlCallback() {
    // Control the robot movement based on laser scan data

    if (!scan_init_done_) {
      return;
    }

    // Reset segment maximums and minimums
    right_max_ = scan_msg_.range_min, right_min_ = scan_msg_.range_max;
    front_max_ = scan_msg_.range_min, front_min_ = scan_msg_.range_max;
    left_max_ = scan_msg_.range_min, left_min_ = scan_msg_.range_max;

    // Iterate through the frontal 180 degrees of laser scan
    for (int i = right_index_; i <= left_index_; i++) {
      float scan_range = scan_msg_.ranges[i];
      if (!std::isinf(scan_range)) {
        // Get right segment
        if ((i >= right_index_min_) && (i < right_index_max_)) {
          if (scan_range > right_max_) {
            right_max_ = scan_range;
            right_max_idx_ = i;
          }
          if (scan_range < right_min_) {
            right_min_ = scan_range;
          }
        }
        // Get front segment
        if ((i >= front_index_min_) && (i < front_index_max_)) {
          if (scan_range > front_max_) {
            front_max_ = scan_range;
            front_max_idx_ = i;
          }
          if (scan_range < front_min_) {
            front_min_ = scan_range;
          }
        }
        // Get left segment
        if ((i >= left_index_min_) && (i < left_index_max_)) {
          if (scan_range > left_max_) {
            left_max_ = scan_range;
            left_max_idx_ = i;
          }
          if (scan_range < left_min_) {
            left_min_ = scan_range;
          }
        }
      } else {
        // do nothing
      }
    }

    // Define linear and angular velocities
    float linear_vel = 0.1;
    float angular_vel = 0.0;

    // RCLCPP_INFO(this->get_logger(), "left_max_: %+0.3f, right_max_: %+0.3f",
    //             left_max_, right_max_);
    // RCLCPP_INFO(this->get_logger(), "left_min_: %+0.3f, right_min_: %+0.3f",
    //             left_min_, right_min_);
    // RCLCPP_INFO(this->get_logger(), "left_max_idx_: %d, right_max_idx_: %d",
    //             left_max_idx_, right_max_idx_);
    // RCLCPP_INFO(this->get_logger(), "ranges_size_: %d, segment_size_: %d",
    //             ranges_size_, segment_size_);

    // Set the value for direction variable
    if (front_min_ > front_threshold_) {
      // If front section is free then move straight
      direction_ = 0.0;
    } else {
      if (right_max_ > left_max_) {
        // If right segment is greater than left, calculate direction
        direction_ = (180 - (right_max_idx_ * (360.0 / ranges_size_)));
      } else {
        // If left segment is greater than right, calculate direction
        direction_ = (180 - (left_max_idx_ * (360.0 / ranges_size_)));
      }
      // The direction sign must then be inverted
      // Since laser scan index starts from the back of the robot
      // The value must be sign-inversed to suit the 180 degree offset
      direction_ *= (-1.0);
    }

    // Determine robot's angular velocity based on direction_ variable
    if (direction_ == 0.0) {
      if ((right_min_ < side_threshold_) && (left_min_ > side_threshold_)) {
        angular_vel = 0.1;
      } else if ((left_min_ < side_threshold_) &&
                 (right_min_ > side_threshold_)) {
        angular_vel = -0.1;
      } else {
        angular_vel = 0.0;
      }
    } else {
      // The value of direction is divided by 10
      // The value is converted to radians from degrees
      angular_vel = (direction_ / 2.0) * (3.14159 / 180.0);
    }

    // Set robot velocities
    twist_msg_.linear.x = linear_vel;
    twist_msg_.angular.z = angular_vel;

    // Publish velocity command
    this->cmd_pub_->publish(twist_msg_);

    // Publish information message about linear and angular velocities
    RCLCPP_INFO(this->get_logger(),
                "Linear Velocity: %+0.3f, Angular Velocity: %+0.3f",
                twist_msg_.linear.x, twist_msg_.angular.z);
  }

  // Member variables
  rclcpp::CallbackGroup::SharedPtr scan_sub_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  geometry_msgs::msg::Twist twist_msg_;
  sensor_msgs::msg::LaserScan scan_msg_;

  bool scan_init_done_ = false;
  int ranges_size_ = 0;
  int segment_size_ = 0;
  int right_index_ = 0;     // Index value of right range
  int right_index_min_ = 0; // Start index value of right segment
  int right_index_max_ = 0; // End index value of right segment
  int front_index_ = 0;     // Index value of front range
  int front_index_min_ = 0; // Start index value of front segment
  int front_index_max_ = 0; // End index value of front segment
  int left_index_ = 0;      // Index value of left range
  int left_index_min_ = 0;  // Start index value of left segment
  int left_index_max_ = 0;  // End index value of left segment

  float right_max_ = 0.0; // Maximum scan value of right segment
  float right_min_ = 0.0; // Minimum scan value of right segment
  int right_max_idx_ = 0; // Index of maximum scan value of right segment
  float front_max_ = 0.0; // Maximum scan value of front segment
  float front_min_ = 0.0; // Minimum scan value of front segment
  int front_max_idx_ = 0; // Index of maximum scan value of front segment
  float left_max_ = 0.0;  // Maximum scan value of left segment
  float left_min_ = 0.0;  // Minimum scan value of left segment
  int left_max_idx_ = 0;  // Index of maximum scan value of left segment

  float direction_ = 0.0;
  const float front_threshold_ = 0.350;
  const float side_threshold_ = 0.120;
};

int main(int argc, char **argv) {
  // Initialize ROS2 node
  rclcpp::init(argc, argv);

  // Create an instance of the Patrol class
  std::shared_ptr<Patrol> patrol_node = std::make_shared<Patrol>();

  // Create a multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;

  // Add the Patrol node to the executor
  executor.add_node(patrol_node);

  // Spin the executor to handle callbacks
  executor.spin();

  // Shutdown ROS2 node
  rclcpp::shutdown();

  return 0;
}
