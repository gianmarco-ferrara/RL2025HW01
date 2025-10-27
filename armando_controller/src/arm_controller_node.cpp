#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// Subscriber Node to monitor joint states
class JointStateSubscriber : public rclcpp::Node
{
public:
  JointStateSubscriber()
  : Node("joint_state_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&JointStateSubscriber::topic_callback, this, _1)
    );
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "---- Joint Positions ----");
    for (size_t i = 0; i < std::min((size_t)4, msg.position.size()); i++)
    {
      RCLCPP_INFO(this->get_logger(), "Joint %zu: %.3f", i+1, msg.position[i]);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

// Publisher Node to send target positions to the controller
class ControllerPublisher : public rclcpp::Node
{
public:
  ControllerPublisher()
  : Node("controller_publisher"),
    current_step_(0)
  {
    // Declare and get parameter to choose controller type
    this->declare_parameter<bool>("use_trajectory", false);
    use_trajectory_ = this->get_parameter("use_trajectory").as_bool();

    target_pos_ = {
      {0.0, 0.0, 0.0, 1.0},
      {0.7, 0.0, 0.7, 1.0},
      {0.7, -1.0, 0.7, 0.0},
      {0.0, 0.0, 0.0, 0.0}
    };
    // Initialize publisher based on controller type choosing the appropriate topic
    if (use_trajectory_)
    {
      publisher_traj_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/trajectory_controller/joint_trajectory", 10);
      RCLCPP_INFO(this->get_logger(), "Using Trajectory Controller");
    }
    else
    {
      publisher_pos_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_controller/commands", 10);
      RCLCPP_INFO(this->get_logger(), "Using Position Controller");
    }
    // Create a timer to publish commands every 5 seconds
    timer_ = this->create_wall_timer(
      5000ms, std::bind(&ControllerPublisher::timer_callback, this)
    );
  }
private:
// Timer callback to be executed periodically
  void timer_callback()
  {
    const auto &pos = target_pos_[current_step_];

    if (use_trajectory_)
    {
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = {"j0", "j1", "j2", "j3"};
      traj.points.resize(1);
      traj.points[0].positions = pos;
      traj.points[0].time_from_start.sec = 5;
      traj.points[0].time_from_start.nanosec = 0;
      publisher_traj_->publish(traj);
      RCLCPP_INFO(this->get_logger(),
                  "Sent Trajectory step %zu", current_step_);
    }
    else
    {
      std_msgs::msg::Float64MultiArray cmd;
      cmd.data = pos;
      publisher_pos_->publish(cmd);
      RCLCPP_INFO(this->get_logger(),
                  "Sent Position step %zu", current_step_);
    }

    current_step_ = (current_step_ + 1) % target_pos_.size();
  }

  bool use_trajectory_;
  size_t current_step_;
  std::vector<std::vector<double>> target_pos_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_pos_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_traj_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create nodes
  auto joint_sub = std::make_shared<JointStateSubscriber>();
  auto controller_pub = std::make_shared<ControllerPublisher>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(joint_sub);
  executor.add_node(controller_pub);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}