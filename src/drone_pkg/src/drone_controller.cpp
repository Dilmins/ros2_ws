// ─────────────────────────────────────────────────────────────────────────────
// drone_controller.cpp
// Phase 5 — Altitude Hold PID
// Q/E adjusts target height. PID auto-corrects thrust to hold it.
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <chrono>
#include <algorithm>
#include <string>

using namespace std::chrono_literals;

static constexpr double MASS_KG     = 1.7;
static constexpr double GRAVITY     = 9.81;
static constexpr double HOVER_FORCE = MASS_KG * GRAVITY;   // 16.677 N
static constexpr double MAX_THRUST  = HOVER_FORCE * 2.0;   // 33.354 N
static constexpr double MAX_TORQUE  = 0.5;
static constexpr double MOTOR_TAU   = 0.1;
static constexpr double CTRL_HZ     = 50.0;
static constexpr double DT          = 1.0 / CTRL_HZ;
static constexpr double CMD_TIMEOUT_S = 0.5;   // teleop publishes at 20 Hz

// ── PID gains (tune these) ────────────────────────────────────────────────────
static constexpr double KP          = 8.0;   // proportional — main correction
static constexpr double KI          = 0.5;   // integral     — removes steady-state error
static constexpr double KD          = 4.0;   // derivative   — damps oscillation
static constexpr double I_CLAMP     = 5.0;   // prevents integral windup
// ─────────────────────────────────────────────────────────────────────────────

class DroneController : public rclcpp::Node
{
public:
  DroneController()
  : Node("drone_controller"),
    current_z_(0.0), target_z_(0.0),
    pid_integral_(0.0), prev_error_(0.0),
    fx_(0.0), fy_(0.0), tz_(0.0),
    target_fx_(0.0), target_fy_(0.0), target_tz_(0.0),
    altitude_hold_active_(false)
  {
    wrench_pub_ = create_publisher<geometry_msgs::msg::Wrench>(
      "/drone/gazebo_ros_force", 10);

    // Teleop commands
    // linear.z  > 0 → raise target height, < 0 → lower
    // linear.x/y → pitch/roll (unchanged)
    // angular.z  → yaw (unchanged)
    // linear.z == 99 → special: lock current height (SPACE)
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/drone/cmd", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (msg->linear.z == 99.0) {
          // SPACE — lock current height
          target_z_ = current_z_;
          altitude_hold_active_ = true;
          RCLCPP_INFO(get_logger(), "Altitude locked at %.2f m", target_z_);
        } else if (msg->linear.z == -99.0) {
          // X key — kill (disable hold, zero thrust)
          altitude_hold_active_ = false;
          target_z_ = 0.0;
        } else {
          // Q/E — nudge target height up or down by 0.2 m per keypress
          target_z_ += msg->linear.z * 0.2;
          target_z_  = std::max(0.0, target_z_);
          if (target_z_ > 0.0) altitude_hold_active_ = true;
        }
        last_cmd_time_ = now();
        target_fx_ = std::clamp(msg->linear.x,  -1.0, 1.0) * 3.0;
        target_fy_ = std::clamp(msg->linear.y,  -1.0, 1.0) * 3.0;
        target_tz_ = std::clamp(msg->angular.z, -1.0, 1.0) * MAX_TORQUE;
      });

    // Read drone's actual Z position from Gazebo
    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      "/gazebo/model_states", 10,
      [this](const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (msg->name[i] == "quadrotor") {
            current_z_ = msg->pose[i].position.z;
            break;
          }
        }
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(DT),
      std::bind(&DroneController::control_loop, this));

    RCLCPP_INFO(get_logger(),
      "\n[DroneController] Phase 5 — Altitude Hold PID"
      "\n  Q/E  : raise/lower target height (0.2 m steps)"
      "\n  SPACE: lock current height"
      "\n  X    : kill motors"
      "\n  Kp=%.1f  Ki=%.1f  Kd=%.1f", KP, KI, KD);
  }

private:
  void control_loop()
  {
    // Command watchdog: if the teleop goes silent, stop lateral/yaw motion.
    // (Altitude target is kept, so the drone hovers instead of falling.)
    if ((now() - last_cmd_time_).seconds() > CMD_TIMEOUT_S) {
      target_fx_ = target_fy_ = target_tz_ = 0.0;
    }

    const double alpha = DT / (MOTOR_TAU + DT);

    // Lateral/yaw motor lag (unchanged from Phase 4)
    fx_ += alpha * (target_fx_ - fx_);
    fy_ += alpha * (target_fy_ - fy_);
    tz_ += alpha * (target_tz_ - tz_);

    geometry_msgs::msg::Wrench w;

    if (!altitude_hold_active_) {
      // Motors off
      w.force.z = 0.0;
    } else {
      // ── PID ───────────────────────────────────────────────────────────────
      double error      = target_z_ - current_z_;
      pid_integral_    += error * DT;
      pid_integral_     = std::clamp(pid_integral_, -I_CLAMP, I_CLAMP);
      double derivative = (error - prev_error_) / DT;
      prev_error_       = error;

      // PID output = force correction above/below hover
      double pid_out = KP * error + KI * pid_integral_ + KD * derivative;

      // Add hover feedforward so PID only corrects the delta
      w.force.z = std::clamp(HOVER_FORCE + pid_out, 0.0, MAX_THRUST);
    }

    w.force.x  = fx_;
    w.force.y  = fy_;
    w.torque.z = tz_;

    wrench_pub_->publish(w);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "target_z=%.2f | current_z=%.2f | Fz=%.2fN",
      target_z_, current_z_, w.force.z);
  }

  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr      wrench_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    cmd_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  double current_z_, target_z_;
  double pid_integral_, prev_error_;
  double fx_, fy_, tz_;
  double target_fx_, target_fy_, target_tz_;
  bool   altitude_hold_active_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneController>());
  rclcpp::shutdown();
  return 0;
}