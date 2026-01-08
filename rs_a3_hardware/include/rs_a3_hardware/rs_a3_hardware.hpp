/**
 * @file rs_a3_hardware.hpp
 * @brief ROS2 Control hardware interface for RS-A3 robot arm
 */

#ifndef RS_A3_HARDWARE__RS_A3_HARDWARE_HPP_
#define RS_A3_HARDWARE__RS_A3_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rs_a3_hardware/robstride_can_driver.hpp"

namespace rs_a3_hardware
{

/**
 * @brief Joint configuration
 */
struct JointConfig
{
  std::string name;
  uint8_t motor_id;
  MotorType motor_type;
  double position_offset;
  double direction;  // 1.0 or -1.0
};

/**
 * @brief ROS2 Control hardware interface for RS-A3
 */
class RsA3HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RsA3HardwareInterface)

  RsA3HardwareInterface();
  ~RsA3HardwareInterface() override;

  // SystemInterface methods
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo& info) override;
  
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;
  
  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  /**
   * @brief Parse joint configuration from hardware info
   */
  bool parseJointConfig(const hardware_interface::HardwareInfo& info);
  
  // CAN driver
  std::unique_ptr<RobstrideCanDriver> can_driver_;
  std::string can_interface_;
  uint8_t host_can_id_;
  
  // Joint configurations
  std::vector<JointConfig> joint_configs_;
  
  // State interfaces data
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  
  // Command interfaces data
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  
  // Control parameters
  double position_kp_;
  double position_kd_;
  double velocity_limit_;
  
  // 位置命令平滑滤波器（带速度/加速度限制）
  std::vector<double> smoothed_positions_;    // 平滑后的位置命令
  std::vector<double> smoothed_velocities_;   // 平滑后的速度（用于加速度限制）
  double smoothing_alpha_;                    // 平滑系数 (0-1，越小越平滑)
  double max_velocity_;                       // 最大速度限制 (rad/s)
  double max_acceleration_;                   // 最大加速度限制 (rad/s²)
  bool first_command_;                        // 第一次命令标记
  double control_period_;                     // 控制周期 (s)
  
  // Control mode
  enum class ControlMode
  {
    POSITION,    // CSP mode
    VELOCITY,
    EFFORT
  };
  ControlMode control_mode_;
  
  bool use_mock_hardware_;
};

}  // namespace rs_a3_hardware

#endif  // RS_A3_HARDWARE__RS_A3_HARDWARE_HPP_

