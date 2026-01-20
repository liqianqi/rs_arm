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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "rs_a3_hardware/robstride_can_driver.hpp"
#include "rs_a3_hardware/s_curve_generator.hpp"

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
  double lower_limit;  // 关节下限 (rad)
  double upper_limit;  // 关节上限 (rad)
  double kp;           // 关节独立Kp (0表示使用全局值)
  double kd;           // 关节独立Kd (0表示使用全局值)
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
  
  // 位置命令平滑滤波器（带速度/加速度/加加速度限制 - S曲线规划）
  std::vector<double> smoothed_positions_;    // 平滑后的位置命令
  std::vector<double> smoothed_velocities_;   // 平滑后的速度（用于加速度限制）
  std::vector<double> smoothed_accelerations_; // 平滑后的加速度（用于S曲线规划）
  
  // 速度前馈计算相关
  std::vector<double> last_cmd_positions_;     // 上一周期的命令位置（用于计算速度前馈）
  std::vector<double> cmd_velocities_;         // 计算出的命令速度
  std::vector<double> filtered_cmd_velocities_; // 滤波后的命令速度
  double velocity_filter_alpha_;               // 速度滤波系数 (0-1)
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
  
  // S曲线轨迹生成器（每个关节一个）
  std::vector<std::unique_ptr<SCurveGenerator>> s_curve_generators_;
  bool s_curve_enabled_;                      // 是否启用S曲线规划
  
  // ============ 零力矩模式与重力补偿 ============
  // 零力矩模式标志
  bool zero_torque_mode_;           // 是否启用零力矩模式
  double zero_torque_kd_;           // 零力矩模式的阻尼系数
  
  // 重力补偿参数 (每个关节: τ = sin_coeff * sin(θ) + cos_coeff * cos(θ) + offset)
  struct GravityCompParams {
    double sin_coeff;
    double cos_coeff;
    double offset;
  };
  std::vector<GravityCompParams> gravity_params_;
  bool gravity_comp_enabled_;       // 是否启用重力补偿
  double gravity_feedforward_ratio_;          // 重力补偿前馈比例 (0-1, 默认0.5=50%)
  
  // 计算关节重力补偿力矩
  double computeGravityTorque(size_t joint_idx, double position);
  
  // 零力矩模式服务回调
  void zeroTorqueModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  
  // 零力矩模式服务
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr zero_torque_srv_;
  
  // 调试发布器
  rclcpp::Node::SharedPtr debug_node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hw_cmd_pub_;      // 控制器发来的命令
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr smoothed_cmd_pub_; // 平滑后发给电机的命令
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gravity_torque_pub_; // 重力补偿力矩
  
  // ============ 关节限位保护 ============
  double limit_margin_;           // 限位边界距离 (rad)，开始减速的距离
  double limit_stop_margin_;      // 硬停止边界 (rad)，完全停止的距离
  double limit_decel_factor_;     // 接近限位时的减速系数 (0-1)
  double max_jerk_;               // 最大加加速度限制 (rad/s³) - S曲线规划
  std::vector<bool> joint_at_limit_;  // 各关节是否处于限位状态
  std::vector<int> limit_warn_counter_;  // 限位警告计数器（防止刷屏）
  
  /**
   * @brief 计算限位保护系数
   * @param joint_idx 关节索引
   * @param current_pos 当前位置
   * @param target_pos 目标位置
   * @return 速度缩放系数 (0.0-1.0)
   */
  double computeLimitProtectionFactor(size_t joint_idx, double current_pos, double target_pos);
  
  /**
   * @brief 应用限位保护，钳位目标位置
   * @param joint_idx 关节索引
   * @param target_pos 目标位置（会被修改）
   * @return 是否触发了限位
   */
  bool applyJointLimitProtection(size_t joint_idx, double& target_pos);
};

}  // namespace rs_a3_hardware

#endif  // RS_A3_HARDWARE__RS_A3_HARDWARE_HPP_

