/**
 * @file rs_a3_hardware.cpp
 * @brief Implementation of ROS2 Control hardware interface for RS-A3 robot arm
 */

#include "rs_a3_hardware/rs_a3_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rs_a3_hardware
{

RsA3HardwareInterface::RsA3HardwareInterface()
  : can_interface_("can0")
  , host_can_id_(0xFD)
  , position_kp_(100.0)   // 增大Kp以获得更好的位置跟踪
  , position_kd_(3.0)
  , velocity_limit_(10.0)
  , control_mode_(ControlMode::POSITION)
  , use_mock_hardware_(false)
{
}

RsA3HardwareInterface::~RsA3HardwareInterface()
{
  on_shutdown(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_init(
  const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse parameters
  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  }
  if (info_.hardware_parameters.count("host_can_id")) {
    host_can_id_ = std::stoi(info_.hardware_parameters.at("host_can_id"));
  }
  if (info_.hardware_parameters.count("position_kp")) {
    position_kp_ = std::stod(info_.hardware_parameters.at("position_kp"));
  }
  if (info_.hardware_parameters.count("position_kd")) {
    position_kd_ = std::stod(info_.hardware_parameters.at("position_kd"));
  }
  if (info_.hardware_parameters.count("velocity_limit")) {
    velocity_limit_ = std::stod(info_.hardware_parameters.at("velocity_limit"));
  }
  if (info_.hardware_parameters.count("use_mock_hardware")) {
    use_mock_hardware_ = info_.hardware_parameters.at("use_mock_hardware") == "true";
  }

  // Parse joint configurations
  if (!parseJointConfig(info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Resize state and command vectors
  size_t num_joints = joint_configs_.size();
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_efforts_.resize(num_joints, 0.0);
  hw_commands_positions_.resize(num_joints, 0.0);
  hw_commands_velocities_.resize(num_joints, 0.0);
  hw_commands_efforts_.resize(num_joints, 0.0);
  
  // 初始化位置命令平滑滤波器（带速度/加速度限制）
  smoothed_positions_.resize(num_joints, 0.0);
  smoothed_velocities_.resize(num_joints, 0.0);
  
  // 默认参数
  smoothing_alpha_ = 0.15;      // 平滑系数
  max_velocity_ = 2.0;          // 最大速度 2 rad/s
  max_acceleration_ = 10.0;     // 最大加速度 10 rad/s²
  control_period_ = 0.005;      // 默认200Hz -> 5ms周期
  first_command_ = true;
  
  // 从参数读取平滑系数
  if (info_.hardware_parameters.count("smoothing_alpha")) {
    smoothing_alpha_ = std::stod(info_.hardware_parameters.at("smoothing_alpha"));
    smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.01, 1.0);
  }
  
  // 从参数读取速度限制
  if (info_.hardware_parameters.count("max_velocity")) {
    max_velocity_ = std::stod(info_.hardware_parameters.at("max_velocity"));
  }
  
  // 从参数读取加速度限制
  if (info_.hardware_parameters.count("max_acceleration")) {
    max_acceleration_ = std::stod(info_.hardware_parameters.at("max_acceleration"));
  }

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Initialized with %zu joints on %s, smoothing_alpha=%.2f, max_vel=%.1f rad/s, max_acc=%.1f rad/s²", 
              num_joints, can_interface_.c_str(), smoothing_alpha_, max_velocity_, max_acceleration_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool RsA3HardwareInterface::parseJointConfig(const hardware_interface::HardwareInfo& info)
{
  joint_configs_.clear();
  
  for (const auto& joint : info.joints) {
    JointConfig config;
    config.name = joint.name;
    
    // Parse motor_id
    if (joint.parameters.count("motor_id")) {
      config.motor_id = std::stoi(joint.parameters.at("motor_id"));
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "Joint %s missing motor_id parameter", joint.name.c_str());
      return false;
    }
    
    // Parse motor_type
    if (joint.parameters.count("motor_type")) {
      std::string type_str = joint.parameters.at("motor_type");
      if (type_str == "RS00") {
        config.motor_type = MotorType::RS00;
      } else if (type_str == "RS05") {
        config.motor_type = MotorType::RS05;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                     "Unknown motor_type %s for joint %s", type_str.c_str(), joint.name.c_str());
        return false;
      }
    } else {
      // Default based on motor_id: 1-3 = RS00, 4-6 = RS05
      config.motor_type = (config.motor_id <= 3) ? MotorType::RS00 : MotorType::RS05;
    }
    
    // Parse offset
    if (joint.parameters.count("position_offset")) {
      config.position_offset = std::stod(joint.parameters.at("position_offset"));
    } else {
      config.position_offset = 0.0;
    }
    
    // Parse direction
    if (joint.parameters.count("direction")) {
      config.direction = std::stod(joint.parameters.at("direction"));
    } else {
      config.direction = 1.0;
    }
    
    joint_configs_.push_back(config);
    
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Joint %s: motor_id=%d, type=%s, offset=%.3f, dir=%.1f",
                config.name.c_str(), config.motor_id,
                config.motor_type == MotorType::RS00 ? "RS00" : "RS05",
                config.position_offset, config.direction);
  }
  
  return true;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (use_mock_hardware_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Using mock hardware - skipping CAN initialization");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Create and initialize CAN driver
  can_driver_ = std::make_unique<RobstrideCanDriver>(can_interface_, host_can_id_);
  
  if (!can_driver_->init()) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                 "Failed to initialize CAN driver on %s", can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set motor types for each joint (with delay to avoid CAN buffer overflow)
  for (const auto& config : joint_configs_) {
    can_driver_->setMotorType(config.motor_id, config.motor_type);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 添加延迟
  }

  // Start receive thread
  can_driver_->startReceiveThread();

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "Hardware configured on %s", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (can_driver_) {
    can_driver_->stopReceiveThread();
    can_driver_->close();
    can_driver_.reset();
  }

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "Hardware cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (use_mock_hardware_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Mock hardware activated");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // 使用运控模式 (Motion Control Mode) - 纯位置控制
  // 增加延迟时间避免CAN缓冲区溢出
  for (const auto& config : joint_configs_) {
    // 1. 先停止电机
    can_driver_->disableMotor(config.motor_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 增加延迟
    
    // 2. 设置运行模式为运控模式 (run_mode = 0)
    if (!can_driver_->setRunMode(config.motor_id, RunMode::MOTION_CONTROL)) {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "Failed to set Motion Control mode for motor %d", config.motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 增加延迟
    
    // 3. 使能电机
    if (!can_driver_->enableMotor(config.motor_id)) {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "Failed to enable motor %d", config.motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Motor %d enabled in Motion Control mode (Kp=%.0f, Kd=%.1f)", 
                config.motor_id, position_kp_, position_kd_);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 增加延迟避免缓冲区溢出
  }

  // Wait for feedback
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Read initial positions and initialize smoothed positions
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    auto feedback = can_driver_->getMotorFeedback(config.motor_id);
    if (feedback.is_valid) {
      hw_positions_[i] = (feedback.position - config.position_offset) * config.direction;
      hw_commands_positions_[i] = hw_positions_[i];
      smoothed_positions_[i] = hw_positions_[i];  // 初始化平滑位置为当前位置
      smoothed_velocities_[i] = 0.0;              // 初始速度为0
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "Motor %d initial position: %.4f rad", config.motor_id, hw_positions_[i]);
    } else {
      hw_commands_positions_[i] = 0.0;
      smoothed_positions_[i] = 0.0;
      smoothed_velocities_[i] = 0.0;
      RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                  "Motor %d feedback not valid, using zero position", config.motor_id);
    }
  }
  
  // 重置首次命令标记，因为已经用实际位置初始化了
  first_command_ = false;

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), 
              "Hardware activated with CSP Position mode");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!use_mock_hardware_ && can_driver_) {
    // Disable all motors
    for (const auto& config : joint_configs_) {
      can_driver_->disableMotor(config.motor_id);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  on_deactivate(rclcpp_lifecycle::State());
  on_cleanup(rclcpp_lifecycle::State());
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "Hardware shutdown");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_error(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!use_mock_hardware_ && can_driver_) {
    // Emergency stop - disable all motors and clear faults
    for (const auto& config : joint_configs_) {
      can_driver_->disableMotor(config.motor_id, true);
    }
  }

  RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"), "Hardware error occurred");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RsA3HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RsA3HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }
  
  return command_interfaces;
}

hardware_interface::return_type RsA3HardwareInterface::read(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  static int debug_counter = 0;
  
  if (use_mock_hardware_) {
    // Mock hardware: use commanded positions as current positions
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      hw_positions_[i] = hw_commands_positions_[i];
      hw_velocities_[i] = 0.0;
      hw_efforts_[i] = 0.0;
    }
    return hardware_interface::return_type::OK;
  }

  if (!can_driver_ || !can_driver_->isConnected()) {
    return hardware_interface::return_type::ERROR;
  }

  // Read feedback from all motors
  bool any_valid = false;
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    auto feedback = can_driver_->getMotorFeedback(config.motor_id);
    
    if (feedback.is_valid) {
      any_valid = true;
      // Apply direction and offset transformations
      hw_positions_[i] = (feedback.position - config.position_offset) * config.direction;
      hw_velocities_[i] = feedback.velocity * config.direction;
      hw_efforts_[i] = feedback.torque * config.direction;
    }
  }
  
  // Debug log every 100 cycles (1 second at 100Hz)
  if (++debug_counter >= 100) {
    debug_counter = 0;
    if (!any_valid) {
      RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                  "No valid feedback received from motors");
    } else {
      RCLCPP_DEBUG(rclcpp::get_logger("RsA3HardwareInterface"),
                   "Motor 1 pos: %.4f rad", hw_positions_[0]);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RsA3HardwareInterface::write(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  static int write_counter = 0;
  
  if (use_mock_hardware_) {
    return hardware_interface::return_type::OK;
  }

  if (!can_driver_ || !can_driver_->isConnected()) {
    return hardware_interface::return_type::ERROR;
  }

  // 获取实际控制周期
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {
    dt = control_period_;  // 使用默认周期
  }

  // 【平滑滤波】第一次命令时直接设置，避免从0开始平滑
  if (first_command_) {
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      smoothed_positions_[i] = hw_commands_positions_[i];
      smoothed_velocities_[i] = 0.0;
    }
    first_command_ = false;
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "First command received, initializing smoothed positions");
  }
  
  // 使用运控模式发送命令
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    
    // ============ 带速度/加速度限制的平滑滤波器 ============
    // 1. 计算目标位置（低通滤波后的目标）
    double target_pos = smoothing_alpha_ * hw_commands_positions_[i] + 
                        (1.0 - smoothing_alpha_) * smoothed_positions_[i];
    
    // 2. 计算期望速度
    double desired_velocity = (target_pos - smoothed_positions_[i]) / dt;
    
    // 3. 应用加速度限制
    double velocity_change = desired_velocity - smoothed_velocities_[i];
    double max_velocity_change = max_acceleration_ * dt;
    velocity_change = std::clamp(velocity_change, -max_velocity_change, max_velocity_change);
    double new_velocity = smoothed_velocities_[i] + velocity_change;
    
    // 4. 应用速度限制
    new_velocity = std::clamp(new_velocity, -max_velocity_, max_velocity_);
    
    // 5. 更新平滑位置
    smoothed_positions_[i] += new_velocity * dt;
    smoothed_velocities_[i] = new_velocity;
    
    // 将平滑后的关节坐标转换为电机坐标
    double cmd_position = smoothed_positions_[i] * config.direction + config.position_offset;
    
    // Clamp position to valid range
    auto params = getMotorParams(config.motor_type);
    cmd_position = std::clamp(cmd_position, params.p_min, params.p_max);
    
    // Debug: 定期输出日志
    if (write_counter % 1000 == 0 && i == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("RsA3HardwareInterface"),
                  "[Smooth] cmd=%.4f, smooth=%.4f, vel=%.4f rad/s", 
                  hw_commands_positions_[0], smoothed_positions_[0], smoothed_velocities_[0]);
    }
    
    // 使用运控模式发送命令
    double motor_kp = std::clamp(position_kp_, 0.0, 500.0);
    double motor_kd = std::clamp(position_kd_, 0.0, 5.0);
    
    if (!can_driver_->sendMotionControl(
          config.motor_id,
          config.motor_type,
          cmd_position,
          0.0,       // velocity = 0，纯位置控制
          motor_kp,
          motor_kd,
          0.0        // torque = 0
        )) {
      // 使用静态计数器替代 THROTTLE 避免 Clock 问题
      static int warn_counter = 0;
      if (warn_counter++ % 1000 == 0) {
        RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
          "Failed to send motion control command to motor %d", config.motor_id);
      }
    }
  }

  write_counter++;
  return hardware_interface::return_type::OK;
}

}  // namespace rs_a3_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rs_a3_hardware::RsA3HardwareInterface,
  hardware_interface::SystemInterface)

