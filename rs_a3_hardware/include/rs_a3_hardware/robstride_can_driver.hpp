/**
 * @file robstride_can_driver.hpp
 * @brief Robstride motor CAN communication driver using candlight/gs_usb
 * 
 * Supports RS00 and RS05 series motors with private protocol
 */

#ifndef RS_A3_HARDWARE__ROBSTRIDE_CAN_DRIVER_HPP_
#define RS_A3_HARDWARE__ROBSTRIDE_CAN_DRIVER_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace rs_a3_hardware
{

/**
 * @brief Motor type enumeration
 */
enum class MotorType
{
  RS00,  // 1-3号关节: 力矩±14Nm, 速度±33rad/s
  RS05   // 4-6号关节: 力矩±5.5Nm, 速度±50rad/s
};

/**
 * @brief Motor run mode
 */
enum class RunMode : uint8_t
{
  MOTION_CONTROL = 0,  // 运控模式
  POSITION_PP = 1,     // 位置模式(PP)
  VELOCITY = 2,        // 速度模式
  CURRENT = 3,         // 电流模式
  POSITION_CSP = 5     // 位置模式(CSP)
};

/**
 * @brief Motor feedback data
 */
struct MotorFeedback
{
  uint8_t motor_id;
  double position;      // rad
  double velocity;      // rad/s
  double torque;        // Nm
  double temperature;   // °C
  uint8_t mode_state;   // 0: Reset, 1: Cali, 2: Motor
  uint8_t fault_code;
  bool is_valid;
};

/**
 * @brief Motor parameters for different motor types
 */
struct MotorParams
{
  double p_min;   // Position min (rad)
  double p_max;   // Position max (rad)
  double v_min;   // Velocity min (rad/s)
  double v_max;   // Velocity max (rad/s)
  double t_min;   // Torque min (Nm)
  double t_max;   // Torque max (Nm)
  double kp_min;  // Kp min
  double kp_max;  // Kp max
  double kd_min;  // Kd min
  double kd_max;  // Kd max
};

/**
 * @brief Get motor parameters by type
 */
inline MotorParams getMotorParams(MotorType type)
{
  MotorParams params;
  params.p_min = -12.57;
  params.p_max = 12.57;
  params.kp_min = 0.0;
  params.kd_min = 0.0;
  
  // 所有电机 Kp/Kd 映射范围相同: Kp 0~500, Kd 0~5
  params.kp_max = 500.0;
  params.kd_max = 5.0;
  
  switch (type)
  {
    case MotorType::RS00:
      params.v_min = -33.0;
      params.v_max = 33.0;
      params.t_min = -14.0;
      params.t_max = 14.0;
      break;
    case MotorType::RS05:
      params.v_min = -50.0;
      params.v_max = 50.0;
      params.t_min = -5.5;
      params.t_max = 5.5;
      break;
  }
  return params;
}

/**
 * @brief Robstride CAN driver class
 */
class RobstrideCanDriver
{
public:
  /**
   * @brief Construct a new Robstride CAN Driver
   * @param can_interface CAN interface name (e.g., "can0")
   * @param host_can_id Host CAN ID (default 0xFD)
   */
  explicit RobstrideCanDriver(const std::string& can_interface, uint8_t host_can_id = 0xFD);
  
  ~RobstrideCanDriver();
  
  /**
   * @brief Initialize the CAN interface
   * @return true if successful
   */
  bool init();
  
  /**
   * @brief Close the CAN interface
   */
  void close();
  
  /**
   * @brief Check if driver is connected
   */
  bool isConnected() const { return socket_fd_ >= 0; }
  
  /**
   * @brief Enable motor
   * @param motor_id Motor CAN ID
   * @return true if successful
   */
  bool enableMotor(uint8_t motor_id);
  
  /**
   * @brief Disable motor
   * @param motor_id Motor CAN ID
   * @param clear_fault Clear fault flag
   * @return true if successful
   */
  bool disableMotor(uint8_t motor_id, bool clear_fault = false);
  
  /**
   * @brief Set motor zero position
   * @param motor_id Motor CAN ID
   * @return true if successful
   */
  bool setZeroPosition(uint8_t motor_id);
  
  /**
   * @brief Send motion control command (Type 1)
   * @param motor_id Motor CAN ID
   * @param motor_type Motor type for parameter mapping
   * @param position Target position (rad)
   * @param velocity Target velocity (rad/s)
   * @param kp Position gain
   * @param kd Velocity gain
   * @param torque Feedforward torque (Nm)
   * @return true if successful
   */
  bool sendMotionControl(
    uint8_t motor_id,
    MotorType motor_type,
    double position,
    double velocity,
    double kp,
    double kd,
    double torque = 0.0);
  
  /**
   * @brief Write single parameter (Type 18)
   * @param motor_id Motor CAN ID
   * @param param_index Parameter index
   * @param value Parameter value (float)
   * @return true if successful
   */
  bool writeParameter(uint8_t motor_id, uint16_t param_index, float value);
  
  /**
   * @brief Set run mode
   * @param motor_id Motor CAN ID
   * @param mode Run mode
   * @return true if successful
   */
  bool setRunMode(uint8_t motor_id, RunMode mode);
  
  /**
   * @brief Set CSP position command
   * @param motor_id Motor CAN ID
   * @param position Target position (rad)
   * @return true if successful
   */
  bool setPositionCSP(uint8_t motor_id, double position);
  
  /**
   * @brief Set velocity limit for CSP mode
   * @param motor_id Motor CAN ID
   * @param velocity_limit Maximum velocity (rad/s)
   * @return true if successful
   */
  bool setVelocityLimit(uint8_t motor_id, double velocity_limit);
  
  /**
   * @brief Get motor feedback
   * @param motor_id Motor CAN ID
   * @return Motor feedback data
   */
  MotorFeedback getMotorFeedback(uint8_t motor_id);
  
  /**
   * @brief Set motor type for a specific motor ID
   * @param motor_id Motor CAN ID
   * @param type Motor type (RS00 or RS05)
   */
  void setMotorType(uint8_t motor_id, MotorType type);
  
  /**
   * @brief Start receive thread
   */
  void startReceiveThread();
  
  /**
   * @brief Stop receive thread
   */
  void stopReceiveThread();

private:
  /**
   * @brief Build extended CAN ID for private protocol
   * @param comm_type Communication type (0-26)
   * @param data_area2 Data area 2 (host CAN ID etc.)
   * @param target_id Target motor CAN ID
   * @return 29-bit extended CAN ID
   */
  uint32_t buildExtendedCanId(uint8_t comm_type, uint16_t data_area2, uint8_t target_id);
  
  /**
   * @brief Send CAN frame
   */
  bool sendFrame(const can_frame& frame);
  
  /**
   * @brief Receive CAN frame
   */
  bool receiveFrame(can_frame& frame, int timeout_ms = 100);
  
  /**
   * @brief Receive thread function
   */
  void receiveThreadFunc();
  
  /**
   * @brief Parse motor feedback from CAN frame
   */
  void parseFeedback(const can_frame& frame, MotorType motor_type);
  
  /**
   * @brief Convert float to uint16 for protocol
   */
  static uint16_t floatToUint16(double x, double x_min, double x_max);
  
  /**
   * @brief Convert uint16 to float from protocol
   */
  static double uint16ToFloat(uint16_t x_int, double x_min, double x_max);
  
  std::string can_interface_;
  uint8_t host_can_id_;
  int socket_fd_;
  
  std::mutex send_mutex_;
  std::mutex feedback_mutex_;
  
  std::atomic<bool> receive_running_;
  std::thread receive_thread_;
  
  // Motor feedback storage (indexed by motor_id)
  std::vector<MotorFeedback> motor_feedbacks_;
  std::vector<MotorType> motor_types_;
};

}  // namespace rs_a3_hardware

#endif  // RS_A3_HARDWARE__ROBSTRIDE_CAN_DRIVER_HPP_

