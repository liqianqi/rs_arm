/**
 * @file s_curve_generator.hpp
 * @brief Standard 7-segment S-curve trajectory generator for smooth motion control
 * 
 * Implements a standard 7-segment S-curve velocity profile:
 * 1. Jerk-up (acceleration increasing)
 * 2. Constant acceleration
 * 3. Jerk-down (acceleration decreasing to 0)
 * 4. Constant velocity (cruise)
 * 5. Jerk-down (deceleration increasing)
 * 6. Constant deceleration
 * 7. Jerk-up (deceleration decreasing to 0)
 * 
 * Features:
 * - Real-time incremental update for joystick control
 * - Full trajectory generation for MoveIt path re-parameterization
 * - Configurable max_velocity, max_acceleration, max_jerk
 */

#ifndef RS_A3_HARDWARE__S_CURVE_GENERATOR_HPP_
#define RS_A3_HARDWARE__S_CURVE_GENERATOR_HPP_

#include <cmath>
#include <algorithm>
#include <vector>

namespace rs_a3_hardware
{

/**
 * @brief S-curve motion profile for a single segment
 */
struct SCurveProfile
{
  // Time durations for each of the 7 segments
  double t1;  // Jerk-up time (acceleration phase)
  double t2;  // Constant acceleration time
  double t3;  // Jerk-down time (end of acceleration phase)
  double t4;  // Constant velocity (cruise) time
  double t5;  // Jerk-down time (deceleration phase)
  double t6;  // Constant deceleration time
  double t7;  // Jerk-up time (end of deceleration phase)
  
  double total_time;
  
  // Motion constraints
  double j_max;   // Maximum jerk (rad/s³)
  double a_max;   // Maximum acceleration (rad/s²)
  double v_max;   // Maximum velocity (rad/s)
  
  // Motion parameters
  double distance;      // Total distance to travel
  double direction;     // +1 or -1
  double v_cruise;      // Actual cruise velocity achieved
  double a_limit;       // Actual acceleration limit achieved
  
  // Initial conditions
  double p0;  // Initial position
  double v0;  // Initial velocity
  double a0;  // Initial acceleration
  
  SCurveProfile()
    : t1(0), t2(0), t3(0), t4(0), t5(0), t6(0), t7(0)
    , total_time(0)
    , j_max(50.0), a_max(10.0), v_max(3.0)
    , distance(0), direction(1.0), v_cruise(0), a_limit(0)
    , p0(0), v0(0), a0(0)
  {}
};

/**
 * @brief Real-time state for incremental S-curve control
 */
struct SCurveState
{
  double position;
  double velocity;
  double acceleration;
  double jerk;
  
  // Target tracking
  double target_position;
  bool is_moving;
  
  SCurveState()
    : position(0), velocity(0), acceleration(0), jerk(0)
    , target_position(0), is_moving(false)
  {}
};

/**
 * @brief S-curve trajectory generator
 * 
 * Provides two modes of operation:
 * 1. Real-time mode: Continuously updates position based on target changes
 * 2. Trajectory mode: Generates complete trajectory for a given displacement
 */
class SCurveGenerator
{
public:
  /**
   * @brief Constructor with motion constraints
   * @param max_velocity Maximum velocity (rad/s)
   * @param max_acceleration Maximum acceleration (rad/s²)
   * @param max_jerk Maximum jerk (rad/s³)
   */
  SCurveGenerator(double max_velocity = 3.0, 
                  double max_acceleration = 10.0, 
                  double max_jerk = 50.0);
  
  /**
   * @brief Set motion constraints
   */
  void setConstraints(double max_velocity, double max_acceleration, double max_jerk);
  
  /**
   * @brief Get current constraints
   */
  void getConstraints(double& max_velocity, double& max_acceleration, double& max_jerk) const;
  
  /**
   * @brief Initialize the generator with current state
   * @param position Current position
   * @param velocity Current velocity (default 0)
   * @param acceleration Current acceleration (default 0)
   */
  void initialize(double position, double velocity = 0.0, double acceleration = 0.0);
  
  /**
   * @brief Reset to initial state
   */
  void reset();
  
  // ==================== Real-time Mode ====================
  
  /**
   * @brief Update target position (real-time mode)
   * @param target_position New target position
   */
  void setTarget(double target_position);
  
  /**
   * @brief Compute next position using S-curve profile (real-time mode)
   * @param dt Time step (seconds)
   * @return Updated position
   * 
   * This method implements real-time S-curve control by:
   * 1. Computing desired velocity based on position error
   * 2. Limiting acceleration change (jerk limiting)
   * 3. Limiting acceleration
   * 4. Limiting velocity
   * 5. Integrating to get new position
   */
  double update(double dt);
  
  /**
   * @brief Get current state
   */
  const SCurveState& getState() const { return state_; }
  
  /**
   * @brief Get current position
   */
  double getPosition() const { return state_.position; }
  
  /**
   * @brief Get current velocity
   */
  double getVelocity() const { return state_.velocity; }
  
  /**
   * @brief Get current acceleration
   */
  double getAcceleration() const { return state_.acceleration; }
  
  /**
   * @brief Check if currently moving
   */
  bool isMoving() const { return state_.is_moving; }
  
  // ==================== Trajectory Mode ====================
  
  /**
   * @brief Calculate complete S-curve profile for point-to-point motion
   * @param start_position Starting position
   * @param end_position Ending position
   * @param start_velocity Initial velocity (default 0)
   * @param end_velocity Final velocity (default 0)
   * @return Calculated profile
   */
  SCurveProfile calculateProfile(double start_position, double end_position,
                                  double start_velocity = 0.0, double end_velocity = 0.0);
  
  /**
   * @brief Get position at time t for a given profile
   * @param profile The S-curve profile
   * @param t Time from start
   * @return Position at time t
   */
  double getPositionAtTime(const SCurveProfile& profile, double t) const;
  
  /**
   * @brief Get velocity at time t for a given profile
   * @param profile The S-curve profile
   * @param t Time from start
   * @return Velocity at time t
   */
  double getVelocityAtTime(const SCurveProfile& profile, double t) const;
  
  /**
   * @brief Get acceleration at time t for a given profile
   * @param profile The S-curve profile
   * @param t Time from start
   * @return Acceleration at time t
   */
  double getAccelerationAtTime(const SCurveProfile& profile, double t) const;
  
  /**
   * @brief Generate trajectory points at regular intervals
   * @param profile The S-curve profile
   * @param dt Time step between points
   * @param positions Output: position at each time step
   * @param velocities Output: velocity at each time step
   * @param accelerations Output: acceleration at each time step
   */
  void generateTrajectory(const SCurveProfile& profile, double dt,
                          std::vector<double>& positions,
                          std::vector<double>& velocities,
                          std::vector<double>& accelerations) const;

private:
  // Motion constraints
  double max_velocity_;
  double max_acceleration_;
  double max_jerk_;
  
  // Real-time state
  SCurveState state_;
  
  // Position tracking gain for real-time mode
  double position_gain_;
  
  // Small value threshold
  static constexpr double EPSILON = 1e-9;
  static constexpr double VELOCITY_THRESHOLD = 1e-6;
  static constexpr double POSITION_THRESHOLD = 1e-7;
  
  /**
   * @brief Calculate time to reach zero velocity with max deceleration
   */
  double calculateStoppingTime(double velocity, double acceleration) const;
  
  /**
   * @brief Calculate stopping distance with S-curve deceleration
   */
  double calculateStoppingDistance(double velocity, double acceleration) const;
  
  /**
   * @brief Compute jerk-limited acceleration update
   */
  double computeJerkLimitedAcceleration(double current_acc, double desired_acc, double dt) const;
  
  /**
   * @brief Determine which segment we're in and compute jerk
   */
  double computeSegmentJerk(const SCurveProfile& profile, double t) const;
  
  /**
   * @brief Calculate profile for short distance (may not reach max velocity)
   */
  void calculateShortProfile(SCurveProfile& profile) const;
  
  /**
   * @brief Calculate profile for long distance (reaches max velocity)
   */
  void calculateLongProfile(SCurveProfile& profile) const;
  
  /**
   * @brief Clamp value to range
   */
  static double clamp(double value, double min_val, double max_val)
  {
    return std::max(min_val, std::min(max_val, value));
  }
  
  /**
   * @brief Sign function
   */
  static double sign(double value)
  {
    if (value > EPSILON) return 1.0;
    if (value < -EPSILON) return -1.0;
    return 0.0;
  }
};

}  // namespace rs_a3_hardware

#endif  // RS_A3_HARDWARE__S_CURVE_GENERATOR_HPP_
