#pragma once

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>
#include <ros/time.h>

namespace diff_drive_controller_rbs {

namespace bacc = boost::accumulators;

/**
 * \brief The Odometry class handles odometry readings
 *        (2D pose and velocity with related timestamp) for a differential
 *        drive robot
 */
class Odometry {
 public:
  typedef boost::function<void(double, double)> IntegrationFunction;
      //!< Integration function used to integrate the odometry

 private:
  typedef bacc::accumulator_set
      <double, bacc::stats<bacc::tag::rolling_mean>> RollingMeanAcc;
      //!< Rolling mean accumulator type
  typedef bacc::tag::rolling_window RollingWindow;  //!< Rolling window type

  ros::Time timestamp_; //!< current timestamp (ros::Time)

  double x_;  //!< current pose: x direction in odometry frame [m]
  double y_;  //!< current pose: y direction in odometry frame [m]
  double th_; //!< current pose: orientation in odometry frame [rad]

  double vel_lin_;  //!< current linear velocity [m.s⁻¹]
  double vel_ang_;  //!< current angular velocity [rad.s⁻¹]

  double wh_dist_;      //!< distance between wheels [m]
  double wh_l_radius_;  //!< left wheel radius [m]
  double wh_r_radius_;  //!< right wheel radius [m]

  double wh_l_pos_old_; //!< previous position of left wheel [rad]
  double wh_r_pos_old_; //!< previous position of right wheel [rad]

  size_t vel_rolling_window_size_;  //!< size of the velocity rolling window
  RollingMeanAcc vel_lin_rolling_window_; //!< linear velocity rolling window
  RollingMeanAcc vel_ang_rolling_window_; //!< angular velocity rolling window

  IntegrationFunction integrate_fun_; //!< integration function used to
                                      //!< integrate odometry

 public:
  /**
   * \brief Constructor: Timestamp will get the current time value.
   *        Value will be set to zero.
   * \param vel_rolling_window_size rolling window size used to compute the
   *                                velocity mean
   */
  explicit Odometry(size_t vel_rolling_window_size = 10);

  /**
   * \brief Initialize the odometry
   * \param[in] time current time
   */
  void Init(const ros::Time &time);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param[in] wh_l_pos left wheel position [rad]
   * \param[in] wh_r_pos right wheel position [rad]
   * \param[in] time     current time
   * \return             true if the odometry is actually updated
   */
  bool Update(double wh_l_pos, double wh_r_pos, const ros::Time &time);

  /**
   * \brief Updates the odometry class with latest velocity command
   * \param vel_lin linear velocity [m.s⁻¹]
   * \param vel_ang angular velocity [rad.s⁻¹]
   * \param time    current time
   */
  void UpdateOpenLoop(double vel_lin, double vel_ang, const ros::Time &time);

  /**
   * \brief Heading getter
   * \return heading [rad]
   */
  double GetHeading() const {
    return th_;
  }

  /**
   * \brief x position getter
   * \return x position [m]
   */
  double GetX() const {
    return x_;
  }

  /**
   * \brief y position getter
   * \return y position [m]
   */
  double GetY() const {
    return y_;
  }

  /**
   * \brief Linear velocity getter
   * \return linear velocity [m/s]
   */
  double GetLinear() const {
    return vel_lin_;
  }

  /**
   * \brief Angular velocity getter
   * \return angular velocity [rad/s]
   */
  double GetAngular() const {
    return vel_ang_;
  }

  /**
   * \brief Sets the wheel parameters: radius and separation
   * \param wh_dist     separation between left and right wheels [m]
   * \param wh_l_radius left wheel radius [m]
   * \param wh_r_radius right wheel radius [m]
   */
  void SetWheelParams(double wh_dist, double wh_l_radius, double wh_r_radius);

  /**
   * \brief Velocity rolling window size setter
   * \param vel_rolling_window_size velocity rolling window size
   */
  void SetVelocityRollingWindowSize(size_t vel_rolling_window_size);

 private:
  /**
   * \brief Integrates the velocities (linear and angular) using 2nd order
   *        Runge-Kutta
   * \param vel_lin linear velocity [m] (linear displacement, m/s * dt)
   *                computed by encoders
   * \param vel_ang angular velocity [rad] (angular displacement, rad/s * dt)
   *                computed by encoders
   */
  void IntegrateRungeKutta2(double vel_lin, double vel_ang);

  /**
   * \brief Integrates the velocities (linear and angular) using exact method
   * \param vel_lin linear velocity [m] (linear displacement, m/s * dt)
   *                computed by encoders
   * \param vel_ang angular velocity [rad] (angular displacement, rad/s * dt)
   *                computed by encoders
   */
  void IntegrateExact(double vel_lin, double vel_ang);

  /**
   * \brief Reset linear and angular accumulators
   */
  void ResetAccumulators();
};

} // namespace diff_drive_controller_rbs
