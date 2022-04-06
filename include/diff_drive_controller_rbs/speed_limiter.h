#pragma once

namespace diff_drive_controller_rbs {

/**
 * \brief Speed limiter for the robot's motion
 *
 * The class implements the following limiters:
 * - velocity (m.s⁻¹)
 * - acceleration (m.s⁻²)
 * - jerk (m.s⁻³)
 *
 * \see http://en.wikipedia.org/wiki/Jerk_%28physics%29#Motion_control
 */
class SpeedLimiter {
 public:
  bool has_vel_lim;   //!< enable velocity limit
  bool has_acc_lim;   //!< enable acceleration limit
  bool has_jerk_lim;  //!< enable jerk limit
  double min_vel; //!< minimum velocity limit
  double max_vel; //!< maximum velocity limit
  double min_acc; //!< minimum acceleration limit
  double max_acc; //!< maximum acceleration limit
  double min_jerk;  //!< minimum jerk limit
  double max_jerk;  //!< maximum jerk limit

 public:
  /**
   * \brief Constructor
   * \param [in] has_vel_lim  if true, applies velocity limits
   * \param [in] has_acc_lim  if true, applies acceleration limits
   * \param [in] has_jerk_lim if true, applies jerk limits
   * \param [in] min_vel      minimum velocity [m/s] (usually <= 0)
   * \param [in] max_vel      maximum velocity [m/s] (usually >= 0)
   * \param [in] min_acc      minimum acceleration [m/s^2] (usually <= 0)
   * \param [in] max_acc      maximum acceleration [m/s^2] (usually >= 0)
   * \param [in] min_jerk     minimum jerk [m/s^3] (usually <= 0)
   * \param [in] max_jerk     maximum jerk [m/s^3] (usually >= 0)
   */
  explicit SpeedLimiter(
      bool has_vel_lim=false, bool has_acc_lim=false, bool has_jerk_lim=false,
      double min_vel=0.0, double max_vel=0.0,
      double min_acc=0.0, double max_acc=0.0,
      double min_jerk=0.0, double max_jerk=0.0);

  /**
   * \brief Limit the velocity, acceleration, and jerk
   * \param [in,out] v  velocity [m/s]
   * \param [in]     v0 previous velocity to v  [m/s]
   * \param [in]     v1 previous velocity to v0 [m/s]
   * \param [in]     dt time step [s]
   * \return            limiting factor (1.0 if none)
   */
  double Limit(double& v, double v0, double v1, double dt) const;

  /**
   * \brief Limit the velocity
   * \param [in,out] v velocity [m/s]
   * \return           limiting factor (1.0 if none)
   */
  double LimitVelocity(double& v) const;

  /**
   * \brief Limit the acceleration
   * \param [in,out] v  Velocity [m/s]
   * \param [in]     v0 previous velocity to v  [m/s]
   * \param [in]     dt time step [s]
   * \return            limiting factor (1.0 if none)
   */
  double LimitAcceleration(double& v, double v0, double dt) const;

  /**
   * \brief Limit the jerk
   * \param [in,out] v  velocity [m/s]
   * \param [in]     v0 previous velocity to v  [m/s]
   * \param [in]     v1 previous velocity to v0 [m/s]
   * \param [in]     dt time step [s]
   * \return            limiting factor (1.0 if none)
   */
  double LimitJerk(double& v, double v0, double v1, double dt) const;
};

} // namespace diff_drive_controller_rbs
