/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

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
