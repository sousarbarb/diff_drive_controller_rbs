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

#include "diff_drive_controller_rbs/speed_limiter.h"

#include <algorithm>

template<typename T>
T clamp(T x, T min, T max) {
  return std::min(std::max(min, x), max);
}

namespace diff_drive_controller_rbs {

SpeedLimiter::SpeedLimiter(
    bool has_vel_lim, bool has_acc_lim, bool has_jerk_lim,
    double min_vel, double max_vel,
    double min_acc, double max_acc,
    double min_jerk, double max_jerk)
    : has_vel_lim(has_vel_lim) , has_acc_lim(has_acc_lim) ,
      has_jerk_lim(has_jerk_lim) ,
      min_vel(min_vel) , max_vel(max_vel) ,
      min_acc(min_acc) , max_acc(max_acc) ,
      min_jerk(min_jerk) , max_jerk(max_jerk) { }

double SpeedLimiter::Limit(double& v, double v0, double v1, double dt) const {
  const double tmp = v;

  LimitJerk(v, v0, v1, dt);
  LimitAcceleration(v, v0, dt);
  LimitVelocity(v);

  return tmp != 0.0 ? v / tmp : 1.0;
}

double SpeedLimiter::LimitVelocity(double& v) const {
  const double tmp = v;

  if (has_vel_lim) {
    v = clamp(v, min_vel, max_vel);
  }

  return tmp != 0.0 ? v / tmp : 1.0;
}

double SpeedLimiter::LimitAcceleration(double& v, double v0, double dt) const {
  const double tmp = v;

  if (has_acc_lim) {
    const double dv_min = min_acc * dt;
    const double dv_max = max_acc * dt;

    const double dv = clamp(v - v0, dv_min, dv_max);

    v = v0 + dv;
  }

  return tmp != 0.0 ? v / tmp : 1.0;
}

double SpeedLimiter::LimitJerk(
    double& v, double v0, double v1, double dt) const {
  const double tmp = v;

  if (has_jerk_lim) {
    const double dv  = v  - v0;
    const double dv0 = v0 - v1;

    const double dt2 = 2. * dt * dt;

    const double da_min = min_jerk * dt2;
    const double da_max = max_jerk * dt2;

    const double da = clamp(dv - dv0, da_min, da_max);

    v = v0 + dv0 + da;
  }

  return tmp != 0.0 ? v / tmp : 1.0;
}

} // namespace diff_drive_controller_rbs
