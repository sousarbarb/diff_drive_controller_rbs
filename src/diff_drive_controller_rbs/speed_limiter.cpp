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
