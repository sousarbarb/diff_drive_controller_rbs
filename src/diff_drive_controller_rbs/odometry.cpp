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

#include "diff_drive_controller_rbs/odometry.h"

#include <boost/bind.hpp>

namespace diff_drive_controller_rbs {

namespace bacc = boost::accumulators;

Odometry::Odometry(size_t vel_rolling_window_size)
    : timestamp_(0.0) ,
      x_(0.0) , y_(0.0) , th_(0.0) , vel_lin_(0.0) , vel_ang_(0.0) ,
      wh_dist_(0.0) , wh_l_radius_(0.0) , wh_r_radius_(0.0) ,
      wh_l_pos_old_(0.0) , wh_r_pos_old_(0.0) ,
      vel_rolling_window_size_(vel_rolling_window_size) ,
      vel_lin_rolling_window_(RollingWindow::window_size =
          vel_rolling_window_size) ,
      vel_ang_rolling_window_(RollingWindow::window_size =
          vel_rolling_window_size) ,
      integrate_fun_(boost::bind(&Odometry::IntegrateExact, this, _1, _2)) { }

void Odometry::Init(const ros::Time& time) {
  ResetAccumulators();
  timestamp_ = time;
}

bool Odometry::Update(double wh_l_pos, double wh_r_pos, const ros::Time &time) {
  /// Get current wheel joint positions:
  const double wh_l_pos_cur = wh_l_pos * wh_l_radius_;
  const double wh_r_pos_cur = wh_r_pos * wh_r_radius_;

  /// Estimate velocity of wheels using old and current position:
  const double wh_l_vel_est = wh_l_pos_cur - wh_l_pos_old_;
  const double wh_r_vel_est = wh_r_pos_cur - wh_r_pos_old_;

  /// Update old position with current:
  wh_l_pos_old_ = wh_l_pos_cur;
  wh_r_pos_old_ = wh_r_pos_cur;

  /// Compute linear and angular diff:
  const double linear  = (wh_r_vel_est + wh_l_vel_est) * 0.5 ;
  const double angular = (wh_r_vel_est - wh_l_vel_est) / wh_dist_;

  /// Integrate odometry:
  integrate_fun_(linear, angular);

  /// We cannot estimate the speed with very small time intervals:
  const double dt = (time - timestamp_).toSec();
  if (dt < 0.0001)
    return false; // Interval too small to integrate with

  timestamp_ = time;

  /// Estimate speeds using a rolling mean to filter them out:
  vel_lin_rolling_window_(linear/dt);
  vel_ang_rolling_window_(angular/dt);

  vel_lin_ = bacc::rolling_mean(vel_lin_rolling_window_);
  vel_ang_ = bacc::rolling_mean(vel_ang_rolling_window_);

  return true;
}

void Odometry::UpdateOpenLoop(
    double vel_lin, double vel_ang, const ros::Time &time) {
  /// Save last linear and angular velocity:
  vel_lin_ = vel_lin;
  vel_ang_ = vel_ang;

  /// Integrate odometry:
  const double dt = (time - timestamp_).toSec();
  timestamp_ = time;
  integrate_fun_(vel_lin * dt, vel_ang * dt);
}

void Odometry::SetWheelParams(
    double wh_dist, double wh_l_radius, double wh_r_radius) {
  wh_dist_     = wh_dist;
  wh_l_radius_ = wh_l_radius;
  wh_r_radius_ = wh_r_radius;
}

void Odometry::SetVelocityRollingWindowSize(size_t vel_rolling_window_size) {
  vel_rolling_window_size_ = vel_rolling_window_size;
  ResetAccumulators();
}

void Odometry::IntegrateRungeKutta2(double vel_lin, double vel_ang)
{
  const double th = th_ + vel_ang * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_  += vel_lin * cos(th);
  y_  += vel_lin * sin(th);
  th_ += vel_ang;
}

void Odometry::IntegrateExact(double vel_lin, double vel_ang) {
  if (fabs(vel_ang) < 1e-6)
    IntegrateRungeKutta2(vel_lin, vel_ang);
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double th_old = th_;
    const double r = vel_lin/vel_ang;
    th_ += vel_ang;
    x_  +=  r * (sin(th_) - sin(th_old));
    y_  += -r * (cos(th_) - cos(th_old));
  }
}

void Odometry::ResetAccumulators()
{
  vel_lin_rolling_window_ = RollingMeanAcc(
      RollingWindow::window_size = vel_rolling_window_size_);
  vel_ang_rolling_window_ = RollingMeanAcc(
      RollingWindow::window_size = vel_rolling_window_size_);
}

} // namespace diff_drive_controller_rbs
