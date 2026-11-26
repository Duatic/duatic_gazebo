/*
 * Copyright 2026 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "duatic_gazebo/sim_duadrive_law.hpp"

#include <algorithm>
#include <cmath>

namespace duatic_gazebo
{

DriveLaw select_law(bool position, bool velocity, bool effort)
{
  if (position) return DriveLaw::Pd;
  if (velocity) return DriveLaw::Velocity;
  if (effort) return DriveLaw::Effort;
  return DriveLaw::Pd;
}

DriveCommand sanitize(const DriveCommand & incoming, const DriveCommand & previous)
{
  const auto target = [](double in, double prev) { return std::isfinite(in) ? in : prev; };
  const auto gain = [](double in, double prev) { return std::isfinite(in) && in >= 0.0 ? in : prev; };
  return {
    target(incoming.position, previous.position), target(incoming.velocity, previous.velocity),
    target(incoming.effort, previous.effort),     gain(incoming.p_gain, previous.p_gain),
    gain(incoming.i_gain, previous.i_gain),       gain(incoming.d_gain, previous.d_gain)};
}

double pd_torque(
  const DriveCommand & cmd, const DriveState & state, double dt, double effort_limit,
  double & integral)
{
  const double error = cmd.position - state.position;
  // A diverged state or limit must not reach the output or poison the integral.
  if (!std::isfinite(error) || !std::isfinite(state.velocity) || !std::isfinite(effort_limit)) {
    return 0.0;
  }
  if (cmd.i_gain > 0.0) {
    if (dt > 0.0) integral += error * dt;
    const double bound = effort_limit / cmd.i_gain;
    integral = std::clamp(integral, -bound, bound);
  } else {
    integral = 0.0;
  }
  const double tau = cmd.p_gain * error + cmd.d_gain * (cmd.velocity - state.velocity) +
                     cmd.i_gain * integral + cmd.effort;
  return std::clamp(tau, -effort_limit, effort_limit);
}

}  // namespace duatic_gazebo
