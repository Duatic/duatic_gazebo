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

#pragma once

namespace duatic_gazebo
{

struct DriveCommand
{
  double position;
  double velocity;
  double effort;
  double p_gain;
  double i_gain;
  double d_gain;
};

struct DriveState
{
  double position;
  double velocity;
};

enum class DriveLaw
{
  Pd,
  Effort,
  Velocity
};

// Position claimed or nothing claimed gives Pd, effort only gives Effort, velocity without position gives Velocity.
DriveLaw select_law(bool position, bool velocity, bool effort);

// Replaces non-finite targets and non-finite or negative gains in `incoming` by the value in `previous`.
DriveCommand sanitize(const DriveCommand & incoming, const DriveCommand & previous);

// Returns 0 and leaves `integral` unchanged for a non-finite state, position target or limit.
// PID torque clamped to +-effort_limit; `integral` advances only for dt > 0
// and is bounded so |i_gain * integral| <= effort_limit.
double pd_torque(
  const DriveCommand & cmd, const DriveState & state, double dt, double effort_limit,
  double & integral);

}  // namespace duatic_gazebo
