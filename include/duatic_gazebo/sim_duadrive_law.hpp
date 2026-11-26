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

// PID torque clamped to +-effort_limit; `integral` only advances for dt > 0 and is bounded so |i_gain * integral| <= effort_limit.
double pd_torque(
  const DriveCommand & cmd, const DriveState & state, double dt, double effort_limit,
  double & integral);

}  // namespace duatic_gazebo
