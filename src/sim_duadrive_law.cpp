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
