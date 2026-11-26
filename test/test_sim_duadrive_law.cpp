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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "duatic_gazebo/sim_duadrive_law.hpp"

using duatic_gazebo::DriveCommand;
using duatic_gazebo::DriveLaw;
using duatic_gazebo::pd_torque;
using duatic_gazebo::sanitize;
using duatic_gazebo::select_law;

TEST(SelectLaw, FollowsClaimedInterfaces)
{
  EXPECT_EQ(select_law(true, true, true), DriveLaw::Pd);
  EXPECT_EQ(select_law(true, true, false), DriveLaw::Pd);
  EXPECT_EQ(select_law(false, false, false), DriveLaw::Pd);
  EXPECT_EQ(select_law(false, false, true), DriveLaw::Effort);
  EXPECT_EQ(select_law(false, true, false), DriveLaw::Velocity);
}

TEST(PdTorque, MatchesFormula)
{
  double integral = 0.0;
  // 100*(1.0-0.5) + 20*(0.2-0.1) + 3.0 = 55.0
  EXPECT_DOUBLE_EQ(
    pd_torque({1.0, 0.2, 3.0, 100.0, 0.0, 20.0}, {0.5, 0.1}, 0.002, 120.0, integral), 55.0);
  EXPECT_DOUBLE_EQ(integral, 0.0);  // i_gain 0 keeps the integral at zero
}

TEST(PdTorque, ClampsToEffortLimit)
{
  double integral = 0.0;
  EXPECT_DOUBLE_EQ(pd_torque({10.0, 0, 0, 1000.0, 0, 0}, {0, 0}, 0.002, 40.0, integral), 40.0);
  EXPECT_DOUBLE_EQ(pd_torque({-10.0, 0, 0, 1000.0, 0, 0}, {0, 0}, 0.002, 40.0, integral), -40.0);
}

TEST(PdTorque, IntegralAccumulatesAndIsClamped)
{
  double integral = 0.0;
  pd_torque({1.0, 0, 0, 0.0, 10.0, 0.0}, {0, 0}, 0.5, 40.0, integral);
  EXPECT_DOUBLE_EQ(integral, 0.5);
  for (int i = 0; i < 100; ++i) pd_torque({1.0, 0, 0, 0.0, 10.0, 0.0}, {0, 0}, 0.5, 40.0, integral);
  EXPECT_DOUBLE_EQ(integral, 4.0);  // 10 * 4.0 = effort limit
}

TEST(PdTorque, ZeroDtIsSafe)
{
  double integral = 0.0;
  EXPECT_TRUE(
    std::isfinite(pd_torque({1.0, 0, 0, 100.0, 10.0, 20.0}, {0, 0}, 0.0, 40.0, integral)));
  EXPECT_DOUBLE_EQ(integral, 0.0);
}

TEST(Sanitize, KeepsPreviousForBadValues)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const DriveCommand prev{0.1, 0.0, 0.0, 60.0, 0.0, 3.0};
  const DriveCommand out = sanitize({nan, 0.5, nan, -1.0, nan, 5.0}, prev);
  EXPECT_DOUBLE_EQ(out.position, 0.1);
  EXPECT_DOUBLE_EQ(out.velocity, 0.5);
  EXPECT_DOUBLE_EQ(out.effort, 0.0);
  EXPECT_DOUBLE_EQ(out.p_gain, 60.0);
  EXPECT_DOUBLE_EQ(out.i_gain, 0.0);
  EXPECT_DOUBLE_EQ(out.d_gain, 5.0);
}

TEST(PdTorque, NonFiniteInputsGiveZeroAndKeepIntegral)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  double integral = 0.5;
  const DriveCommand cmd{1.0, 0.0, 0.0, 100.0, 10.0, 0.0};
  EXPECT_DOUBLE_EQ(pd_torque(cmd, {nan, 0.0}, 0.002, 40.0, integral), 0.0);
  EXPECT_DOUBLE_EQ(pd_torque(cmd, {0.0, nan}, 0.002, 40.0, integral), 0.0);
  EXPECT_DOUBLE_EQ(pd_torque(cmd, {0.0, 0.0}, 0.002, nan, integral), 0.0);
  EXPECT_DOUBLE_EQ(integral, 0.5);
  // The integral stays usable once the state recovers.
  EXPECT_TRUE(std::isfinite(pd_torque(cmd, {0.0, 0.0}, 0.002, 40.0, integral)));
  EXPECT_TRUE(std::isfinite(integral));
}
