// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Derived from gz_ros2_control 1.2.20, include/gz_ros2_control/gz_system.hpp

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace duatic_gazebo
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SimDuaDriveSystemPrivate;

// GazeboSimSystem whose joints with a sim_p_gain param run a DuaDrive PD loop on every physics step.
class SimDuaDriveSystem : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  SimDuaDriveSystem();
  ~SimDuaDriveSystem() override;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool initSim(
    rclcpp::Node::SharedPtr & model_nh, std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info, sim::EntityComponentManager & _ecm,
    unsigned int update_rate) override;

private:
  void registerSensors(const hardware_interface::HardwareInfo & hardware_info);

  std::unique_ptr<SimDuaDriveSystemPrivate> dataPtr;
};

}  // namespace duatic_gazebo
