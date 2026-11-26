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

// Derived from gz_ros2_control 1.2.20, src/gz_system.cpp

#include "duatic_gazebo/sim_duadrive_system.hpp"

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/wrench.pb.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/physics/Geometry.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/ForceTorque.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "duatic_gazebo/sim_duadrive_law.hpp"

#ifdef ERROR
#undef ERROR
#endif

struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Joint's type.
  sdf::JointType joint_type;

  /// \brief Joint's axis.
  sdf::JointAxis joint_axis;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint position
  double joint_position_cmd;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd;

  /// \brief Current cmd joint effort
  double joint_effort_cmd;

  /// \brief flag if joint is actuated (has command interfaces) or passive
  bool is_actuated;

  /// \brief handles to the joints from within Gazebo
  sim::Entity sim_joint;

  /// \brief Control method defined in the URDF for each joint.
  gz_ros2_control::GazeboSimSystemInterface::ControlMethod joint_control_method;

  /// \brief Precomputed interface names ("joint_name/position" etc.) to avoid
  /// per-call string allocations in perform_command_mode_switch.
  std::string if_name_position;
  std::string if_name_velocity;
  std::string if_name_effort;

  /// \brief Joint carries a sim_p_gain param and is driven by the simulated DuaDrive.
  bool is_drive = false;

  /// \brief Gain command interfaces of a drive joint.
  double p_gain_cmd = 0.0;
  double i_gain_cmd = 0.0;
  double d_gain_cmd = 0.0;

  /// \brief Last command that passed sanitize, used in place of rejected values.
  duatic_gazebo::DriveCommand accepted_cmd{};

  /// \brief Torque limit of a drive joint, from the URDF joint limits.
  double effort_limit = 0.0;

  /// \brief Integral of the position error of a drive joint.
  double integral = 0.0;

  /// \brief Claimed command interfaces of a drive joint as position, velocity, effort flags.
  std::array<bool, 3> claimed{};

  /// \brief Law a drive joint applies, chosen from its claimed command interfaces.
  duatic_gazebo::DriveLaw law = duatic_gazebo::DriveLaw::Pd;
};

class ForceTorqueData
{
public:
  /// \brief force torque sensor's name.
  std::string name{};

  /// \brief force torque sensor's topic name.
  std::string topicName{};

  /// \brief handles to the force torque from within Gazebo
  sim::Entity sim_ft_sensors_ = sim::kNullEntity;

  /// \brief An array per FT
  std::array<double, 6> ft_sensor_data_;

  /// \brief callback to get the Force Torque topic values
  void OnForceTorque(const gz::msgs::Wrench & _msg);
};

void ForceTorqueData::OnForceTorque(const gz::msgs::Wrench & _msg)
{
  this->ft_sensor_data_[0] = _msg.force().x();
  this->ft_sensor_data_[1] = _msg.force().y();
  this->ft_sensor_data_[2] = _msg.force().z();
  this->ft_sensor_data_[3] = _msg.torque().x();
  this->ft_sensor_data_[4] = _msg.torque().y();
  this->ft_sensor_data_[5] = _msg.torque().z();
}

class ImuData
{
public:
  /// \brief imu's name.
  std::string name{};

  /// \brief imu's topic name.
  std::string topicName{};

  /// \brief handles to the imu from within Gazebo
  sim::Entity sim_imu_sensors_ = sim::kNullEntity;

  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
  std::array<double, 10> imu_sensor_data_;

  /// \brief callback to get the IMU topic values
  void OnIMU(const gz::msgs::IMU & _msg);
};

void ImuData::OnIMU(const gz::msgs::IMU & _msg)
{
  this->imu_sensor_data_[0] = _msg.orientation().x();
  this->imu_sensor_data_[1] = _msg.orientation().y();
  this->imu_sensor_data_[2] = _msg.orientation().z();
  this->imu_sensor_data_[3] = _msg.orientation().w();
  this->imu_sensor_data_[4] = _msg.angular_velocity().x();
  this->imu_sensor_data_[5] = _msg.angular_velocity().y();
  this->imu_sensor_data_[6] = _msg.angular_velocity().z();
  this->imu_sensor_data_[7] = _msg.linear_acceleration().x();
  this->imu_sensor_data_[8] = _msg.linear_acceleration().y();
  this->imu_sensor_data_[9] = _msg.linear_acceleration().z();
}

class duatic_gazebo::SimDuaDriveSystemPrivate
{
public:
  SimDuaDriveSystemPrivate() = default;

  ~SimDuaDriveSystemPrivate() = default;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief write() has run before, so last_update_sim_time_ros_ holds its time.
  bool has_last_write_ = false;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  /// \brief vector with the imus.
  std::vector<std::shared_ptr<ImuData>> imus_;

  /// \brief vector with the force torque sensors.
  std::vector<std::shared_ptr<ForceTorqueData>> ft_sensors_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  sim::EntityComponentManager * ecm;

  /// \brief controller update rate
  unsigned int update_rate;

  /// \brief Gazebo communication node.
  gz::transport::Node node;

  /// \brief Gain which converts position error to a velocity command
  double position_proportional_gain_;

  // Should hold the joints if no control_mode is active
  bool hold_joints_ = true;
};

namespace duatic_gazebo
{

SimDuaDriveSystem::SimDuaDriveSystem() = default;

SimDuaDriveSystem::~SimDuaDriveSystem() = default;

bool SimDuaDriveSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, sim::Entity> & enableJoints,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & _ecm,
  unsigned int update_rate)
{
  this->dataPtr = std::make_unique<SimDuaDriveSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->dataPtr->ecm = &_ecm;

  this->dataPtr->update_rate = update_rate;

  try {
    this->dataPtr->hold_joints_ = this->nh_->get_parameter("hold_joints").as_bool();
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'hold_joints' not initialized, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(), "Using default value: " << this->dataPtr->hold_joints_);
  } catch (rclcpp::exceptions::ParameterNotDeclaredException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'hold_joints' not declared, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(), "Using default value: " << this->dataPtr->hold_joints_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'hold_joints' has wrong type: %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(), "Using default value: " << this->dataPtr->hold_joints_);
  }
  RCLCPP_DEBUG_STREAM(
    this->nh_->get_logger(), "hold_joints (system): " << this->dataPtr->hold_joints_ << std::endl);


  this->dataPtr->joints_.resize(hardware_info.joints.size());

  try {
    this->dataPtr->position_proportional_gain_ =
      this->nh_->get_parameter("position_proportional_gain").as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'position_proportional_gain' not initialized, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(),
      "Using default value: " << this->dataPtr->position_proportional_gain_);
  } catch (rclcpp::exceptions::ParameterNotDeclaredException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'position_proportional_gain' not declared, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(),
      "Using default value: " << this->dataPtr->position_proportional_gain_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'position_proportional_gain' has wrong type: %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(),
      "Using default value: " << this->dataPtr->position_proportional_gain_);
  }

  RCLCPP_INFO_STREAM(
    this->nh_->get_logger(),
    "The position_proportional_gain has been set to: " <<
      this->dataPtr->position_proportional_gain_);

  if (this->dataPtr->joints_.empty()) {
    RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "There is no joint available");
    return false;
  }

  for (unsigned int j = 0; j < this->dataPtr->joints_.size(); j++) {
    auto & joint_info = hardware_info.joints[j];
    std::string joint_name = this->dataPtr->joints_[j].name = joint_info.name;

    auto it_joint = enableJoints.find(joint_name);
    if (it_joint == enableJoints.end()) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the gazebo model.");
      continue;
    }

    sim::Entity simjoint = it_joint->second;
    this->dataPtr->joints_[j].sim_joint = simjoint;
    this->dataPtr->joints_[j].joint_type = _ecm.Component<sim::components::JointType>(
      simjoint)->Data();
    this->dataPtr->joints_[j].joint_axis = _ecm.Component<sim::components::JointAxis>(
      simjoint)->Data();

    // Create joint position component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointPosition().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointPosition());
    }

    // Create joint velocity component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointVelocity().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointVelocity());
    }

    // Create joint transmitted wrench component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointTransmittedWrench().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointTransmittedWrench());
    }

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);

    // Precompute interface names once to avoid allocations in perform_command_mode_switch
    this->dataPtr->joints_[j].if_name_position =
      joint_name + "/" + hardware_interface::HW_IF_POSITION;
    this->dataPtr->joints_[j].if_name_velocity =
      joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
    this->dataPtr->joints_[j].if_name_effort =
      joint_name + "/" + hardware_interface::HW_IF_EFFORT;

    // check if joint is mimicked
    auto it = std::find_if(
      hardware_info.mimic_joints.begin(),
      hardware_info.mimic_joints.end(),
      [j](const hardware_interface::MimicJoint & mj) {
        return mj.joint_index == j;
      });

    if (it != hardware_info.mimic_joints.end()) {
      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(),
        "Joint '" << joint_name << "'is mimicking joint '" <<
          this->dataPtr->joints_[it->mimicked_joint_index].name <<
          "' with multiplier: " << it->multiplier << " and offset: " << it->offset);
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

    auto get_initial_value =
      [this, joint_name](const hardware_interface::InterfaceInfo & interface_info) {
        double initial_value{0.0};
        if (!interface_info.initial_value.empty()) {
          try {
            initial_value = hardware_interface::stod(interface_info.initial_value);
            RCLCPP_INFO(this->nh_->get_logger(), "\t\t\t found initial value: %f", initial_value);
          } catch (std::invalid_argument &) {
            RCLCPP_ERROR_STREAM(
              this->nh_->get_logger(),
              "Failed converting initial_value string to real number for the joint "
                << joint_name
                << " and state interface " << interface_info.name
                << ". Actual value of parameter: " << interface_info.initial_value
                << ". Initial value will be set to 0.0");
            throw std::invalid_argument("Failed converting initial_value string");
          }
        }
        return initial_value;
      };

    double initial_position = std::numeric_limits<double>::quiet_NaN();
    double initial_velocity = std::numeric_limits<double>::quiet_NaN();
    double initial_effort = std::numeric_limits<double>::quiet_NaN();

    // register the state handles
    for (unsigned int i = 0; i < joint_info.state_interfaces.size(); ++i) {
      if (joint_info.state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position);
        initial_position = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_position = initial_position;
      }
      if (joint_info.state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity);
        initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
      }
      if (joint_info.state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joints_[j].joint_effort);
        initial_effort = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_effort = initial_effort;
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < joint_info.command_interfaces.size(); ++i) {
      if (joint_info.command_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position_cmd);
        if (!std::isnan(initial_position)) {
          this->dataPtr->joints_[j].joint_position_cmd = initial_position;
        }
      } else if (joint_info.command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity_cmd);
        if (!std::isnan(initial_velocity)) {
          this->dataPtr->joints_[j].joint_velocity_cmd = initial_velocity;
        }
      } else if (joint_info.command_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joints_[j].joint_effort_cmd);
        if (!std::isnan(initial_effort)) {
          this->dataPtr->joints_[j].joint_effort_cmd = initial_effort;
        }
      }
      // independently of existence of command interface set initial value if defined
      if (!std::isnan(initial_position)) {
        this->dataPtr->joints_[j].joint_position = initial_position;
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[j].sim_joint,
          sim::components::JointPositionReset({initial_position}));
      }
      if (!std::isnan(initial_velocity)) {
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[j].sim_joint,
          sim::components::JointVelocityReset({initial_velocity}));
      }
    }

    // check if joint is actuated (has command interfaces) or passive
    this->dataPtr->joints_[j].is_actuated = (joint_info.command_interfaces.size() > 0);

    // Gain values and limits of a drive joint are read and checked in on_init.
    if (joint_info.parameters.count("sim_p_gain") > 0) {
      auto & joint = this->dataPtr->joints_[j];
      joint.is_drive = true;
      joint.joint_position_cmd = std::isnan(initial_position) ? 0.0 : initial_position;
      joint.joint_velocity_cmd = 0.0;
      joint.joint_effort_cmd = 0.0;
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t p_gain, i_gain, d_gain (sim drive)");
      this->dataPtr->command_interfaces_.emplace_back(joint_name, "p_gain", &joint.p_gain_cmd);
      this->dataPtr->command_interfaces_.emplace_back(joint_name, "i_gain", &joint.i_gain_cmd);
      this->dataPtr->command_interfaces_.emplace_back(joint_name, "d_gain", &joint.d_gain_cmd);
    }
  }

  registerSensors(hardware_info);

  return true;
}

void SimDuaDriveSystem::registerSensors(
  const hardware_interface::HardwareInfo & hardware_info)
{
  // Reference sensor components directly from hardware_info, no copy needed.
  const auto & sensor_components_ = hardware_info.sensors;

  // Build a name→ComponentInfo lookup map so each ECM Each<> callback is O(1)
  // instead of doing a linear scan over all sensor_components_ entries.
  std::unordered_map<std::string, const hardware_interface::ComponentInfo *> sensor_map;
  sensor_map.reserve(sensor_components_.size());
  for (const auto & comp : sensor_components_) {
    sensor_map.emplace(comp.name, &comp);
  }

  // This is split in two steps: Count the number and type of sensor and associate the interfaces
  // So we have resize only once the structures where the data will be stored, and we can safely
  // use pointers to the structures

  this->dataPtr->ecm->Each<sim::components::Imu,
    sim::components::Name>(
    [&](const sim::Entity & _entity,
    const sim::components::Imu *,
    const sim::components::Name * _name) -> bool
    {
      auto imuData = std::make_shared<ImuData>();
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << _name->Data());

      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(_entity);
      if (sensorTopicComp) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
      }

      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(), "\tState:");
      imuData->name = _name->Data();
      imuData->sim_imu_sensors_ = _entity;

      auto sensor_it = sensor_map.find(_name->Data());
      if (sensor_it == sensor_map.end()) {
        RCLCPP_WARN_STREAM(
          this->nh_->get_logger(),
          "IMU sensor '" << _name->Data() << "' not found in hardware_info, skipping.");
        return true;
      }
      const hardware_interface::ComponentInfo & component = *sensor_it->second;

      static const std::map<std::string, size_t> interface_name_map = {
        {"orientation.x", 0},
        {"orientation.y", 1},
        {"orientation.z", 2},
        {"orientation.w", 3},
        {"angular_velocity.x", 4},
        {"angular_velocity.y", 5},
        {"angular_velocity.z", 6},
        {"linear_acceleration.x", 7},
        {"linear_acceleration.y", 8},
        {"linear_acceleration.z", 9},
      };

      for (const auto & state_interface : component.state_interfaces) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

        size_t data_index = interface_name_map.at(state_interface.name);
        this->dataPtr->state_interfaces_.emplace_back(
          imuData->name,
          state_interface.name,
          &imuData->imu_sensor_data_[data_index]);
      }
      this->dataPtr->imus_.push_back(imuData);
      return true;
    });

  this->dataPtr->ecm->Each<sim::components::ForceTorque,
    sim::components::Name>(
    [&](const sim::Entity & _entity,
    const sim::components::ForceTorque *,
    const sim::components::Name * _name) -> bool
    {
      auto ftData = std::make_shared<ForceTorqueData>();
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << _name->Data());

      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(_entity);
      if (sensorTopicComp) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
      }

      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(), "\tState:");
      ftData->name = _name->Data();
      ftData->sim_ft_sensors_ = _entity;

      auto sensor_it = sensor_map.find(_name->Data());
      if (sensor_it == sensor_map.end()) {
        RCLCPP_WARN_STREAM(
          this->nh_->get_logger(),
          "ForceTorque sensor '" << _name->Data() << "' not found in hardware_info, skipping.");
        return true;
      }
      const hardware_interface::ComponentInfo & component = *sensor_it->second;

      static const std::map<std::string, size_t> interface_name_map = {
        {"force.x", 0},
        {"force.y", 1},
        {"force.z", 2},
        {"torque.x", 3},
        {"torque.y", 4},
        {"torque.z", 5},
      };

      for (const auto & state_interface : component.state_interfaces) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

        size_t data_index = interface_name_map.at(state_interface.name);
        this->dataPtr->state_interfaces_.emplace_back(
          ftData->name,
          state_interface.name,
          &ftData->ft_sensor_data_[data_index]);
      }
      this->dataPtr->ft_sensors_.push_back(ftData);
      return true;
    });
}

CallbackReturn
SimDuaDriveSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & info = params.hardware_info;

  for (unsigned int j = 0; j < this->dataPtr->joints_.size(); j++) {
    auto & joint = this->dataPtr->joints_[j];
    if (!joint.is_drive) {
      continue;
    }
    const auto & joint_params = info.joints[j].parameters;
    double * gains[] = {&joint.p_gain_cmd, &joint.i_gain_cmd, &joint.d_gain_cmd};
    const char * names[] = {"sim_p_gain", "sim_i_gain", "sim_d_gain"};
    for (int g = 0; g < 3; ++g) {
      const auto it = joint_params.find(names[g]);
      if (it == joint_params.end()) {
        RCLCPP_ERROR(
          this->nh_->get_logger(), "Drive joint '%s' has no '%s' param", joint.name.c_str(),
          names[g]);
        return CallbackReturn::ERROR;
      }
      try {
        *gains[g] = hardware_interface::stod(it->second);
      } catch (const std::invalid_argument &) {
        *gains[g] = std::numeric_limits<double>::quiet_NaN();
      }
      if (!std::isfinite(*gains[g]) || *gains[g] < 0.0) {
        RCLCPP_ERROR(
          this->nh_->get_logger(), "Drive joint '%s': '%s' must be a non-negative number, got '%s'",
          joint.name.c_str(), names[g], it->second.c_str());
        return CallbackReturn::ERROR;
      }
    }

    const auto limits = info_.limits.find(joint.name);
    if (
      limits == info_.limits.end() || !limits->second.has_effort_limits ||
      !std::isfinite(limits->second.max_effort) || limits->second.max_effort <= 0.0)
    {
      RCLCPP_ERROR(
        this->nh_->get_logger(), "Drive joint '%s' has no positive effort limit in the URDF",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    joint.effort_limit = limits->second.max_effort;
    joint.accepted_cmd = {joint.joint_position_cmd, joint.joint_velocity_cmd,
      joint.joint_effort_cmd, joint.p_gain_cmd, joint.i_gain_cmd, joint.d_gain_cmd};
    RCLCPP_INFO(
      this->nh_->get_logger(), "Drive joint '%s': p %g, i %g, d %g, effort limit %g",
      joint.name.c_str(), joint.p_gain_cmd, joint.i_gain_cmd, joint.d_gain_cmd,
      joint.effort_limit);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn SimDuaDriveSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    this->nh_->get_logger(), "System Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SimDuaDriveSystem::export_state_interfaces()
{
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
SimDuaDriveSystem::export_command_interfaces()
{
  return std::move(this->dataPtr->command_interfaces_);
}

CallbackReturn SimDuaDriveSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn SimDuaDriveSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type SimDuaDriveSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity) {
      continue;
    }

    // Get the joint velocity
    const auto * jointVelocity =
      this->dataPtr->ecm->Component<sim::components::JointVelocity>(
      this->dataPtr->joints_[i].sim_joint);

    // Get the joint force via joint transmitted wrench
    const auto * jointWrench =
      this->dataPtr->ecm->Component<sim::components::JointTransmittedWrench>(
      this->dataPtr->joints_[i].sim_joint);

    // Get the joint position
    const auto * jointPositions =
      this->dataPtr->ecm->Component<sim::components::JointPosition>(
      this->dataPtr->joints_[i].sim_joint);

    if (!jointPositions || !jointVelocity || !jointWrench) {
      continue;
    }

    this->dataPtr->joints_[i].joint_position = jointPositions->Data()[0];
    this->dataPtr->joints_[i].joint_velocity = jointVelocity->Data()[0];
    gz::physics::Vector3d force_or_torque;
    if (this->dataPtr->joints_[i].joint_type == sdf::JointType::PRISMATIC) {
      force_or_torque = {jointWrench->Data().force().x(),
        jointWrench->Data().force().y(), jointWrench->Data().force().z()};
    } else {  // REVOLUTE and CONTINUOUS
      force_or_torque = {jointWrench->Data().torque().x(),
        jointWrench->Data().torque().y(), jointWrench->Data().torque().z()};
    }
    // Calculate the scalar effort along the joint axis
    this->dataPtr->joints_[i].joint_effort = force_or_torque.dot(
      gz::physics::Vector3d{this->dataPtr->joints_[i].joint_axis.Xyz()[0],
        this->dataPtr->joints_[i].joint_axis.Xyz()[1],
        this->dataPtr->joints_[i].joint_axis.Xyz()[2]});
  }

  for (unsigned int i = 0; i < this->dataPtr->imus_.size(); ++i) {
    if (this->dataPtr->imus_[i]->topicName.empty()) {
      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(this->dataPtr->imus_[i]->sim_imu_sensors_);
      if (sensorTopicComp) {
        this->dataPtr->imus_[i]->topicName = sensorTopicComp->Data();
        RCLCPP_INFO_STREAM(
          this->nh_->get_logger(), "IMU " << this->dataPtr->imus_[i]->name <<
            " has a topic name: " << sensorTopicComp->Data());

        this->dataPtr->node.Subscribe(
          this->dataPtr->imus_[i]->topicName, &ImuData::OnIMU,
          this->dataPtr->imus_[i].get());
      }
    }
  }

  for (unsigned int i = 0; i < this->dataPtr->ft_sensors_.size(); ++i) {
    if (this->dataPtr->ft_sensors_[i]->topicName.empty()) {
      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(this->dataPtr->ft_sensors_[i]->sim_ft_sensors_);
      if (sensorTopicComp) {
        this->dataPtr->ft_sensors_[i]->topicName = sensorTopicComp->Data();
        RCLCPP_INFO_STREAM(
          this->nh_->get_logger(), "ForceTorque " << this->dataPtr->ft_sensors_[i]->name <<
            " has a topic name: " << sensorTopicComp->Data());

        this->dataPtr->node.Subscribe(
          this->dataPtr->ft_sensors_[i]->topicName, &ForceTorqueData::OnForceTorque,
          this->dataPtr->ft_sensors_[i].get());
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SimDuaDriveSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  constexpr ControlMethod_ kAllControlMethodBits =
    static_cast<ControlMethod_>(POSITION | VELOCITY | EFFORT);

  for (unsigned int j = 0; j < this->dataPtr->joints_.size(); j++) {
    for (const std::string & interface_name : stop_interfaces) {
      // Clear joint control method bits corresponding to stop interfaces
      if (interface_name == this->dataPtr->joints_[j].if_name_position) {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(kAllControlMethodBits ^ POSITION);
      } else if (interface_name == this->dataPtr->joints_[j].if_name_velocity) {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(kAllControlMethodBits ^ VELOCITY);
      } else if (interface_name == this->dataPtr->joints_[j].if_name_effort) {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(kAllControlMethodBits ^ EFFORT);
      }
    }

    // Set joint control method bits corresponding to start interfaces
    for (const std::string & interface_name : start_interfaces) {
      if (interface_name == this->dataPtr->joints_[j].if_name_position) {
        this->dataPtr->joints_[j].joint_control_method |= POSITION;
      } else if (interface_name == this->dataPtr->joints_[j].if_name_velocity) {
        this->dataPtr->joints_[j].joint_control_method |= VELOCITY;
      } else if (interface_name == this->dataPtr->joints_[j].if_name_effort) {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
      }
    }
  }

  for (auto & joint : this->dataPtr->joints_) {
    if (!joint.is_drive) {
      continue;
    }
    const std::array<bool, 3> claimed{static_cast<bool>(joint.joint_control_method & POSITION),
      static_cast<bool>(joint.joint_control_method & VELOCITY),
      static_cast<bool>(joint.joint_control_method & EFFORT)};
    if (claimed == joint.claimed) {
      continue;
    }
    const duatic_gazebo::DriveLaw previous_law = joint.law;
    joint.claimed = claimed;
    joint.law = duatic_gazebo::select_law(claimed[0], claimed[1], claimed[2]);
    // A target in force under Pd is kept; after Effort or Velocity it is stale and re-seeded.
    if (previous_law != duatic_gazebo::DriveLaw::Pd) {
      if (std::isfinite(joint.joint_position)) {
        joint.joint_position_cmd = joint.joint_position;
      }
      joint.accepted_cmd.position = joint.joint_position_cmd;
    }
    joint.joint_velocity_cmd = 0.0;
    joint.joint_effort_cmd = 0.0;
    joint.accepted_cmd.velocity = 0.0;
    joint.accepted_cmd.effort = 0.0;
    joint.integral = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SimDuaDriveSystem::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  // dt for the integral term; 0 on the first call or when time does not advance.
  double dt = 0.0;
  if (this->dataPtr->has_last_write_) {
    dt = std::max(
      0.0, (time.nanoseconds() - this->dataPtr->last_update_sim_time_ros_.nanoseconds()) * 1e-9);
  }
  this->dataPtr->last_update_sim_time_ros_ = time;
  this->dataPtr->has_last_write_ = true;

  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity) {
      continue;
    }

    if (this->dataPtr->joints_[i].is_drive) {
      auto & joint = this->dataPtr->joints_[i];
      const auto * position = this->dataPtr->ecm->Component<sim::components::JointPosition>(
        joint.sim_joint);
      const auto * velocity = this->dataPtr->ecm->Component<sim::components::JointVelocity>(
        joint.sim_joint);
      if (!position || position->Data().empty() || !velocity || velocity->Data().empty()) {
        continue;
      }

      const duatic_gazebo::DriveCommand incoming{joint.joint_position_cmd,
        joint.joint_velocity_cmd, joint.joint_effort_cmd, joint.p_gain_cmd, joint.i_gain_cmd,
        joint.d_gain_cmd};
      joint.accepted_cmd = duatic_gazebo::sanitize(incoming, joint.accepted_cmd);
      // NaN compares unequal, so this catches NaN as well as negative gains.
      if (
        incoming.p_gain != joint.accepted_cmd.p_gain ||
        incoming.i_gain != joint.accepted_cmd.i_gain ||
        incoming.d_gain != joint.accepted_cmd.d_gain)
      {
        RCLCPP_WARN_THROTTLE(
          this->nh_->get_logger(), *this->nh_->get_clock(), 1000,
          "Drive joint '%s': rejected a NaN or negative gain, keeping the previous one",
          joint.name.c_str());
      }

      if (joint.law == duatic_gazebo::DriveLaw::Velocity) {
        // Gazebo ignores a joint's velocity command while it has a force command.
        if (this->dataPtr->ecm->Component<sim::components::JointForceCmd>(joint.sim_joint)) {
          this->dataPtr->ecm->RemoveComponent<sim::components::JointForceCmd>(joint.sim_joint);
        }
        auto * vel = this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
          joint.sim_joint);
        if (vel == nullptr) {
          this->dataPtr->ecm->CreateComponent(
            joint.sim_joint, sim::components::JointVelocityCmd({joint.accepted_cmd.velocity}));
        } else if (!vel->Data().empty()) {
          vel->Data()[0] = joint.accepted_cmd.velocity;
        }
        continue;
      }

      const double torque = joint.law == duatic_gazebo::DriveLaw::Effort ?
        std::clamp(joint.accepted_cmd.effort, -joint.effort_limit, joint.effort_limit) :
        duatic_gazebo::pd_torque(
        joint.accepted_cmd, {position->Data()[0], velocity->Data()[0]}, dt, joint.effort_limit,
        joint.integral);
      auto * force = this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
        joint.sim_joint);
      if (force == nullptr) {
        this->dataPtr->ecm->CreateComponent(
          joint.sim_joint, sim::components::JointForceCmd({torque}));
      } else if (!force->Data().empty()) {
        force->Data()[0] = torque;
      }
      continue;
    }

    if (this->dataPtr->joints_[i].joint_control_method & VELOCITY) {
      if (!this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
          this->dataPtr->joints_[i].sim_joint))
      {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          sim::components::JointVelocityCmd({0}));
      } else {
        const auto jointVelCmd =
          this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
          this->dataPtr->joints_[i].sim_joint);
        *jointVelCmd = sim::components::JointVelocityCmd(
          {this->dataPtr->joints_[i].joint_velocity_cmd});
      }
    } else if (this->dataPtr->joints_[i].joint_control_method & POSITION) {
      // Get error in position
      double error;
      error = (this->dataPtr->joints_[i].joint_position -
        this->dataPtr->joints_[i].joint_position_cmd) * this->dataPtr->update_rate;

      // Calculate target velcity
      double target_vel = -this->dataPtr->position_proportional_gain_ * error;

      auto vel =
        this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
        this->dataPtr->joints_[i].sim_joint);

      if (vel == nullptr) {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          sim::components::JointVelocityCmd({target_vel}));
      } else if (!vel->Data().empty()) {
        vel->Data()[0] = target_vel;
      }
    } else if (this->dataPtr->joints_[i].joint_control_method & EFFORT) {
      if (!this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
          this->dataPtr->joints_[i].sim_joint))
      {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          sim::components::JointForceCmd({0}));
      } else {
        const auto jointEffortCmd =
          this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
          this->dataPtr->joints_[i].sim_joint);
        *jointEffortCmd = sim::components::JointForceCmd(
          {this->dataPtr->joints_[i].joint_effort_cmd});
      }
    } else if (this->dataPtr->joints_[i].is_actuated && this->dataPtr->hold_joints_) {
      // Fallback case is a velocity command of zero (only for actuated joints)
      double target_vel = 0.0;
      auto vel =
        this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
        this->dataPtr->joints_[i].sim_joint);

      if (vel == nullptr) {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          sim::components::JointVelocityCmd({target_vel}));
      } else if (!vel->Data().empty()) {
        vel->Data()[0] = target_vel;
      }
    }
  }

  // set values of all mimic joints with respect to mimicked joint
  for (const auto & mimic_joint : this->info_.mimic_joints) {
    // Get the joint position
    double position_mimicked_joint =
      this->dataPtr->ecm->Component<sim::components::JointPosition>(
      this->dataPtr->joints_[mimic_joint.mimicked_joint_index].sim_joint)->Data()[0];

    double position_mimic_joint =
      this->dataPtr->ecm->Component<sim::components::JointPosition>(
      this->dataPtr->joints_[mimic_joint.joint_index].sim_joint)->Data()[0];

    double position_error =
      position_mimic_joint -
      (position_mimicked_joint * mimic_joint.multiplier + mimic_joint.offset);

    double velocity_sp = (-1.0) * position_error * this->dataPtr->update_rate;

    auto vel =
      this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
      this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);

    if (vel == nullptr) {
      this->dataPtr->ecm->CreateComponent(
        this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
        sim::components::JointVelocityCmd({velocity_sp}));
    } else if (!vel->Data().empty()) {
      vel->Data()[0] = velocity_sp;
    }
  }

  return hardware_interface::return_type::OK;
}
}  // namespace duatic_gazebo

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(duatic_gazebo::SimDuaDriveSystem, gz_ros2_control::GazeboSimSystemInterface)
