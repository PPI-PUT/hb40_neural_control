// Copyright 2024 Maciej Krupka
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HB40_NEURAL_CONTROLLER__HB40_NEURAL_CONTROLLER_HPP_
#define HB40_NEURAL_CONTROLLER__HB40_NEURAL_CONTROLLER_HPP_

#include <cstdint>
#include <torch/script.h>
#include <ATen/ATen.h>
#include <Eigen/Dense>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <hb40_commons/msg/bridge_data.hpp>
#include <hb40_commons/msg/robot_state.hpp>
#include "hb40_neural_controller/hb40_neural_controller_constants.hpp"
#include "hb40_neural_controller/visibility_control.hpp"


namespace hb40_neural_controller
{
using Quaternionf = Eigen::Quaternionf;
using QuaternionMsg = geometry_msgs::msg::Quaternion;
using Vector3f = Eigen::Vector3f;
using Vector3Msg = geometry_msgs::msg::Vector3;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using Twist = geometry_msgs::msg::Twist;
using BridgeData = hb40_commons::msg::BridgeData;
using RobotState = hb40_commons::msg::RobotState;
using Imu = sensor_msgs::msg::Imu;
using WrenchStamped = geometry_msgs::msg::WrenchStamped;
using JointsArray = std::array<float, static_cast<size_t>(Joints::SIZE)>;
using LegsArray = std::array<float, static_cast<size_t>(Legs::SIZE)>;
using TensorArray = std::array<float, TENSOR_SIZE>;
using GravityArray = std::array<float, VECTOR3_SIZE>;
class HB40_NEURAL_CONTROLLER_PUBLIC Hb40NeuralController
{
public:
  Hb40NeuralController();
  Hb40NeuralController(
    const std::string model_path);
  JointsArray modelForward(
    const std::shared_ptr<BridgeData> & bridge,
    const std::shared_ptr<RobotState> & robot,
    const std::shared_ptr<Twist> & twist);
  JointsArray getNominal();
  LegsArray getFootContact();
  LegsArray getCyclesSinceLastContact();
  JointsArray getJointPosition();
  JointsArray getJointVelocity();
  JointsArray getAction();
  TensorArray getTensor();
  GravityArray getGravity();

private:
  std::unique_ptr<torch::jit::script::Module> module_;
  double scaled_factor_{SCALED_FACTOR};
  JointsArray nominal_joint_position_;
  LegsArray foot_contact_;
  LegsArray cycles_since_last_contact_;
  GravityArray gravity_;
  JointsArray joint_position_;
  JointsArray joint_velocity_;
  JointsArray last_action_;
  TensorArray input_tensor_;
  GravityArray convertToGravityVector(
    const QuaternionMsg & orientation);
  void setTensor(
    const std::shared_ptr<BridgeData> & bridge,
    const std::shared_ptr<RobotState> & robot,
    const std::shared_ptr<Twist> & twist);
  void updateCycleSinceLastContact();
  void loadModel(std::string path);
};

}  // namespace hb40_neural_controller

#endif  // HB40_NEURAL_CONTROLLER__HB40_NEURAL_CONTROLLER_HPP_
