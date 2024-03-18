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

#include "hb40_neural_controller/hb40_neural_controller.hpp"
#include <iostream>

namespace hb40_neural_controller
{

Hb40NeuralController::Hb40NeuralController()
{
  // Empty constructor
}
Hb40NeuralController::Hb40NeuralController(
  const std::string model_path, std::vector<float> nominal_joint_position)
{
  cycles_since_last_contact_ = std::vector<float>(4, 0.0f);
  foot_contact_ = std::vector<float>(4, 0.0f);
  joint_position_ = std::vector<float>(nominal_joint_position.size(), 0.0f);
  joint_velocity_ = std::vector<float>(nominal_joint_position.size(), 0.0f);
  gravity_ = std::vector<float>(3, 0.0f);
  this->loadModel(model_path);
  nominal_ = nominal_joint_position;
  if (nominal_.size() != 13) {
    throw std::invalid_argument("Nominal joint position size is not 13");
  }
  last_action_ = std::vector<float>(13, 0.0f);

}

void Hb40NeuralController::loadModel(std::string path)
{
  std::cout << "Loading model: " << path << std::endl;
  try {
    auto module = std::make_shared<torch::jit::script::Module>(torch::jit::load(path));
    module_ = module;
  } catch (const c10::Error & e) {
    std::cerr << "Error loading the model\n";
  }
}

std::vector<float> Hb40NeuralController::modelForward(
  const std::shared_ptr<BridgeData> & bridge,
  const std::shared_ptr<RobotState> & robot,
  const std::shared_ptr<Twist> & twist)
{
  auto state = this->createTensor(bridge, robot, twist);
  std::cout << state.size() << std::endl;
  
  last_state_ = state;

  auto tensor = torch::from_blob(state.data(), {1, static_cast<long>(state.size())});

  at::Tensor action = module_->forward({tensor}).toTensor();

  std::vector<float> action_vec(action.data_ptr<float>(),
    action.data_ptr<float>() + action.numel());
  last_action_ = action_vec;

  std::transform(
    nominal_.begin(), nominal_.end(),
    action_vec.begin(), action_vec.begin(),
    [&](double a, double b)
    {return a + (b * scaled_factor_);});

  std::vector<float> joint_command = std::vector<float>(action_vec.size(), 0.0f);
  joint_command[MAB_FR_HIP] = action_vec[FR_HIP];
  joint_command[MAB_FR_THIGH] = action_vec[FR_THIGH];
  joint_command[MAB_FR_CALF] = action_vec[FR_CALF];
  joint_command[MAB_FL_HIP] = action_vec[FL_HIP];
  joint_command[MAB_FL_THIGH] = action_vec[FL_THIGH];
  joint_command[MAB_FL_CALF] = action_vec[FL_CALF];
  joint_command[MAB_RR_HIP] = action_vec[RR_HIP];
  joint_command[MAB_RR_THIGH] = action_vec[RR_THIGH];
  joint_command[MAB_RR_CALF] = action_vec[RR_CALF];
  joint_command[MAB_RL_HIP] = action_vec[RL_HIP];
  joint_command[MAB_RL_THIGH] = action_vec[RL_THIGH];
  joint_command[MAB_RL_CALF] = action_vec[RL_CALF];
  joint_command[MAB_SPINE_0] = action_vec[SPINE_0];
  return joint_command;
}

std::vector<float> Hb40NeuralController::createTensor(
  const std::shared_ptr<BridgeData> & bridge,
  const std::shared_ptr<RobotState> & robot,
  const std::shared_ptr<Twist> & twist)
{
  std::vector<float> tensor;

  // // Joint position
  auto normalize = [](float pose, float nominal) -> float {
      return pose - nominal;
    };

  // throw exception if nominal size is not bridge.joint_posiotn
  if (nominal_.size() != bridge->joint_position.size()) {
    throw std::invalid_argument(
            "Nominal joint position size is not equal to bridge joint position size");
  }
  if (nominal_.size() != bridge->joint_velocity.size()) {
    throw std::invalid_argument(
            "Nominal joint position size is not equal to bridge joint velocity size");
  }
  if (bridge->joint_position.size() != bridge->joint_position.size()) {
    throw std::invalid_argument(
            "Bridge joint position size is not equal to bridge joint velocity size");
  }
  joint_position_[SPINE_0] = normalize(bridge->joint_position[SPINE_0], nominal_[SPINE_0]);
  joint_position_[FR_HIP] = normalize(bridge->joint_position[MAB_FR_HIP], nominal_[FR_HIP]);
  joint_position_[FR_THIGH] = normalize(bridge->joint_position[MAB_FR_THIGH], nominal_[FR_THIGH]);
  joint_position_[FR_CALF] = normalize(bridge->joint_position[MAB_FR_CALF], nominal_[FR_CALF]);

  joint_position_[FL_HIP] = normalize(bridge->joint_position[MAB_FL_HIP], nominal_[FL_HIP]);
  joint_position_[FL_THIGH] = normalize(bridge->joint_position[MAB_FL_THIGH], nominal_[FL_THIGH]);
  joint_position_[FL_CALF] = normalize(bridge->joint_position[MAB_FL_CALF], nominal_[FL_CALF]);

  joint_position_[RR_HIP] = normalize(bridge->joint_position[MAB_RR_HIP], nominal_[RR_HIP]);
  joint_position_[RR_THIGH] = normalize(bridge->joint_position[MAB_RR_THIGH], nominal_[RR_THIGH]);
  joint_position_[RR_CALF] = normalize(bridge->joint_position[MAB_RR_CALF], nominal_[RR_CALF]);

  joint_position_[RL_HIP] = normalize(bridge->joint_position[MAB_RL_HIP], nominal_[RL_HIP]);
  joint_position_[RL_THIGH] = normalize(bridge->joint_position[MAB_RL_THIGH], nominal_[RL_THIGH]);
  joint_position_[RL_CALF] = normalize(bridge->joint_position[MAB_RL_CALF], nominal_[RL_CALF]);


  tensor.insert(tensor.end(), joint_position_.begin(), joint_position_.end());

  // //  Imu angular velocity
  tensor.push_back(static_cast<float>(bridge->angular_velocity.x));
  tensor.push_back(static_cast<float>(bridge->angular_velocity.y));
  tensor.push_back(static_cast<float>(bridge->angular_velocity.z));

  // Joint velocity
  joint_velocity_[SPINE_0] = bridge->joint_velocity[SPINE_0];

  joint_velocity_[FR_HIP] = bridge->joint_velocity[MAB_FR_HIP];
  joint_velocity_[FR_THIGH] = bridge->joint_velocity[MAB_FR_THIGH];
  joint_velocity_[FR_CALF] = bridge->joint_velocity[MAB_FR_CALF];

  joint_velocity_[FL_HIP] = bridge->joint_velocity[MAB_FL_HIP];
  joint_velocity_[FL_THIGH] = bridge->joint_velocity[MAB_FL_THIGH];
  joint_velocity_[FL_CALF] = bridge->joint_velocity[MAB_FL_CALF];

  joint_velocity_[RR_HIP] = bridge->joint_velocity[MAB_RR_HIP];
  joint_velocity_[RR_THIGH] = bridge->joint_velocity[MAB_RR_THIGH];
  joint_velocity_[RR_CALF] = bridge->joint_velocity[MAB_RR_CALF];

  joint_velocity_[RL_HIP] = bridge->joint_velocity[MAB_RL_HIP];
  joint_velocity_[RL_THIGH] = bridge->joint_velocity[MAB_RL_THIGH];
  joint_velocity_[RL_CALF] = bridge->joint_velocity[MAB_RL_CALF];

  tensor.insert(tensor.end(), joint_velocity_.begin(), joint_velocity_.end());

  // Goal velocity
  tensor.push_back(static_cast<float>(twist->linear.x));
  tensor.push_back(static_cast<float>(twist->linear.y));
  tensor.push_back(static_cast<float>(twist->angular.z));
  // Foot contact
  foot_contact_[FR_CONTACT] = robot->leg[MAB_FR].contact ? 1.0f : 0.0f;
  foot_contact_[FL_CONTACT] = robot->leg[MAB_FL].contact ? 1.0f : 0.0f;
  foot_contact_[RL_CONTACT] = robot->leg[MAB_RL].contact ? 1.0f : 0.0f;
  foot_contact_[RR_CONTACT] = robot->leg[MAB_RR].contact ? 1.0f : 0.0f;
  // tensor.insert(tensor.end(), foot_contact_.begin(), foot_contact_.end());

  // // Cycles since last contact
  cycles_since_last_contact_[FR_CYCLE] =
    foot_contact_[FR_CONTACT] ? 0.0f : cycles_since_last_contact_[FR_CONTACT] + 1.0f;
  cycles_since_last_contact_[FL_CYCLE] =
    foot_contact_[FL_CONTACT] ? 0.0f : cycles_since_last_contact_[FL_CONTACT] + 1.0f;
  cycles_since_last_contact_[RL_CYCLE] =
    foot_contact_[RL_CONTACT] ? 0.0f : cycles_since_last_contact_[RL_CONTACT] + 1.0f;
  cycles_since_last_contact_[RR_CYCLE] =
    foot_contact_[RR_CONTACT] ? 0.0f : cycles_since_last_contact_[RR_CONTACT] + 1.0f;

  // Gravity vector
  Quaternionf orientation(
    bridge->orientation.w, bridge->orientation.x, bridge->orientation.y, bridge->orientation.z);
  Vector3f gravity = orientation * Vector3f(0.0f, 0.0f, -1.0f);
  gravity.normalize();
  gravity_[0] = static_cast<float>(gravity.x());
  gravity_[1] = static_cast<float>(gravity.y());
  gravity_[2] = static_cast<float>(gravity.z());

  tensor.insert(tensor.end(), gravity_.begin(), gravity_.end());

  // // Last action
  tensor.insert(tensor.end(), last_action_.begin(), last_action_.end());

  // // Cycles since last contact
  // tensor.insert(tensor.end(), cycles_since_last_contact_.begin(), cycles_since_last_contact_.end());

  return tensor;
}

std::vector<float> Hb40NeuralController::getNominal()
{
  return nominal_;
}

std::vector<float> Hb40NeuralController::getFootContact()
{
  return foot_contact_;
}

std::vector<float> Hb40NeuralController::getCyclesSinceLastContact()
{
  return cycles_since_last_contact_;
}

std::vector<float> Hb40NeuralController::getAction()
{
  return last_action_;
}

std::vector<float> Hb40NeuralController::getTensor()
{
  return last_state_;
}

std::vector<float> Hb40NeuralController::getGravity()
{
  return gravity_;
}

std::vector<float> Hb40NeuralController::getJointPosition()
{
  return joint_position_;
}
std::vector<float> Hb40NeuralController::getJointVelocity()
{
  return joint_velocity_;
}

}  // namespace hb40_neural_controller
