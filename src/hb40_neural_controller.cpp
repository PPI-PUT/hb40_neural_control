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
  const std::string model_path, JointsArray nominal_joint_position)
{
  cycles_since_last_contact_ = LegsArray{};
  foot_contact_ = LegsArray{};
  joint_position_ = JointsArray{};
  joint_velocity_ = JointsArray{};
  tensor_ = TensorArray{};
  gravity_ = GravityArray{};
  if (nominal_.size() != nominal_joint_position.size()) {
    throw std::invalid_argument("Nominal joint position size is not correct");
  }
  std::copy(
    nominal_joint_position.begin(), nominal_joint_position.end(), nominal_.begin());
  last_action_ = JointsArray{};
  this->loadModel(model_path);

}

void Hb40NeuralController::loadModel(std::string path)
{
  try {
    auto module = std::make_shared<torch::jit::script::Module>(torch::jit::load(path));
    module_ = module;
  } catch (const c10::Error & e) {
    std::cerr << "Error loading the model\n";
    std::cerr << e.what();
    std::exit(1);
  }
}

JointsArray Hb40NeuralController::modelForward(
  const std::shared_ptr<BridgeData> & bridge,
  const std::shared_ptr<RobotState> & robot,
  const std::shared_ptr<Twist> & twist)
{
  this->createTensor(bridge, robot, twist);

  auto tensor = torch::from_blob(tensor_.data(), {1, static_cast<long>(tensor_.size())});
  std::cout << tensor << std::endl;
  at::Tensor action = module_->forward({tensor}).toTensor();
  JointsArray action_arr;

  if (action.numel() == action_arr.size()) {
    std::copy_n(action.data_ptr<float>(), action_arr.size(), action_arr.begin());
  } else {
    std::cerr << "Model output size is not equal to action array size\n";
  }
  std::memcpy(last_action_.data(), action_arr.data(), action_arr.size() * sizeof(float));
  std::transform(
    nominal_.begin(), nominal_.end(),
    action_arr.begin(), action_arr.begin(),
    [&](double a, double b)
    {return a + (b * scaled_factor_);});
  return action_arr;
}

void Hb40NeuralController::createTensor(
  const std::shared_ptr<BridgeData> & bridge,
  const std::shared_ptr<RobotState> & robot,
  const std::shared_ptr<Twist> & twist)
{
  auto* tensorPtr = &this->tensor_;
  // tensor_.fill(0.0f);
  auto index = 0;
  auto copyData = [&index, tensorPtr](const auto & data) {
      std::memcpy(tensorPtr->data() + index, data.data(), data.size() * sizeof(float));
      index += data.size();
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


  std::vector<float> normalized_joint_position(bridge->joint_position.size());
  normalized_joint_position = bridge->joint_position;
  // normalized_joint_position = reorderVector(
  //   bridge->joint_position,
  //   typename ReorderToTensor<JointsOrder>::type{});

  std::transform(
    nominal_.begin(), nominal_.end(),
    normalized_joint_position.begin(), normalized_joint_position.begin(),
    [](float nominal, float pose) {return -1 * nominal + pose;}
  );
  
  copyData(normalized_joint_position);
  tensor_[index++] = static_cast<float>(bridge->angular_velocity.x);
  tensor_[index++] = static_cast<float>(bridge->angular_velocity.y);
  tensor_[index++] = static_cast<float>(bridge->angular_velocity.z);

  std::vector<float> reorder_joint_velocity(bridge->joint_velocity.size());
  reorder_joint_velocity = bridge->joint_velocity;
  // reorder_joint_velocity = reorderVector(
  //   bridge->joint_velocity,
  //   typename ReorderToTensor<JointsOrder>::type{});
  copyData(reorder_joint_velocity);
  tensor_[index++] = static_cast<float>(twist->linear.x);
  tensor_[index++] = static_cast<float>(twist->linear.y);
  tensor_[index++] = static_cast<float>(twist->angular.z);


  // // Foot contact and cycles since last contact
  for (auto & leg : robot->leg) {
    auto update_contact = [&](NetworkLegs legEnum) {
        auto idx = static_cast<size_t>(legEnum);
        foot_contact_[idx] = static_cast<float>(leg.contact);
        cycles_since_last_contact_[idx] =
          foot_contact_[idx] ? 0.0f : cycles_since_last_contact_[idx] + 1.0f;
      };
    if (leg.leg_name == "FR") {
      update_contact(NetworkLegs::FR);
    } else if (leg.leg_name == "FL") {
      update_contact(NetworkLegs::FL);
    } else if (leg.leg_name == "RL") {
      update_contact(NetworkLegs::RL);
    } else if (leg.leg_name == "RR") {
      update_contact(NetworkLegs::RR);
    }
  }
  // Gravity vector
  Quaternionf orientation(
    bridge->orientation.w, bridge->orientation.x, bridge->orientation.y, bridge->orientation.z);
  Vector3f gravity = orientation * Vector3f(0.0f, 0.0f, -1.0f);
  gravity.normalize();
  gravity_[0] = static_cast<float>(gravity.x());
  gravity_[1] = static_cast<float>(gravity.y());
  gravity_[2] = static_cast<float>(gravity.z());
  tensor_[index++] = gravity_[0];
  tensor_[index++] = gravity_[1];
  tensor_[index++] = gravity_[2];

  // Last action
  copyData(last_action_);
  // std::cout << "LAST ACTION: " << last_action_[0] << std::endl;

  // // Cycles since last contact
  // copyData(cycles_since_last_contact_);
}

JointsArray Hb40NeuralController::getNominal()
{
  return nominal_;
}

LegsArray Hb40NeuralController::getFootContact()
{
  return foot_contact_;
}

LegsArray Hb40NeuralController::getCyclesSinceLastContact()
{
  return cycles_since_last_contact_;
}

JointsArray Hb40NeuralController::getAction()
{
  return last_action_;
}

GravityArray Hb40NeuralController::getGravity()
{
  return gravity_;
}

JointsArray Hb40NeuralController::getJointPosition()
{
  return joint_position_;
}
JointsArray Hb40NeuralController::getJointVelocity()
{
  return joint_velocity_;
}
TensorArray Hb40NeuralController::getTensor()
{
  return tensor_;
}
}  // namespace hb40_neural_controller
