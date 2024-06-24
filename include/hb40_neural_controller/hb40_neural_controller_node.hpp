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

#ifndef HB40_NEURAL_CONTROLLER__HB40_NEURAL_CONTROLLER_NODE_HPP_
#define HB40_NEURAL_CONTROLLER__HB40_NEURAL_CONTROLLER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "hb40_neural_controller/hb40_neural_controller.hpp"
#include "hb40_commons/msg/joint_command.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

namespace hb40_neural_controller
{
using Hb40NeuralControllerPtr = std::unique_ptr<hb40_neural_controller::Hb40NeuralController>;
using JointCommand = hb40_commons::msg::JointCommand;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<RobotState, BridgeData>;
using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
using SubscriberBridge = message_filters::Subscriber<BridgeData>;
using SubscriberRobot = message_filters::Subscriber<RobotState>;
using Twist = geometry_msgs::msg::Twist;
using VectorFloatMsg = std_msgs::msg::Float32MultiArray;
class HB40_NEURAL_CONTROLLER_PUBLIC Hb40NeuralControllerNode : public rclcpp::Node
{
public:
  explicit Hb40NeuralControllerNode(const rclcpp::NodeOptions & options);

private:
  Hb40NeuralControllerPtr hb40_neural_controller_{nullptr};
  float kp_{0.0f};
  float kd_{0.0f};
  bool activate_node_{false};
  JointCommand cmd_msg_;
  std::shared_ptr<RobotState> robot_state_msg_;
  std::shared_ptr<BridgeData> bridge_data_msg_;
  std::shared_ptr<Twist> cmd_vel_msg_;
  // std::shared_ptr<SubscriberRobot> robot_state_;
  // std::shared_ptr<SubscriberBridge> joint_state_;
  // std::shared_ptr<Synchronizer> sync_;
  rclcpp::Subscription<Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<BridgeData>::SharedPtr sub_bridge_;
  rclcpp::Subscription<RobotState>::SharedPtr sub_state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_;
  rclcpp::Publisher<JointCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<JointCommand>::SharedPtr pub_cmd_debug_;
  rclcpp::Publisher<VectorFloatMsg>::SharedPtr pub_action_;
  rclcpp::Publisher<VectorFloatMsg>::SharedPtr pub_tensor_;
  rclcpp::TimerBase::SharedPtr control_loop_;
  void robotStateCallback(RobotState::ConstSharedPtr robot, BridgeData::ConstSharedPtr bridge);
  void robotCallback(RobotState::SharedPtr robot);
  void bridgeCallback(BridgeData::SharedPtr bridge);
  void cmdVelCallback(Twist::SharedPtr twist); // TODO: change Twist to TwistStamped!
  void systemCmdCallback(std_msgs::msg::String::SharedPtr msg);
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);
  void pubTensor();
  void pubAction();
  void controlLoop();
};
}  // namespace hb40_neural_controller

#endif  // HB40_NEURAL_CONTROLLER__HB40_NEURAL_CONTROLLER_NODE_HPP_
