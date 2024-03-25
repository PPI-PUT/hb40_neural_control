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

#include "hb40_neural_controller/hb40_neural_controller_node.hpp"

#include "hb40_commons/msg/joint_command.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

namespace hb40_neural_controller
{

Hb40NeuralControllerNode::Hb40NeuralControllerNode(const rclcpp::NodeOptions & options)
:  Node("hb40_neural_controller", options)
{
  auto model_path = this->declare_parameter("model_path", std::string("default"));
  auto nominal_joint_position = this->declare_parameter(
    "nominal_joint_position",
    std::vector<float>());
  kp_ = this->declare_parameter("kp", 2.0);
  kd_ = this->declare_parameter("kd", 0.2);
  // check twice:
  std::array<float, 13> x = {
    -0.1f, 1.0f, -1.5f,
    0.1f, -0.8f, 1.5f,
    -0.1f, -1.0f, 1.5f,
    0.1f, 0.8f, -1.5f,
    0.0f};


  hb40_neural_controller_ = std::make_unique<hb40_neural_controller::Hb40NeuralController>(
    model_path, x);

  auto qosRT = rclcpp::QoS(1).keep_last(1).best_effort().durability_volatile();
  auto qos = rclcpp::QoS(1).keep_last(1).reliable().durability_volatile();
  // TODO fix bridge to add header
  // rmw_qos_profile_t qos_filter = rmw_qos_profile_default;
  // qos_filter.depth = 1;
  // qos_filter.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  // qos_filter.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  // robot_state_.reset(
  //   new message_filters::Subscriber<RobotState>(this, "~/input/robot_state", qos_filter));
  // joint_state_.reset(
  //   new message_filters::Subscriber<BridgeData>(this, "~/input/bridge_data", qos_filter));
  // sync_.reset(
  //   new Synchronizer(
  //     SyncPolicy(2), *robot_state_, *joint_state_));
  // sync_->registerCallback(&Hb40NeuralControllerNode::robotStateCallback, this);
  auto joint_name = std::vector<std::string>{"fr_j0", "fr_j1", "fr_j2", "fl_j0", "fl_j1",
    "fl_j2", "rl_j0", "rl_j1", "rl_j2", "rr_j0", "rr_j1", "rr_j2", "sp_j0"};
  cmd_msg_ = hb40_commons::build<JointCommand>()
    .header(std_msgs::msg::Header())
    .source_node("hb40_neural_controller")
    .name(joint_name)
    .kp(std::vector<float>(joint_name.size(), kp_))
    .kd(std::vector<float>(joint_name.size(), kd_))
    .t_pos(std::vector<float>(joint_name.size(), 0.0))
    .t_vel(std::vector<float>(joint_name.size(), 0.0))
    .t_trq(std::vector<float>(joint_name.size(), 0.0));
  robot_state_msg_ = std::make_shared<RobotState>();
  bridge_data_msg_ = std::make_shared<BridgeData>();

  cmd_vel_msg_ = std::make_shared<Twist>();
  control_loop_ =
    this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&Hb40NeuralControllerNode::controlLoop, this));
  sub_bridge_ = this->create_subscription<BridgeData>(
    "~/input/bridge_data", qosRT,
    std::bind(&Hb40NeuralControllerNode::bridgeCallback, this, std::placeholders::_1));
  sub_state_ = this->create_subscription<RobotState>(
    "~/input/robot_state", qosRT,
    std::bind(&Hb40NeuralControllerNode::robotCallback, this, std::placeholders::_1));
  sub_cmd_vel_ = this->create_subscription<Twist>(
    "~/input/cmd_vel", qos,
    std::bind(&Hb40NeuralControllerNode::cmdVelCallback, this, std::placeholders::_1));
  sub_cmd_ = this->create_subscription<std_msgs::msg::String>(
    "~/input/system_cmd", qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      // how to compare strings in ROS2?
      if (msg->data == "nn_activate" && activate_ == false) {
        activate_ = true;
      } else if (msg->data == "nn_activate" && activate_ == true) {
        activate_ = false;
      }
      RCLCPP_INFO(this->get_logger(), "Received command: '%s'", msg->data.c_str());
    });
  pub_cmd_ = this->create_publisher<JointCommand>("~/output/joint_command", qosRT);
  pub_cmd_debug_ = this->create_publisher<JointCommand>("~/output/debug/joint_command", qos);

  //debug
  pub_nominal_ = this->create_publisher<VectorFloatMsg>("~/output/debug/nominal", qosRT);
  pub_foot_contact_ = this->create_publisher<VectorFloatMsg>("~/output/debug/foot_contact", qosRT);
  pub_cycles_since_last_contact_ = this->create_publisher<VectorFloatMsg>(
    "~/output/debug/cycles_since_last_contact", qosRT);
  pub_joint_position_ = this->create_publisher<VectorFloatMsg>(
    "~/output/debug/joint_position",
    qosRT);
  pub_joint_velocity_ = this->create_publisher<VectorFloatMsg>(
    "~/output/debug/joint_velocity",
    qosRT);
  pub_action_ = this->create_publisher<VectorFloatMsg>("~/output/debug/action", qosRT);
  pub_tensor_ = this->create_publisher<VectorFloatMsg>("~/output/debug/tensor", qosRT);
  pub_gravity_ = this->create_publisher<VectorFloatMsg>("~/output/debug/gravity", qosRT);
}

void Hb40NeuralControllerNode::robotStateCallback(
  RobotState::ConstSharedPtr robot,
  BridgeData::ConstSharedPtr bridge)
{
  // do something with the robot state
  RCLCPP_INFO(this->get_logger(), "Received robot state");
  // TODO: Fixe timestamp in bridge msg
}

void Hb40NeuralControllerNode::cmdVelCallback(Twist::SharedPtr msg)
{
  cmd_vel_msg_ = msg;
}

void Hb40NeuralControllerNode::robotCallback(RobotState::SharedPtr msg)
{
  robot_state_msg_ = msg;
}

void Hb40NeuralControllerNode::bridgeCallback(BridgeData::SharedPtr msg)
{
  bridge_data_msg_ = msg;
}

void Hb40NeuralControllerNode::controlLoop()
{
  cmd_msg_.source_node = activate_ ? "hb40_neural_controller" : "hb40_neural_controller_deactivate";
  kp_ = this->get_parameter("kp").as_double();
  kd_ = this->get_parameter("kd").as_double();
  cmd_msg_.kp = std::vector<float>(cmd_msg_.name.size(), kp_);
  cmd_msg_.kd = std::vector<float>(cmd_msg_.name.size(), kd_);
  cmd_msg_.header.stamp = this->now();
  if (bridge_data_msg_->joint_position.size() == 0 || robot_state_msg_->leg.size() == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "No bridge data or robot state received");
    return;
  }
  cmd_msg_.name = bridge_data_msg_->joint_name;
  auto pos =
    hb40_neural_controller_->modelForward(bridge_data_msg_, robot_state_msg_, cmd_vel_msg_);
  cmd_msg_.t_pos.assign(pos.begin(), pos.end());
  cmd_msg_.t_vel = std::vector<float>(bridge_data_msg_->joint_name.size(), 0.0f);
  cmd_msg_.t_trq = std::vector<float>(bridge_data_msg_->joint_name.size(), 0.0f);
  if (activate_) {
    pub_cmd_->publish(cmd_msg_);
  }

  pub_cmd_debug_->publish(cmd_msg_);

  VectorFloatMsg tensor_msg;
  auto tensor = hb40_neural_controller_->getTensor();
  tensor_msg.data.resize(tensor.size());
  tensor_msg.data = std::vector<float>(tensor.begin(), tensor.end());
  pub_tensor_->publish(tensor_msg);
}

}  // namespace hb40_neural_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hb40_neural_controller::Hb40NeuralControllerNode)
