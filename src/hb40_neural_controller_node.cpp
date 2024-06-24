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
  using std::placeholders::_1;
  auto model_path = this->declare_parameter("model_path", std::string("default"));
  kp_ = this->declare_parameter("kp", 2.0);
  kd_ = this->declare_parameter("kd", 0.2);
  activate_node_ = this->declare_parameter("activate_node", false);
  hb40_neural_controller_ = std::make_unique<hb40_neural_controller::Hb40NeuralController>(
    model_path);

  auto qosRT = rclcpp::QoS(1).keep_last(1).best_effort().durability_volatile();
  auto qos = rclcpp::QoS(1).keep_last(1).reliable().durability_volatile();
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
  set_param_res_ =
    this->add_on_set_parameters_callback(
    std::bind(
      &Hb40NeuralControllerNode::onSetParam, this,
      _1));
  
  control_loop_ =
    this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&Hb40NeuralControllerNode::controlLoop, this));

  sub_bridge_ = this->create_subscription<BridgeData>(
    "~/input/bridge_data", qosRT,
    std::bind(&Hb40NeuralControllerNode::bridgeCallback, this, _1));
  sub_state_ = this->create_subscription<RobotState>(
    "~/input/robot_state", qosRT,
    std::bind(&Hb40NeuralControllerNode::robotCallback, this, _1));
  sub_cmd_vel_ = this->create_subscription<Twist>(
    "~/input/cmd_vel", qos,
    std::bind(&Hb40NeuralControllerNode::cmdVelCallback, this, _1));
  sub_cmd_ = this->create_subscription<std_msgs::msg::String>(
    "~/input/system_cmd", qos,
    std::bind(&Hb40NeuralControllerNode::systemCmdCallback, this, _1));

  pub_cmd_ = this->create_publisher<JointCommand>("~/output/joint_command", qosRT);
  pub_cmd_debug_ = this->create_publisher<JointCommand>("~/output/debug/joint_command", qos);
  pub_action_ = this->create_publisher<VectorFloatMsg>("~/output/debug/action", qos);
  pub_tensor_ = this->create_publisher<VectorFloatMsg>("~/output/debug/tensor", qos);
  
  // TODO fix bridge topic and add header to robot interface
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
}

void Hb40NeuralControllerNode::systemCmdCallback(std_msgs::msg::String::SharedPtr msg)
{
  if(msg->data == "nn_activate" && activate_node_ == false)
  {
    activate_node_ = true;
  } else if ( msg->data == "nn_activate" && activate_node_ == true)
  {
    activate_node_ = false;
  }
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
  // yaw in robot interface -2.0 to 2.0 nn inputs are -1.0 to 1.0
  cmd_vel_msg_->angular.z *= 0.5;
}

void Hb40NeuralControllerNode::robotCallback(RobotState::SharedPtr msg)
{
  robot_state_msg_ = msg;
}

void Hb40NeuralControllerNode::bridgeCallback(BridgeData::SharedPtr msg)
{
  bridge_data_msg_ = msg;
}

void Hb40NeuralControllerNode::pubTensor()
{
  VectorFloatMsg tensor_msg;
  auto tensor = hb40_neural_controller_->getTensor();
  tensor_msg.data.resize(tensor.size());
  tensor_msg.data = std::vector<float>(tensor.begin(), tensor.end());
  pub_tensor_->publish(tensor_msg);
}

void Hb40NeuralControllerNode::pubAction()
{
  VectorFloatMsg action_msg;
  auto action = hb40_neural_controller_->getAction();
  action_msg.data.resize(action.size());
  action_msg.data = std::vector<float>(action.begin(), action.end());
  pub_action_->publish(action_msg);
}

void Hb40NeuralControllerNode::controlLoop()
{
  cmd_msg_.source_node = activate_node_ ? "hb40_neural_controller" : "hb40_neural_controller_deactivate";
  cmd_msg_.header.stamp = this->now();
  if (bridge_data_msg_->joint_position.size() == 0 || robot_state_msg_->leg.size() == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "No bridge data or robot state received");
    return;
  }
  cmd_msg_.name = bridge_data_msg_->joint_name;
  auto target_joint_positions =
    hb40_neural_controller_->modelForward(bridge_data_msg_, robot_state_msg_, cmd_vel_msg_);
  cmd_msg_.t_pos.assign(target_joint_positions.begin(), target_joint_positions.end());
  if (activate_node_) {
    pub_cmd_->publish(cmd_msg_);
  }
  pub_cmd_debug_->publish(cmd_msg_);

  this->pubAction();
  this->pubTensor();
}

rcl_interfaces::msg::SetParametersResult Hb40NeuralControllerNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  auto check_gain = [](const double & param)->bool
    {
      return param > 0.0;
    };
  result.successful = false;
  result.reason = "Failed to set parameters";
  try {
    {
      for (const auto & param : params) {
        if (param.get_name() == "kp") {
          if (check_gain(param.as_double())) {
            cmd_msg_.kp = std::vector<float>(
              cmd_msg_.name.size(),
              param.as_double());
            result.successful = true;
            result.reason = "Successfully set kp";
          }
        } else if (param.get_name() == "kd") {
          if (check_gain(param.as_double())) {
            cmd_msg_.kd = std::vector<float>(
              cmd_msg_.name.size(),
              param.as_double());
            result.successful = true;
            result.reason = "Successfully set kd";
          }
        } else if (param.get_name() == "activate") {
          activate_node_ = param.as_bool();
          auto msg = activate_node_ ? "Activated" : "Deactivated";
          result.successful = true;
          result.reason = msg;
        }
      }
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }
  return result;
}
}  // namespace hb40_neural_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hb40_neural_controller::Hb40NeuralControllerNode)
