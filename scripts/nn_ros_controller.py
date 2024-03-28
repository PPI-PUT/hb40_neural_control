import numpy as np
from scipy.spatial.transform import Rotation as R
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from hb40_commons.msg import BridgeData
from hb40_commons.msg import JointCommand
from geometry_msgs.msg import Twist


class RobotHandler(Node):
    def __init__(self):
        super().__init__("robot_handler")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.bridge_data_subscription = self.create_subscription(
            BridgeData,
            "/hb40/bridge_data",
            self.bridge_data_callback,
            qos_profile=qos_profile)

        self.velocity_command_subscription = self.create_subscription(
            Twist,
            "/hb40/velocity_command",
            self.velocity_command_callback,
            qos_profile=qos_profile)

        self.joint_commands_publisher = self.create_publisher(
            JointCommand,
            "/hb40/joint_command",
            qos_profile=qos_profile)
        timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.joint_positions = np.zeros(13)
        self.joint_velocities = np.zeros(13)
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.angular_velocity = np.zeros(3)
        self.x_goal_velocity = 0.0
        self.y_goal_velocity = 0.0
        self.yaw_goal_velocity = 0.0

        self.kp = [20.0,] * 13
        self.kd = [0.5,] * 13
        self.scaling_factor = 0.25
        self.nominal_joint_positions = np.array([
            -0.1, 1.0, -1.5,
            0.1, -0.8, 1.5,
            -0.1, -1.0, 1.5,
            0.1, 0.8, -1.5,
            0.0
        ])

        self.previous_action = np.zeros(13)

        neural_network_name = "../policy/policy.pt"
        self.model = torch.jit.load(neural_network_name)

    def bridge_data_callback(self, msg):
        self.joint_positions = np.array(msg.joint_position)
        self.joint_velocities = np.array(msg.joint_velocity)
        self.orientation = np.array(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.angular_velocity = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    def velocity_command_callback(self, msg):
        self.x_goal_velocity = msg.linear.x
        self.y_goal_velocity = msg.linear.y
        # Topic gives -2 to 2, but we want -1 to 1
        self.yaw_goal_velocity = msg.angular.z * 0.5

    def timer_callback(self):
        transposed_trunk_rotation_matrix = R.from_quat(
            self.orientation).as_matrix().T
        qpos = self.joint_positions - self.nominal_joint_positions
        qvel = self.joint_velocities
        ang_vel = self.angular_velocity
        projected_gravity_vector = np.matmul(
            transposed_trunk_rotation_matrix, np.array([0.0, 0.0, -1.0]))
        observation = np.concatenate([
            qpos, ang_vel, qvel,
            [self.x_goal_velocity, self.y_goal_velocity, self.yaw_goal_velocity],
            projected_gravity_vector, self.previous_action
        ])

        with torch.no_grad(), torch.inference_mode():
            action = self.model(torch.tensor(observation.reshape(
                1, -1), dtype=torch.float32)).numpy()[0]

        target_joint_positions = self.nominal_joint_positions + self.scaling_factor * action

        joint_command_msg = JointCommand()
        joint_command_msg.source_node = "neural_controller_python"
        joint_command_msg.header.stamp = self.get_clock().now().to_msg()
        joint_command_msg.kp = self.kp
        joint_command_msg.kd = self.kd
        joint_command_msg.t_pos = target_joint_positions.tolist()
        joint_command_msg.t_vel = [0.0,] * 13
        joint_command_msg.t_trq = [0.0,] * 13

        self.joint_commands_publisher.publish(joint_command_msg)

        self.previous_action = action


def main(args=None):
    rclpy.init(args=args)
    robot_handler = RobotHandler()
    rclpy.spin(robot_handler)
    robot_handler.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
