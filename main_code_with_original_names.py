import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__("arm_gripper_status")

        self.arm_joints = [
            "arm_sh0",
            "arm_sh1",
            "arm_hr0",
            "arm_el0",
            "arm_el1",
            "arm_wr0",
            "arm_wr1",
        ]
        self.arm_stowed_values = [
            -0.0001456737518310547,
            -3.115017890930176,
            0.0,
            3.132694959640503,
            1.570010781288147,
            0.0025632381439208984,
            -1.56974196434021,
        ]

        self.joint_state_subscription = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 1
        )
        self.arm_stowed_publisher = self.create_publisher(Bool, "arm_stowed", 1)
        self.gripper_open_publisher = self.create_publisher(Bool, "gripper_open", 1)

    def get_joint_position(self, joint_name, joint_state_msg):
        index = joint_state_msg.name.index(joint_name)
        return joint_state_msg.position[index]

    def is_arm_in_stowed_position(self, arm_values):
        joint_stowed = [
            abs(real - desired) < 0.1
            for real, desired in zip(arm_values, self.arm_stowed_values)
        ]

        return all(joint_stowed)

    def checking_gripper_position(self, gripper_position):
        return -0.04 <= gripper_position <= -0.02

    def joint_state_callback(self, msg):
        self.get_logger().debug(f"Received joint state: {msg}")

        try:
            gripper_position = self.get_joint_position("arm_f1x", msg)
        except ValueError:
            self.get_logger().debug(
                "This joint state does not contain the right joints - ignoring."
            )

        gripper_open_msg = Bool()
        gripper_open_msg.data = self.checking_gripper_position(gripper_position)
        self.gripper_open_publisher.publish(gripper_open_msg)

        arm_values = [self.get_joint_position(joint, msg) for joint in self.arm_joints]

        arm_stowed = self.is_arm_in_stowed_position(arm_values)
        arm_stowed_msg = Bool()
        arm_stowed_msg.data = arm_stowed
        self.arm_stowed_publisher.publish(arm_stowed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
