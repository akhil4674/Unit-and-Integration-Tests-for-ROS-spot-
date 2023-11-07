import rclpy
import pytest
from spot_arm_status.spot_arm_status_pub import JointStateSubscriber


@pytest.fixture(scope="module")
def ros_init():
    rclpy.init()


def test_arm_stowed_correctly_evaluated(ros_init):
    # given

    arm_stowed_values = [
        -0.0001456737518310547,
        -3.115017890930176,
        0.0,
        3.132694959640503,
        1.570010781288147,
        0.0025632381439208984,
        -1.56974196434021,
    ]

    node = JointStateSubscriber()

    # then

    # expected
    assert node.is_arm_in_stowed_position(arm_stowed_values) is True


def test_gripper_open_correctly_evaluated(ros_init):
    gripper_values = -0.05

    node = JointStateSubscriber()

    assert node.is_position_within_gripper_range(gripper_values) is False


def test_arm_not_stowed_correctly_evaluated(ros_init):
    # given

    arm_values = [0, 1, 2, 3, 4, 5, 6]

    node = JointStateSubscriber()

    # then

    # expected
    assert node.is_arm_stowed(arm_values) is False

    
