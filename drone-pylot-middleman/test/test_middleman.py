import pytest

import rclpy, os, time
from rclpy.node import Node

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image


path = os.path.dirname(os.path.realpath(__file__)).replace("slam_traffic/test", "")


def test_node_initialization():
    rclpy.init(args=None)
    try:
        pass
        # node = TrafficSignRecognizer(debug=True, bb_only=False)
        # assert node is not None

        # assert node.get_name() == "traffic_sign_recognizer"

        # assert hasattr(node, 'subscription')
        # assert node.subscription.topic_name == "/camera/image_raw"

        # assert hasattr(node, 'publisher_')
        # assert node.publisher_.topic_name == "/traffic_sign_detection"
        # assert hasattr(node, 'box_edges_pub')
        # assert node.box_edges_pub.topic_name == "/box_edges"

        # assert hasattr(node, 'bridge')
        # assert node.bridge is not None

    finally:
        # node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    pytest.main(['-v'])