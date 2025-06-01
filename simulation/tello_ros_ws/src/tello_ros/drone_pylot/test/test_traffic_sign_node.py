import pytest

import rclpy, os, time
from rclpy.node import Node

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
from slam_traffic.traffic_sign_recognizer_node import TrafficSignRecognizer


path = os.path.dirname(os.path.realpath(__file__)).replace("slam_traffic/test", "")


def test_node_initialization():
    rclpy.init(args=None)
    try:
        node = TrafficSignRecognizer(debug=True, bb_only=False)
        assert node is not None

        assert node.get_name() == "traffic_sign_recognizer"

        assert hasattr(node, 'subscription')
        assert node.subscription.topic_name == "/camera/image_raw"

        assert hasattr(node, 'publisher_')
        assert node.publisher_.topic_name == "/traffic_sign_detection"
        assert hasattr(node, 'box_edges_pub')
        assert node.box_edges_pub.topic_name == "/box_edges"

        assert hasattr(node, 'bridge')
        assert node.bridge is not None

        # assert path == "/home/olivia/Experimental/dev_ws/src/"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_image_bounding_box_():

    rclpy.init(args=None)
    assert rclpy.ok()
    node = TrafficSignRecognizer(debug=True, bb_only=False)
    workdir = os.path.join(path, "ros2_bag_to_image/ros_bags")
    bag_paths = [
        # os.path.join(workdir, "rosbag2_2025_03_20-16_49_11"),
        # os.path.join(workdir, "rosbag2_2025_03_20-16_54_08"),
        # os.path.join(workdir, "rosbag2_2025_03_20-16_56_50"),
        os.path.join(workdir, "rosbag2_2025_04_11-15_04_58"),
        
    ]

    for bag_index, bag_path in enumerate(bag_paths):
        node.get_logger().info(f"TrafficSign: Processing bag [{bag_index}]: {bag_path}")
        storage_options = StorageOptions(uri=bag_path,
                                         storage_id="sqlite3")

        converter_options = ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic == "/camera/image_raw":
                # time.sleep(1) # note --> without this the bb detect cant keep up need timer for node
                image_msg = deserialize_message(data, Image)
                node.image_callback(image_msg)

    node.destroy_node()
    rclpy.shutdown()



# def test_traffic_recognition():

#     rclpy.init(args=None)
#     node = TrafficSignRecognizer(debug=True, bb_only=False)
#     workdir = os.path.join(path, "ros2_bag_to_image/ros_bags")
#     bag_paths = [
#         os.path.join(workdir, "rosbag2_2025_03_20-16_49_11"),
#         os.path.join(workdir, "rosbag2_2025_03_20-16_54_08"),
#         os.path.join(workdir, "rosbag2_2025_03_20-16_56_50"),
#     ]

#     for bag_index, bag_path in enumerate(bag_paths):
#         node.get_logger().info(f"TrafficSign: Processing bag [{bag_index}]: {bag_path}")
#         storage_options = StorageOptions(bag_path,
#                                          storage_id="sqlite3")

#         converter_options = ConverterOptions(
#             input_serialization_format="cdr",
#             output_serialization_format="cdr"
#         )

#         reader = SequentialReader()
#         reader.open(storage_options, converter_options)

#         while reader.has_next():
#             topic, data, t = reader.read_next()
#             if topic == "/camera/image_raw":
#                 time.sleep(1)
#                 image_msg = deserialize_message(data, Image)
#                 node.image_callback(image_msg)


#     node.destroy_node()
#     rclpy.shutdown()


if __name__ == "__main__":
    pytest.main(['-v'])