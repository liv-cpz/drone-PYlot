// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2_ros/transform_broadcaster.h>


// class MirrorCorrectionNode : public rclcpp::Node
// {
// public:
//   MirrorCorrectionNode()
//   : Node("mirror_correction_node")
//   {
//     sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//       "/aruco/pose", 10,
//       std::bind(&MirrorCorrectionNode::pose_callback, this, std::placeholders::_1));

//     pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
//       "/aruco/pose_corrected", 10);

//     tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

//     RCLCPP_INFO(this->get_logger(), "Mirror Correction Node started.");
//   }

// private:
//   void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//   {
//     RCLCPP_INFO(this->get_logger(), "Received ArUco pose");

//     geometry_msgs::msg::PoseStamped corrected_msg = *msg;

//     // z to neg z
//     corrected_msg.pose.position.z = -corrected_msg.pose.position.z;

//     tf2::Quaternion q_orig;
//     tf2::fromMsg(msg->pose.orientation, q_orig);
//     double roll, pitch, yaw;
//     tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

//     // yaw to neg yaw (or pitch??)
//     yaw = -yaw;

//     tf2::Quaternion q_corrected;
//     q_corrected.setRPY(roll, pitch, yaw);
//     q_corrected.normalize();
//     corrected_msg.pose.orientation = tf2::toMsg(q_corrected);

//     pub_->publish(corrected_msg);

//     // --- TF transform ---
//     geometry_msgs::msg::TransformStamped transform;
//     transform.header.stamp = this->now();
//     transform.header.frame_id = "camera_link";              // base frame
//     transform.child_frame_id = "aruco_corrected";           // tf name

//     transform.transform.translation.x = corrected_msg.pose.position.x;
//     transform.transform.translation.y = corrected_msg.pose.position.y;
//     transform.transform.translation.z = corrected_msg.pose.position.z;
//     transform.transform.rotation = corrected_msg.pose.orientation;

//     tf_broadcaster_->sendTransform(transform);
//   }

//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
//   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
//   std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MirrorCorrectionNode>());
//   rclcpp::shutdown();
//   return 0;
// }

// // #include <rclcpp/rclcpp.hpp>
// // #include <geometry_msgs/msg/transform_stamped.hpp>
// // #include <tf2_ros/transform_broadcaster.h>

// // class TestTfNode : public rclcpp::Node {
// // public:
// //   TestTfNode() : Node("test_tf_node") {
// //     tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
// //     timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
// //       std::bind(&TestTfNode::publish_tf, this));
// //   }

// // private:
// //   void publish_tf() {
// //     geometry_msgs::msg::TransformStamped transform;
// //     transform.header.stamp = this->now();
// //     transform.header.frame_id = "base_link";
// //     transform.child_frame_id = "test_frame";
// //     transform.transform.translation.x = 1.0;
// //     transform.transform.rotation.w = 1.0;
// //     tf_broadcaster_->sendTransform(transform);  
// //   }

// //   std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
// //   rclcpp::TimerBase::SharedPtr timer_;
// // };
