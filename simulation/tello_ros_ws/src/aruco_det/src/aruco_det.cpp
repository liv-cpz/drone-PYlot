#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <yaml-cpp/yaml.h>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class ArucoDetectorNode : public rclcpp::Node
{
public:
    ArucoDetectorNode() : Node("aruco_det_node")
    {
        this->declare_parameter<int>("dictionary_id");
        this->declare_parameter<int>("aruco_id");
        this->declare_parameter<double>("aruco_length");
        this->declare_parameter<std::string>("camera_param_path");
        this->declare_parameter<std::string>("sub_image_topic");

        this->get_parameter("dictionary_id", dictionary_id_);
        this->get_parameter("aruco_id", aruco_id_);
        this->get_parameter("aruco_length", aruco_length_);
        this->get_parameter("camera_param_path", camera_param_path_);
        this->get_parameter("sub_image_topic", sub_image_topic_);
        if (sub_image_topic_.empty()) {
            sub_image_topic_ = "/image_raw";
        }

        YAML::Node config = YAML::LoadFile(camera_param_path_);
        double fx = config["fx"].as<double>();
        double fy = config["fy"].as<double>();
        double cx = config["x0"].as<double>();
        double cy = config["y0"].as<double>();
        double k1 = config["k1"].as<double>();
        double k2 = config["k2"].as<double>();
        double p1 = config["p1"].as<double>();
        double p2 = config["p2"].as<double>();
        double k3 = config["k3"].as<double>();

        camera_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                                    0, fy, cy,
                                                    0, 0, 1);
        dist_coeffs_ = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

        dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id_);
        parameters_ = cv::aruco::DetectorParameters::create();

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aruco_det_image", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("aruco/pose", 10);
        bbox_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("aruco_bbox", 10);
        visible_pub_ = this->create_publisher<std_msgs::msg::Bool>("aruco_visible", 10);

        auto qos = rclcpp::SensorDataQoS();
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            sub_image_topic_, qos,
            std::bind(&ArucoDetectorNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Aruco detector node started.");
        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", sub_image_topic_.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        if (frame.empty()) return;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
        cv::aruco::detectMarkers(gray, dictionary_, marker_corners, marker_ids, parameters_, rejected_candidates);

        std_msgs::msg::Bool visible_msg;
        visible_msg.data = false;

        if (!marker_ids.empty()) {
            RCLCPP_INFO(this->get_logger(), "Detected %ld markers", marker_ids.size());
            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(marker_corners, aruco_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            for (size_t i = 0; i < marker_ids.size(); ++i) {
                // if (marker_ids[i] != aruco_id_) continue;

                visible_msg.data = true;

                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = this->now();
                pose_msg.header.frame_id = "camera_frame";
                pose_msg.pose.position.x = tvecs[i][0];
                pose_msg.pose.position.y = tvecs[i][1];
                pose_msg.pose.position.z = tvecs[i][2];

                cv::Mat rot_mat;
                cv::Rodrigues(rvecs[i], rot_mat);
                double w = std::sqrt(1 + rot_mat.at<double>(0, 0) + rot_mat.at<double>(1, 1) + rot_mat.at<double>(2, 2)) / 2;
                double x = (rot_mat.at<double>(2, 1) - rot_mat.at<double>(1, 2)) / (4 * w);
                double y = (rot_mat.at<double>(0, 2) - rot_mat.at<double>(2, 0)) / (4 * w);
                double z = (rot_mat.at<double>(1, 0) - rot_mat.at<double>(0, 1)) / (4 * w);

                pose_msg.pose.orientation.w = w;
                pose_msg.pose.orientation.x = x;
                pose_msg.pose.orientation.y = y;
                pose_msg.pose.orientation.z = z;

                pose_pub_->publish(pose_msg);

                float half_len = static_cast<float>(aruco_length_ / 2.0);
                std::vector<cv::Point3f> obj_points = {
                    {-half_len,  half_len, 0.0f},
                    { half_len,  half_len, 0.0f},
                    { half_len, -half_len, 0.0f},
                    {-half_len, -half_len, 0.0f}
                };

                std::vector<cv::Point3f> corners_3d;
                for (const auto& pt : obj_points) {
                    cv::Mat pt_mat = (cv::Mat_<double>(3,1) << pt.x, pt.y, pt.z);
                    cv::Mat cam_pt = rot_mat * pt_mat + cv::Mat(tvecs[i]);
                    corners_3d.emplace_back(cam_pt.at<double>(0), cam_pt.at<double>(1), cam_pt.at<double>(2));
                }

                float x_min = corners_3d[0].x, y_min = corners_3d[0].y;
                float x_max = corners_3d[0].x, y_max = corners_3d[0].y;
                for (const auto& p : corners_3d) {
                    x_min = std::min(x_min, p.x);
                    y_min = std::min(y_min, p.y);
                    x_max = std::max(x_max, p.x);
                    y_max = std::max(y_max, p.y);
                }

                std_msgs::msg::Float32MultiArray bbox_msg;
                bbox_msg.data = {x_min, y_min, x_max, y_max};
                bbox_pub_->publish(bbox_msg);

                break;
            }
        }
        visible_pub_->publish(visible_msg);

        cv_bridge::CvImage out_msg;
        out_msg.encoding = "bgr8";
        out_msg.image = frame;
        sensor_msgs::msg::Image ros2_msg = *out_msg.toImageMsg();
        image_pub_->publish(ros2_msg);
    }

    int dictionary_id_;
    int aruco_id_;
    double aruco_length_;
    std::string camera_param_path_;
    std::string sub_image_topic_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visible_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    cv::Mat camera_matrix_, dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
