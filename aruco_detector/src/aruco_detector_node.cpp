#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <iostream>
#include <chrono>
#include <functional>

// ROS2
#include "rclcpp/rclcpp.hpp"
// #include "image_transport/image_transport.hpp" // Completely removed image_transport
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "aruco_interfaces/msg/marker.hpp"
#include "aruco_interfaces/msg/marker_array.hpp" // Corrected namespace

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using Marker = aruco_interfaces::msg::Marker;
using MarkerArray = aruco_interfaces::msg::MarkerArray; // Corrected namespace

class ArucoDetectorNode : public rclcpp::Node
{
public:
    ArucoDetectorNode() : Node("aruco_detector_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("aruco_dictionary_name", "DICT_APRILTAG_36h11");
        this->declare_parameter<double>("marker_size", 0.1);
        this->declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
        this->declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
        this->declare_parameter<bool>("image_is_rectified", false);
        this->declare_parameter<std::string>("marker_topic", "aruco_markers");
        this->declare_parameter<std::string>("marker_tf_prefix", "aruco_marker_");
        this->declare_parameter<std::string>("debug_image_topic", "aruco_debug_image");

        // Get parameters
        this->get_parameter("aruco_dictionary_name", aruco_dictionary_name_);
        this->get_parameter("marker_size", marker_size_);
        image_topic_ = this->get_parameter("image_topic").as_string();
        camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
        image_is_rectified_ = this->get_parameter("image_is_rectified").as_bool();
        marker_topic_ = this->get_parameter("marker_topic").as_string();
        marker_tf_prefix_ = this->get_parameter("marker_tf_prefix").as_string();
        debug_image_topic_ = this->get_parameter("debug_image_topic").as_string();

        if (!initializeArucoDictionary()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize ArUco dictionary '%s'", aruco_dictionary_name_.c_str());
            return;
        }

        detector_params_ = cv::makePtr<cv::aruco::DetectorParameters>();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        markers_publisher_ = this->create_publisher<MarkerArray>(marker_topic_, 10);

        camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&ArucoDetectorNode::camera_info_callback, this, std::placeholders::_1)
        );
        
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&ArucoDetectorNode::image_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribing to image topic (rclcpp native): %s", image_topic_.c_str());

        debug_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, 10); 
        RCLCPP_INFO(this->get_logger(), "Publishing debug image on topic (rclcpp native): %s", debug_image_topic_.c_str());

        // --- Diagnostic Timer ---
        // Corrected: Do not capture member variables directly in lambda init-capture list.
        // Access them via 'this->' inside the lambda body.

        RCLCPP_INFO(this->get_logger(), "ArucoDetectorNode (pure rclcpp) has been initialized.");
    }

private:
    bool initializeArucoDictionary()
    {
        const auto& dict_map = getArucoDictionaryMap();
        auto it = dict_map.find(aruco_dictionary_name_);
        if (it != dict_map.end()) {
            dictionary_ = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(it->second));
            return true;
        }
        return false;
    }

    static const std::unordered_map<std::string, int>& getArucoDictionaryMap() {
        static std::unordered_map<std::string, int> dict_map = {
            {"DICT_4X4_50", static_cast<int>(cv::aruco::DICT_4X4_50)},
            {"DICT_4X4_100", static_cast<int>(cv::aruco::DICT_4X4_100)},
            {"DICT_4X4_250", static_cast<int>(cv::aruco::DICT_4X4_250)},
            {"DICT_4X4_1000", static_cast<int>(cv::aruco::DICT_4X4_1000)},
            {"DICT_5X5_50", static_cast<int>(cv::aruco::DICT_5X5_50)},
            {"DICT_5X5_100", static_cast<int>(cv::aruco::DICT_5X5_100)},
            {"DICT_5X5_250", static_cast<int>(cv::aruco::DICT_5X5_250)},
            {"DICT_5X5_1000", static_cast<int>(cv::aruco::DICT_5X5_1000)},
            {"DICT_6X6_50", static_cast<int>(cv::aruco::DICT_6X6_50)},
            {"DICT_6X6_100", static_cast<int>(cv::aruco::DICT_6X6_100)},
            {"DICT_6X6_250", static_cast<int>(cv::aruco::DICT_6X6_250)},
            {"DICT_6X6_1000", static_cast<int>(cv::aruco::DICT_6X6_1000)},
            {"DICT_7X7_50", static_cast<int>(cv::aruco::DICT_7X7_50)},
            {"DICT_7X7_100", static_cast<int>(cv::aruco::DICT_7X7_100)},
            {"DICT_7X7_250", static_cast<int>(cv::aruco::DICT_7X7_250)},
            {"DICT_7X7_1000", static_cast<int>(cv::aruco::DICT_7X7_1000)},
            {"DICT_ARUCO_ORIGINAL", static_cast<int>(cv::aruco::DICT_ARUCO_ORIGINAL)},
            {"DICT_APRILTAG_36h11", static_cast<int>(cv::aruco::DICT_APRILTAG_36h11)}
        };
        return dict_map;
    }

    // 添加相机信息回调函数
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!camera_info_received_) {
            camera_matrix_ = cv::Mat(3, 3, CV_64F);
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    camera_matrix_.at<double>(i,j) = msg->k[i*3 + j];
                }
            }

            dist_coeffs_ = cv::Mat(1, msg->d.size(), CV_64F);
            for (size_t i = 0; i < msg->d.size(); i++) {
                dist_coeffs_.at<double>(0,i) = msg->d[i];
            }

            camera_info_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera calibration received from topic: %s", camera_info_topic_.c_str());
            camera_info_subscription_.reset();
            RCLCPP_INFO(this->get_logger(), "Camera info subscription closed after initial calibration message");
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        RCLCPP_DEBUG(this->get_logger(), "image_callback called (pure rclcpp) with timestamp: %u.%09u", msg->header.stamp.sec, msg->header.stamp.nanosec);
        try {

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image_copy = cv_ptr->image.clone();

            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;
            cv::aruco::detectMarkers(image_copy, dictionary_, marker_corners, marker_ids, detector_params_);

            bool markers_found = !marker_ids.empty();

            if (markers_found) {
                cv::aruco::drawDetectedMarkers(image_copy, marker_corners, marker_ids);

                for (size_t i = 0; i < marker_ids.size(); ++i) {
                    RCLCPP_DEBUG(this->get_logger(), "Detected marker ID %d", marker_ids[i]);
                }

                if (!camera_info_received_) {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000,
                        "Markers detected but camera_info has not been received on %s yet; pose estimation skipped",
                        camera_info_topic_.c_str()
                    );
                } else {
                    auto markers_msg = MarkerArray();
                    markers_msg.header = msg->header;

                    cv::Mat dist_coeffs = dist_coeffs_.clone();
                    if (image_is_rectified_) {
                        dist_coeffs = cv::Mat::zeros(dist_coeffs_.size(), dist_coeffs_.type());
                    }

                    std::vector<cv::Vec3d> rvecs, tvecs;
                    cv::aruco::estimatePoseSingleMarkers(
                        marker_corners,
                        marker_size_,
                        camera_matrix_,
                        dist_coeffs,
                        rvecs,
                        tvecs
                    );

                    for (size_t i = 0; i < marker_ids.size(); ++i) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Marker ID %d: tvec = [%.3f, %.3f, %.3f] (meters)",
                            marker_ids[i], tvecs[i][0], tvecs[i][1], tvecs[i][2]);
                    }

                    for (size_t i = 0; i < marker_ids.size(); ++i) {
                        cv::drawFrameAxes(image_copy, camera_matrix_, dist_coeffs, rvecs[i], tvecs[i], marker_size_ * 0.5f);

                        Marker marker_msg;
                        marker_msg.id = marker_ids[i];
                        marker_msg.pose.position.x = tvecs[i][0];
                        marker_msg.pose.position.y = tvecs[i][1];
                        marker_msg.pose.position.z = tvecs[i][2];

                        cv::Mat rotation_matrix;
                        cv::Rodrigues(rvecs[i], rotation_matrix);

                        tf2::Matrix3x3 tf2_matrix(
                            rotation_matrix.at<double>(0,0), rotation_matrix.at<double>(0,1), rotation_matrix.at<double>(0,2),
                            rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(1,1), rotation_matrix.at<double>(1,2),
                            rotation_matrix.at<double>(2,0), rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2)
                        );

                        tf2::Quaternion tf2_quat;
                        tf2_matrix.getRotation(tf2_quat);

                        marker_msg.pose.orientation.x = tf2_quat.x();
                        marker_msg.pose.orientation.y = tf2_quat.y();
                        marker_msg.pose.orientation.z = tf2_quat.z();
                        marker_msg.pose.orientation.w = tf2_quat.w();

                        markers_msg.markers.push_back(marker_msg);

                        geometry_msgs::msg::TransformStamped t;
                        t.header.stamp = msg->header.stamp;
                        t.header.frame_id = msg->header.frame_id;
                        t.child_frame_id = marker_tf_prefix_ + std::to_string(marker_ids[i]);
                        t.transform.translation.x = tvecs[i][0];
                        t.transform.translation.y = tvecs[i][1];
                        t.transform.translation.z = tvecs[i][2];
                        t.transform.rotation.x = tf2_quat.x();
                        t.transform.rotation.y = tf2_quat.y();
                        t.transform.rotation.z = tf2_quat.z();
                        t.transform.rotation.w = tf2_quat.w();
                        tf_broadcaster_->sendTransform(t);
                    }

                    markers_publisher_->publish(markers_msg);
                    RCLCPP_DEBUG(this->get_logger(), "Published %zu markers.", marker_ids.size());
                }
            } else {
                 RCLCPP_DEBUG(this->get_logger(), "No markers detected.");
            }

            // 在图像中心绘制一个点
            int center_x = image_copy.cols / 2;
            int center_y = image_copy.rows / 2;
            cv::circle(image_copy, cv::Point(center_x, center_y), 3, cv::Scalar(0, 0, 255), -1);

            // Always publish debug image using rclcpp native publisher
            // Corrected: Use the correctly named publisher variable and pass CvImage directly
            cv_bridge::CvImage debug_img_bridge;
            debug_img_bridge.header = msg->header; // Use the same header as input
            debug_img_bridge.encoding = sensor_msgs::image_encodings::BGR8;
            debug_img_bridge.image = image_copy;

            // 修改发布调试图像的方式
            sensor_msgs::msg::Image debug_img_msg;
            debug_img_bridge.toImageMsg(debug_img_msg);  // 先转换为ROS消息
            debug_image_publisher_->publish(debug_img_msg);  // 然后发布

            RCLCPP_DEBUG(this->get_logger(), "Debug image published (rclcpp native).");

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Standard exception in image_callback: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Unknown exception in image_callback");
        }
    }

    // --- Member variables ---
    std::string aruco_dictionary_name_;
    double marker_size_;
    std::string image_topic_;
    std::string camera_info_topic_;
    bool image_is_rectified_;
    std::string marker_topic_;
    std::string marker_tf_prefix_;
    std::string debug_image_topic_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<MarkerArray>::SharedPtr markers_publisher_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_; 

    // 添加相机信息相关成员变量
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool camera_info_received_ = false;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

