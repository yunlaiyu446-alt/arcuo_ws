#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <deque>
#include <functional>
#include <optional>
// 包含 Eigen 用于矩阵运算
#include <Eigen/Dense>
// 包含 TF2 用于四元数运算
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "aruco_interfaces/msg/marker_array.hpp" // 使用 aruco_interfaces
#include "cargo_locator/Kalman_filter.h"

using std::placeholders::_1;
using Marker = aruco_interfaces::msg::Marker;
using MarkerArray = aruco_interfaces::msg::MarkerArray;// 使用 aruco_interfaces


// 辅助结构体，用于存储货物框架中标记的位置（假设没有相对旋转）
struct MarkerPosition {
    int id;
    double x, y, z; // 标记中心在货物框架中的位置

    // 创建逆变换 T_marker_cargo（假设标记相对于货物的方向为单位矩阵）
    geometry_msgs::msg::TransformStamped toInverseTransform(const std::string& cargo_frame) const {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.frame_id = cargo_frame + "_marker_" + std::to_string(id); // 标记框架
        tf_msg.child_frame_id = cargo_frame; // 货物框架
        // 逆向平移
        tf_msg.transform.translation.x = -x;
        tf_msg.transform.translation.y = -y;
        tf_msg.transform.translation.z = -z;
        // 单位四元数旋转（标记和货物具有相同的方向）
        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;
        return tf_msg;
    }
};

class CargoLocatorNode : public rclcpp::Node
{
public:
    // 构造函数
    CargoLocatorNode() : Node("cargo_locator_node_ekf"), ekf_initialized_(false)
    {
        // 声明参数
        this->declare_parameter<std::string>("marker_topic", "/aruco_markers");
        this->declare_parameter<std::string>("camera_frame_id", "camera_color_optical_frame");
        this->declare_parameter<std::string>("cargo_frame_id", "cargo");
        this->declare_parameter<std::string>("marker_frame_id_prefix", "aruco_marker_");
        this->declare_parameter<std::string>("hanger_pose_topic", "/cargo_locator/hanger_pose_in_cargo");
        this->declare_parameter<std::vector<double>>("hanger_position_in_camera_frame", std::vector<double>({0.0, 0.105, 0.15}));
        this->declare_parameter<bool>("enable_fusion", true);
        this->declare_parameter<std::vector<std::string>>("marker_positions", std::vector<std::string>{});

        // EKF 参数 (初始状态现在来自首次检测)
        this->declare_parameter<std::vector<double>>("ekf_initial_covariance_diagonal", std::vector<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));
        this->declare_parameter<std::vector<double>>("ekf_process_noise_diagonal", std::vector<double>({1e-6, 1e-6, 1.0e-6, 1e-6, 1e-6, 1e-6, 1e-6}));
        this->declare_parameter<std::vector<double>>("ekf_measurement_noise_diagonal", std::vector<double>({0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01}));

        // --- 新增参数：状态重置 ---
        this->declare_parameter<double>("reset_position_threshold", 0.3); // 米
        this->declare_parameter<double>("reset_angle_threshold", 30.0);   // 度
        this->declare_parameter<bool>("enable_reset", true); // 是否启用重置功能
        this->declare_parameter<bool>("publish_predicted_pose_on_marker_loss", true);
        this->declare_parameter<double>("max_marker_loss_duration", 0.2); // 秒

        // 获取参数
        marker_topic_ = this->get_parameter("marker_topic").as_string();
        camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
        cargo_frame_id_ = this->get_parameter("cargo_frame_id").as_string();
        marker_frame_id_prefix_ = this->get_parameter("marker_frame_id_prefix").as_string();
        hanger_pose_topic_ = this->get_parameter("hanger_pose_topic").as_string();
        hanger_position_in_camera_frame_ = this->get_parameter("hanger_position_in_camera_frame").as_double_array();
        if (hanger_position_in_camera_frame_.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "hanger_position_in_camera_frame 参数必须包含 3 个值 [x, y, z]，将使用默认值 [0.0, 0.105, 0.15]。");
            hanger_position_in_camera_frame_ = {0.0, 0.105, 0.15};
        }
        enable_fusion_ = this->get_parameter("enable_fusion").as_bool();

        std::vector<std::string> marker_positions_strings = this->get_parameter("marker_positions").as_string_array();
        parse_marker_positions(marker_positions_strings);

        if (known_marker_positions_.empty()) {
            RCLCPP_WARN(this->get_logger(), "未加载已知标记位置。此节点将不会发布吊挂位姿数据。");
        }

        // 初始化 EKF 组件（不包含初始状态）
        auto ekf_init_cov_diag = this->get_parameter("ekf_initial_covariance_diagonal").as_double_array();
        auto ekf_proc_noise_diag = this->get_parameter("ekf_process_noise_diagonal").as_double_array();
        auto ekf_meas_noise_diag = this->get_parameter("ekf_measurement_noise_diagonal").as_double_array();

        // --- 获取新增参数：状态重置 ---
        reset_position_threshold_ = this->get_parameter("reset_position_threshold").as_double();
        reset_angle_threshold_ = this->get_parameter("reset_angle_threshold").as_double();
        enable_reset_ = this->get_parameter("enable_reset").as_bool();
        publish_predicted_pose_on_marker_loss_ = this->get_parameter("publish_predicted_pose_on_marker_loss").as_bool();
        max_marker_loss_duration_ = this->get_parameter("max_marker_loss_duration").as_double();

        // 检查参数向量大小
        if (ekf_init_cov_diag.size() != StaticPoseEKF::STATE_SIZE ||
            ekf_proc_noise_diag.size() != StaticPoseEKF::STATE_SIZE ||
            ekf_meas_noise_diag.size() != StaticPoseEKF::STATE_SIZE) {
             RCLCPP_ERROR(this->get_logger(), "EKF 参数向量大小必须为 %d。", StaticPoseEKF::STATE_SIZE);
             // 处理错误，例如关闭节点
        } else {
            // 初始化协方差矩阵
            StaticPoseEKF::StateCovariance initial_covariance = StaticPoseEKF::StateCovariance::Zero();
            for(int i = 0; i < StaticPoseEKF::STATE_SIZE; ++i) {
                 initial_covariance(i, i) = ekf_init_cov_diag[i];
            }

            // 初始化过程噪声矩阵
            StaticPoseEKF::StateCovariance process_noise = StaticPoseEKF::StateCovariance::Zero();
            for(int i = 0; i < StaticPoseEKF::STATE_SIZE; ++i) {
                 process_noise(i, i) = ekf_proc_noise_diag[i];
            }

            // 初始化测量噪声矩阵
            StaticPoseEKF::MeasurementCovariance measurement_noise = StaticPoseEKF::MeasurementCovariance::Zero();
            for(int i = 0; i < StaticPoseEKF::STATE_SIZE; ++i) {
                 measurement_noise(i, i) = ekf_meas_noise_diag[i];
            }

            // 创建 EKF 实例
            ekf_ = std::make_unique<StaticPoseEKF>(initial_covariance, process_noise, measurement_noise);
            RCLCPP_INFO(this->get_logger(), "6DoF EKF 已创建（等待首次检测以初始化）。");
        }

        // 初始化 TF 监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        hanger_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(hanger_pose_topic_, 10);

        // 创建订阅者
        marker_subscription_ = this->create_subscription<MarkerArray>(
            marker_topic_,
            10,
            std::bind(&CargoLocatorNode::marker_callback, this, _1)
        );

        // 打印节点初始化信息
        RCLCPP_INFO(this->get_logger(), "货物定位节点 (使用六自由度 EKF，初始位姿来自首次检测) 已初始化。");
        RCLCPP_INFO(this->get_logger(), "  - 标记话题: '%s'", marker_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - 相机框架: '%s'", camera_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - 货物框架: '%s'", cargo_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - 吊挂位姿话题: '%s'", hanger_pose_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - 吊挂在相机坐标系下的位置: (%.3f, %.3f, %.3f) m",
                    hanger_position_in_camera_frame_[0], hanger_position_in_camera_frame_[1], hanger_position_in_camera_frame_[2]);
        RCLCPP_INFO(this->get_logger(), "  - 融合使能: %s", enable_fusion_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  - 已加载的已知标记数量: %zu", known_marker_positions_.size());
        // --- 打印新增参数：状态重置 ---
        RCLCPP_INFO(this->get_logger(), "  - 状态重置使能: %s", enable_reset_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  - 状态重置位置阈值: %.3f m", reset_position_threshold_);
        RCLCPP_INFO(this->get_logger(), "  - 状态重置角度阈值: %.1f deg", reset_angle_threshold_);
        RCLCPP_INFO(this->get_logger(), "  - 短时丢失 marker 继续发布: %s", publish_predicted_pose_on_marker_loss_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  - marker 丢失最大保持时间: %.3f s", max_marker_loss_duration_);
    }

private:
    // 解析标记位置参数
    void parse_marker_positions(const std::vector<std::string>& position_strings) {
        known_marker_positions_.clear();
        for (const auto& pos_str : position_strings) {
            std::istringstream iss(pos_str);
            std::string token;
            std::vector<std::string> tokens;

            while(std::getline(iss, token, ',')) {
                tokens.push_back(token);
            }

            if (tokens.size() != 4) {
                RCLCPP_WARN(this->get_logger(), "无效的标记位置字符串格式 (期望 'id,x,y,z'): '%s'", pos_str.c_str());
                continue;
            }

            try {
                MarkerPosition mp;
                mp.id = std::stoi(tokens[0]);
                mp.x = std::stod(tokens[1]);
                mp.y = std::stod(tokens[2]);
                mp.z = std::stod(tokens[3]);

                known_marker_positions_[mp.id] = mp;

                RCLCPP_INFO(this->get_logger(), "已加载标记 %d: 位置(%.3f, %.3f, %.3f)",
                            mp.id, mp.x, mp.y, mp.z);

            } catch (const std::exception& e) {
                 RCLCPP_ERROR(this->get_logger(), "解析标记位置字符串 '%s' 时出错: %s", pos_str.c_str(), e.what());
            }
        }
    }

    geometry_msgs::msg::PoseStamped getHangerPoseInCargoFrame(const rclcpp::Time& stamp) const
    {
        geometry_msgs::msg::TransformStamped tf_camera_to_cargo =
            ekf_->getStateAsTransform(camera_frame_id_, cargo_frame_id_, stamp);

        tf2::Transform tf2_camera_to_cargo;
        tf2::fromMsg(tf_camera_to_cargo.transform, tf2_camera_to_cargo);

        tf2::Transform tf2_cargo_to_camera = tf2_camera_to_cargo.inverse();
        tf2::Transform tf2_camera_to_hanger(
            tf2::Quaternion::getIdentity(),
            tf2::Vector3(
                hanger_position_in_camera_frame_[0],
                hanger_position_in_camera_frame_[1],
                hanger_position_in_camera_frame_[2]));
        tf2::Transform tf2_cargo_to_hanger = tf2_cargo_to_camera * tf2_camera_to_hanger;

        geometry_msgs::msg::PoseStamped hanger_pose;
        hanger_pose.header.stamp = stamp;
        hanger_pose.header.frame_id = cargo_frame_id_;
        hanger_pose.pose.position.x = tf2_cargo_to_hanger.getOrigin().x();
        hanger_pose.pose.position.y = tf2_cargo_to_hanger.getOrigin().y();
        hanger_pose.pose.position.z = tf2_cargo_to_hanger.getOrigin().z();

        tf2::Quaternion q = tf2_cargo_to_hanger.getRotation();
        q.normalize();
        hanger_pose.pose.orientation.x = q.x();
        hanger_pose.pose.orientation.y = q.y();
        hanger_pose.pose.orientation.z = q.z();
        hanger_pose.pose.orientation.w = q.w();

        return hanger_pose;
    }

    geometry_msgs::msg::TransformStamped markerPoseToTransform(
        const MarkerArray& markers_msg,
        const Marker& marker) const
    {
        geometry_msgs::msg::TransformStamped tf_camera_to_marker;
        tf_camera_to_marker.header.stamp = markers_msg.header.stamp;
        tf_camera_to_marker.header.frame_id = markers_msg.header.frame_id.empty()
            ? camera_frame_id_
            : markers_msg.header.frame_id;
        tf_camera_to_marker.child_frame_id = marker_frame_id_prefix_ + std::to_string(marker.id);

        tf_camera_to_marker.transform.translation.x = marker.pose.position.x;
        tf_camera_to_marker.transform.translation.y = marker.pose.position.y;
        tf_camera_to_marker.transform.translation.z = marker.pose.position.z;

        tf2::Quaternion q(
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w);
        if (q.length2() > 1e-24) {
            q.normalize();
        } else {
            q = tf2::Quaternion::getIdentity();
            RCLCPP_WARN(this->get_logger(), "标记 %d 的姿态四元数无效，已使用单位四元数。", marker.id);
        }

        tf_camera_to_marker.transform.rotation.x = q.x();
        tf_camera_to_marker.transform.rotation.y = q.y();
        tf_camera_to_marker.transform.rotation.z = q.z();
        tf_camera_to_marker.transform.rotation.w = q.w();

        return tf_camera_to_marker;
    }

    void marker_callback(const MarkerArray::SharedPtr msg)
    {
        if (known_marker_positions_.empty() || !ekf_) {
            return;
        }

        if (msg->markers.empty()) {
            handle_marker_loss(msg->header.stamp);
            return;
        }

        if (!msg->header.frame_id.empty() && msg->header.frame_id != camera_frame_id_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "marker 消息坐标系 '%s' 与配置 camera_frame_id '%s' 不一致，将直接使用 marker.pose 的数值。",
                msg->header.frame_id.c_str(), camera_frame_id_.c_str());
        }

        // --- 使用首次检测初始化 EKF ---
        if (!ekf_initialized_) {
            RCLCPP_DEBUG(this->get_logger(), "EKF 尚未初始化。正在寻找首次有效检测...");
            for (const auto& marker : msg->markers) {
                int marker_id = marker.id;
                auto it = known_marker_positions_.find(marker_id);
                if (it != known_marker_positions_.end()) {
                    const MarkerPosition& marker_in_cargo = it->second;
                    geometry_msgs::msg::TransformStamped tf_camera_to_marker =
                        markerPoseToTransform(*msg, marker);

                    // 计算标记到货物的逆变换
                    geometry_msgs::msg::TransformStamped tf_marker_to_cargo = marker_in_cargo.toInverseTransform(cargo_frame_id_);
                    
                    // 将变换转换为 TF2 格式
                    tf2::Transform tf2_camera_to_marker, tf2_marker_to_cargo, tf2_camera_to_cargo_result;
                    tf2::fromMsg(tf_camera_to_marker.transform, tf2_camera_to_marker);
                    tf2::fromMsg(tf_marker_to_cargo.transform, tf2_marker_to_cargo);
                    // 计算相机到货物的变换: T_camera_cargo = T_camera_marker * T_marker_cargo
                    tf2_camera_to_cargo_result = tf2_camera_to_marker * tf2_marker_to_cargo;

                    // --- 使用此首次估计初始化 EKF ---
                    StaticPoseEKF::StateVector initial_state;
                    initial_state(0) = tf2_camera_to_cargo_result.getOrigin().x();
                    initial_state(1) = tf2_camera_to_cargo_result.getOrigin().y();
                    initial_state(2) = tf2_camera_to_cargo_result.getOrigin().z();
                    tf2::Quaternion q = tf2_camera_to_cargo_result.getRotation();
                    // 确保从 TF 获取的四元数在使用前已归一化
                    q.normalize();
                    initial_state(3) = q.x();
                    initial_state(4) = q.y();
                    initial_state(5) = q.z();
                    initial_state(6) = q.w();

                    // 使用配置的初始协方差
                    auto ekf_init_cov_diag = this->get_parameter("ekf_initial_covariance_diagonal").as_double_array();
                    StaticPoseEKF::StateCovariance initial_covariance = StaticPoseEKF::StateCovariance::Zero();
                    for(int i = 0; i < StaticPoseEKF::STATE_SIZE; ++i) {
                         initial_covariance(i, i) = ekf_init_cov_diag[i];
                    }

                    // 初始化 EKF
                    ekf_->initialize(msg->header.stamp, initial_state, initial_covariance);
                    ekf_initialized_ = true;
                    last_valid_marker_stamp_ = rclcpp::Time(msg->header.stamp);

                    RCLCPP_INFO(this->get_logger(), "EKF 已使用标记 ID %d 的首次检测进行初始化。", marker_id);

                    // 重要：使用第一个找到的标记初始化后，跳出循环
                    break;
                }
            }
            // 如果在此回调中未初始化 EKF，则返回并等待下一次检测
            if (!ekf_initialized_) {
                 RCLCPP_DEBUG(this->get_logger(), "在此回调中未找到有效的标记用于 EKF 初始化。");
                 return;
            }
        }


        // --- EKF 预测步骤 (仅在已初始化时执行) ---
        if (ekf_initialized_) {
            ekf_->predict();
        } else {
            return; // 等待初始化
        }

        // 存储候选测量值
        std::vector<StaticPoseEKF::MeasurementVector> candidate_measurements;

        // 遍历检测到的标记
        for (const auto& marker : msg->markers) {
            int marker_id = marker.id;

            auto it = known_marker_positions_.find(marker_id);
            if (it != known_marker_positions_.end()) {
                const MarkerPosition& marker_in_cargo = it->second;

                geometry_msgs::msg::TransformStamped tf_camera_to_marker =
                    markerPoseToTransform(*msg, marker);

                // 计算标记到货物的逆变换
                geometry_msgs::msg::TransformStamped tf_marker_to_cargo = marker_in_cargo.toInverseTransform(cargo_frame_id_);

                // 将变换转换为 TF2 格式并计算相机到货物的变换
                tf2::Transform tf2_camera_to_marker, tf2_marker_to_cargo, tf2_camera_to_cargo_result;
                tf2::fromMsg(tf_camera_to_marker.transform, tf2_camera_to_marker);
                tf2::fromMsg(tf_marker_to_cargo.transform, tf2_marker_to_cargo);
                tf2_camera_to_cargo_result = tf2_camera_to_marker * tf2_marker_to_cargo;

                // 构建候选 TF 消息
                geometry_msgs::msg::TransformStamped tf_camera_to_cargo_candidate;
                tf_camera_to_cargo_candidate.header.stamp = msg->header.stamp;
                tf_camera_to_cargo_candidate.header.frame_id = camera_frame_id_;
                tf_camera_to_cargo_candidate.child_frame_id = cargo_frame_id_;
                tf2::toMsg(tf2_camera_to_cargo_result, tf_camera_to_cargo_candidate.transform);

                // --- 为 EKF 准备测量值 ---
                StaticPoseEKF::MeasurementVector measurement;
                measurement(0) = tf_camera_to_cargo_candidate.transform.translation.x;
                measurement(1) = tf_camera_to_cargo_candidate.transform.translation.y;
                measurement(2) = tf_camera_to_cargo_candidate.transform.translation.z;
                measurement(3) = tf_camera_to_cargo_candidate.transform.rotation.x;
                measurement(4) = tf_camera_to_cargo_candidate.transform.rotation.y;
                measurement(5) = tf_camera_to_cargo_candidate.transform.rotation.z;
                measurement(6) = tf_camera_to_cargo_candidate.transform.rotation.w;

                // 在使用前归一化测量值中的四元数部分
                double meas_norm = sqrt(measurement(3)*measurement(3) + measurement(4)*measurement(4) +
                                        measurement(5)*measurement(5) + measurement(6)*measurement(6));
                if (meas_norm > 1e-12) {
                    measurement.segment<4>(3) /= meas_norm;
                } else {
                    measurement(3) = 0.0; measurement(4) = 0.0; measurement(5) = 0.0; measurement(6) = 1.0;
                     RCLCPP_WARN(this->get_logger(), "标记 %d 的测量四元数范数接近零。", marker_id);
                }

                // 输出每个标记估计出的货物与相机的位姿
                RCLCPP_INFO(this->get_logger(), "标记 %d 估计位姿: 位置(%.3f, %.3f, %.3f), 四元数(%.3f, %.3f, %.3f, %.3f)",
                            marker_id,
                            measurement(0), measurement(1), measurement(2),
                            measurement(3), measurement(4), measurement(5), measurement(6));

                // 添加到候选测量值列表，EKF 更新使用固定测量噪声参数
                candidate_measurements.push_back(measurement);

                // 如果未启用融合，则仅使用第一个标记进行 EKF 更新
                if (!enable_fusion_) {
                     RCLCPP_DEBUG(this->get_logger(), "融合已禁用。仅使用第一个标记 (ID %d) 进行 EKF 更新。", marker_id);
                     break;
                }
            }
        }

        // --- EKF 更新或重置步骤 ---
        if (!candidate_measurements.empty()) {

            // 选择一个测量值用于更新或重置决策 (这里选择第一个，也可以选择置信度最高的)
            const StaticPoseEKF::MeasurementVector& best_measurement = candidate_measurements.front();

            // --- 检查是否需要重置 EKF 状态 ---
            bool should_reset = false;
            if (ekf_initialized_ && enable_reset_) {
                StaticPoseEKF::StateVector current_state = ekf_->get_state();
                // 计算当前位置差异
                double reset_pos_diff = sqrt(
                    pow(best_measurement(0) - current_state(0), 2) +
                    pow(best_measurement(1) - current_state(1), 2) +
                    pow(best_measurement(2) - current_state(2), 2)
                );

                // 计算当前角度差异
                tf2::Quaternion q_meas(best_measurement(3), best_measurement(4), best_measurement(5), best_measurement(6));
                tf2::Quaternion q_state(current_state(3), current_state(4), current_state(5), current_state(6));
                double reset_dot_product = q_meas.dot(q_state);
                reset_dot_product = std::max(-1.0, std::min(1.0, reset_dot_product));
                double reset_angle_diff_rad = 2.0 * acos(std::abs(reset_dot_product));
                double reset_angle_diff_deg = reset_angle_diff_rad * (180.0 / M_PI);

                if (reset_pos_diff > reset_position_threshold_ || reset_angle_diff_deg > reset_angle_threshold_) {
                    RCLCPP_INFO(this->get_logger(), "EKF 状态与最佳测量值差异过大，触发重置: 位置差=%.3fm (阈值=%.3fm), 角度差=%.1fdeg (阈值=%.1fdeg)",
                                reset_pos_diff, reset_position_threshold_, reset_angle_diff_deg, reset_angle_threshold_);
                    should_reset = true;
                }
            }

            if (should_reset) {
                // --- 执行重置 ---
                // 使用最佳测量值作为新的 EKF 状态
                // 重置协方差到初始值（或一个较大的值，表示不确定性增加）
                auto ekf_init_cov_diag = this->get_parameter("ekf_initial_covariance_diagonal").as_double_array();
                StaticPoseEKF::StateCovariance reset_covariance = StaticPoseEKF::StateCovariance::Zero();
                for(int i = 0; i < StaticPoseEKF::STATE_SIZE; ++i) {
                     reset_covariance(i, i) = ekf_init_cov_diag[i];
                }
                ekf_->initialize(msg->header.stamp, best_measurement, reset_covariance); // initialize 会设置 initialized_ 为 true
                RCLCPP_INFO(this->get_logger(), "EKF 状态已重置。");

            } else if (enable_fusion_ && candidate_measurements.size() > 1) {
                // --- 融合所有候选测量值 (简单平均位置和四元数) ---
                Eigen::Vector3d fused_position = Eigen::Vector3d::Zero();
                Eigen::Quaterniond fused_orientation(1, 0, 0, 0); // 初始化为单位四元数
                Eigen::Matrix<double, 3, 3> rotation_sum = Eigen::Matrix<double, 3, 3>::Zero(); // 用于平均旋转的另一种方法

                size_t num_valid_measurements = candidate_measurements.size();
                for (const auto& meas : candidate_measurements) {
                    fused_position += Eigen::Vector3d(meas(0), meas(1), meas(2));
                    // 累加旋转矩阵进行平均
                    Eigen::Quaterniond q(meas(6), meas(3), meas(4), meas(5)); // w, x, y, z
                    q.normalize();
                    rotation_sum += q.toRotationMatrix();
                }
                fused_position /= static_cast<double>(num_valid_measurements);
                // 计算平均旋转矩阵的 SVD 来得到最接近的旋转矩阵
                Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd(rotation_sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::Matrix<double, 3, 3> R_avg = svd.matrixU() * svd.matrixV().transpose();
                // 如果行列式为 -1，则需要翻转符号以确保是有效的旋转矩阵
                if (R_avg.determinant() < 0) {
                    Eigen::Matrix<double, 3, 3> tmp = svd.matrixU();
                    tmp.col(2) *= -1; // 翻转第三列
                    R_avg = tmp * svd.matrixV().transpose();
                }
                fused_orientation = Eigen::Quaterniond(R_avg);
                fused_orientation.normalize();

                // 构建融合后的测量向量
                StaticPoseEKF::MeasurementVector fused_measurement;
                fused_measurement << fused_position, fused_orientation.x(), fused_orientation.y(), fused_orientation.z(), fused_orientation.w();

                RCLCPP_DEBUG(this->get_logger(), "融合 %zu 个标记估计值用于 EKF 更新。", num_valid_measurements);
                ekf_->update(fused_measurement);

            } else {
                // --- 使用单个测量值更新 (已在循环中处理，或这里处理单个情况) ---
                // 如果未启用融合或只有一个测量值，则使用该测量值更新
                RCLCPP_DEBUG(this->get_logger(), "使用单个有效标记测量值更新 EKF。");
                ekf_->update(best_measurement);
            }

            last_valid_marker_stamp_ = rclcpp::Time(msg->header.stamp);

        }


        // --- 发布当前 EKF 估计值：货物坐标系下的吊挂位姿数据 ---
        if (ekf_->is_initialized()) {
            geometry_msgs::msg::PoseStamped hanger_pose = getHangerPoseInCargoFrame(msg->header.stamp);
            hanger_pose_publisher_->publish(hanger_pose);
            RCLCPP_DEBUG(this->get_logger(), "已发布 EKF 估计的吊挂位姿数据: frame='%s', topic='%s'",
                         cargo_frame_id_.c_str(), hanger_pose_topic_.c_str());
        }
    }

    void handle_marker_loss(const builtin_interfaces::msg::Time& stamp_msg)
    {
        if (!publish_predicted_pose_on_marker_loss_ || !ekf_initialized_ || !ekf_->is_initialized() ||
            !last_valid_marker_stamp_) {
            return;
        }

        const rclcpp::Time stamp(stamp_msg);
        double loss_duration = (stamp - last_valid_marker_stamp_.value()).seconds();
        if (loss_duration < 0.0) {
            loss_duration = 0.0;
        }

        if (loss_duration > max_marker_loss_duration_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "marker 已丢失 %.3fs，超过保持时间 %.3fs，暂停发布吊挂位姿。",
                loss_duration, max_marker_loss_duration_);
            return;
        }

        ekf_->predict();
        geometry_msgs::msg::PoseStamped hanger_pose = getHangerPoseInCargoFrame(stamp);
        hanger_pose_publisher_->publish(hanger_pose);
        RCLCPP_DEBUG(this->get_logger(), "marker 短时丢失 %.3fs，已发布当前 EKF 估计吊挂位姿。", loss_duration);
    }

    // --- 成员变量 ---
    std::string marker_topic_;
    std::string camera_frame_id_;
    std::string cargo_frame_id_;
    std::string marker_frame_id_prefix_;
    std::string hanger_pose_topic_;
    std::vector<double> hanger_position_in_camera_frame_;
    bool enable_fusion_;

    std::map<int, MarkerPosition> known_marker_positions_;

    rclcpp::Subscription<MarkerArray>::SharedPtr marker_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr hanger_pose_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<StaticPoseEKF> ekf_;
    bool ekf_initialized_; // 标记 EKF 是否已初始化的标志

    double reset_position_threshold_; // 重置位置差异阈值 (米)
    double reset_angle_threshold_;    // 重置角度差异阈值 (度)
    bool enable_reset_;               // 是否启用重置功能
    bool publish_predicted_pose_on_marker_loss_;
    double max_marker_loss_duration_;
    std::optional<rclcpp::Time> last_valid_marker_stamp_;
};

// 主函数
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CargoLocatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
