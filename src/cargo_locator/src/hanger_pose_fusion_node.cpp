#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using PoseStamped = geometry_msgs::msg::PoseStamped;

class HangerPoseFusionNode : public rclcpp::Node
{
public:
    HangerPoseFusionNode() : Node("hanger_pose_fusion_node")
    {
        this->declare_parameter<std::string>("left_pose_topic", "/left/hanger_pose_in_cargo");
        this->declare_parameter<std::string>("right_pose_topic", "/right/hanger_pose_in_cargo");
        this->declare_parameter<std::string>("output_pose_topic", "/hanger_pose_in_cargo");
        this->declare_parameter<std::string>("output_frame_id", "cargo");
        this->declare_parameter<double>("max_sync_time_diff", 0.05);
        this->declare_parameter<double>("position_consistency_threshold", 0.05);
        this->declare_parameter<double>("angle_consistency_threshold", 15.0);
        this->declare_parameter<double>("left_weight", 1.0);
        this->declare_parameter<double>("right_weight", 1.0);
        this->declare_parameter<bool>("publish_single_observation", true);

        left_pose_topic_ = this->get_parameter("left_pose_topic").as_string();
        right_pose_topic_ = this->get_parameter("right_pose_topic").as_string();
        output_pose_topic_ = this->get_parameter("output_pose_topic").as_string();
        output_frame_id_ = this->get_parameter("output_frame_id").as_string();
        max_sync_time_diff_ = this->get_parameter("max_sync_time_diff").as_double();
        position_consistency_threshold_ = this->get_parameter("position_consistency_threshold").as_double();
        angle_consistency_threshold_ = this->get_parameter("angle_consistency_threshold").as_double();
        left_weight_ = this->get_parameter("left_weight").as_double();
        right_weight_ = this->get_parameter("right_weight").as_double();
        publish_single_observation_ = this->get_parameter("publish_single_observation").as_bool();

        if (left_weight_ < 0.0 || right_weight_ < 0.0 || (left_weight_ + right_weight_) <= 1e-12) {
            RCLCPP_WARN(this->get_logger(), "左右权重无效，已重置为 1.0 / 1.0。");
            left_weight_ = 1.0;
            right_weight_ = 1.0;
        }

        fused_pose_publisher_ = this->create_publisher<PoseStamped>(output_pose_topic_, 10);
        left_pose_subscription_ = this->create_subscription<PoseStamped>(
            left_pose_topic_, 10,
            [this](const PoseStamped::SharedPtr msg) {
                latest_left_pose_ = *msg;
                fuseAndPublish(Source::Left);
            });
        right_pose_subscription_ = this->create_subscription<PoseStamped>(
            right_pose_topic_, 10,
            [this](const PoseStamped::SharedPtr msg) {
                latest_right_pose_ = *msg;
                fuseAndPublish(Source::Right);
            });

        RCLCPP_INFO(this->get_logger(), "吊挂双相机位姿融合节点已启动。");
        RCLCPP_INFO(this->get_logger(), "  - 左输入: '%s'", left_pose_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - 右输入: '%s'", right_pose_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - 输出: '%s'", output_pose_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - 输出坐标系: '%s'", output_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - 时间窗口: %.3f s", max_sync_time_diff_);
        RCLCPP_INFO(this->get_logger(), "  - 位置一致性阈值: %.3f m", position_consistency_threshold_);
        RCLCPP_INFO(this->get_logger(), "  - 角度一致性阈值: %.1f deg", angle_consistency_threshold_);
        RCLCPP_INFO(this->get_logger(), "  - 左右权重: %.3f / %.3f", left_weight_, right_weight_);
    }

private:
    enum class Source
    {
        Left,
        Right
    };

    void fuseAndPublish(Source source)
    {
        const PoseStamped& current_pose = source == Source::Left ? latest_left_pose_.value() : latest_right_pose_.value();

        if (!latest_left_pose_ || !latest_right_pose_) {
            publishSinglePose(current_pose, source);
            return;
        }

        const PoseStamped& left_pose = latest_left_pose_.value();
        const PoseStamped& right_pose = latest_right_pose_.value();
        const double time_diff = std::abs((rclcpp::Time(left_pose.header.stamp) -
                                           rclcpp::Time(right_pose.header.stamp)).seconds());

        if (time_diff > max_sync_time_diff_) {
            RCLCPP_DEBUG(this->get_logger(), "左右吊挂位姿时间差 %.3fs 超过窗口 %.3fs，使用最新单路观测。",
                         time_diff, max_sync_time_diff_);
            publishSinglePose(current_pose, source);
            return;
        }

        const double pos_diff = positionDistance(left_pose, right_pose);
        const double angle_diff = orientationDistanceDeg(left_pose, right_pose);
        if (pos_diff > position_consistency_threshold_ || angle_diff > angle_consistency_threshold_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "左右吊挂位姿不一致，位置差=%.3fm(阈值=%.3fm)，角度差=%.1fdeg(阈值=%.1fdeg)，使用最新单路观测。",
                pos_diff, position_consistency_threshold_, angle_diff, angle_consistency_threshold_);
            publishSinglePose(current_pose, source);
            return;
        }

        PoseStamped fused_pose = fusePose(left_pose, right_pose);
        fused_pose_publisher_->publish(fused_pose);
        RCLCPP_DEBUG(this->get_logger(), "已融合左右吊挂位姿并发布。");
    }

    void publishSinglePose(const PoseStamped& pose, Source source)
    {
        if (!publish_single_observation_) {
            return;
        }

        PoseStamped output_pose = pose;
        output_pose.header.frame_id = output_frame_id_;
        normalizeOrientation(output_pose);
        fused_pose_publisher_->publish(output_pose);

        RCLCPP_DEBUG(this->get_logger(), "已发布%s单路吊挂位姿。",
                     source == Source::Left ? "左" : "右");
    }

    PoseStamped fusePose(const PoseStamped& left_pose, const PoseStamped& right_pose) const
    {
        PoseStamped fused_pose;
        const rclcpp::Time left_stamp(left_pose.header.stamp);
        const rclcpp::Time right_stamp(right_pose.header.stamp);
        fused_pose.header.stamp = left_stamp.nanoseconds() >= right_stamp.nanoseconds()
            ? left_pose.header.stamp
            : right_pose.header.stamp;
        fused_pose.header.frame_id = output_frame_id_;

        const double weight_sum = left_weight_ + right_weight_;
        const double wl = left_weight_ / weight_sum;
        const double wr = right_weight_ / weight_sum;

        fused_pose.pose.position.x = wl * left_pose.pose.position.x + wr * right_pose.pose.position.x;
        fused_pose.pose.position.y = wl * left_pose.pose.position.y + wr * right_pose.pose.position.y;
        fused_pose.pose.position.z = wl * left_pose.pose.position.z + wr * right_pose.pose.position.z;

        tf2::Quaternion q_left = normalizedQuaternion(left_pose);
        tf2::Quaternion q_right = normalizedQuaternion(right_pose);
        if (q_left.dot(q_right) < 0.0) {
            q_right = tf2::Quaternion(-q_right.x(), -q_right.y(), -q_right.z(), -q_right.w());
        }

        tf2::Quaternion q_fused(
            wl * q_left.x() + wr * q_right.x(),
            wl * q_left.y() + wr * q_right.y(),
            wl * q_left.z() + wr * q_right.z(),
            wl * q_left.w() + wr * q_right.w());
        normalizeQuaternion(q_fused);

        fused_pose.pose.orientation.x = q_fused.x();
        fused_pose.pose.orientation.y = q_fused.y();
        fused_pose.pose.orientation.z = q_fused.z();
        fused_pose.pose.orientation.w = q_fused.w();

        return fused_pose;
    }

    static double positionDistance(const PoseStamped& a, const PoseStamped& b)
    {
        const double dx = a.pose.position.x - b.pose.position.x;
        const double dy = a.pose.position.y - b.pose.position.y;
        const double dz = a.pose.position.z - b.pose.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    static double orientationDistanceDeg(const PoseStamped& a, const PoseStamped& b)
    {
        tf2::Quaternion q_a = normalizedQuaternion(a);
        tf2::Quaternion q_b = normalizedQuaternion(b);
        const double dot = std::clamp(std::abs(q_a.dot(q_b)), 0.0, 1.0);
        return 2.0 * std::acos(dot) * 180.0 / M_PI;
    }

    static tf2::Quaternion normalizedQuaternion(const PoseStamped& pose)
    {
        tf2::Quaternion q(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w);
        normalizeQuaternion(q);
        return q;
    }

    static void normalizeOrientation(PoseStamped& pose)
    {
        tf2::Quaternion q = normalizedQuaternion(pose);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
    }

    static void normalizeQuaternion(tf2::Quaternion& q)
    {
        const double norm = std::sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w());
        if (norm > 1e-12) {
            q = tf2::Quaternion(q.x() / norm, q.y() / norm, q.z() / norm, q.w() / norm);
        } else {
            q = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
        }
    }

    std::string left_pose_topic_;
    std::string right_pose_topic_;
    std::string output_pose_topic_;
    std::string output_frame_id_;
    double max_sync_time_diff_;
    double position_consistency_threshold_;
    double angle_consistency_threshold_;
    double left_weight_;
    double right_weight_;
    bool publish_single_observation_;

    std::optional<PoseStamped> latest_left_pose_;
    std::optional<PoseStamped> latest_right_pose_;

    rclcpp::Subscription<PoseStamped>::SharedPtr left_pose_subscription_;
    rclcpp::Subscription<PoseStamped>::SharedPtr right_pose_subscription_;
    rclcpp::Publisher<PoseStamped>::SharedPtr fused_pose_publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HangerPoseFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
