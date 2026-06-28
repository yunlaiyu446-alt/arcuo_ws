#include "cargo_locator/Kalman_filter.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

StaticPoseEKF::StaticPoseEKF(const StateCovariance& initial_covariance,
                             const StateCovariance& process_noise, const MeasurementCovariance& measurement_noise)
                            :covariance_(initial_covariance), 
                            process_noise_(process_noise), measurement_noise_(measurement_noise),
                            initialized_(false){

                            }

void StaticPoseEKF::initialize(const rclcpp::Time&, const StateVector& initial_state, const StateCovariance& initial_covariance){
    state_ = initial_state;
    covariance_ = initial_covariance;
    normalizeQuaternion();
    initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("ekf"), "EKF 已使用首次检测初始化。");
}

bool StaticPoseEKF::is_initialized() const {return initialized_; }

void StaticPoseEKF::predict(){
    if (!initialized_) return;
    covariance_ += process_noise_;
}

void StaticPoseEKF::update(const MeasurementVector& measurement){

    if (!initialized_) {
             RCLCPP_WARN(rclcpp::get_logger("ekf"), "EKF 未初始化，无法更新。");
             return;
        }

    // 观测矩阵H
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> H = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();

    // S = H * P * H_{T}
    MeasurementCovariance S = H * covariance_ * H.transpose() + measurement_noise_;
    // 卡尔曼增益K
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> K = covariance_ * H.transpose() * S.inverse();

    // 计算y
    MeasurementVector y = measurement - H * state_;

    // 更新状态
    state_ += K * y;

    // 更新协方差
    covariance_ = (Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity() - K * H) * covariance_;

    // 至关重要：重新归一化状态向量中的四元数部分
        normalizeQuaternion();
}

const StaticPoseEKF::StateVector& StaticPoseEKF::get_state() const{ return state_; }

const StaticPoseEKF::StateCovariance& StaticPoseEKF::get_covariance() const { return covariance_; }

geometry_msgs::msg::TransformStamped StaticPoseEKF::getStateAsTransform (const std::string& parent_frame, const std::string& child_frame, 
                                                                const rclcpp::Time& stamp) const {
    // 定义坐标系变量ts
    geometry_msgs::msg::TransformStamped ts;

    // 设置好ts时间戳和坐标系变换
    ts.header.stamp = stamp;
    ts.header.frame_id = parent_frame;
    ts.child_frame_id = child_frame;

    // 设置ts的平移部分
    ts.transform.translation.x = state_(0);
    ts.transform.translation.y = state_(1);
    ts.transform.translation.z = state_(2);

    //设置好ts的旋转部分
    ts.transform.rotation.x = state_(3);
    ts.transform.rotation.y = state_(4);
    ts.transform.rotation.z = state_(5);
    ts.transform.rotation.w = state_(6);

    tf2::Quaternion q(ts.transform.rotation.x, ts.transform.rotation.y,
                      ts.transform.rotation.z, ts.transform.rotation.w);
    q.normalize();
    
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();

    return ts;
}

void StaticPoseEKF::normalizeQuaternion(){

    // 计算归一化系数
    double norm = sqrt(state_(3) * state_(3) + state_(4) * state_(4) + state_(5) * state_(5) + state_(6) * state_(6));

    // 进行归一化
    if(norm > 1e-12){
        state_(3) /= norm;
        state_(4) /= norm;
        state_(5) /= norm;
        state_(6) /= norm;
    } else{
        state_(3) = 0;
        state_(4) = 0;
        state_(5) = 0;
        state_(6) = 1;
        RCLCPP_WARN(rclcpp::get_logger("ekf"), "四元数范数接近零，重置为单位四元数。");
    }
}
