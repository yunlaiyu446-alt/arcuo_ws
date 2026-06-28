#pragma once 

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class StaticPoseEKF{
    public:
        static constexpr int STATE_SIZE = 7;
        using StateVector = Eigen::Matrix<double, STATE_SIZE, 1>;
        using StateCovariance = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;
        using MeasurementVector = Eigen::Matrix<double, STATE_SIZE, 1>;
        using MeasurementCovariance = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;

        StaticPoseEKF(const StateCovariance& initial_covariance,
                    const StateCovariance& process_noise,const MeasurementCovariance& measurement_noise);
                    

        void initialize(const rclcpp::Time&, const StateVector& initial_state, const StateCovariance& initial_covariance);

        bool is_initialized() const;

        void predict();

        void update(const MeasurementVector& measurement);

        const StateVector& get_state() const;

        const StateCovariance& get_covariance() const;

        geometry_msgs::msg::TransformStamped getStateAsTransform(const std::string& parent_frame, const std::string& chind_frame, 
                                                                const rclcpp::Time& stamp) const;

    private:
        void normalizeQuaternion();

        StateVector state_; // 在构造函数中不初始化
        StateCovariance covariance_;
        StateCovariance process_noise_;
        MeasurementCovariance measurement_noise_;
        bool initialized_;
};
