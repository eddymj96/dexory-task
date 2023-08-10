#ifndef DEX_CONTROLLER__SIMPLE_PID_HPP_
#define DEX_CONTROLLER__SIMPLE_PID_HPP_

#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <optional>


namespace dex_controller
{

/**
 * @class dex_controller::SimplePID
 * @brief Simple PID controller
 * @tparam T Floating point type
*/

template<typename T>
class SimplePID
{
private:
    T kp_, ki_, kd_;
    T setpoint_;
    T integral_ = 0;
    T windup_limit_ = std::numeric_limits<T>::max();

public:

    SimplePID() = default;

    /**
     * @brief Constructor for dex_controller::SimplePID
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param windup_limit Windup limit for integral term
     * @param setpoint Setpoint for controller
     * @param logger Optional logger for debugging
    */

    SimplePID(const T kp, const T ki, const T kd, const T windup_limit, const T setpoint)
        : kp_(kp_), ki_(ki_), kd_(kd_), setpoint_(setpoint), windup_limit_(windup_limit), integral_(0) {};

    
    /**
     * @brief Set gains for controller
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
    */
    void setGains(const T kp, const T ki, const T kd) 
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;   
    }

    /**
     * @brief Set setpoint for controller
     * @param setpoint Setpoint for controller
    */

    void setSetpoint(const T setpoint) 
    {
        setpoint_ = setpoint;
    }

    /**
     * @brief Set windup limit for controller
     * @param windup_limit Windup limit for integral term
    */

    void setWindUpLimit(const T windup_limit) 
    {
        windup_limit_ = windup_limit;
    }

    /**
     * @brief Update controller with measured value
     * @param measured_value Measured value
     * @param dt Time step
     * @param logger Optional logger for debugging
     * @return Output of controller
    */

    T update(const T measured_value, const T measured_value_derivative, const T dt, std::optional<rclcpp::Logger> logger = std::nullopt) 
    {
        const T error = setpoint_ - measured_value;
        integral_ += error * dt;

        if (abs(integral_) > windup_limit_) 
        {
            integral_ = windup_limit_ * (integral_ / abs(integral_));
        }

        const auto output = kp_*error + ki_*integral_ + ki_*measured_value_derivative;

        if (logger.has_value()) 
        {
            RCLCPP_DEBUG(logger.value(), "PID error: %f", error);
            RCLCPP_DEBUG(logger.value(), "PID integral: %f", integral_);
            RCLCPP_DEBUG(logger.value(), "PID derivative: %f", measured_value_derivative);
            RCLCPP_DEBUG(logger.value(), "PID output: %f", output);
        }

        return output;
    }

    /**
     * @brief Reset integral term
    */

    void reset() 
    {
        integral_ = 0;
    }
};

}  // namespace dex_controller

#endif  // DEX_CONTROLLER__SIMPLE_PID_HPP_

