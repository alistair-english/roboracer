#include "wheel_odometry.hpp"

#include <algorithm>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace localization::wheel_odometry {

namespace {

double stamp_to_sec(const builtin_interfaces::msg::Time& s) {
    return static_cast<double>(s.sec) + static_cast<double>(s.nanosec) * 1e-9;
}

double encoder_position(const sensor_msgs::msg::JointState& js) {
    // The bridge publishes each side's cumulative wheel angle (radians) as a
    // single-element position[]. position[] could in principle be empty if the
    // simulator misbehaves; guarantee a numeric answer either way.
    return js.position.empty() ? 0.0 : js.position.front();
}

void on_encoders(
    std::shared_ptr<Session> sn,
    sensor_msgs::msg::JointState::ConstSharedPtr left,
    sensor_msgs::msg::JointState::ConstSharedPtr right
) {
    const double t = std::max(stamp_to_sec(left->header.stamp), stamp_to_sec(right->header.stamp));
    const double left_pos = encoder_position(*left);
    const double right_pos = encoder_position(*right);

    if (!sn->last_sample.has_value()) {
        sn->last_sample = EncoderSample{t, left_pos, right_pos};
        return;
    }

    const auto& prev = *sn->last_sample;
    const double dt = t - prev.t_sec;
    if (dt < sn->params.min_dt_sec) {
        return;
    }

    const double dtheta_l = left_pos - prev.left_pos;
    const double dtheta_r = right_pos - prev.right_pos;
    const double r = sn->params.wheel_radius;
    const double v_x = 0.5 * r * (dtheta_l + dtheta_r) / dt;

    sn->last_sample = EncoderSample{t, left_pos, right_pos};

    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    msg.header.stamp.sec = static_cast<int32_t>(t);
    msg.header.stamp.nanosec = static_cast<uint32_t>((t - msg.header.stamp.sec) * 1e9);
    msg.header.frame_id = sn->params.base_frame;
    msg.twist.twist.linear.x = v_x;

    // Only linear.x is observed; mark every other component with a large
    // variance so robot_localization downweights it even if accidentally fused.
    constexpr double kLargeVar = 1.0e6;
    auto& cov = msg.twist.covariance;
    cov.fill(0.0);
    cov[0 * 6 + 0] = sn->params.linear_x_variance; // vx
    cov[1 * 6 + 1] = kLargeVar;                    // vy
    cov[2 * 6 + 2] = kLargeVar;                    // vz
    cov[3 * 6 + 3] = kLargeVar;                    // vroll
    cov[4 * 6 + 4] = kLargeVar;                    // vpitch
    cov[5 * 6 + 5] = kLargeVar;                    // vyaw

    sn->publishers.twist->publish(msg);
}

} // namespace

CallbackReturn on_configure(std::shared_ptr<Session> sn) {
    sn->subscribers.encoders->set_callback(on_encoders);
    return CallbackReturn::SUCCESS;
}

} // namespace localization::wheel_odometry
