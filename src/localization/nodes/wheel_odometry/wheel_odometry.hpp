#pragma once

#include <memory>
#include <optional>

#include <localization/wheel_odometry_interface.hpp>

namespace localization::wheel_odometry {

struct EncoderSample {
    double t_sec;
    double left_pos;
    double right_pos;
};

struct Session : WheelOdometrySession<Session> {
    using WheelOdometrySession::WheelOdometrySession;
    std::optional<EncoderSample> last_sample;
};

CallbackReturn on_configure(std::shared_ptr<Session> sn);

using WheelOdometry = WheelOdometryBase<Session, on_configure>;

} // namespace localization::wheel_odometry
