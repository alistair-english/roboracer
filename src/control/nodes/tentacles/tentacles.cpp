#include "tentacles.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <racer_interfaces/msg/wall_segment_array.hpp>

namespace control::tentacles {

namespace {

struct Vec2 {
    float x;
    float y;
};

struct Segment {
    Vec2 a;
    Vec2 b;
};

float dot(Vec2 u, Vec2 v) { return u.x * v.x + u.y * v.y; }

// Squared distance from point p to closed segment s.
float dist_sq_point_segment(Vec2 p, const Segment& s) {
    const Vec2 ab{s.b.x - s.a.x, s.b.y - s.a.y};
    const Vec2 ap{p.x - s.a.x, p.y - s.a.y};
    const float ab_len_sq = dot(ab, ab);
    if (ab_len_sq < 1e-12f) {
        return ap.x * ap.x + ap.y * ap.y;
    }
    const float t = std::clamp(dot(ap, ab) / ab_len_sq, 0.0f, 1.0f);
    const float dx = ap.x - t * ab.x;
    const float dy = ap.y - t * ab.y;
    return dx * dx + dy * dy;
}

bool point_collides(Vec2 p, const std::vector<Segment>& segments, float radius_sq) {
    for (const auto& s : segments) {
        if (dist_sq_point_segment(p, s) < radius_sq) {
            return true;
        }
    }
    return false;
}

// Body-frame position along a constant-curvature arc at arc length s.
// Bicycle model: heading 0, origin (0,0), curvature κ. κ > 0 turns left (+y).
Vec2 arc_point(float kappa, float s) {
    if (std::abs(kappa) < 1e-6f) {
        return {s, 0.0f};
    }
    const float r = 1.0f / kappa;
    return {r * std::sin(kappa * s), r * (1.0f - std::cos(kappa * s))};
}

struct Tentacle {
    float steering;
    float kappa;
    float clear_length;
    bool blocked_at_start;
    std::vector<Vec2> points;
};

Tentacle evaluate_tentacle(
    float steering,
    float wheelbase,
    float horizon,
    float step,
    const std::vector<Segment>& segments,
    float safety_radius
) {
    Tentacle t;
    t.steering = steering;
    t.kappa = std::tan(steering) / wheelbase;
    t.clear_length = 0.0f;
    t.blocked_at_start = false;

    const float radius_sq = safety_radius * safety_radius;
    const auto cap = static_cast<std::size_t>(horizon / step) + 2u;
    t.points.reserve(cap);
    t.points.push_back({0.0f, 0.0f});

    float s = step;
    bool first = true;
    while (s <= horizon + 1e-6f) {
        const Vec2 p = arc_point(t.kappa, s);
        if (point_collides(p, segments, radius_sq)) {
            if (first) {
                t.blocked_at_start = true;
            }
            break;
        }
        t.points.push_back(p);
        t.clear_length = s;
        s += step;
        first = false;
    }
    return t;
}

std::vector<Segment> to_segments(const racer_interfaces::msg::WallSegmentArray& walls) {
    std::vector<Segment> out;
    out.reserve(walls.segments.size());
    for (const auto& w : walls.segments) {
        out.push_back({
            {static_cast<float>(w.start.x), static_cast<float>(w.start.y)},
            {static_cast<float>(w.end.x), static_cast<float>(w.end.y)},
        });
    }
    return out;
}

geometry_msgs::msg::Point to_point(Vec2 v) {
    geometry_msgs::msg::Point p;
    p.x = v.x;
    p.y = v.y;
    p.z = 0.0;
    return p;
}

void set_lifetime(visualization_msgs::msg::Marker& m, double sec) {
    const auto secs = static_cast<int32_t>(sec);
    m.lifetime.sec = secs;
    m.lifetime.nanosec = static_cast<uint32_t>((sec - secs) * 1e9);
}

visualization_msgs::msg::Marker base_marker(
    const std_msgs::msg::Header& header,
    const std::string& ns,
    int id,
    int32_t type,
    double lifetime_sec
) {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    set_lifetime(m, lifetime_sec);
    return m;
}

void on_walls(std::shared_ptr<Session> sn, racer_interfaces::msg::WallSegmentArray::ConstSharedPtr walls) {
    const auto segments = to_segments(*walls);

    const int n = static_cast<int>(sn->params.n_tentacles);
    const float max_steer = static_cast<float>(sn->params.max_steer_rad);
    const float wheelbase = static_cast<float>(sn->params.wheelbase);
    const float horizon = static_cast<float>(sn->params.horizon);
    const float step = static_cast<float>(sn->params.arc_step);
    const float radius = static_cast<float>(sn->params.safety_radius);
    const float alpha = static_cast<float>(sn->params.curvature_penalty);
    const float min_thr = static_cast<float>(sn->params.min_throttle);
    const float max_thr = static_cast<float>(sn->params.max_throttle);
    const double lifetime = sn->params.debug_marker_lifetime_sec;

    const float kappa_max = std::tan(max_steer) / wheelbase;

    std::vector<Tentacle> tentacles;
    tentacles.reserve(n);
    for (int i = 0; i < n; ++i) {
        const float t = (n == 1) ? 0.5f : static_cast<float>(i) / static_cast<float>(n - 1);
        const float steering = -max_steer + t * (2.0f * max_steer);
        tentacles.push_back(evaluate_tentacle(steering, wheelbase, horizon, step, segments, radius));
    }

    int best = -1;
    float best_score = -1.0f;
    for (int i = 0; i < n; ++i) {
        const auto& tentacle = tentacles[i];
        const float kappa_norm = (kappa_max > 1e-9f) ? std::abs(tentacle.kappa) / kappa_max : 0.0f;
        const float score = tentacle.clear_length * (1.0f - alpha * kappa_norm);
        if (score > best_score) {
            best_score = score;
            best = i;
        }
    }

    // /autodrive/roboracer_1/steering_command is normalized to [-1, 1] across [-max_steer, +max_steer].
    std_msgs::msg::Float32 steer_msg;
    std_msgs::msg::Float32 throttle_msg;
    if (best >= 0 && !tentacles[best].blocked_at_start) {
        const auto& sel = tentacles[best];
        const float kappa_norm = (kappa_max > 1e-9f) ? std::abs(sel.kappa) / kappa_max : 0.0f;
        steer_msg.data = sel.steering / max_steer;
        throttle_msg.data = max_thr - (max_thr - min_thr) * kappa_norm;
    } else {
        // Even the best arc collides at the first sample → stop, but still steer toward it
        // so the car turns on the spot rather than freezing pointed at a wall.
        steer_msg.data = (best >= 0) ? tentacles[best].steering / max_steer : 0.0f;
        throttle_msg.data = 0.0f;
    }
    sn->publishers.steering_command->publish(steer_msg);
    sn->publishers.throttle_command->publish(throttle_msg);

    visualization_msgs::msg::MarkerArray tentacle_markers;
    for (int i = 0; i < n; ++i) {
        const auto& tentacle = tentacles[i];
        if (tentacle.points.size() < 2) {
            // Emit an explicit DELETE so the previous frame's arc at this id clears
            // immediately rather than waiting for lifetime expiry.
            visualization_msgs::msg::Marker del;
            del.header = walls->header;
            del.ns = "tentacles";
            del.id = i;
            del.action = visualization_msgs::msg::Marker::DELETE;
            tentacle_markers.markers.push_back(std::move(del));
            continue;
        }
        auto strip = base_marker(walls->header, "tentacles", i,
                                 visualization_msgs::msg::Marker::LINE_STRIP, lifetime);
        strip.scale.x = 0.02;
        const bool full = (tentacle.clear_length >= horizon - 1e-3f);
        if (full) {
            strip.color.r = 0.2f;
            strip.color.g = 0.8f;
            strip.color.b = 0.2f;
        } else {
            strip.color.r = 0.9f;
            strip.color.g = 0.3f;
            strip.color.b = 0.2f;
        }
        strip.color.a = 0.5f;
        strip.points.reserve(tentacle.points.size());
        for (const auto& p : tentacle.points) {
            strip.points.push_back(to_point(p));
        }
        tentacle_markers.markers.push_back(std::move(strip));
    }
    sn->publishers.debug_tentacles->publish(tentacle_markers);

    visualization_msgs::msg::MarkerArray selected_markers;
    if (best >= 0 && tentacles[best].points.size() >= 2) {
        auto sel = base_marker(walls->header, "selected", 0,
                               visualization_msgs::msg::Marker::LINE_STRIP, lifetime);
        sel.scale.x = 0.06;
        sel.color.r = 0.2f;
        sel.color.g = 0.6f;
        sel.color.b = 1.0f;
        sel.color.a = 1.0f;
        sel.points.reserve(tentacles[best].points.size());
        for (const auto& p : tentacles[best].points) {
            sel.points.push_back(to_point(p));
        }
        selected_markers.markers.push_back(std::move(sel));
    } else {
        visualization_msgs::msg::Marker del;
        del.header = walls->header;
        del.ns = "selected";
        del.id = 0;
        del.action = visualization_msgs::msg::Marker::DELETE;
        selected_markers.markers.push_back(std::move(del));
    }
    sn->publishers.debug_selected->publish(selected_markers);
}

} // namespace

CallbackReturn on_configure(std::shared_ptr<Session> sn) {
    sn->subscribers.walls->set_callback(on_walls);
    return CallbackReturn::SUCCESS;
}

} // namespace control::tentacles
