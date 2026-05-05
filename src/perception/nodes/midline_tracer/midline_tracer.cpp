#include "midline_tracer.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <racer_interfaces/msg/midline.hpp>
#include <racer_interfaces/msg/wall_segment_array.hpp>

namespace perception::midline_tracer {

namespace {

struct Vec2 {
    float x;
    float y;
};

struct Segment {
    Vec2 a;
    Vec2 b;
};

struct CastResult {
    bool hit;
    Vec2 point; // hit location, or sample + cap*dir on miss
};

struct TraceSample {
    Vec2 sample;
    CastResult left;
    CastResult right;
    Vec2 midpoint;
};

Vec2 normalize(Vec2 v) {
    const float n = std::sqrt(v.x * v.x + v.y * v.y);
    if (n < 1e-9f) {
        return {1.0f, 0.0f};
    }
    return {v.x / n, v.y / n};
}

// 90 deg counter-clockwise: points to the car's left when applied to a forward tangent.
Vec2 perp_left(Vec2 v) {
    return {-v.y, v.x};
}

// Ray-vs-segment intersection. Returns the closest hit within `cap`, or a capped
// miss point so the visualization still shows the cast extent.
CastResult raycast(Vec2 origin, Vec2 dir, const std::vector<Segment>& segments, float cap) {
    float best_t = cap;
    bool found = false;
    for (const auto& s : segments) {
        const float vx = s.b.x - s.a.x;
        const float vy = s.b.y - s.a.y;
        const float wx = s.a.x - origin.x;
        const float wy = s.a.y - origin.y;
        // Solve [dir.x  -vx] [t]   [wx]
        //       [dir.y  -vy] [u] = [wy]
        const float det = dir.y * vx - dir.x * vy;
        if (std::abs(det) < 1e-9f) {
            continue; // ray parallel to segment
        }
        const float t = (vx * wy - vy * wx) / det;
        const float u = (dir.x * wy - dir.y * wx) / det;
        if (t >= 0.0f && t <= best_t && u >= 0.0f && u <= 1.0f) {
            best_t = t;
            found = true;
        }
    }
    return {found, {origin.x + best_t * dir.x, origin.y + best_t * dir.y}};
}

geometry_msgs::msg::Point to_point(Vec2 v) {
    geometry_msgs::msg::Point p;
    p.x = v.x;
    p.y = v.y;
    p.z = 0.0;
    return p;
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

struct TraceParams {
    float step;
    float track_width;
    float cast_cap;
    float max_length;
    int max_iterations;
    float ema_alpha;
    int miss_terminate_count;
};

std::vector<TraceSample> trace_midline(const std::vector<Segment>& segments, const TraceParams& p) {
    std::vector<TraceSample> samples;

    Vec2 prev_mid{0.0f, 0.0f};      // M[0] = car origin in vehicle frame
    Vec2 tangent{1.0f, 0.0f};       // initial tangent = car heading (+x)
    float total_length = 0.0f;
    int consecutive_misses = 0;

    for (int i = 0; i < p.max_iterations; ++i) {
        const Vec2 sample{prev_mid.x + p.step * tangent.x, prev_mid.y + p.step * tangent.y};
        const Vec2 cast_dir = perp_left(tangent);
        const Vec2 right_dir{-cast_dir.x, -cast_dir.y};

        const CastResult left = raycast(sample, cast_dir, segments, p.cast_cap);
        const CastResult right = raycast(sample, right_dir, segments, p.cast_cap);

        Vec2 mid;
        if (left.hit && right.hit) {
            mid = {0.5f * (left.point.x + right.point.x), 0.5f * (left.point.y + right.point.y)};
            consecutive_misses = 0;
        } else if (left.hit) {
            // Virtual wall at track_width to the right of the left hit; midpoint sits
            // track_width/2 from the visible wall in the -cast_dir direction.
            mid = {left.point.x - 0.5f * p.track_width * cast_dir.x,
                   left.point.y - 0.5f * p.track_width * cast_dir.y};
            consecutive_misses = 0;
        } else if (right.hit) {
            mid = {right.point.x + 0.5f * p.track_width * cast_dir.x,
                   right.point.y + 0.5f * p.track_width * cast_dir.y};
            consecutive_misses = 0;
        } else {
            ++consecutive_misses;
            if (consecutive_misses >= p.miss_terminate_count) {
                break;
            }
            // Coast forward along the current tangent so the trace can recover
            // if walls reappear within the next few steps.
            mid = sample;
        }

        samples.push_back({sample, left, right, mid});

        const float dx = mid.x - prev_mid.x;
        const float dy = mid.y - prev_mid.y;
        total_length += std::sqrt(dx * dx + dy * dy);

        const Vec2 raw = normalize({dx, dy});
        tangent = normalize({
            p.ema_alpha * raw.x + (1.0f - p.ema_alpha) * tangent.x,
            p.ema_alpha * raw.y + (1.0f - p.ema_alpha) * tangent.y,
        });
        prev_mid = mid;

        if (total_length >= p.max_length) {
            break;
        }
    }

    return samples;
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

    const TraceParams p{
        static_cast<float>(sn->params.step),
        static_cast<float>(sn->params.track_width),
        static_cast<float>(sn->params.cast_cap),
        static_cast<float>(sn->params.max_midline_length),
        static_cast<int>(sn->params.max_iterations),
        static_cast<float>(sn->params.tangent_ema_alpha),
        static_cast<int>(sn->params.miss_terminate_count),
    };
    const auto samples = trace_midline(segments, p);

    racer_interfaces::msg::Midline midline;
    midline.header = walls->header;
    midline.points.reserve(samples.size() + 1);
    midline.points.push_back(to_point({0.0f, 0.0f})); // M[0]
    for (const auto& ts : samples) {
        midline.points.push_back(to_point(ts.midpoint));
    }
    sn->publishers.midline->publish(midline);

    const double lifetime = sn->params.debug_marker_lifetime_sec;

    visualization_msgs::msg::MarkerArray midline_markers;
    {
        auto strip = base_marker(walls->header, "midline", 0,
                                 visualization_msgs::msg::Marker::LINE_STRIP, lifetime);
        strip.scale.x = 0.04;
        strip.color.r = 0.2f;
        strip.color.g = 0.6f;
        strip.color.b = 1.0f;
        strip.color.a = 1.0f;
        strip.points = midline.points;
        midline_markers.markers.push_back(std::move(strip));

        auto pts = base_marker(walls->header, "midpoints", 0,
                               visualization_msgs::msg::Marker::SPHERE_LIST, lifetime);
        pts.scale.x = pts.scale.y = pts.scale.z = 0.06;
        pts.color.r = 1.0f;
        pts.color.g = 0.2f;
        pts.color.b = 0.2f;
        pts.color.a = 1.0f;
        pts.points = midline.points;
        midline_markers.markers.push_back(std::move(pts));
    }
    sn->publishers.debug_midline->publish(midline_markers);

    visualization_msgs::msg::MarkerArray cast_markers;
    {
        auto rays = base_marker(walls->header, "casts", 0,
                                visualization_msgs::msg::Marker::LINE_LIST, lifetime);
        rays.scale.x = 0.015;
        rays.color.r = 1.0f;
        rays.color.g = 1.0f;
        rays.color.b = 0.2f;
        rays.color.a = 0.6f;
        rays.points.reserve(samples.size() * 4);
        for (const auto& ts : samples) {
            rays.points.push_back(to_point(ts.sample));
            rays.points.push_back(to_point(ts.left.point));
            rays.points.push_back(to_point(ts.sample));
            rays.points.push_back(to_point(ts.right.point));
        }
        cast_markers.markers.push_back(std::move(rays));

        auto hits = base_marker(walls->header, "cast_hits", 0,
                                visualization_msgs::msg::Marker::SPHERE_LIST, lifetime);
        hits.scale.x = hits.scale.y = hits.scale.z = 0.04;
        hits.color.r = 0.2f;
        hits.color.g = 1.0f;
        hits.color.b = 0.2f;
        hits.color.a = 1.0f;
        for (const auto& ts : samples) {
            if (ts.left.hit) {
                hits.points.push_back(to_point(ts.left.point));
            }
            if (ts.right.hit) {
                hits.points.push_back(to_point(ts.right.point));
            }
        }
        cast_markers.markers.push_back(std::move(hits));
    }
    sn->publishers.debug_casts->publish(cast_markers);
}

} // namespace

CallbackReturn on_configure(std::shared_ptr<Session> sn) {
    sn->subscribers.walls->set_callback(on_walls);
    return CallbackReturn::SUCCESS;
}

} // namespace perception::midline_tracer
