#include "wall_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <racer_interfaces/msg/wall_segment.hpp>
#include <racer_interfaces/msg/wall_segment_array.hpp>

namespace perception::wall_extractor {

namespace {

struct Vec2 {
    float x;
    float y;
};

struct Segment {
    Vec2 start;
    Vec2 end;
    int n_points;
};

float perp_distance(Vec2 p, Vec2 a, Vec2 b) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len_sq = dx * dx + dy * dy;
    if (len_sq < 1e-12f) {
        const float ex = p.x - a.x;
        const float ey = p.y - a.y;
        return std::sqrt(ex * ex + ey * ey);
    }
    const float cross = dx * (p.y - a.y) - dy * (p.x - a.x);
    return std::abs(cross) / std::sqrt(len_sq);
}

void iepf(
    const std::vector<Vec2>& pts,
    std::size_t lo,
    std::size_t hi,
    float threshold,
    std::vector<Segment>& out
) {
    if (hi <= lo) {
        return;
    }
    if (hi == lo + 1) {
        out.push_back({pts[lo], pts[hi], 2});
        return;
    }

    const Vec2 a = pts[lo];
    const Vec2 b = pts[hi];
    std::size_t max_idx = lo;
    float max_dev = 0.0f;
    for (std::size_t i = lo + 1; i < hi; ++i) {
        const float dev = perp_distance(pts[i], a, b);
        if (dev > max_dev) {
            max_dev = dev;
            max_idx = i;
        }
    }

    if (max_dev < threshold) {
        out.push_back({a, b, static_cast<int>(hi - lo + 1)});
    } else {
        iepf(pts, lo, max_idx, threshold, out);
        iepf(pts, max_idx, hi, threshold, out);
    }
}

// Walk the scan in beam order, accumulating contiguous valid points into runs.
// A run breaks on an invalid return or on a range jump bigger than the threshold.
std::vector<std::vector<Vec2>> extract_runs(
    const sensor_msgs::msg::LaserScan& scan,
    float range_jump_threshold
) {
    std::vector<std::vector<Vec2>> runs;
    std::vector<Vec2> current;
    float prev_range = std::numeric_limits<float>::quiet_NaN();

    const std::size_t n = scan.ranges.size();
    for (std::size_t i = 0; i < n; ++i) {
        const float r = scan.ranges[i];
        const bool valid = std::isfinite(r) && r >= scan.range_min && r <= scan.range_max;
        if (!valid) {
            if (!current.empty()) {
                runs.push_back(std::move(current));
                current.clear();
            }
            prev_range = std::numeric_limits<float>::quiet_NaN();
            continue;
        }

        if (!current.empty() && std::isfinite(prev_range)
            && std::abs(r - prev_range) > range_jump_threshold) {
            runs.push_back(std::move(current));
            current.clear();
        }

        const float angle = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
        current.push_back({r * std::cos(angle), r * std::sin(angle)});
        prev_range = r;
    }
    if (!current.empty()) {
        runs.push_back(std::move(current));
    }
    return runs;
}

geometry_msgs::msg::Point32 to_point32(Vec2 v) {
    geometry_msgs::msg::Point32 p;
    p.x = v.x;
    p.y = v.y;
    p.z = 0.0f;
    return p;
}

geometry_msgs::msg::Point to_point(Vec2 v) {
    geometry_msgs::msg::Point p;
    p.x = v.x;
    p.y = v.y;
    p.z = 0.0;
    return p;
}

visualization_msgs::msg::Marker build_marker(
    const std_msgs::msg::Header& header,
    const std::vector<Segment>& segments,
    double lifetime_sec
) {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = "wall_segments";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.03; // line thickness in m
    m.color.r = 0.1f;
    m.color.g = 1.0f;
    m.color.b = 0.1f;
    m.color.a = 1.0f;
    const auto secs = static_cast<int32_t>(lifetime_sec);
    m.lifetime.sec = secs;
    m.lifetime.nanosec = static_cast<uint32_t>((lifetime_sec - secs) * 1e9);
    m.points.reserve(segments.size() * 2);
    for (const auto& s : segments) {
        m.points.push_back(to_point(s.start));
        m.points.push_back(to_point(s.end));
    }
    return m;
}

void on_scan(std::shared_ptr<Session> sn, sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    const auto runs = extract_runs(*scan, static_cast<float>(sn->params.range_jump_threshold));

    std::vector<Segment> segments;
    const auto threshold = static_cast<float>(sn->params.iepf_threshold);
    for (const auto& run : runs) {
        if (run.size() < 2) {
            continue;
        }
        iepf(run, 0, run.size() - 1, threshold, segments);
    }

    const auto min_n = static_cast<int>(sn->params.min_segment_points);
    segments.erase(
        std::remove_if(
            segments.begin(),
            segments.end(),
            [min_n](const Segment& s) { return s.n_points < min_n; }
        ),
        segments.end()
    );

    racer_interfaces::msg::WallSegmentArray walls;
    walls.header = scan->header;
    walls.segments.reserve(segments.size());
    for (const auto& s : segments) {
        racer_interfaces::msg::WallSegment w;
        w.start = to_point32(s.start);
        w.end = to_point32(s.end);
        walls.segments.push_back(w);
    }
    sn->publishers.walls->publish(walls);

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(build_marker(scan->header, segments, sn->params.debug_marker_lifetime_sec));
    sn->publishers.debug_walls->publish(markers);
}

} // namespace

CallbackReturn on_configure(std::shared_ptr<Session> sn) {
    sn->subscribers.scan->set_callback(on_scan);
    return CallbackReturn::SUCCESS;
}

} // namespace perception::wall_extractor
