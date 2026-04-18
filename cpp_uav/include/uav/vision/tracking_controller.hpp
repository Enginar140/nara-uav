#pragma once

#include <optional>
#include <unordered_map>
#include <vector>

namespace uav::vision {

struct Detection {
    double x, y;
    double width, height;
    double confidence;
    int class_id;
};

struct TrackedObject {
    int id;
    double x, y;
    double vx = 0, vy = 0;
    double confidence;
    int consecutive_frames = 0;
    int frames_since_seen = 0;
};

class TrackingController {
public:
    struct Config {
        int max_age_frames = 30;
        int min_hits = 3;
        double dist_threshold = 100.0;
    };

    explicit TrackingController(Config config = {});

    void update(const std::vector<Detection>& detections);
    std::vector<TrackedObject> getActiveTracks() const;
    std::optional<TrackedObject> getTrack(int id) const;

private:
    Config config_;
    std::unordered_map<int, TrackedObject> tracks_;
    int next_id_ = 0;

    double distance(const TrackedObject& track, const Detection& det) const;
};

} // namespace uav::vision
