#include "uav/vision/tracking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace uav::vision {

TrackingController::TrackingController(Config config) : config_(config) {}

void TrackingController::update(const std::vector<Detection>& detections) {
    for (auto& [id, track] : tracks_) {
        track.frames_since_seen++;
        track.x += track.vx;
        track.y += track.vy;
    }

    std::vector<bool> det_matched(detections.size(), false);

    for (auto& [id, track] : tracks_) {
        double best_dist = config_.dist_threshold;
        int best_idx = -1;

        for (size_t i = 0; i < detections.size(); ++i) {
            if (det_matched[i]) continue;
            double d = distance(track, detections[i]);
            if (d < best_dist) {
                best_dist = d;
                best_idx = static_cast<int>(i);
            }
        }

        if (best_idx >= 0) {
            const auto& det = detections[best_idx];
            track.vx = det.x - track.x;
            track.vy = det.y - track.y;
            track.x = det.x;
            track.y = det.y;
            track.confidence = det.confidence;
            track.consecutive_frames++;
            track.frames_since_seen = 0;
            det_matched[best_idx] = true;
        }
    }

    for (size_t i = 0; i < detections.size(); ++i) {
        if (det_matched[i]) continue;
        TrackedObject track;
        track.id = next_id_++;
        track.x = detections[i].x;
        track.y = detections[i].y;
        track.confidence = detections[i].confidence;
        track.consecutive_frames = 1;
        tracks_[track.id] = track;
    }

    for (auto it = tracks_.begin(); it != tracks_.end();) {
        if (it->second.frames_since_seen > config_.max_age_frames) {
            it = tracks_.erase(it);
        } else {
            ++it;
        }
    }
}

std::vector<TrackedObject> TrackingController::getActiveTracks() const {
    std::vector<TrackedObject> result;
    for (const auto& [id, track] : tracks_) {
        if (track.consecutive_frames >= config_.min_hits) {
            result.push_back(track);
        }
    }
    return result;
}

std::optional<TrackedObject> TrackingController::getTrack(int id) const {
    auto it = tracks_.find(id);
    if (it != tracks_.end()) return it->second;
    return std::nullopt;
}

double TrackingController::distance(const TrackedObject& track,
                                    const Detection& det) const {
    double dx = track.x - det.x;
    double dy = track.y - det.y;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace uav::vision
