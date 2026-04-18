#pragma once

#include "uav/core/types.hpp"
#include "uav/comm/flight_commander.hpp"
#include "uav/world_model/world_model.hpp"

#include <chrono>
#include <memory>
#include <mutex>

namespace uav::nav {

/**
 * @brief Velocity Controller - Local control için velocity guidance
 * 
 * Bu sınıf, hedef takibi (vision tracking) için pursuit guidance sağlar.
 * Fixed-wing aircraft için velocity vector kontrolü kullanır.
 * 
 * Kullanım senaryoları:
 * - Vision-based target tracking
 * - Pursuit mode navigation
 * - Dynamic obstacle avoidance
 */
class VelocityController {
public:
    struct Config {
        // Pursuit guidance parametreleri
        float max_speed_ms = 35.0f;          // Maksimum hız (m/s)
        float min_speed_ms = 12.0f;          // Minimum hız (m/s)
        float approach_speed_ms = 20.0f;     // Yaklaşma hızı (m/s)
        float pursuit_speed_ms = 30.0f;      // Takip hızı (m/s)
        
        // Guidance parametreleri
        float pursuit_gain = 3.0f;           // Pure Navigation gain
        float max_lead_deg = 45.0f;          // Maksimum lead angle (derece)
        float capture_radius_m = 80.0f;      // Hedefe varma yarıçapı (m)
        float min_command_interval_ms = 250; // Minimum komut aralığı
        
        // ROI (Region of Interest) parametreleri
        bool use_roi_for_heading = true;     // ROI ile heading kontrolü
        float roi_update_interval_s = 0.5f;  // ROI güncelleme aralığı
        
        // Safety parametreleri
        float max_accel_ms2 = 5.0f;          // Maksimum ivme (m/s²)
        float max_vertical_speed_ms = 5.0f;  // Maksimum dikey hız (m/s)
    };

    struct State {
        core::Position current_position;
        float current_heading_deg = 0.0f;
        float current_speed_ms = 0.0f;
        float current_altitude_m = 0.0f;
        
        core::Position target_position;
        bool target_valid = false;
        float target_distance_m = 0.0f;
        float target_bearing_deg = 0.0f;
        
        std::chrono::steady_clock::time_point last_command_time;
        std::chrono::steady_clock::time_point last_roi_time;
    };

    VelocityController(std::shared_ptr<comm::FlightCommander> flight_cmd,
                       std::shared_ptr<world_model::WorldModel> world_model,
                       Config config = {});

    /**
     * @brief Hedef pozisyonunu güncelle
     * @param target Hedef pozisyon (GPS koordinatları)
     */
    void setTarget(const core::Position& target);

    /**
     * @brief Hedefi temizle
     */
    void clearTarget();

    /**
     * @brief Velocity komutu gönder (Pursuit guidance)
     * @return true komut başarılı, false başarısız
     */
    [[nodiscard]] bool sendVelocityCommand();

    /**
     * @brief Hız değiştir
     * @param speed_ms Hedef hız (m/s)
     * @param use_airspeed true=airspeed, false=groundspeed
     * @return true komut başarılı, false başarısız
     */
    [[nodiscard]] bool changeSpeed(float speed_ms, bool use_airspeed = true);

    /**
     * @brief İrtifa değiştir
     * @param altitude_m Hedef irtifa (m)
     * @return true komut başarılı, false başarısız
     */
    [[nodiscard]] bool changeAltitude(float altitude_m);

    /**
     * @brief ROI (Region of Interest) ile heading kontrolü
     * @param roi_position Hedef konum
     * @return true komut başarılı, false başarısız
     */
    [[nodiscard]] bool setHeadingROI(const core::Position& roi_position);

    /**
     * @brief Pursuit guidance ile velocity hesapla
     * @param vn_out North velocity output (m/s)
     * @param ve_out East velocity output (m/s)
     * @param vd_out Down velocity output (m/s)
     * @return true hesaplama başarılı, false hedef yok
     */
    [[nodiscard]] bool calculatePursuitVelocity(float& vn_out, float& ve_out, float& vd_out);

    /**
     * @brief Hedefe ulaşıldı mı?
     * @return true hedefe ulaşıldı, false henüz ulaşmadı
     */
    [[nodiscard]] bool isTargetReached() const;

    /**
     * @brief Mevcut durumu güncelle
     * WorldModel'den pozisyon, heading, speed bilgilerini alır
     */
    void updateState();

    /**
     * @brief Config güncelle
     */
    void setConfig(const Config& config) { config_ = config; }

    /**
     * @brief Mevcut config'i al
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief Mevcut state'i al (thread-safe)
     */
    State getState() const {
        std::lock_guard lock(state_mutex_);
        return state_;
    }

private:
    std::shared_ptr<comm::FlightCommander> flight_cmd_;
    std::shared_ptr<world_model::WorldModel> world_model_;
    Config config_;

    mutable std::mutex state_mutex_;
    State state_;

    /**
     * @brief Bearing hesapla (derece cinsinden)
     */
    float calculateBearing(const core::Position& from, const core::Position& to) const;

    /**
     * @brief Lead angle hesapla (Pure Navigation guidance)
     */
    float calculateLeadAngle(float target_distance_m, float closing_speed_ms) const;

    /**
     * @brief Hız limitlerini uygula
     */
    float clampSpeed(float speed_ms) const;

    /**
     * @brief Minimum komut aralığını kontrol et
     */
    bool canSendCommand() const;
};

} // namespace uav::nav
