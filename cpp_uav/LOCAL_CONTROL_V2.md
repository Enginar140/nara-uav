# Local Control Implementation - Doğru Yaklaşım (v2)

## Önceki Yaklaşımın Sorunu

Önceki yaklaşımda `MAV_CMD_GUIDED_CHANGE_*` komutlarını (43000, 43001, 43002) kullanmaya çalıştık. Bu komutlar:
- ArduPilot geliştirme (development) dalında tanımlı
- Standart MAVLink protokolünde DEĞİL
- `ardupilotmega.h` dialect'inde tanımlı ancak build'da problem çıkardı
- ArduPilot kaynak kodunu değiştirmek gerekiyor (AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED flag)

## Doğru Yaklaşım: Standart MAVLink Komutları

Araştırma sonucunda, **ArduPilot'u değiştirmeden** kullanabileceğimiz standart komutları buldum:

### 1. Hız Kontrolü: `MAV_CMD_DO_CHANGE_SPEED` (178)

```cpp
// Param1: Speed type (0=Airspeed, 1=Ground Speed)
// Param2: Target speed (m/s)
// Param3: Throttle % (0-100) - Plane için

mavlink_msg_command_long_pack(
    sys_id_, comp_id_, &msg,
    target_sys, target_comp,
    MAV_CMD_DO_CHANGE_SPEED,  // 178
    0,  // confirmation
    0,              // param1: 0=Airspeed, 1=Ground Speed
    target_speed,   // param2: Speed in m/s
    0,              // param3: Throttle (optional)
    0, 0, 0, 0      // unused
);
```

**Önemli Notlar:**
- Airspeed sensörü gerekiyor (TECS aktif olmalı)
- Değişiklikler reboot'a kadar kalıcı
- `WP_SPD` parametresini değiştirir

### 2. Heading Kontrolü: `MAV_CMD_CONDITION_YAW` (115)

```cpp
// Param1: Target heading (degrees, 0-360, 0=North)
// Param2: Yaw speed (deg/s)
// Param3: Direction (-1=CCW, 1=CW, 0=shortest)
// Param4: 0=Absolute, 1=Relative

mavlink_msg_command_long_pack(
    sys_id_, comp_id_, &msg,
    target_sys, target_comp,
    MAV_CMD_CONDITION_YAW,  // 115
    0,  // confirmation
    heading_deg,    // param1: Target heading
    yaw_speed_dps,  // param2: Yaw speed (deg/s)
    0,              // param3: Direction (0=shortest)
    0,              // param4: 0=Absolute
    0, 0, 0         // unused
);
```

**Önemli Notlar:**
- Copter için çalışır
- Plane için **sınırlı destek** - Plane heading'i waypoint'e göre yönetir
- Plane'de alternatif: `DO_SET_ROI` (Region of Interest) ile heading kontrolü

### 3. Altitude Kontrolü: `SET_POSITION_TARGET_GLOBAL_INT`

```cpp
// Type mask ile sadece altitude'ı değiştir
// Position X/Y ignore, Position Z use, Velocity ignore, Yaw ignore

uint16_t type_mask = 0b0000111111000111;  // pos_x/y=ignore, pos_z=use

mavlink_msg_set_position_target_global_int_pack(
    sys_id_, comp_id_, &msg,
    time_boot_ms,
    target_sys, target_comp,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  // frame
    type_mask,
    0, 0,  // x, y (lat, lon) - ignore
    target_altitude * 1e3,  // z (alt * 1000) - use
    0, 0, 0,  // vx, vy, vz - ignore
    0, 0, 0,  // afx, afy, afz - ignore
    0, 0  // yaw, yaw_rate - ignore
);
```

## Plane için Gerçekçi Yaklaşım

Fixed-wing (Plane) için **heading kontrolü ayrı bir komutla yapılmaz**. Bunun yerine:

### Yöntem 1: Velocity Vector ile Yönlendirme

```cpp
// Hedef yön ve hız ile velocity komutu gönder
float vn = speed_ms * cos(heading_rad);
float ve = speed_ms * sin(heading_rad);

mavlink_msg_set_position_target_local_ned_pack(
    sys_id_, comp_id_, &msg,
    0, target_sys, target_comp,
    MAV_FRAME_LOCAL_NED,
    0b0000111111000111,  // pos=ignore, vel=use, yaw=ignore
    0, 0, 0,  // pos
    vn, ve, 0,  // vel - sadece horizontal
    0, 0, 0,  // acc
    0, 0  // yaw, yaw_rate
);
```

### Yöntem 2: Waypoint ile Yönlendirme

```cpp
// Uçağın mevcut konumundan istenen heading yönünde bir waypoint oluştur
double target_lat, target_lon;
calculatePositionFromHeadingBearing(current_lat, current_lon, 
                                   heading_deg, distance_m, 
                                   &target_lat, &target_lon);

flyToPosition({target_lat, target_lon, current_alt}, speed_ms);
```

## Vision Tracking İçin Önerilen Yaklaşım

Hedef takibi için **velocity vector yönlendirme** en uygun:

```cpp
// Hedef relative position (NED frame)
float north_error = target_north_m - current_north_m;
float east_error = target_east_m - current_east_m;

// Normalize et
float dist = sqrt(north_error*north_error + east_error*east_error);
if (dist > 1.0f) {
    north_error /= dist;
    east_error /= dist;
}

// Hedef hız
float target_vn = pursuit_speed_ms * north_error;
float target_ve = pursuit_speed_ms * east_error;

// Velocity komutu gönder
setLocalVelocity(target_vn, target_ve, 0, 0);
```

## Implementasyon Planı

### Phase 1: FlightCommander'a Metodlar Ekle
1. `changeSpeed(float speed_ms, bool use_airspeed = true)` - `MAV_CMD_DO_CHANGE_SPEED`
2. `setHeading(float heading_deg, float yaw_speed_dps = 30.0f)` - `MAV_CMD_CONDITION_YAW` (opsiyonel)
3. `setAltitude(float altitude_m)` - `SET_POSITION_TARGET_GLOBAL_INT`

### Phase 2: Velocity Controller
1. `VelocityController` sınıfı oluştur
2. NED velocity komutları için wrapper metodlar
3. Vision tracking için pursuit guidance

### Phase 3: Integration
1. `FollowMode`'da velocity kontrolü kullan
2. SafetyGate ile velocity limitleri kontrol et
3. Test ve doğrulama

## ArduPilot Parametreleri

Plane için ilgili parametreler:
- `WP_SPD` - Waypoint speed (m/s)
- `WP_SPD_UP` - Climb speed (m/s)
- `WP_SPD_DN` - Descent speed (m/s)
- `WP_ACC_Z` - Vertical acceleration (m/s²)

## MAVLink Header Kullanımı

Standart komutlar `mavlink_msg_command_long_pack` içinde tanımlı:
- `MAV_CMD_DO_CHANGE_SPEED` = 178 (common/mavlink_msg_command_long.h)
- `MAV_CMD_CONDITION_YAW` = 115
- `MAV_CMD_DO_SET_ROI` = 195 (ROI ile heading kontrolü)

Bu değerler zaten mevcut MAVLink header'larında tanımlı!
