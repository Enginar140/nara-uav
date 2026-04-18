# ARDUPILOT VEHICLE KARŞILAŞTIRMASI - LOCAL CONTROL

> **Amaç:** Farklı ArduPilot vehicle'larının local control yeteneklerini karşılaştırmak

---

## 📋 ÖZET KARŞILAŞTIRMA

| Vehicle | Position | Velocity | Acceleration | Yaw/YawRate | Slew Commands |
|---------|----------|----------|--------------|-------------|---------------|
| **Copter** | ✅ FULL | ✅ FULL | ✅ FULL | ✅ FULL | ❌ (gerekmez) |
| **Plane** | ⚠️ ALT ONLY | ✅ | ❌ | ⚠️ HEADING ONLY | ✅ YES |
| **Sub** | ✅ FULL | ✅ FULL | ✅ | ✅ | ❌ |
| **Rover** | ⚠️ LIMITED | ⚠️ LIMITED | ❌ | ❌ | ❌ |
| **Blimp** | ✅ | ✅ | ✅ | ✅ | ❌ |

---

## 1. ARDUCOPTER - FULL 6-DOF CONTROL

### 1.1 SET_POSITION_TARGET_LOCAL_NED

[`GCS_MAVLink_Copter.cpp:971-1080`](cpp_uav_external/ardupilot_repo/ArduCopter/GCS_MAVLink_Copter.cpp:971-1080)

**Desteklenen Frame'ler:**
- `MAV_FRAME_LOCAL_NED` - Earth frame, absolute
- `MAV_FRAME_LOCAL_OFFSET_NED` - Earth frame, offset
- `MAV_FRAME_BODY_NED` - Body frame, absolute
- `MAV_FRAME_BODY_OFFSET_NED` - Body frame, offset

**Type Mask Desteği:**

| Mask | Anlamı | Destek |
|------|--------|--------|
| POS_IGNORE | Position yok | ✅ |
| VEL_IGNORE | Velocity yok | ✅ |
| ACC_IGNORE | Acceleration yok | ✅ |
| YAW_IGNORE | Yaw yok | ✅ |
| YAW_RATE_IGNORE | Yaw rate yok | ✅ |

**Kombinasyonlar:**

```cpp
// 1. Position + Velocity + Acceleration (FULL!)
if (!pos_ignore && !vel_ignore) {
    mode_guided.set_pos_vel_accel_NED_m(
        pos_ned_m, vel_ned_ms, accel_ned_mss,
        !yaw_ignore, yaw_rad,
        !yaw_rate_ignore, yaw_rate_rads,
        yaw_relative
    );
}

// 2. Velocity only
if (pos_ignore && !vel_ignore) {
    mode_guided.set_vel_accel_NED_m(
        vel_ned_ms, accel_ned_mss,
        ...
    );
}

// 3. Acceleration only
if (pos_ignore && vel_ignore && !acc_ignore) {
    mode_guided.set_accel_NED_mss(
        accel_ned_mss, ...
    );
}

// 4. Position only
if (!pos_ignore && vel_ignore && acc_ignore) {
    mode_guided.set_pos_NED_m(
        pos_ned_m, ...
    );
}
```

### 1.2 Özellikler

✅ **Tam 6-DOF kontrolü** (x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate)  
✅ **Body frame desteği** (gemiye göre komut)  
✅ **Offset desteği** (mevcut konuma göre offset)  
✅ **Combined control** (pos + vel + accel aynı anda)  

---

## 2. ARDUPLANE - LIMITED CONTROL

### 2.1 SET_POSITION_TARGET_LOCAL_NED

[`GCS_MAVLink_Plane.cpp:1081-1101`](cpp_uav_external/ardupilot_repo/ArduPlane/GCS_MAVLink_Plane.cpp:1081-1101)

```cpp
void GCS_MAVLINK_Plane::handle_set_position_target_local_ned(...) {
    // sadece local moves for now
    if (packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED) {
        return;
    }
    // just do altitude for now  ⚠️
    plane.next_WP_loc.alt += -packet.z*100.0;
}
```

**Desteklenen:**
- ✅ Altitude (z)
- ❌ X/Y (north/east)
- ❌ Velocity
- ❌ Acceleration
- ❌ Yaw

### 2.2 Guided Slew Commands

**AMAÇ:** Plane'in eksikliklerini telafi et

| Komut | MAV_CMD | Ne Yapar? |
|-------|---------|-----------|
| HEADING | 40002 | Yön kontrolü (heading/COG) |
| SPEED | 40000 | Hız kontrolü |
| ALTITUDE | 40001 | Yükseklik + tırmanma hızı |

**Neden Slew?**
- Fixed-wing aerodinamik gereksinimleri
- Yumuşak maneuver kabiliyeti
- Rate limiting

---

## 3. ARDUSUB - FULL CONTROL

### 3.1 SET_POSITION_TARGET_LOCAL_NED

[`GCS_MAVLink_Sub.cpp:603-...`](cpp_uav_external/ardupilot_repo/ArduSub/GCS_MAVLink_Sub.cpp:603)

Copter ile benzer şekilde full 6-DOF kontrol.

**Özellikler:**
- ✅ Position
- ✅ Velocity
- ✅ Acceleration
- ✅ Yaw/YawRate

---

## 4. ROVER - LIMITED CONTROL

### 4.1 SET_POSITION_TARGET_LOCAL_NED

[`GCS_MAVLink_Rover.cpp:587-754`](cpp_uav_external/ardupilot_repo/Rover/GCS_MAVLink_Plane.cpp:587-754)

**Sınırlamalar:**
```cpp
// ignore any command where acceleration is not ignored
send_acc_ignore_must_be_set_message("SET_POSITION_TARGET_LOCAL_NED");
return;
```

**Desteklenen:**
- ⚠️ Position (sınırlı)
- ⚠️ Velocity (sınırlı)
- ❌ Acceleration (redded!)

---

## 5. NEDEN FARKLILIK VAR?

### 5.1 Aerodinamik Farkları

```
MULTIROTOR (Copter/Sub):
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│    ↑T          ^T                                           │
│    │           │  Her yöne eşit maneuver                    │
│    │           │  6-DOF kontrol mümkün                     │
│   ─┼─         ─┼─                                          │
│    │           │                                           │
│    ↓          ↓                                            │
│                                                             │
│  Hover → Her yöne hareket edebilir                          │
│                                                             │
└─────────────────────────────────────────────────────────────┘

FIXED-WING (Plane):
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│                    →→→→→→→→                                 │
│                    FLIGHT DIRECTION                        │
│                                                             │
│  Sadece ileri yönünde stabil                                │
│  Dönüş için bank angle gerekli                              │
│  X/Y direkt kontrol mümkün değil                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 Kontrol Mimarisi

**Copter:**
```
Position → Position Controller → Velocity → Rate → Motor Mixer → ESC
```

**Plane:**
```
Waypoint → L1 Navigation → Roll/Pitch → Servo → Control Surface
```

---

## 6. PRACTICAL FARKLAR

### 6.1 Örnek: "100m sağa git"

**Copter:**
```cpp
// LOCAL_OFFSET_NED frame
mavlink_msg_set_position_target_local_ned_pack(
    sys_id, comp_id, &msg,
    0, target_sys, target_comp,
    MAV_FRAME_LOCAL_OFFSET_NED,
    0,  // type_mask: nothing ignored
    0, 100, 0,  // x=0, y=100m (sğ), z=0
    0, 0, 0,    // vx, vy, vz
    0, 0, 0,    // afx, afy, afz
    0, 0        // yaw, yaw_rate
);
```

**Plane:**
```cpp
// SLEW HEADING komutu
float current_heading = own_state.heading;
float target_heading = current_heading + 90.0;  // Sağa 90°

mavlink_msg_command_int_pack(
    sys_id, comp_id, &msg,
    target_sys, target_comp,
    MAV_FRAME_GLOBAL_INT,
    MAV_CMD_GUIDED_CHANGE_HEADING,
    0, 0,
    HEADING_TYPE_HEADING,  // param1
    target_heading,         // param2
    0.5f,                   // param3: accel limit
    0, 0, 0, 0, 0
);

// Sonra 100m ileri git (DO_REPOSITION)
```

### 6.2 Örnek: "Hızlan"

**Copter:**
```cpp
// Velocity komutu
mavlink_msg_set_position_target_local_ned_pack(
    ...,
    MAV_FRAME_LOCAL_NED,
    0b0000111111000111,  // pos ignore, vel use
    0, 0, 0,             // x, y, z ignore
    10, 0, 0,            // vx=10m/s, vy=0, vz=0
    ...
);
```

**Plane:**
```cpp
// SLEW SPEED komutu
mavlink_msg_command_int_pack(
    ...,
    MAV_CMD_GUIDED_CHANGE_SPEED,
    0, 0,
    SPEED_TYPE_AIRSPEED,
    25.0f,  // target airspeed
    2.0f,   // acceleration
    ...
);
```

---

## 7. KARAR MATRİSİ

### 7.1 Bizim Sistem İçin

**Sabit kanat (Fixed-wing)** kullanıyoruz → Plane davranışı

**Mevcut Durum:**
- ❌ SET_POSITION_TARGET_LOCAL_NED X/Y çalışmıyor
- ✅ Slew commands mevcut (AMAÇ: Plane)

**Çözüm:**
1. ✅ Guided Slew Commands kullan (native, stable)
2. ⚠️ DO_REPOSITION ile waypoint (büyük hareketler)
3. ⚠️ SET_POSITION_TARGET_LOCAL_NED velocity (fine control)

### 7.2 Hybrid Yaklaşım

```cpp
class HybridController {
public:
    void moveTo(float north_m, float east_m, float down_m) {
        // Büyük hareketler (>100m) → Global position
        float dist = sqrt(north*north + east*east);
        if (dist > 100.0) {
            auto target = calculateGlobalTarget(north, east, down);
            commander_->flyToPosition(target);
            return;
        }
        
        // Küçük hareketler → Heading + speed
        float heading = atan2(east, north) * 180.0 / M_PI;
        commander_->changeHeading(heading, HEADING, 0.5);
        
        if (dist > 10.0) {
            float speed = calculateSpeedForDistance(dist);
            commander_->changeAirspeed(speed, 2.0);
        }
    }
};
```

---

## 8. SONUÇ

### 8.1 Key Takeaways

1. **Copter = Full control** (6-DOF)
2. **Plane = Limited control** (heading/speed/altitude)
3. **Slew commands** Plane'in eksikliklerini kapatıyor
4. **Aerodinamik** mimariyi belirliyor

### 8.2 Bizim İçin En İyi

**Guided Slew Commands** çünkü:
- ✅ Native Plane desteği
- ✅ Rate limiting
- ✅ PID kontrolü
- ✅ Timeout güvenliği
- ✅ Fork gerekmez

---

*Döküman v1.0 - 2026-04-14*
