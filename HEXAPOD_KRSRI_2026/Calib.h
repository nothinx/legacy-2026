#ifndef CALIB_H
#define CALIB_H

// Kalibrasi runtime: satu sumber nilai yang bisa disetel via GUI tuner & disimpan
// di EEPROM. Nama simbol lama (SERVO_OFFSET, HEADING_KP, GAIT_*, ...) diarahkan
// ulang ke store runtime lewat macro -> kode konsumen TIDAK berubah.
//
// Boot: Calib::begin() (di setup(), SEBELUM robot.begin()) -> applyDefaults lalu load().
// config.h tetap murni-const (aman utk host test); file ini Teensy-only saat disave.
#include <stdint.h>
#include "config.h"

// --- indeks parameter skalar (tambah knob = 1 baris di sini + 1 di PARAM_DEFS) ---
enum ParamId {
    K_PULSE_MIN, K_PULSE_MAX,
    K_GAIT_STEP_HEIGHT, K_GAIT_STEP_LENGTH, K_GAIT_CYCLE_TIME, K_GAIT_DUTY,
    K_GAIT_SLEW_RATE, K_GAIT_PROFILE_TAU, K_GAIT_SETTLE_TAU,
    K_STAB_TAU, K_STAB_SIGN_ROLL, K_STAB_SIGN_PITCH,
    K_HEADING_KP, K_HEADING_KD, K_WALL_KP, K_WALL_KD, K_WALL_SETPOINT,
    K_HEAD_UTARA, K_HEAD_TIMUR, K_HEAD_SELATAN, K_HEAD_BARAT,
    // pose lengan: tiap pose 3 entri kontigu (base, shoulder, gripper).
    // urut HARUS sama dgn enum ArmPose {PARK,REACH,GRIP,LIFT,DROP} -> poseData pakai stride 3.
    K_ARM_PARK_B, K_ARM_PARK_S, K_ARM_PARK_G,
    K_ARM_REACH_B, K_ARM_REACH_S, K_ARM_REACH_G,
    K_ARM_GRIP_B, K_ARM_GRIP_S, K_ARM_GRIP_G,
    K_ARM_LIFT_B, K_ARM_LIFT_S, K_ARM_LIFT_G,
    K_ARM_DROP_B, K_ARM_DROP_S, K_ARM_DROP_G,
    N_PARAMS
};

struct ParamDef { const char* name; float def, lo, hi; };

// Blob persist (urutan field = layout EEPROM). crc = CRC16 atas semua byte sebelum crc.
struct CalibBlob {
    char     magic[2];
    uint8_t  version;
    float    param[N_PARAMS];
    float    offset[NUM_SERVOS];   // hanya kaki dipakai; lengan pose-based
    int16_t  trim[NUM_SERVOS];
    uint8_t  invert[NUM_SERVOS];
    uint16_t crc;
};

extern const ParamDef PARAM_DEFS[N_PARAMS];
extern CalibBlob gCalib;

#define gParam  (gCalib.param)
#define gOffset (gCalib.offset)
#define gTrim   (gCalib.trim)
#define gInvert (gCalib.invert)

// --- redireksi nama lama -> runtime (konsumen tak berubah) ---
#define SERVO_PULSE_MIN   ((uint16_t)gParam[K_PULSE_MIN])
#define SERVO_PULSE_MAX   ((uint16_t)gParam[K_PULSE_MAX])
#define SERVO_OFFSET      gOffset
#define SERVO_TRIM_US     gTrim
#define SERVO_INVERT      gInvert

#define GAIT_STEP_HEIGHT  gParam[K_GAIT_STEP_HEIGHT]
#define GAIT_STEP_LENGTH  gParam[K_GAIT_STEP_LENGTH]
#define GAIT_CYCLE_TIME   gParam[K_GAIT_CYCLE_TIME]
#define GAIT_DUTY         gParam[K_GAIT_DUTY]
#define GAIT_SLEW_RATE    gParam[K_GAIT_SLEW_RATE]
#define GAIT_PROFILE_TAU  gParam[K_GAIT_PROFILE_TAU]
#define GAIT_SETTLE_TAU   gParam[K_GAIT_SETTLE_TAU]
#define STAB_TAU          gParam[K_STAB_TAU]

#define HEADING_KP        gParam[K_HEADING_KP]
#define HEADING_KD        gParam[K_HEADING_KD]
#define WALL_KP           gParam[K_WALL_KP]
#define WALL_KD           gParam[K_WALL_KD]
#define WALL_SETPOINT_CM  ((int)gParam[K_WALL_SETPOINT])
#define HEAD_UTARA        gParam[K_HEAD_UTARA]
#define HEAD_TIMUR        gParam[K_HEAD_TIMUR]
#define HEAD_SELATAN      gParam[K_HEAD_SELATAN]
#define HEAD_BARAT        gParam[K_HEAD_BARAT]

// pose lengan: pointer ke 3 float kontigu di gParam (dipakai sbg const float* di HexaArm).
#define ARM_POSE_PARK     (&gParam[K_ARM_PARK_B])
#define ARM_POSE_REACH    (&gParam[K_ARM_REACH_B])
#define ARM_POSE_GRIP     (&gParam[K_ARM_GRIP_B])
#define ARM_POSE_LIFT     (&gParam[K_ARM_LIFT_B])
#define ARM_POSE_DROP     (&gParam[K_ARM_DROP_B])

namespace Calib {
    void  begin();                              // applyDefaults + load (panggil di setup awal)
    void  applyDefaults();                      // isi gCalib dari PARAM_DEFS + default servo
    bool  load();                               // dari EEPROM; false bila invalid (lalu default)
    void  save();                               // ke EEPROM (+ hitung crc)
    int   findParam(const char* name);          // -1 bila tak ada
    bool  setParam(const char* name, float v);  // clamp ke [lo,hi]; false bila nama tak ada
    uint16_t crc16(const uint8_t* p, uint32_t n);
}

#endif
