#ifndef TYPES_H
#define TYPES_H

// File ini MURNI (tanpa Arduino.h) supaya matematika bisa diuji di PC (lihat test/).
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Vec3 {
    float x, y, z;
};

// --- Helper matematika kecil (inline, tanpa dependensi) ---
inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }
inline float rad2deg(float r) { return r * 180.0f / (float)M_PI; }

inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

// Selisih sudut terpendek (-180..180), input/output derajat.
inline float angleDiffDeg(float target, float current) {
    float d = target - current;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// Kontroler PD (tanpa I: hindari windup pada wall-follow/heading).
// step() butuh dt (detik) dari pemanggil; reset() saat target lompat (ganti state).
struct Pid {
    float kp, kd;
    float prev = 0.0f;
    bool  has  = false;            // belum ada sampel sebelumnya -> D=0
    float step(float err, float dt) {
        float d = (has && dt > 1e-4f) ? (err - prev) / dt : 0.0f;
        prev = err; has = true;
        return kp * err + kd * d;
    }
    void reset() { has = false; prev = 0.0f; }
};

// Rotasi titik oleh roll(X), pitch(Y), yaw(Z) dalam radian.
// Urutan: Rz * Ry * Rx (intrinsic). Dipakai untuk body kinematics.
// ponytail: rotasi titik langsung, bukan bangun matriks 4x4 + CMSIS-DSP.
// Untuk 6 titik/loop ini lebih sederhana & cukup cepat di FPU Teensy.
inline Vec3 rotatePoint(const Vec3& p, float roll, float pitch, float yaw) {
    float cr = cosf(roll),  sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw),   sy = sinf(yaw);

    // Rx
    float y1 = cr * p.y - sr * p.z;
    float z1 = sr * p.y + cr * p.z;
    float x1 = p.x;
    // Ry
    float x2 = cp * x1 + sp * z1;
    float z2 = -sp * x1 + cp * z1;
    float y2 = y1;
    // Rz
    float x3 = cy * x2 - sy * y2;
    float y3 = sy * x2 + cy * y2;
    float z3 = z2;

    return { x3, y3, z3 };
}

#endif
