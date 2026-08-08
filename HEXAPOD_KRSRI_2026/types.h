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

// Rotasi titik terhadap sumbu X, Y, Z (radian). Urutan: Rz * Ry * Rx.
//
// PENTING — jangan tertukar. Body frame di sini +X KANAN, +Y DEPAN, +Z ATAS,
// jadi arti fisik tiap sumbu adalah:
//     rotX = PITCH (mendongak/menunduk)   <- bukan roll
//     rotY = ROLL  (miring kanan/kiri)    <- bukan pitch
//     rotZ = YAW
// Versi lama file ini menamai parameternya roll, pitch, yaw dan itu terbalik
// untuk frame ini. Namanya sekarang netral supaya kesalahan itu tidak bisa
// terulang diam-diam; pemanggil yang menentukan pemetaannya (lihat Hexapod).
//
// Rotasi titik langsung, bukan bangun matriks 4x4 + CMSIS-DSP: untuk 6 titik
// per loop ini lebih sederhana & cukup cepat di FPU Teensy.
inline Vec3 rotatePoint(const Vec3& p, float rotX, float rotY, float rotZ) {
    float cx = cosf(rotX), sx = sinf(rotX);
    float cy = cosf(rotY), sy = sinf(rotY);
    float cz = cosf(rotZ), sz = sinf(rotZ);

    // Rx
    float y1 = cx * p.y - sx * p.z;
    float z1 = sx * p.y + cx * p.z;
    float x1 = p.x;
    // Ry
    float x2 = cy * x1 + sy * z1;
    float z2 = -sy * x1 + cy * z1;
    float y2 = y1;
    // Rz
    float x3 = cz * x2 - sz * y2;
    float y3 = sz * x2 + cz * y2;
    float z3 = z2;

    return { x3, y3, z3 };
}

// INVERS SEJATI dari rotatePoint: (Rz*Ry*Rx)^-1 = Rx(-a)*Ry(-b)*Rz(-c).
// Urutannya DIBALIK — bukan cukup sudutnya dinegatifkan.
//
// Kode lama memakai rotatePoint(p, -roll, -pitch, -yaw) untuk membalik body
// transform. Itu tepat kalau hanya SATU sumbu yang tidak nol, dan meleset
// begitu dua sumbu aktif bersamaan — persis kondisi stabilisasi IMU di medan
// miring. Ujung kaki melenceng (diukur di TES_GERAK/test_motion.py):
//     roll+pitch  5 der -> 1,2 mm    15 der -> 9,9 mm    30 der -> 34 mm
// Melencengnya ujung kaki berarti robot mendorong lantai, bukan berdiri di
// atasnya: kaki tergelincir dan badan bergoyang justru saat sedang menyeimbangkan.
inline Vec3 rotatePointInv(const Vec3& p, float rotX, float rotY, float rotZ) {
    float cx = cosf(rotX), sx = sinf(rotX);
    float cy = cosf(rotY), sy = sinf(rotY);
    float cz = cosf(rotZ), sz = sinf(rotZ);

    // Rz(-rotZ)
    float x1 =  cz * p.x + sz * p.y;
    float y1 = -sz * p.x + cz * p.y;
    float z1 =  p.z;
    // Ry(-rotY)
    float x2 =  cy * x1 - sy * z1;
    float z2 =  sy * x1 + cy * z1;
    float y2 =  y1;
    // Rx(-rotX)
    float y3 =  cx * y2 + sx * z2;
    float z3 = -sx * y2 + cx * z2;

    return { x2, y3, z3 };
}

#endif
