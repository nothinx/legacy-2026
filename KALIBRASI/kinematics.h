/* =====================================================================
   kinematics.h — geometri + IK hexapod, IDENTIK dengan firmware
   (config.h + LegIK.cpp + Hexapod::solvePose). Dipakai KALIBRASI agar
   hasil kalibrasi langsung sah untuk firmware.

   Konvensi: +X kanan, +Y depan, +Z atas. Origin = pusat badan.
   ===================================================================== */
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include <math.h>

// Dimensi kaki (mm) — samakan dengan config.h firmware
#define COXA_LEN    20.0f
#define FEMUR_LEN   80.0f
#define TIBIA_LEN   90.0f

#define STAND_H_DEF 100.0f   // tinggi badan default (mm)
#define STAND_R_DEF  70.0f   // bentang kaki dari pangkal coxa (mm)

// Pangkal coxa tiap kaki (x, y) dan arah hadapnya (derajat dari +X, CCW +)
// 0=Ka-Depan 1=Ka-Tengah 2=Ka-Belakang 3=Ki-Belakang 4=Ki-Tengah 5=Ki-Depan
static const float LEG_ORIGIN[6][2] = {
    {  45.0f,  78.0f }, {  90.0f,   0.0f }, {  45.0f, -78.0f },
    { -45.0f, -78.0f }, { -90.0f,   0.0f }, { -45.0f,  78.0f }
};
static const float LEG_ANGLE[6] = { 60.0f, 0.0f, -60.0f, -120.0f, 180.0f, 120.0f };

#define PI_F      3.14159265358979f
#define RAD2DEG   (180.0f / PI_F)
#define DEG2RAD   (PI_F / 180.0f)

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// IK satu kaki — salinan persis LegIK::solve.
// Input (x,y,z) relatif pangkal coxa, sudah di frame kaki (netral menghadap +X).
// Output sudut GEOMETRIS: coxa = simpangan dari arah netral, femur = dari
// horizontal (+ naik), tibia = sudut interior lutut.
// return false bila target di luar jangkauan (sudah di-clamp).
static bool ikSolve(float x, float y, float z,
                    float& coxaDeg, float& femurDeg, float& tibiaDeg) {
    bool inRange = true;
    float coxaRad = atan2f(y, x);
    float horiz = sqrtf(x * x + y * y);
    float Lr = horiz - COXA_LEN;
    float D  = sqrtf(Lr * Lr + z * z);

    const float Dmin = fabsf(FEMUR_LEN - TIBIA_LEN) + 1.0f;
    const float Dmax = (FEMUR_LEN + TIBIA_LEN) - 1.0f;
    if (D < Dmin) { D = Dmin; inRange = false; }
    if (D > Dmax) { D = Dmax; inRange = false; }

    float F2 = FEMUR_LEN * FEMUR_LEN;
    float T2 = TIBIA_LEN * TIBIA_LEN;
    float D2 = D * D;

    float a1 = atan2f(z, Lr);
    float a2 = acosf(clampf((F2 + D2 - T2) / (2.0f * FEMUR_LEN * D), -1.0f, 1.0f));
    float knee = acosf(clampf((F2 + T2 - D2) / (2.0f * FEMUR_LEN * TIBIA_LEN), -1.0f, 1.0f));

    coxaDeg  = coxaRad     * RAD2DEG;
    femurDeg = (a1 + a2)   * RAD2DEG;
    tibiaDeg = knee        * RAD2DEG;
    return inRange;
}

// Titik kaki di frame BADAN -> 3 sudut SERVO (baseline 90), seperti
// Hexapod::solvePose. out[] = {coxa, femur, tibia} dalam derajat 0..180.
static bool footToServo(uint8_t leg, float bx, float by, float bz, float out[3]) {
    float vx = bx - LEG_ORIGIN[leg][0];
    float vy = by - LEG_ORIGIN[leg][1];

    float a  = -LEG_ANGLE[leg] * DEG2RAD;                // rotasi ke frame kaki
    float lx = cosf(a) * vx - sinf(a) * vy;
    float ly = sinf(a) * vx + cosf(a) * vy;

    float c, f, t;
    bool ok = ikSolve(lx, ly, bz, c, f, t);
    out[0] = 90.0f + c;
    out[1] = 90.0f + f;
    out[2] = 90.0f + (t - 90.0f);       // tibia dipusatkan di 90
    return ok;
}

// Titik kaki netral (berdiri diam) di frame badan.
static void footHome(uint8_t leg, float radius, float height, float out[3]) {
    float a = LEG_ANGLE[leg] * DEG2RAD;
    out[0] = LEG_ORIGIN[leg][0] + radius * cosf(a);
    out[1] = LEG_ORIGIN[leg][1] + radius * sinf(a);
    out[2] = -height;
}

#endif
