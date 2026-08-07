/* =====================================================================
   motion.h — MESIN GERAK: gait tripod + body kinematics + profil medan

   Ini port dari HexaGait.cpp + Hexapod::solvePose firmware, dengan tiga
   perbedaan yang DISENGAJA (lihat README bagian "Beda dari firmware"):
     1. fase gait diakumulasi dari dt, bukan (millis()-cycleStart)/cycleTime
     2. invers body transform sungguhan (urutan dibalik), bukan sudut dinegatifkan
     3. GaitProfile punya standR — tanpa itu profil SEMPIT (R11) mustahil

   Semua berbasis dt yang DIBERIKAN PEMANGGIL, bukan millis() internal.
   Itu bukan gaya-gayaan: karena dt datang dari luar, mesin yang sama bisa
   dijalankan cepat-cepat di RAM untuk simulasi kering ('S' di TES_GERAK)
   tanpa menggerakkan satu servo pun.

   Konvensi frame: +X kanan, +Y depan, +Z atas, origin pusat badan.
     ROLL  = putar terhadap sumbu DEPAN (+Y) -> badan miring kanan/kiri
     PITCH = putar terhadap sumbu KANAN (+X) -> badan mendongak/menunduk
     YAW   = putar terhadap +Z, positif = berlawanan jarum jam (belok KIRI)
   types.h firmware menamai rotasi-X "roll" dan rotasi-Y "pitch" — untuk
   frame ini terbalik. Nama di sini mengikuti FISIKA, bukan firmware.
   ===================================================================== */
#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>
#include <math.h>
#include "kinematics.h"

// ---- default: sama persis dengan PARAM_DEFS di Calib.cpp firmware ----
#define MO_STEP_H_DEF       40.0f    // gait.step_height
#define MO_STEP_L_DEF       60.0f    // gait.step_length
#define MO_CYCLE_DEF       900.0f    // gait.cycle_time (ms)
#define MO_DUTY_DEF          0.5f    // gait.duty  (porsi fase tumpu)
#define MO_SLEW_DEF          3.0f    // gait.slew_rate (unit/detik)
#define MO_PROF_TAU_DEF      0.25f   // gait.profile_tau
#define MO_SETTLE_TAU_DEF    0.10f   // gait.settle_tau

// RDS3235: 0,15 detik per 60 derajat tanpa beban -> 400 der/detik.
// Dengan beban realistis anggap 60-70% dari itu.
#define SERVO_MAX_DPS      400.0f

// Radius acuan rotasi. sx/sy dibagi 100 mm supaya vyaw=1 berarti "kaki di
// radius 100 mm melangkah sepanjang stepLength penuh" — SAMA dengan firmware,
// jangan diubah kalau tidak sekalian mengubah HexaGait.cpp.
#define MO_YAW_RADIUS      100.0f

struct GaitProfile {
    float stepH;      // tinggi angkat kaki (mm)
    float stepL;      // panjang langkah (mm)
    float cycleMs;    // lama 1 siklus penuh (ms)
    float standH;     // tinggi badan (mm), foot z = -standH
    float standR;     // bentang kaki dari pangkal coxa (mm)
};

// Empat profil medan.
// SEMPIT untuk celah 30 cm di R11: bentang = 2*(90 + standR) = 270 mm pada
// standR 45, sisa 3 cm untuk lebar telapak + galat kemudi. Angka 45 ini hasil
// pencarian ('N300'), bukan tebakan — 48 mm sudah tidak muat.
// TANGGA menyalin profileStairs() firmware apa adanya. Perintah 'S' akan
// mengeluhkan femur-nya terlalu cepat; itu memang temuan tentang firmware,
// jangan "dibetulkan" di sini tanpa mengubah firmware juga.
static const GaitProfile PROFIL[4] = {
    {  40.0f, 60.0f,  900.0f, 100.0f, 70.0f },   // 0 DATAR
    {  75.0f, 80.0f, 1200.0f, 110.0f, 70.0f },   // 1 TANGGA
    {  40.0f, 55.0f,  900.0f,  80.0f, 70.0f },   // 2 MERUNDUK
    {  30.0f, 45.0f, 1000.0f, 100.0f, 45.0f }    // 3 SEMPIT
};
static const char* const PROFIL_NAMA[4] = { "DATAR", "TANGGA", "MERUNDUK", "SEMPIT" };

// ---------------------------------------------------------------- rotasi
// Invers dari (Rz(yaw) * Ry(roll) * Rx(pitch)) = Rx(-pitch)*Ry(-roll)*Rz(-yaw).
// URUTAN DIBALIK, bukan sekadar sudutnya dinegatifkan. Firmware memakai fungsi
// maju yang sama dengan sudut negatif; itu benar hanya untuk sudut kecil, dan
// galatnya orde-2 (di 15 der sudah ~1 mm di ujung kaki, di 30 der ~5 mm).
static inline void moRotInv(float x, float y, float z,
                            float pitchRad, float rollRad, float yawRad,
                            float out[3]) {
    float c, s;
    c = cosf(yawRad);  s = sinf(yawRad);            // Rz(-yaw)
    float x1 =  c * x + s * y;
    float y1 = -s * x + c * y;
    float z1 =  z;
    c = cosf(rollRad); s = sinf(rollRad);           // Ry(-roll)
    float x2 =  c * x1 - s * z1;
    float z2 =  s * x1 + c * z1;
    float y2 =  y1;
    c = cosf(pitchRad); s = sinf(pitchRad);         // Rx(-pitch)
    float y3 =  c * y2 + s * z2;
    float z3 = -s * y2 + c * z2;
    out[0] = x2; out[1] = y3; out[2] = z3;
}

// ================================================================ Motion
class Motion {
public:
    // --- keluaran ---
    float legTargets[6][3];      // titik ujung kaki, frame badan (mm)

    // --- pose badan (derajat & mm), dipakai saat solve() ---
    float bodyRoll  = 0, bodyPitch = 0, bodyYaw = 0;
    float bodyTx    = 0, bodyTy    = 0, bodyTz  = 0;

    // --- knob yang di firmware ada di Calib ---
    float duty      = MO_DUTY_DEF;
    float slewRate  = MO_SLEW_DEF;
    float profTau   = MO_PROF_TAU_DEF;
    float settleTau = MO_SETTLE_TAU_DEF;

    GaitProfile prof, tgtProf;

    void begin() {
        prof = tgtProf = PROFIL[0];
        _tX = _tY = _tYaw = _cX = _cY = _cYaw = 0;
        _run = false; _ph = 0;
        computeHome();
        setHome();
    }

    void computeHome() {
        for (uint8_t i = 0; i < 6; i++)
            footHome(i, prof.standR, prof.standH, _home[i]);
    }

    // Paksa kaki ke titik netral tanpa menunggu settle (untuk simulasi & 's').
    void setHome() {
        computeHome();
        for (uint8_t i = 0; i < 6; i++)
            for (uint8_t k = 0; k < 3; k++) legTargets[i][k] = _home[i][k];
    }

    void setMove(float vx, float vy, float vyaw) { _tX = vx; _tY = vy; _tYaw = vyaw; }
    void setProfile(uint8_t i)                   { if (i < 4) tgtProf = PROFIL[i]; }
    void setProfileNow(uint8_t i)                { if (i < 4) { tgtProf = PROFIL[i]; prof = PROFIL[i]; } }

    float velX()   const { return _cX; }
    float velY()   const { return _cY; }
    float velYaw() const { return _cYaw; }
    float tgtX()   const { return _tX; }      // sebelum di-slew — dipakai 'S'
    float tgtY()   const { return _tY; }
    float tgtYaw() const { return _tYaw; }
    float phase()  const { return _ph; }
    bool  running() const { return _run; }

    // dt DETIK. Dipanggil pemanggil, bukan diambil dari millis().
    void update(float dt) {
        if (dt < 0) dt = 0;
        if (dt > 0.05f) dt = 0.05f;               // jeda besar tidak boleh jadi lompatan

        // 1) ramp vektor gerak -> start/stop/belok mulus (ease-in otomatis)
        _cX   = slew(_cX,   _tX,   slewRate, dt);
        _cY   = slew(_cY,   _tY,   slewRate, dt);
        _cYaw = slew(_cYaw, _tYaw, slewRate, dt);

        // 2) ramp profil medan -> ganti profil tanpa badan melonjak
        float ap = dt / (profTau + dt);
        prof.stepH   = lerp(prof.stepH,   tgtProf.stepH,   ap);
        prof.stepL   = lerp(prof.stepL,   tgtProf.stepL,   ap);
        prof.cycleMs = lerp(prof.cycleMs, tgtProf.cycleMs, ap);
        prof.standH  = lerp(prof.standH,  tgtProf.standH,  ap);
        prof.standR  = lerp(prof.standR,  tgtProf.standR,  ap);
        computeHome();

        bool moving = (fabsf(_cX) + fabsf(_cY) + fabsf(_cYaw)) > 0.002f;
        if (moving && !_run) { _run = true; _ph = 0; }

        if (!moving) {
            _run = false;
            float as = dt / (settleTau + dt);      // settle berbasis waktu
            for (uint8_t i = 0; i < 6; i++)
                for (uint8_t k = 0; k < 3; k++)
                    legTargets[i][k] = lerp(legTargets[i][k], _home[i][k], as);
            return;
        }

        // 3) fase DIAKUMULASI. Firmware memakai fmod(elapsed, cycleTime)/cycleTime;
        // saat cycleTime ikut di-ramp (ganti profil), pembaginya berubah di
        // tengah jalan dan fase MELOMPAT — persis di saat robot sedang ganti
        // medan. Menambah dt/cycleTime kebal terhadap itu.
        if (prof.cycleMs < 100.0f) prof.cycleMs = 100.0f;
        _ph += dt * 1000.0f / prof.cycleMs;
        while (_ph >= 1.0f) _ph -= 1.0f;

        for (uint8_t leg = 0; leg < 6; leg++) {
            // tripod: {0,2,4} sefase, {1,3,5} geser setengah siklus
            float lp = _ph + ((leg % 2 == 0) ? 0.0f : 0.5f);
            if (lp >= 1.0f) lp -= 1.0f;

            // vektor langkah kaki ini = translasi + rotasi (v = omega x r)
            float rx = _home[leg][0], ry = _home[leg][1];
            float sx = (_cX + (-_cYaw * ry / MO_YAW_RADIUS)) * prof.stepL;
            float sy = (_cY + ( _cYaw * rx / MO_YAW_RADIUS)) * prof.stepL;

            float dx, dy, dz;
            if (lp < duty) {
                // TUMPU: geser lurus +1/2 -> -1/2, kecepatan konstan -> tidak selip
                float s = lp / duty;
                float k = 0.5f - s;
                dx = sx * k; dy = sy * k; dz = 0.0f;
            } else {
                // AYUN SIKLOID: kecepatan nol saat angkat & mendarat -> tidak menyentak
                float s = (lp - duty) / (1.0f - duty);
                float w = 2.0f * PI_F * s;
                float k = -0.5f + (s - sinf(w) / (2.0f * PI_F));
                dx = sx * k; dy = sy * k;
                dz = prof.stepH * (1.0f - cosf(w)) * 0.5f;
            }
            legTargets[leg][0] = _home[leg][0] + dx;
            legTargets[leg][1] = _home[leg][1] + dy;
            legTargets[leg][2] = _home[leg][2] + dz;
        }
    }

    // legTargets + pose badan -> 18 sudut servo. false = ada yang di luar
    // jangkauan (sudah di-clamp oleh ikSolve).
    bool solve(float out[18]) {
        bool ok = true;
        float pr = bodyPitch * DEG2RAD;
        float rr = bodyRoll  * DEG2RAD;
        float yr = bodyYaw   * DEG2RAD;
        for (uint8_t leg = 0; leg < 6; leg++) {
            float p[3], b[3], d[3];
            p[0] = legTargets[leg][0] - bodyTx;
            p[1] = legTargets[leg][1] - bodyTy;
            p[2] = legTargets[leg][2] - bodyTz;
            moRotInv(p[0], p[1], p[2], pr, rr, yr, b);
            if (!footToServo(leg, b[0], b[1], b[2], d)) ok = false;
            for (uint8_t j = 0; j < 3; j++) out[leg * 3 + j] = d[j];
        }
        return ok;
    }

    // Bentang kiri-kanan TITIK UJUNG KAKI pada pose sekarang (mm).
    // Pose berdiri baku memberi 2*(90+70) = 320 mm = 32 cm, cocok dengan
    // pengukuran fisik. Lebar telapak & bagian yang menonjol BELUM termasuk —
    // itu masuk sebagai margin di 'N'.
    float bentangX() const {
        float mn = 1e9f, mx = -1e9f;
        for (uint8_t i = 0; i < 6; i++) {
            if (legTargets[i][0] < mn) mn = legTargets[i][0];
            if (legTargets[i][0] > mx) mx = legTargets[i][0];
        }
        return mx - mn;
    }

    // Laju jalan teoretis (mm/detik): 1 siklus = 1 langkah stepL per kaki.
    float lajuTeoretis() const { return prof.stepL / (prof.cycleMs / 1000.0f); }

private:
    float _home[6][3];
    float _tX, _tY, _tYaw;      // vektor gerak target
    float _cX, _cY, _cYaw;      // vektor gerak aktual (di-slew)
    float _ph;                  // fase 0..1
    bool  _run;

    static float slew(float cur, float tgt, float rate, float dt) {
        float step = rate * dt;
        if (tgt > cur) return (cur + step < tgt) ? cur + step : tgt;
        if (tgt < cur) return (cur - step > tgt) ? cur - step : tgt;
        return tgt;
    }
    static float lerp(float a, float b, float t) { return a + (b - a) * t; }
};

#endif
