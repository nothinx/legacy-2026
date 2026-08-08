#include "HexaGait.h"
#include <Arduino.h>

HexaGait::HexaGait() {
    _prof = _tgtProf = { GAIT_STEP_HEIGHT, GAIT_STEP_LENGTH, GAIT_CYCLE_TIME,
                         STAND_HEIGHT, STAND_RADIUS };
    _tgtX = _tgtY = _tgtYaw = 0;
    _curX = _curY = _curYaw = 0;
    _phase = 0;
    _running = false;
    _lastUpdate = 0;
}

void HexaGait::computeHome() {
    // Kaki netral = pangkal coxa + standRadius searah hadap kaki.
    // standRadius ikut PROFIL (bukan #define lagi): tanpa itu profileNarrow()
    // mustahil — satu-satunya cara mengecilkan badan hexapod adalah menarik
    // kakinya masuk.
    for (int i = 0; i < 6; i++) {
        float a = deg2rad(BODY_LEG_ANGLE[i]);
        _footHome[i].x = BODY_LEG_ORIGINS[i][0] + _prof.standRadius * cosf(a);
        _footHome[i].y = BODY_LEG_ORIGINS[i][1] + _prof.standRadius * sinf(a);
        _footHome[i].z = -_prof.standHeight;
    }
}

void HexaGait::begin() {
    computeHome();
    for (int i = 0; i < 6; i++) legTargets[i] = _footHome[i];
    _lastUpdate = millis();
}

void HexaGait::setMoveVector(float vx, float vy, float vyaw) {
    _tgtX = vx; _tgtY = vy; _tgtYaw = vyaw;   // di-slew di update()
}

// dt detik sejak update terakhir, di-clamp agar aman saat jeda besar.
float HexaGait::dtSeconds() {
    unsigned long now = millis();
    float dt = (now - _lastUpdate) / 1000.0f;
    _lastUpdate = now;
    return clampf(dt, 0.0f, 0.05f);
}

// ramp 'cur' menuju 'tgt' dengan laju maks (unit/detik).
static float slew(float cur, float tgt, float rate, float dt) {
    float step = rate * dt;
    if (tgt > cur) return (cur + step < tgt) ? cur + step : tgt;
    if (tgt < cur) return (cur - step > tgt) ? cur - step : tgt;
    return tgt;
}

void HexaGait::update() {
    float dt = dtSeconds();

    // 1) Ramp vektor gerak (start/stop/belok mulus + ease-in otomatis).
    _curX   = slew(_curX,   _tgtX,   GAIT_SLEW_RATE, dt);
    _curY   = slew(_curY,   _tgtY,   GAIT_SLEW_RATE, dt);
    _curYaw = slew(_curYaw, _tgtYaw, GAIT_SLEW_RATE, dt);

    // 2) Ramp profil medan (transisi datar<->tangga<->narrow tanpa lonjakan).
    float ap = dt / (GAIT_PROFILE_TAU + dt);
    _prof.stepHeight  = lerpf(_prof.stepHeight,  _tgtProf.stepHeight,  ap);
    _prof.stepLength  = lerpf(_prof.stepLength,  _tgtProf.stepLength,  ap);
    _prof.cycleTime   = lerpf(_prof.cycleTime,   _tgtProf.cycleTime,   ap);
    _prof.standHeight = lerpf(_prof.standHeight, _tgtProf.standHeight, ap);
    _prof.standRadius = lerpf(_prof.standRadius, _tgtProf.standRadius, ap);

    computeHome();

    bool moving = (fabsf(_curX) + fabsf(_curY) + fabsf(_curYaw)) > 0.002f;
    if (moving && !_running) { _running = true; _phase = 0.0f; }
    if (!moving) {
        _running = false;
        // Settle ke home, berbasis waktu (konstan tau, tak tergantung loop).
        float as = dt / (GAIT_SETTLE_TAU + dt);
        for (int i = 0; i < 6; i++) {
            legTargets[i].x = lerpf(legTargets[i].x, _footHome[i].x, as);
            legTargets[i].y = lerpf(legTargets[i].y, _footHome[i].y, as);
            legTargets[i].z = lerpf(legTargets[i].z, _footHome[i].z, as);
        }
        return;
    }

    // FASE DIAKUMULASI, bukan fmod(millis()-_cycleStart, cycleTime).
    // Cara lama: elapsed terus tumbuh, jadi saat cycleTime ikut di-ramp
    // (profileStairs sambil berjalan) pembaginya berubah di tengah jalan dan
    // fase MELOMPAT — 0,48 setelah berjalan 30 detik, hampir setengah siklus.
    // Artinya kedua tripod bertukar peran seketika: tiga kaki yang menapak
    // langsung disuruh mengayun, tiga yang di udara langsung disuruh menapak,
    // dan badan jatuh — tepat di saat paling berbahaya, memasuki tangga.
    if (_prof.cycleTime < 100.0f) _prof.cycleTime = 100.0f;
    _phase += dt * 1000.0f / _prof.cycleTime;
    while (_phase >= 1.0f) _phase -= 1.0f;

    // Vektor langkah tiap kaki (mm) = translasi + rotasi(yaw).
    // Rotasi: v = omega x r  -> (-yaw*ry, yaw*rx).
    float sxa[6], sya[6], magMax = 0.0f;
    for (int leg = 0; leg < 6; leg++) {
        float rx = _footHome[leg].x, ry = _footHome[leg].y;
        sxa[leg] = (_curX + (-_curYaw * ry / 100.0f)) * _prof.stepLength;
        sya[leg] = (_curY + ( _curYaw * rx / 100.0f)) * _prof.stepLength;
        float m = sqrtf(sxa[leg] * sxa[leg] + sya[leg] * sya[leg]);
        if (m > magMax) magMax = m;
    }
    // NORMALISASI. Perintah maju dan putar SALING MENAMBAH: "maju 0,8 +
    // putar 0,7" — persis yang dihasilkan followWall() setiap saat — membuat
    // kaki terluar melangkah 115 mm padahal stepLength 60 mm, dan coxa diminta
    // 417 der/s (RDS3235 realistis ~260 der/s berbeban). Servo tertinggal,
    // kaki mendarat di tempat yang salah, robot ngesot bukan melangkah.
    // Satu faktor skala untuk KEENAM kaki supaya arah tiap vektor tidak
    // berubah: putaran murni tetap putaran murni, hanya amplitudonya dibatasi.
    if (magMax > _prof.stepLength && magMax > 0.001f) {
        float f = _prof.stepLength / magMax;
        for (int leg = 0; leg < 6; leg++) { sxa[leg] *= f; sya[leg] *= f; }
    }

    for (int leg = 0; leg < 6; leg++) {
        // Tripod: grup {0,2,4} fase 0, grup {1,3,5} geser 0.5.
        float legPhase = _phase + ((leg % 2 == 0) ? 0.0f : 0.5f);
        if (legPhase >= 1.0f) legPhase -= 1.0f;

        float sx = sxa[leg], sy = sya[leg];
        float dx, dy, dz;
        if (legPhase < GAIT_DUTY) {
            // STANCE: geser lurus +1/2 -> -1/2 (dorong badan maju), kecepatan konstan.
            float s = legPhase / GAIT_DUTY;          // 0..1
            float k = 0.5f - s;
            dx = sx * k; dy = sy * k; dz = 0.0f;
        } else {
            // SWING SIKLOID: kecepatan nol di liftoff & touchdown -> mendarat lembut.
            float s = (legPhase - GAIT_DUTY) / (1.0f - GAIT_DUTY); // 0..1
            float twoPiS = 2.0f * (float)M_PI * s;
            float k = -0.5f + (s - sinf(twoPiS) / (2.0f * (float)M_PI)); // -0.5 -> +0.5
            dx = sx * k; dy = sy * k;
            dz = _prof.stepHeight * (1.0f - cosf(twoPiS)) * 0.5f;        // 0 -> peak -> 0
        }

        legTargets[leg].x = _footHome[leg].x + dx;
        legTargets[leg].y = _footHome[leg].y + dy;
        legTargets[leg].z = _footHome[leg].z + dz;
    }
}
