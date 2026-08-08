#include "Hexapod.h"
#include <Arduino.h>

Hexapod::Hexapod() : _armR(&_servos, ARM_PIN_MAP_R), _armL(&_servos, ARM_PIN_MAP_L) {
    _rotX = _rotY = _rotZ = 0.0f;
    _trans = {0, 0, 0};
    _lastStabT = 0;
}

void Hexapod::begin() {
    _servos.begin();
    _gait.begin();
    _armR.begin();
    _armL.begin();
    profileFlat();
}

void Hexapod::update() {
    _gait.update();
    solvePose();
    _armR.update();
    _armL.update();
    _servos.commit();
}

void Hexapod::walk(float forward, float strafe, float turn) {
    // gait: vx=strafe, vy=forward, vyaw=turn
    _gait.setMoveVector(strafe, forward, turn);
}

void Hexapod::stop() { _gait.setMoveVector(0, 0, 0); }

void Hexapod::setStabilization(float rollDeg, float pitchDeg) {
#if STAB_SWAP_ROLL_PITCH
    // IMU terpasang menyudut 90 der terhadap badan (lihat config.h).
    float tmp = rollDeg; rollDeg = pitchDeg; pitchDeg = tmp;
#endif
    // deadband
    if (fabsf(rollDeg)  < STAB_DEADBAND_DEG) rollDeg  = 0;
    if (fabsf(pitchDeg) < STAB_DEADBAND_DEG) pitchDeg = 0;
    // clamp
    rollDeg  = clampf(rollDeg,  -STAB_MAX_DEG, STAB_MAX_DEG);
    pitchDeg = clampf(pitchDeg, -STAB_MAX_DEG, STAB_MAX_DEG);
    // low-pass berbasis dt (konstan tau -> kehalusan tak tergantung kecepatan loop)
    uint32_t now = millis();
    float dt = _lastStabT ? (now - _lastStabT) / 1000.0f : 0.02f;
    _lastStabT = now;
    dt = clampf(dt, 0.0f, 0.05f);
    float a = dt / (STAB_TAU + dt);
    // PEMETAAN SUMBU — inilah yang dulu tertukar:
    //   roll (miring kanan/kiri) = rotasi terhadap sumbu DEPAN  +Y  -> _rotY
    //   pitch (dongak/menunduk)  = rotasi terhadap sumbu KANAN  +X  -> _rotX
    _rotY = lerpf(_rotY, deg2rad(rollDeg),  a);
    _rotX = lerpf(_rotX, deg2rad(pitchDeg), a);
}

void Hexapod::setBodyTranslation(float x, float y, float z) { _trans = {x, y, z}; }

void Hexapod::jog(uint8_t tuneId, uint16_t pulseUs) {
    if (tuneId >= NUM_TUNE_SERVOS) return;
    _servos.writeRaw(TUNE_PIN_MAP[tuneId][0], TUNE_PIN_MAP[tuneId][1], pulseUs);
}

// Selisih profil ditulis relatif GAIT_* supaya penyetelan lewat GUI tuner ikut
// terbawa. Angka siklusnya BUKAN pilihan estetika: hasil pencarian di
// TES_GERAK/test_motion.py supaya laju sendi tertinggi tetap di bawah ~260
// der/s (RDS3235 berbeban). Ubah salah satunya -> jalankan lagi test itu.
void Hexapod::profileFlat() {
    // laju sendi maks 256 der/s — tepat di batas, tanpa cadangan.
    // Kalau servo terlihat tertinggal, naikkan gait.cycle_time ke 1000.
    _gait.setProfile({ GAIT_STEP_HEIGHT, GAIT_STEP_LENGTH, GAIT_CYCLE_TIME,
                       STAND_HEIGHT, STAND_RADIUS });
}
void Hexapod::profileStairs() {
    // Angkat kaki jauh lebih tinggi (harus melewati anak tangga) dan JAUH
    // lebih lambat. Versi lama (+300 ms) meminta femur 388 der/s — mustahil
    // berbeban; +900 ms menurunkannya ke 252 der/s. Jalan jadi 3,9 cm/detik,
    // dan itu memang harga menaiki tangga.
    _gait.setProfile({ GAIT_STEP_HEIGHT + 35.0f, GAIT_STEP_LENGTH + 10.0f,
                       GAIT_CYCLE_TIME + 900.0f, STAND_HEIGHT + 10.0f, STAND_RADIUS });
}
void Hexapod::profileCrouch() {
    // Menunduk untuk masuk celah / ambil korban rendah. Badan rendah membuat
    // femur bekerja lebih keras (303 der/s pada siklus lama), jadi ikut
    // diperlambat ke 1100 ms -> 248 der/s.
    _gait.setProfile({ GAIT_STEP_HEIGHT, GAIT_STEP_LENGTH - 5.0f,
                       GAIT_CYCLE_TIME + 200.0f, STAND_HEIGHT - 20.0f, STAND_RADIUS });
}
void Hexapod::profileNarrow() {
    // R11: celah 30 cm sedangkan berdiri normal 32 cm. Satu-satunya cara
    // mengecilkan hexapod adalah MENARIK KAKI MASUK -> standRadius 70 -> 45,
    // bentang 32,0 -> 27,0 cm, sisa 3 cm untuk lebar telapak & galat kemudi.
    // (48 mm sudah tidak muat — dicari di TES_GERAK 'N300'.)
    //
    // BATASAN: yang melebarkan badan adalah MENGGESER MENYAMPING (strafe) —
    // bentang jadi 31,5 cm, lebih lebar dari celahnya. BERBELOK tidak: kaki
    // tengah, yang paling lebar, bergerak searah badan saat berputar. Jadi
    // koreksi heading di dalam celah aman; strafe tidak. Navigation memang
    // tak pernah mengisi NavCmd.strafe — jangan mulai sekarang.
    _gait.setProfile({ GAIT_STEP_HEIGHT - 10.0f, GAIT_STEP_LENGTH - 15.0f,
                       GAIT_CYCLE_TIME + 100.0f, STAND_HEIGHT, STAND_RADIUS - 25.0f });
}

// geoAngle (derajat) -> pulse, dengan kalibrasi per-servo.
// servoAngle = baseline + offset + (invert? -geo : geo); lalu map ke pulse + trim.
uint16_t Hexapod::angleToPulse(uint8_t id, float geoAngleDeg, float baseline) {
    float s = SERVO_INVERT[id] ? -geoAngleDeg : geoAngleDeg;
    float servoAngle = baseline + SERVO_OFFSET[id] + s;
    servoAngle = clampf(servoAngle, 0.0f, 180.0f);
    int pulse = SERVO_PULSE_MIN +
        (int)((servoAngle / 180.0f) * (SERVO_PULSE_MAX - SERVO_PULSE_MIN));
    pulse += SERVO_TRIM_US[id];
    return (uint16_t)constrain(pulse, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
}

void Hexapod::solvePose() {
    for (int leg = 0; leg < 6; leg++) {
        Vec3 foot = _gait.legTargets[leg];

        // 1) Body transform (INVERS): supaya kaki tetap di target saat badan
        // miring/geser. rotatePointInv, bukan rotatePoint dengan sudut
        // dinegatifkan — yang terakhir bukan invers begitu dua sumbu aktif
        // bersamaan (lihat types.h; meleset ~10 mm pada roll+pitch 15 der).
        Vec3 p = { foot.x - _trans.x, foot.y - _trans.y, foot.z - _trans.z };
        Vec3 pb = rotatePointInv(p, _rotX, _rotY, _rotZ);

        // 2) Relatif pangkal coxa.
        float vx = pb.x - BODY_LEG_ORIGINS[leg][0];
        float vy = pb.y - BODY_LEG_ORIGINS[leg][1];
        float vz = pb.z - BODY_LEG_ORIGINS[leg][2];

        // 3) Rotasi ke frame kaki (neutral menghadap +X).
        float a = -deg2rad(BODY_LEG_ANGLE[leg]);
        float lx = cosf(a) * vx - sinf(a) * vy;
        float ly = sinf(a) * vx + cosf(a) * vy;
        float lz = vz;

        // 4) IK.
        float coxa, femur, tibia;
        LegIK::solve(lx, ly, lz, coxa, femur, tibia);

        // 5) Ke pulse (baseline 90 untuk coxa/femur; tibia dipusatkan di 90).
        uint8_t c = leg * 3 + 0, f = leg * 3 + 1, t = leg * 3 + 2;
        _servos.setLegPulse(c, angleToPulse(c, coxa,          90.0f));
        _servos.setLegPulse(f, angleToPulse(f, femur,         90.0f));
        _servos.setLegPulse(t, angleToPulse(t, tibia - 90.0f, 90.0f));
    }
}
