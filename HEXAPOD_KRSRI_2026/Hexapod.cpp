#include "Hexapod.h"
#include <Arduino.h>

Hexapod::Hexapod() : _armR(&_servos, ARM_PIN_MAP_R), _armL(&_servos, ARM_PIN_MAP_L) {
    _roll = _pitch = _yaw = 0.0f;
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
    _roll  = lerpf(_roll,  deg2rad(rollDeg),  a);
    _pitch = lerpf(_pitch, deg2rad(pitchDeg), a);
}

void Hexapod::setBodyTranslation(float x, float y, float z) { _trans = {x, y, z}; }

void Hexapod::profileFlat() {
    _gait.setProfile({ GAIT_STEP_HEIGHT, GAIT_STEP_LENGTH, GAIT_CYCLE_TIME, STAND_HEIGHT });
}
void Hexapod::profileStairs() {
    // langkah lebih tinggi & panjang, badan sedikit lebih tinggi, lebih lambat.
    _gait.setProfile({ GAIT_STEP_HEIGHT + 35.0f, GAIT_STEP_LENGTH + 20.0f,
                       GAIT_CYCLE_TIME + 300.0f, STAND_HEIGHT + 10.0f });
}
void Hexapod::profileCrouch() {
    // menunduk untuk masuk celah / ambil korban rendah.
    _gait.setProfile({ GAIT_STEP_HEIGHT, GAIT_STEP_LENGTH, GAIT_CYCLE_TIME, STAND_HEIGHT - 20.0f });
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

        // 1) Body transform (inverse): supaya kaki tetap di target saat badan miring/geser.
        Vec3 p = { foot.x - _trans.x, foot.y - _trans.y, foot.z - _trans.z };
        Vec3 pb = rotatePoint(p, -_roll, -_pitch, -_yaw);

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
