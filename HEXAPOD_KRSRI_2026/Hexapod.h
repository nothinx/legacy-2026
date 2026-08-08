#ifndef HEXAPOD_H
#define HEXAPOD_H

// Fasad: SATU-SATUNYA class yang dipakai di .ino & misi.
// Mengurus: gait -> body transform -> IK per kaki -> pulse servo (terkalibrasi).
#include "Calib.h"   // config.h + kalibrasi runtime
#include "types.h"
#include "HexaServos.h"
#include "HexaGait.h"
#include "HexaArm.h"
#include "LegIK.h"

class Hexapod {
public:
    Hexapod();
    void begin();
    void update();                       // panggil tiap loop

    // Gerak: maju(+)/mundur(-), geser kanan(+)/kiri(-), putar CCW(+)/CW(-). -1..1
    void walk(float forward, float strafe = 0.0f, float turn = 0.0f);
    void stop();

    // Stabilisasi badan dari IMU (derajat).
    //   rollDeg  = miring kanan(+)/kiri(-)   -> rotasi terhadap sumbu +Y (depan)
    //   pitchDeg = mendongak(+)/menunduk(-)  -> rotasi terhadap sumbu +X (kanan)
    // Di-clamp + deadband + smooth. Lihat STAB_SWAP_ROLL_PITCH di config.h
    // kalau IMU terpasang menyudut 90 derajat terhadap badan.
    void setStabilization(float rollDeg, float pitchDeg);

    // Pose badan manual (derajat & mm) -- untuk menunduk/menjinjit dsb.
    void setBodyTranslation(float x, float y, float z);

    // Profil gait medan.
    void profileFlat();
    void profileStairs();
    void profileCrouch();
    void profileNarrow();     // celah sempit (R11) — menarik kaki masuk
    void setGaitProfile(const GaitProfile& p) { _gait.setProfile(p); }

    HexaArm* armRight() { return &_armR; }
    HexaArm* armLeft()  { return &_armL; }
    HexaArm* arm()      { return &_armR; }  // default = kanan

    // Tuner: gerak mentah 1 servo (id 0..NUM_TUNE_SERVOS-1, lihat TUNE_PIN_MAP). Langsung, tanpa gait.
    void jog(uint8_t tuneId, uint16_t pulseUs);

private:
    HexaServos _servos;
    HexaGait   _gait;
    HexaArm    _armR;
    HexaArm    _armL;

    // Pose badan (radian, mm), hasil smoothing. Dinamai per SUMBU, bukan
    // roll/pitch: di frame ini (+X kanan, +Y depan) rotasi terhadap X adalah
    // pitch dan terhadap Y adalah roll — mudah tertukar, dan pernah tertukar.
    float _rotX, _rotY, _rotZ;   // X=pitch(dongak) Y=roll(miring) Z=yaw
    Vec3  _trans;
    uint32_t _lastStabT;   // untuk low-pass stabilisasi berbasis dt

    void solvePose();
    uint16_t angleToPulse(uint8_t servoID, float geoAngleDeg, float baseline);
};

#endif
