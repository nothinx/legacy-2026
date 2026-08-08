#ifndef HEXAGAIT_H
#define HEXAGAIT_H

// Generator tripod gait (mulus).
//  - stance: kaki menapak, geser LURUS (kecepatan konstan) -> tak slip.
//  - swing : trajektori SIKLOID (kecepatan nol di liftoff & touchdown) -> tak menyentak.
//  - YAW   : tiap kaki ikut komponen rotasi (cross product) -> bisa belok.
//  - vektor langkah DINORMALKAN -> maju+putar tidak saling menambah sampai
//    melewati kemampuan servo (lihat update()).
//  - vektor gerak di-SLEW (ramp) -> start/stop/belok mulus, sekaligus ease-in.
//  - profil medan di-RAMP (interpolasi) -> ganti medan tanpa badan melonjak.
//  - fase DIAKUMULASI dari dt, bukan fmod(elapsed, cycleTime) -> kebal
//    terhadap cycleTime yang berubah saat ganti profil (lihat update()).
//  Semua berbasis waktu (dt), tidak tergantung kecepatan loop.
//
// Output -> legTargets[6] (Vec3) di FRAME BADAN, dibaca oleh Hexapod.
#include "Calib.h"   // GAIT_* runtime
#include "types.h"

struct GaitProfile {
    float stepHeight;   // mm
    float stepLength;   // mm, panjang langkah kaki TERPANJANG (setelah normalisasi)
    float cycleTime;    // ms
    float standHeight;  // mm (foot z = -standHeight)
    float standRadius;  // mm, bentang kaki dari pangkal coxa
};

class HexaGait {
public:
    HexaGait();
    void begin();
    void update();                       // hitung legTargets
    void setMoveVector(float vx, float vy, float vyaw); // -1..1 strafe, maju, putar
    void setProfile(const GaitProfile& p) { _tgtProf = p; }  // di-ramp di update()
    GaitProfile profile() const { return _prof; }

    Vec3 legTargets[6];

private:
    GaitProfile _prof, _tgtProf;     // profil aktif (di-ramp) & target
    Vec3 _footHome[6];               // posisi netral ujung kaki (frame badan)
    float _tgtX, _tgtY, _tgtYaw;     // vektor gerak target
    float _curX, _curY, _curYaw;     // vektor gerak aktual (di-slew)
    float _phase;                    // 0..1, diakumulasi dari dt
    bool _running;
    unsigned long _lastUpdate;
    void computeHome();
    float dtSeconds();
};

#endif
