#ifndef MISSION_H
#define MISSION_H

// State machine misi KRSRI (closed-loop, NON-BLOCKING).
// Mengganti rantai while-blocking dead-reckoning generasi lama.
// Blueprint urutan dari firmware kontes 2024:
//   korban1 -> ambil -> SZ1 -> korban2 -> SZ2 -> lantai2 -> ambil2 -> SZ3 -> tangga
//
// Tiap state: aksi closed-loop + syarat keluar + TIMEOUT (recovery).
// Nilai ambang & heading ditandai TUNE -> kalibrasi di arena.
#include "Hexapod.h"
#include "Imu.h"
#include "LidarArray.h"
#include "Navigation.h"   // Pid + NavCmd

enum MissionState {
    M_IDLE,            // tunggu tombol
    M_TO_VICTIM1,      // navigasi ke korban 1
    M_PICK_VICTIM1,    // ambil korban 1 dgn lengan
    M_TO_SZ1,          // ke safe zone 1
    M_DROP_SZ1,
    M_TO_VICTIM2,
    M_PICK_VICTIM2,
    M_TO_SZ2,
    M_TO_STAIRS,       // ke tangga
    M_CLIMB_STAIRS,    // naik tangga (profil khusus)
    M_DONE
};

class Mission {
public:
    Mission(Hexapod* robot, Imu* imu, LidarArray* lidar);
    void start();                  // mulai dari awal
    void update();                 // panggil tiap loop
    MissionState state() const { return _state; }

private:
    Hexapod* _r; Imu* _imu; LidarArray* _l;
    MissionState _state;
    uint32_t _stateT0;             // waktu masuk state (untuk timeout)
    uint32_t _lastT;               // untuk dt loop (PD)
    int _subStep;                  // langkah dalam sekuens lengan
    Pid _headPid{ HEADING_KP, HEADING_KD };
    Pid _wallPid{ WALL_KP, WALL_KD };

    void enter(MissionState s);
    uint32_t elapsed() const { return millis() - _stateT0; }
    void apply(const NavCmd& c);
};

#endif
