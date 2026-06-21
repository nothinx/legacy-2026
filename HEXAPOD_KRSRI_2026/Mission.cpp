#include "Mission.h"
#include "Navigation.h"

// --- AMBANG ARENA (TUNE di lapangan) ---
static const int  VICTIM_REACH_CM = 12;   // jarak depan dianggap "sampai korban"
static const int  SZ_REACH_CM     = 12;   // jarak dianggap "sampai safe zone"
static const int  STAIR_REACH_CM  = 15;   // jarak dianggap "di depan tangga"
static const uint32_t STATE_TIMEOUT = 25000; // ms, recovery bila tersangkut

Mission::Mission(Hexapod* robot, Imu* imu, LidarArray* lidar)
    : _r(robot), _imu(imu), _l(lidar) {
    _state = M_IDLE; _stateT0 = 0; _lastT = 0; _subStep = 0;
}

void Mission::start() { enter(M_TO_VICTIM1); }

void Mission::enter(MissionState s) {
    _state = s; _stateT0 = millis(); _subStep = 0;
    _headPid.reset(); _wallPid.reset();   // target lompat antar-state -> reset D
    _r->stop();
}

void Mission::apply(const NavCmd& c) { _r->walk(c.forward, c.strafe, c.turn); }

// Wall-follow dgn dukungan arena cermin (arena.mirror=1 -> ikut dinding KIRI, heading tercermin).
// Cermin kiri-kanan: heading' = (360 - heading) mod 360 (Timur<->Barat, Utara/Selatan tetap).
static NavCmd followWall(Pid& hp, Pid& wp, bool mir, float yaw, float head,
                         int front, int right, int left, float dt) {
    float h = mir ? fmodf(360.0f - head, 360.0f) : head;
    return mir ? Nav::followWallLeft (hp, wp, yaw, h, front, left,  dt)
               : Nav::followWallRight(hp, wp, yaw, h, front, right, dt);
}
static float mirrorHead(bool mir, float head) { return mir ? fmodf(360.0f - head, 360.0f) : head; }

// Sekuens lengan ambil korban (non-blocking). return true bila selesai.
// ponytail: default lengan kanan; pilih sisi via x_norm vision saat itu disambung.
static bool runPick(Hexapod* r, int& step, HexaArm* a) {
    switch (step) {
        case 0: r->stop(); r->profileCrouch(); a->goTo(ARM_REACH); step++; break;
        case 1: if (!a->isMoving()) { a->goTo(ARM_GRIP); step++; } break;
        case 2: if (!a->isMoving()) { a->goTo(ARM_LIFT); r->profileFlat(); step++; } break;
        case 3: if (!a->isMoving()) return true; break;
    }
    return false;
}
// Sekuens taruh korban di safe zone.
static bool runDrop(Hexapod* r, int& step, HexaArm* a) {
    switch (step) {
        case 0: r->stop(); a->goTo(ARM_DROP); step++; break;
        case 1: if (!a->isMoving()) { a->goTo(ARM_PARK); step++; } break;
        case 2: if (!a->isMoving()) return true; break;
    }
    return false;
}

void Mission::update() {
    uint32_t now = millis();
    float dt = (_lastT == 0) ? 0.0f : (now - _lastT) * 0.001f;  // detik; dt=0 -> D=0
    _lastT = now;

    // Ambil gain PID dari kalibrasi runtime (tuning live via GUI berlaku tanpa restart).
    _headPid.kp = HEADING_KP; _headPid.kd = HEADING_KD;
    _wallPid.kp = WALL_KP;    _wallPid.kd = WALL_KD;

    float yaw  = _imu->yawDeg();
    int front  = _l->getDistance(LIDAR_FRONT);
    int right  = _l->getDistance(LIDAR_RIGHT);
    int left   = _l->getDistance(LIDAR_LEFT);

    // Arena cermin (hadap kiri): ikut dinding kiri + heading tercermin + lengan sisi kiri.
    bool mir = gParam[K_ARENA_MIRROR] >= 0.5f;
    HexaArm* pickArm = mir ? _r->armLeft() : _r->armRight();

    // recovery global: state navigasi yang kelamaan -> mundur sebentar lalu lanjut.
    bool navState = (_state == M_TO_VICTIM1 || _state == M_TO_SZ1 ||
                     _state == M_TO_VICTIM2 || _state == M_TO_SZ2 ||
                     _state == M_TO_STAIRS);
    if (navState && elapsed() > STATE_TIMEOUT) {
        _r->walk(-0.6f, 0, 0);   // mundur lepas dari sangkutan
        if (elapsed() > STATE_TIMEOUT + 1500) _stateT0 = millis(); // reset window
        return;
    }

    switch (_state) {
        case M_IDLE:
            _r->stop();
            break;

        case M_TO_VICTIM1:
            apply(followWall(_headPid, _wallPid, mir, yaw, HEAD_UTARA, front, right, left, dt));
            if (front >= 0 && front <= VICTIM_REACH_CM) enter(M_PICK_VICTIM1);
            break;

        case M_PICK_VICTIM1:
            if (runPick(_r, _subStep, pickArm)) enter(M_TO_SZ1);
            break;

        case M_TO_SZ1:
            apply(followWall(_headPid, _wallPid, mir, yaw, HEAD_BARAT, front, right, left, dt));
            if (front >= 0 && front <= SZ_REACH_CM) enter(M_DROP_SZ1);
            break;

        case M_DROP_SZ1:
            if (runDrop(_r, _subStep, pickArm)) enter(M_TO_VICTIM2);
            break;

        case M_TO_VICTIM2:
            apply(followWall(_headPid, _wallPid, mir, yaw, HEAD_SELATAN, front, right, left, dt));
            if (front >= 0 && front <= VICTIM_REACH_CM) enter(M_PICK_VICTIM2);
            break;

        case M_PICK_VICTIM2:
            if (runPick(_r, _subStep, pickArm)) enter(M_TO_SZ2);
            break;

        case M_TO_SZ2:
            apply(followWall(_headPid, _wallPid, mir, yaw, HEAD_TIMUR, front, right, left, dt));
            if (front >= 0 && front <= SZ_REACH_CM) { runDrop(_r, _subStep, pickArm); enter(M_TO_STAIRS); }
            break;

        case M_TO_STAIRS:
            apply(followWall(_headPid, _wallPid, mir, yaw, HEAD_SELATAN, front, right, left, dt));
            if (front >= 0 && front <= STAIR_REACH_CM) enter(M_CLIMB_STAIRS);
            break;

        case M_CLIMB_STAIRS:
            _r->profileStairs();
            apply(Nav::goStraight(_headPid, yaw, mirrorHead(mir, HEAD_SELATAN), dt));
            // selesai naik bila pitch kembali ~datar setelah sempat menanjak (TUNE),
            // atau lidar depan terbuka lagi.
            if (elapsed() > 8000 && front > 40) { _r->profileFlat(); enter(M_DONE); }
            break;

        case M_DONE:
            _r->stop();
            break;
    }
}
