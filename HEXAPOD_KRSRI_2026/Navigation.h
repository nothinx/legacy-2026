#ifndef NAVIGATION_H
#define NAVIGATION_H

// Helper navigasi closed-loop (header-only, inline).
// Menghasilkan perintah gerak {forward, strafe, turn} dari sensor.
// FSM misi yang memanggil robot.walk(...) tiap loop -> non-blocking.
#include "Calib.h"   // WALL_SETPOINT_CM runtime
#include "types.h"

struct NavCmd { float forward, strafe, turn; };

// Kontroler PD (heading & wall) dimiliki Mission; dt (detik) dihitung dari millis.
namespace Nav {

    // Belok di tempat menuju heading target (derajat kompas).
    inline NavCmd holdHeading(Pid& head, float yawNow, float yawTarget, float dt) {
        float turn = clampf(head.step(angleDiffDeg(yawTarget, yawNow), dt), -1.0f, 1.0f);
        return { 0.0f, 0.0f, turn };
    }
    inline bool atHeading(float yawNow, float yawTarget) {
        return fabsf(angleDiffDeg(yawTarget, yawNow)) <= HEADING_TOL_DEG;
    }

    // Maju sambil ikut dinding KANAN, jaga heading. front/right dalam cm.
    inline NavCmd followWallRight(Pid& head, Pid& wall, float yawNow, float yawTarget,
                                  int frontCm, int rightCm, float dt) {
        if (frontCm >= 0 && frontCm < FRONT_STOP_CM) {
            head.reset(); wall.reset();                  // hindari lonjakan D saat resume
            return { 0.0f, 0.0f, 0.6f };                 // depan mentok -> putar kiri
        }
        float h = head.step(angleDiffDeg(yawTarget, yawNow), dt);
        // turn>0 = CCW(kiri). Terlalu jauh dari dinding kanan -> belok kanan (turn<0).
        float w = (rightCm >= 0) ? -wall.step((float)(rightCm - WALL_SETPOINT_CM), dt) : 0.0f;
        float turn = clampf(h + w, -1.0f, 1.0f);
        return { NAV_FWD_SPEED, 0.0f, turn };
    }

    // Maju sambil ikut dinding KIRI.
    inline NavCmd followWallLeft(Pid& head, Pid& wall, float yawNow, float yawTarget,
                                 int frontCm, int leftCm, float dt) {
        if (frontCm >= 0 && frontCm < FRONT_STOP_CM) {
            head.reset(); wall.reset();
            return { 0.0f, 0.0f, -0.6f };                // depan mentok -> putar kanan
        }
        float h = head.step(angleDiffDeg(yawTarget, yawNow), dt);
        // Terlalu jauh dari dinding kiri -> belok kiri (turn>0).
        float w = (leftCm >= 0) ? wall.step((float)(leftCm - WALL_SETPOINT_CM), dt) : 0.0f;
        float turn = clampf(h + w, -1.0f, 1.0f);
        return { NAV_FWD_SPEED, 0.0f, turn };
    }

    // Maju lurus jaga heading saja (tanpa dinding).
    inline NavCmd goStraight(Pid& head, float yawNow, float yawTarget, float dt) {
        float turn = clampf(head.step(angleDiffDeg(yawTarget, yawNow), dt), -1.0f, 1.0f);
        return { NAV_FWD_SPEED, 0.0f, turn };
    }
}

#endif
