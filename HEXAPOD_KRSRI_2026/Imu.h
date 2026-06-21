#ifndef IMU_H
#define IMU_H

// Pembaca IMU Yahboom 10-axis (protokol WIT, frame 0x55) via UART.
// roll/pitch (derajat) untuk stabilisasi, yaw (0..360) untuk heading/kompas.
// Matang: checksum, gate lonjakan yaw (anti gangguan magnet), tare roll/pitch.
#include <Arduino.h>
#include "config.h"
#include "types.h"

class Imu {
public:
    Imu();
    void begin();
    void update();                 // non-blocking, panggil tiap loop

    // roll/pitch relatif tare (0 saat datar). yaw absolut (kompas).
    float rollDeg()  { return _roll  - _roll0; }
    float pitchDeg() { return _pitch - _pitch0; }
    float yawDeg()   { return _yaw; }

    void tare() { _roll0 = _roll; _pitch0 = _pitch; }  // panggil saat robot datar & diam
    bool hasData() const { return _have; }

private:
    static const int BUF = 11;
    byte _rx[BUF];
    int  _idx;
    bool _frame;
    bool _have;
    float _roll, _pitch, _yaw;
    float _roll0, _pitch0;
};

#endif
