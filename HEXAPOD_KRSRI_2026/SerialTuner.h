#ifndef SERIALTUNER_H
#define SERIALTUNER_H

// Mode tuning via serial (untuk GUI hexapod_tuner.html, Web Serial).
// Protokol baris ASCII @115200. Mode TUNE menyuspend misi & gait; servo bisa di-jog.
// Boot tetap jalur normal: TUNE hanya aktif setelah GUI kirim "MODE TUNE".
#include <Arduino.h>
#include "Calib.h"

class Hexapod;

class SerialTuner {
public:
    SerialTuner(Hexapod* robot, Stream* io = &Serial) : _r(robot), _io(io),
        _tune(false), _jog(false), _len(0) {}

    // Panggil di awal loop(). return true bila mode TUNE aktif -> loop utama harus return.
    bool update();
    bool tuneActive() const { return _tune; }

private:
    Hexapod* _r;
    Stream*  _io;
    bool _tune;          // mode TUNE aktif (misi & stabilisasi disuspend)
    bool _jog;           // sub-mode JOG: gait juga disuspend (servo dipegang manual)
    char _buf[72];
    uint8_t _len;

    void handleLine(char* line);
    void dump();
};

#endif
