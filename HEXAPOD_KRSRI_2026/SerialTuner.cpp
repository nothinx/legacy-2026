#include "SerialTuner.h"
#include "Hexapod.h"
#include <stdlib.h>
#include <string.h>

bool SerialTuner::update() {
    // 1) Baca baris masuk (non-blocking).
    while (_io->available()) {
        char c = (char)_io->read();
        if (c == '\r') continue;
        if (c == '\n') { _buf[_len] = 0; if (_len) handleLine(_buf); _len = 0; }
        else if (_len < sizeof(_buf) - 1) _buf[_len++] = c;
        else { _len = 0; _io->println("ERR overflow"); }
    }
    // 2) HOLD: jalankan gait di pose home agar efek kalibrasi terlihat live.
    //    JOG: gait disuspend, servo dipegang nilai writeRaw terakhir.
    if (_tune && !_jog) { _r->stop(); _r->update(); }
    return _tune;
}

void SerialTuner::handleLine(char* line) {
    char* cmd = strtok(line, " ");
    if (!cmd) return;

    if (!strcmp(cmd, "MODE")) {
        char* m = strtok(NULL, " ");
        if (m && !strcmp(m, "TUNE")) { _tune = true;  _jog = false; _io->println("OK"); }
        else if (m && !strcmp(m, "RUN")) { _tune = false; _jog = false; _io->println("OK"); }
        else _io->println("ERR mode");
    }
    else if (!strcmp(cmd, "DUMP")) {
        dump();
    }
    else if (!strcmp(cmd, "SET")) {
        char* name = strtok(NULL, " ");
        char* val  = strtok(NULL, " ");
        if (name && val && Calib::setParam(name, (float)atof(val))) _io->println("OK");
        else _io->println("ERR set");
    }
    else if (!strcmp(cmd, "SVO")) {
        char* sid = strtok(NULL, " ");
        char* off = strtok(NULL, " ");
        char* trm = strtok(NULL, " ");
        char* inv = strtok(NULL, " ");
        int id = sid ? atoi(sid) : -1;
        if (id >= 0 && id < NUM_SERVOS && off && trm && inv) {
            gOffset[id] = (float)atof(off);
            gTrim[id]   = (int16_t)atoi(trm);
            gInvert[id] = atoi(inv) ? 1 : 0;
            _io->println("OK");
        } else _io->println("ERR svo");
    }
    else if (!strcmp(cmd, "JOG")) {
        char* sid = strtok(NULL, " ");
        char* pul = strtok(NULL, " ");
        if (!_tune) { _io->println("ERR not in tune mode"); return; }
        int id = sid ? atoi(sid) : -1;
        int p  = pul ? atoi(pul) : -1;
        if (id >= 0 && id < NUM_TUNE_SERVOS && p > 0) {
            if (p < SERVO_PULSE_MIN) p = SERVO_PULSE_MIN;
            if (p > SERVO_PULSE_MAX) p = SERVO_PULSE_MAX;
            _jog = true;
            _r->jog((uint8_t)id, (uint16_t)p);
            _io->println("OK");
        } else _io->println("ERR jog");
    }
    else if (!strcmp(cmd, "HOME")) {
        _jog = false;
        if (_tune) _r->profileFlat();
        _io->println("OK");
    }
    else if (!strcmp(cmd, "SAVE"))     { Calib::save();          _io->println("OK"); }
    else if (!strcmp(cmd, "LOAD"))     { Calib::load();          _io->println("OK"); }
    else if (!strcmp(cmd, "DEFAULTS")) { Calib::applyDefaults(); _io->println("OK"); }
    else _io->println("ERR unknown");
}

void SerialTuner::dump() {
    _io->print("INFO "); _io->print(NUM_TUNE_SERVOS);
    _io->print(" ");     _io->println(NUM_SERVOS);          // jog servos, calib servos
    for (int i = 0; i < N_PARAMS; i++) {
        _io->print("P ");  _io->print(PARAM_DEFS[i].name);
        _io->print(" ");   _io->print(gParam[i], 4);
        _io->print(" ");   _io->print(PARAM_DEFS[i].lo, 4);
        _io->print(" ");   _io->println(PARAM_DEFS[i].hi, 4);
    }
    for (int i = 0; i < NUM_SERVOS; i++) {
        _io->print("SVO "); _io->print(i);
        _io->print(" ");    _io->print(gOffset[i], 2);
        _io->print(" ");    _io->print(gTrim[i]);
        _io->print(" ");    _io->println(gInvert[i]);
    }
    _io->println("END");
}
