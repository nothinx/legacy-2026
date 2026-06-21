#include "Calib.h"
#include <string.h>

#ifdef ARDUINO
  #include <EEPROM.h>
#else
  // Stub host (untuk test/ tanpa Teensyduino). Simpan di RAM.
  static uint8_t HOST_EE[1024];
  struct HostEE {
      template <class T> T&       get(int a, T& t)       { memcpy(&t, HOST_EE + a, sizeof(T)); return t; }
      template <class T> const T& put(int a, const T& t) { memcpy(HOST_EE + a, &t, sizeof(T)); return t; }
  } EEPROM;
#endif

#define CALIB_VERSION 1
#define CALIB_ADDR    0

// Default + rentang slider GUI. URUTAN HARUS sama dengan enum ParamId.
const ParamDef PARAM_DEFS[N_PARAMS] = {
    { "pulse.min",        500.0f,  400.0f, 1200.0f },
    { "pulse.max",       2500.0f, 1800.0f, 2600.0f },
    { "gait.step_height",  40.0f,    0.0f,  120.0f },
    { "gait.step_length",  60.0f,    0.0f,  150.0f },
    { "gait.cycle_time",  900.0f,  300.0f, 2000.0f },
    { "gait.duty",          0.5f,    0.3f,    0.7f },
    { "gait.slew_rate",     3.0f,    0.5f,   10.0f },
    { "gait.profile_tau",   0.25f,   0.05f,   1.0f },
    { "gait.settle_tau",    0.10f,   0.02f,   0.5f },
    { "stab.tau",           0.08f,   0.02f,   0.5f },
    { "stab.sign_roll",    -1.0f,   -1.0f,    1.0f },
    { "stab.sign_pitch",   -1.0f,   -1.0f,    1.0f },
    { "heading.kp",         0.020f,  0.0f,    0.1f },
    { "heading.kd",         0.004f,  0.0f,    0.05f },
    { "wall.kp",            0.030f,  0.0f,    0.1f },
    { "wall.kd",            0.010f,  0.0f,    0.05f },
    { "wall.setpoint",     13.0f,    5.0f,   30.0f },
    { "head.utara",         0.0f,    0.0f,  360.0f },
    { "head.timur",        90.0f,    0.0f,  360.0f },
    { "head.selatan",     180.0f,    0.0f,  360.0f },
    { "head.barat",       270.0f,    0.0f,  360.0f },
    { "arm.park.base",     90.0f,    0.0f,  180.0f },
    { "arm.park.shoulder", 30.0f,    0.0f,  180.0f },
    { "arm.park.grip",     20.0f,    0.0f,  180.0f },
    { "arm.reach.base",    90.0f,    0.0f,  180.0f },
    { "arm.reach.shoulder",150.0f,   0.0f,  180.0f },
    { "arm.reach.grip",    70.0f,    0.0f,  180.0f },
    { "arm.grip.base",     90.0f,    0.0f,  180.0f },
    { "arm.grip.shoulder",150.0f,    0.0f,  180.0f },
    { "arm.grip.grip",     10.0f,    0.0f,  180.0f },
    { "arm.lift.base",     90.0f,    0.0f,  180.0f },
    { "arm.lift.shoulder", 60.0f,    0.0f,  180.0f },
    { "arm.lift.grip",     10.0f,    0.0f,  180.0f },
    { "arm.drop.base",     90.0f,    0.0f,  180.0f },
    { "arm.drop.shoulder",120.0f,    0.0f,  180.0f },
    { "arm.drop.grip",     70.0f,    0.0f,  180.0f },
};

CalibBlob gCalib;

uint16_t Calib::crc16(const uint8_t* p, uint32_t n) {
    uint16_t crc = 0xFFFF;                        // CRC16-CCITT
    for (uint32_t i = 0; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

void Calib::applyDefaults() {
    for (int i = 0; i < N_PARAMS; i++) gCalib.param[i] = PARAM_DEFS[i].def;
    for (int i = 0; i < NUM_SERVOS; i++) {
        gCalib.offset[i] = 0.0f;
        gCalib.trim[i]   = 0;
        gCalib.invert[i] = (i >= 9) ? 1 : 0;      // kaki kiri (idx 9..17) terbalik
    }
    gCalib.magic[0] = 'H'; gCalib.magic[1] = 'X';
    gCalib.version  = CALIB_VERSION;
}

void Calib::save() {
    gCalib.magic[0] = 'H'; gCalib.magic[1] = 'X';
    gCalib.version  = CALIB_VERSION;
    gCalib.crc = crc16((const uint8_t*)&gCalib, sizeof(CalibBlob) - sizeof(gCalib.crc));
    EEPROM.put(CALIB_ADDR, gCalib);
}

bool Calib::load() {
    CalibBlob tmp;
    EEPROM.get(CALIB_ADDR, tmp);
    uint16_t want = crc16((const uint8_t*)&tmp, sizeof(CalibBlob) - sizeof(tmp.crc));
    if (tmp.magic[0] == 'H' && tmp.magic[1] == 'X' &&
        tmp.version == CALIB_VERSION && tmp.crc == want) {
        gCalib = tmp;
        return true;
    }
    applyDefaults();   // EEPROM kosong/rusak/versi beda -> default + persist
    save();
    return false;
}

void Calib::begin() { load(); }

int Calib::findParam(const char* name) {
    for (int i = 0; i < N_PARAMS; i++)
        if (strcmp(name, PARAM_DEFS[i].name) == 0) return i;
    return -1;
}

bool Calib::setParam(const char* name, float v) {
    int i = findParam(name);
    if (i < 0) return false;
    if (v < PARAM_DEFS[i].lo) v = PARAM_DEFS[i].lo;
    if (v > PARAM_DEFS[i].hi) v = PARAM_DEFS[i].hi;
    gCalib.param[i] = v;
    return true;
}
