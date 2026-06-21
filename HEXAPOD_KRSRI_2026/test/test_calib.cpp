// Uji mandiri kalibrasi runtime di PC (EEPROM di-stub di Calib.cpp saat non-Arduino).
//   g++ -I.. test_calib.cpp ../Calib.cpp -o tc && ./tc
#include "../Calib.h"
#include <cstdio>
#include <cstring>
#include <cmath>

int main() {
    int fails = 0;

    // 1) Tabel default waras: lo <= def <= hi, nama unik.
    for (int i = 0; i < N_PARAMS; i++) {
        if (!(PARAM_DEFS[i].lo <= PARAM_DEFS[i].def && PARAM_DEFS[i].def <= PARAM_DEFS[i].hi)) {
            printf("FAIL range: %s\n", PARAM_DEFS[i].name); fails++;
        }
        for (int j = i + 1; j < N_PARAMS; j++)
            if (!strcmp(PARAM_DEFS[i].name, PARAM_DEFS[j].name)) {
                printf("FAIL nama duplikat: %s\n", PARAM_DEFS[i].name); fails++;
            }
    }

    // 2) applyDefaults -> gParam == def.
    Calib::applyDefaults();
    for (int i = 0; i < N_PARAMS; i++)
        if (gParam[i] != PARAM_DEFS[i].def) { printf("FAIL default idx %d\n", i); fails++; }

    // 3) save -> kotori RAM -> load: nilai pulih (CRC valid).
    gParam[K_HEADING_KP] = 0.077f;
    Calib::save();
    gParam[K_HEADING_KP] = 0.0f;
    if (!Calib::load()) { printf("FAIL load valid\n"); fails++; }
    if (fabsf(gParam[K_HEADING_KP] - 0.077f) > 1e-6f) {
        printf("FAIL roundtrip: %.4f\n", gParam[K_HEADING_KP]); fails++;
    }

    // 4) CRC sensitif terhadap perubahan 1 byte.
    uint8_t a[4] = {1, 2, 3, 4};
    uint16_t c1 = Calib::crc16(a, 4);
    a[2] ^= 0xFF;
    if (c1 == Calib::crc16(a, 4)) { printf("FAIL crc tak sensitif\n"); fails++; }

    // 5) setParam: clamp ke rentang, tolak nama tak dikenal.
    Calib::setParam("heading.kp", 999.0f);
    if (gParam[K_HEADING_KP] > PARAM_DEFS[K_HEADING_KP].hi) { printf("FAIL clamp\n"); fails++; }
    if (!Calib::setParam("heading.kp", 0.01f)) { printf("FAIL setParam dikenal\n"); fails++; }
    if (Calib::setParam("tidak.ada", 1.0f))    { printf("FAIL setParam tak dikenal\n"); fails++; }

    printf(fails ? "\n[FAIL] %d kasus gagal\n" : "\n[PASS] Calib konsisten\n", fails);
    return fails ? 1 : 0;
}
