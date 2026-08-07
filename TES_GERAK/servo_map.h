/* =====================================================================
   servo_map.h — struktur & util pemetaan servo (dipakai TES_SERVO & SET_HOME)
   SALIN file ini ke kedua folder sketsa bila diubah (Arduino IDE tidak bisa
   berbagi file antar folder sketsa).

   WIRING SAAT INI (hasil scan I2C, lihat README):
     Wire  (SDA18/SCL19) : TCA9548A 0x70 -> 6x VL53L0X (TOF200C)
     Wire1 (SDA17/SCL16) : PCA9685 @0x41   -> driver 1
     Wire2 (SDA25/SCL24) : PCA9685 @0x40   -> driver 0
   0x70 yang ikut muncul di Wire1/Wire2 = ALL-CALL address bawaan PCA9685
   (bukan mux, bukan hantu). Karena itu PCA9685 TIDAK BOLEH satu bus dengan
   TCA9548A kecuali ALL-CALL dimatikan — di wiring ini sudah aman (beda bus).
   ===================================================================== */
#ifndef SERVO_MAP_H
#define SERVO_MAP_H

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

// ------------------------------------------------------------- wiring
#define BUS_DRV0         Wire2      // PCA9685 @0x40
#define BUS_DRV1         Wire1      // PCA9685 @0x41
#define ADDR_DRV0        0x40
#define ADDR_DRV1        0x41
#define SERVO_I2C_CLOCK  400000UL
#define SERVO_PWM_FREQ   50         // Hz

// ------------------------------------------------------- rentang pulse
#define PULSE_MIN   500             // us @ 0 derajat
#define PULSE_MAX   2500            // us @ 180 derajat
#define PULSE_MID   1500            // us @ 90 derajat (netral)

// --------------------------------------------------------- slot logis
// 0..17  = 6 kaki x 3 sendi (coxa, femur, tibia)
// 18..20 = lengan kanan (base, shoulder, gripper)
// 21..23 = lengan kiri
// Urutan kaki sama dengan config.h firmware:
//   0=Ka-Depan 1=Ka-Tengah 2=Ka-Belakang 3=Ki-Belakang 4=Ki-Tengah 5=Ki-Depan
#define SM_SLOTS   24
#define SM_LEGS     6

static const char* const SLOT_NAME[SM_SLOTS] = {
    "K0_COXA",  "K0_FEMUR",  "K0_TIBIA",     // Ka-Depan
    "K1_COXA",  "K1_FEMUR",  "K1_TIBIA",     // Ka-Tengah
    "K2_COXA",  "K2_FEMUR",  "K2_TIBIA",     // Ka-Belakang
    "K3_COXA",  "K3_FEMUR",  "K3_TIBIA",     // Ki-Belakang
    "K4_COXA",  "K4_FEMUR",  "K4_TIBIA",     // Ki-Tengah
    "K5_COXA",  "K5_FEMUR",  "K5_TIBIA",     // Ki-Depan
    "ARMR_BASE","ARMR_SHOULDER","ARMR_GRIP",
    "ARML_BASE","ARML_SHOULDER","ARML_GRIP"
};

// -------------------------------------------------------- blob EEPROM
// Alamat 1024: JAUH dari blok kalibrasi firmware (Calib.cpp pakai alamat 0,
// ukurannya ~280 byte). Jangan diturunkan tanpa mengecek ulang.
#define SM_EE_ADDR   1024
#define SM_VERSION   1

struct ServoMap {
    char     magic[2];              // 'S','M'
    uint8_t  version;
    int8_t   drv[SM_SLOTS];         // 0/1, -1 = belum dipetakan
    int8_t   ch[SM_SLOTS];          // 0..15, -1 = belum dipetakan
    uint8_t  invert[SM_SLOTS];      // 1 = arah sendi dibalik
    int16_t  trim[SM_SLOTS];        // us, koreksi netral per servo
    uint16_t crc;
};

// ------------------------------------------------------------- util
static uint16_t smCrc16(const uint8_t* p, uint32_t n) {
    uint16_t crc = 0xFFFF;                        // CRC16-CCITT, sama dgn Calib
    for (uint32_t i = 0; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// PEMETAAN HASIL PENGUKURAN (TES_SERVO, Agustus 2026):
//   driver 0 = 0x40 @ Wire2 = sisi KIRI   -> ch 0-2 K5(ki-depan), 4-6 K4, 8-10 K3
//   driver 1 = 0x41 @ Wire1 = sisi KANAN  -> ch 0-2 K2(ka-blkg),  4-6 K1, 8-10 K0
// Kaki DAN lengan sudah diverifikasi fisik (pemetaan ulang, Agustus 2026):
// lengan kanan = drv1 ch 12/13/14, lengan kiri = drv0 ch 12/13/14.
// invert[] = hasil uji arah per sendi ('u<slot>' di SET_HOME), sudah lengkap 18 kaki.
// Polanya TIDAK seragam per kaki, jadi jangan disederhanakan jadi rumus:
//   coxa  : dibalik di KEENAM kaki -> konvensi arah coxa memang berlawanan
//           dengan pemasangan servo di badan
//   femur : dibalik hanya di sisi KANAN
//   tibia : dibalik hanya di sisi KIRI
// Femur & tibia saling melengkapi antar sisi — konsisten dengan modul kaki
// kanan/kiri yang terpasang bercermin.
static void smDefaults(ServoMap& m) {
    static const int8_t D[SM_SLOTS][2] = {
        {1, 8},{1, 9},{1,10},   // K0 kanan-depan     (drv1)
        {1, 4},{1, 5},{1, 6},   // K1 kanan-tengah    (drv1)
        {1, 0},{1, 1},{1, 2},   // K2 kanan-belakang  (drv1)
        {0, 8},{0, 9},{0,10},   // K3 kiri-belakang   (drv0)
        {0, 4},{0, 5},{0, 6},   // K4 kiri-tengah     (drv0)
        {0, 0},{0, 1},{0, 2},   // K5 kiri-depan      (drv0)
        {1,12},{1,13},{1,14},   // lengan kanan (drv1) base, shoulder, grip
        {0,12},{0,13},{0,14}    // lengan kiri  (drv0) base, shoulder, grip
    };
    //                      coxa, femur, tibia
    static const uint8_t INV[SM_SLOTS] = {
        1, 1, 0,    // K0 kanan-depan     ] sisi KANAN:
        1, 1, 0,    // K1 kanan-tengah    ] coxa & femur dibalik
        1, 1, 0,    // K2 kanan-belakang  ]
        1, 0, 1,    // K3 kiri-belakang   ] sisi KIRI:
        1, 0, 1,    // K4 kiri-tengah     ] coxa & tibia dibalik
        1, 0, 1,    // K5 kiri-depan      ]
        0, 0, 0,    // lengan kanan (base, shoulder, grip)
        0, 0, 0     // lengan kiri
    };
    for (uint8_t i = 0; i < SM_SLOTS; i++) {
        m.drv[i]    = D[i][0];
        m.ch[i]     = D[i][1];
        m.trim[i]   = 0;
        m.invert[i] = INV[i];
    }
    m.magic[0] = 'S'; m.magic[1] = 'M'; m.version = SM_VERSION;
}

static void smClear(ServoMap& m) {
    for (uint8_t i = 0; i < SM_SLOTS; i++) {
        m.drv[i] = -1; m.ch[i] = -1; m.invert[i] = 0; m.trim[i] = 0;
    }
    m.magic[0] = 'S'; m.magic[1] = 'M'; m.version = SM_VERSION;
}

static void smSave(ServoMap& m) {
    m.magic[0] = 'S'; m.magic[1] = 'M'; m.version = SM_VERSION;
    m.crc = smCrc16((const uint8_t*)&m, sizeof(ServoMap) - sizeof(m.crc));
    EEPROM.put(SM_EE_ADDR, m);
}

static bool smLoad(ServoMap& m) {
    ServoMap t;
    EEPROM.get(SM_EE_ADDR, t);
    uint16_t want = smCrc16((const uint8_t*)&t, sizeof(ServoMap) - sizeof(t.crc));
    if (t.magic[0] == 'S' && t.magic[1] == 'M' && t.version == SM_VERSION && t.crc == want) {
        m = t; return true;
    }
    return false;
}

// sudut (0..180) -> pulse us, memperhitungkan invert & trim slot
static uint16_t smDegToUs(const ServoMap& m, uint8_t s, float deg) {
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;
    if (s < SM_SLOTS && m.invert[s]) deg = 180.0f - deg;
    float us = PULSE_MIN + (deg / 180.0f) * (float)(PULSE_MAX - PULSE_MIN);
    if (s < SM_SLOTS) us += (float)m.trim[s];
    if (us < PULSE_MIN) us = PULSE_MIN;
    if (us > PULSE_MAX) us = PULSE_MAX;
    return (uint16_t)(us + 0.5f);
}

static void smPrint(const ServoMap& m) {
    Serial.println(F("\nslot  nama            drv  ch  inv  trim"));
    Serial.println(F("-------------------------------------------"));
    uint8_t belum = 0;
    for (uint8_t i = 0; i < SM_SLOTS; i++) {
        Serial.print(' ');
        if (i < 10) Serial.print(' ');
        Serial.print(i); Serial.print(F("   "));
        Serial.print(SLOT_NAME[i]);
        for (uint8_t k = strlen(SLOT_NAME[i]); k < 16; k++) Serial.print(' ');
        if (m.drv[i] < 0) { Serial.println(F(" -    -    -    -   << BELUM")); belum++; continue; }
        Serial.print(' '); Serial.print(m.drv[i]);
        Serial.print(F("   ")); if (m.ch[i] < 10) Serial.print(' '); Serial.print(m.ch[i]);
        Serial.print(F("   ")); Serial.print(m.invert[i]);
        Serial.print(F("   ")); Serial.println(m.trim[i]);
    }
    Serial.print(F("belum dipetakan: ")); Serial.println(belum);
}

// Cetak dalam format siap tempel ke config.h firmware
static void smDumpC(const ServoMap& m) {
    Serial.println(F("\n// ---- tempel ke HEXAPOD_KRSRI_2026/config.h ----"));
    Serial.println(F("const uint8_t SERVO_PIN_MAP[NUM_SERVOS][2] = {"));
    for (uint8_t k = 0; k < SM_LEGS; k++) {
        Serial.print(F("    "));
        for (uint8_t j = 0; j < 3; j++) {
            uint8_t i = k * 3 + j;
            Serial.print('{'); Serial.print(m.drv[i] < 0 ? 0 : m.drv[i]);
            Serial.print(F(", ")); Serial.print(m.ch[i] < 0 ? 0 : m.ch[i]);
            Serial.print(F("}, "));
        }
        Serial.print(F("// Kaki ")); Serial.print(k);
        if (m.drv[k*3] < 0) Serial.print(F("  <-- BELUM DIPETAKAN"));
        Serial.println();
    }
    Serial.println(F("};"));
    Serial.println(F("const uint8_t ARM_PIN_MAP_R[ARM_NUM_SERVOS][2] = { "
                     "/* base, shoulder, grip */"));
    Serial.print(F("    "));
    for (uint8_t i = 18; i < 21; i++) {
        Serial.print('{'); Serial.print(m.drv[i] < 0 ? 0 : m.drv[i]);
        Serial.print(F(", ")); Serial.print(m.ch[i] < 0 ? 0 : m.ch[i]); Serial.print(F("}, "));
    }
    Serial.println(F("\n};"));
    Serial.println(F("const uint8_t ARM_PIN_MAP_L[ARM_NUM_SERVOS][2] = {"));
    Serial.print(F("    "));
    for (uint8_t i = 21; i < 24; i++) {
        Serial.print('{'); Serial.print(m.drv[i] < 0 ? 0 : m.drv[i]);
        Serial.print(F(", ")); Serial.print(m.ch[i] < 0 ? 0 : m.ch[i]); Serial.print(F("}, "));
    }
    Serial.println(F("\n};"));
    Serial.println(F("// invert per slot (untuk Calib::applyDefaults):"));
    Serial.print(F("//   "));
    for (uint8_t i = 0; i < SM_SLOTS; i++) { Serial.print(m.invert[i]); Serial.print(','); }
    Serial.println(F("\n// ------------------------------------------------"));
}

#endif
