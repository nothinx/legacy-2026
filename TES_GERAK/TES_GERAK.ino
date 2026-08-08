/* =====================================================================
   TES_GERAK — uji GAIT, BODY KINEMATICS, dan PIVOT (Teensy 4.1)

   KALIBRASI/ sudah membuktikan robot bisa berjalan, tapi gait-nya di sana
   versi sederhana (ayun sinus, putar didekati) dan dibuat untuk menilai
   KEMULUSAN SERVO, bukan untuk menilai gerakannya sendiri. Yang belum
   pernah diuji sama sekali: body kinematics (badan miring/geser dengan
   kaki tetap menapak) dan pivot terukur (berputar N derajat, bukan
   "berputar sampai kelihatan cukup").

   Alat ini memakai motion.h — port dari HexaGait.cpp + Hexapod::solvePose
   firmware. Jadi yang terbukti di sini langsung sah untuk firmware, dan
   yang gagal di sini akan gagal juga di firmware.

   TIGA JENIS UJI:
     KERING   ('S','bl','N','F') — servo TIDAK bergerak, hanya hitungan.
              Menangkap "sendi harus bergerak 620 der/detik" atau "IK mentok"
              sebelum servo dipaksa melakukannya.
     STATIS   ('br','bp','bw','bx','by','bz','bd') — robot menapak di lantai,
              badan bergerak. Di sinilah tanda & sumbu rotasi diverifikasi.
     BERJALAN ('g','y','O','C','M') — robot benar-benar jalan. Butuh lantai.

   KESELAMATAN
     - Perintah 'n' dan 's' pertama kali: TOPANG ROBOT, servo bisa menyentak.
     - 'g','y','O','C','M' membuat robot BERJALAN — taruh di lantai luas.
     - 'x' berhenti, 'r' melepas semua PWM (servo bebas).
     - Enter kosong = berhenti darurat, sama dengan 'x'.

   Serial Monitor 115200, line ending Newline. Ketik 'H' untuk bantuan.
   ===================================================================== */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include <stddef.h>
#include <Adafruit_PWMServoDriver.h>
#include "servo_map.h"
#include "kinematics.h"
#include "motion.h"

// ------------------------------------------------------------- knob IMU
// Sama dengan TES_IMU: Serial2 = RX2 pin 7 / TX2 pin 8, baud 230400.
#define IMU_SERIAL      Serial2
#define IMU_BAUD_DEF    230400
#define WIT_LEN         11
#define RX_EXTRA        2048

// ------------------------------------------------------------ knob pivot
#define PIVOT_KP        0.020f   // sama dengan heading.kp di Calib.cpp
#define PIVOT_KD        0.004f   // heading.kd — dari gyro Z, bukan turunan yaw
#define PIVOT_MIN_CMD   0.25f    // di bawah ini langkah terlalu kecil untuk
                                 // mengalahkan gesekan: robot "menggeliat" saja
#define PIVOT_TOL_DEG   6.0f
#define PIVOT_DIAM_MS   600      // harus di dalam toleransi selama ini
#define PIVOT_BATAS_MS  25000

// ------------------------------------------------ knob kalibrasi rata badan
#define LVL_PROBE_DEF    5.0f    // sudut uji saat mengenali sumbu IMU (derajat).
                                 // Cukup besar untuk mengalahkan derau (sebar
                                 // roll/pitch ~0,2 der), cukup kecil untuk tidak
                                 // menggeser titik berat sampai kaki terangkat.
#define LVL_SETTLE_MS    700     // servo + badan diam dulu sebelum diukur
#define LVL_AVG_MS       500     // lama merata-rata (>= 100 frame pada 200 Hz)
#define LVL_TOL_DEG      0.3f    // berhenti kalau sisa miring di bawah ini
#define LVL_MAX_ITER     8
#define LVL_MAX_TRIM     15.0f   // lebih dari ini bukan trim lagi, itu kerusakan
#define LVL_DET_MIN      0.25f   // determinan Jacobian minimum yang masih sehat

// ---------------------------------------- knob kalibrasi telapak (per kaki)
#define KAKI_ANGKAT_MM  25.0f    // tinggi angkat tripod yang sedang tidak menumpu
#define KAKI_KASAR_MM    1.5f    // langkah pindai kasar (mencari kurungan)
#define KAKI_HALUS_MM    0.3f    // langkah pindai halus di dalam kurungan
#define KAKI_SETTLE_MS    140    // servo sampai di tempat sebelum IMU dibaca
#define KAKI_AVG_MS       120    // ~24 frame sudut pada 200 Hz
#define KAKI_AMBANG_DEG  0.60f   // ambang minimum "badan mulai terungkit"
#define KAKI_AMBANG_X     5.0f   // ... atau 5x sebar derau, mana yang lebih besar
#define KAKI_MAX_OFF     12.0f   // batas offset AKHIR yang masih dianggap wajar
#define KAKI_PINDAI_MM   18.0f   // jangkauan pindai, sengaja LEBIH LEBAR dari
                                 // KAKI_MAX_OFF: pengukuran mentah bisa ~2x
                                 // offset akhir karena komponen bidang belum
                                 // dibuang. Kaki meleset 14 mm terukur -14 mm
                                 // tapi berakhir di 7 mm — jendela +-12 akan
                                 // menolaknya padahal hasilnya sehat.
#define KAKI_MIN_N          6    // sampel minimum per langkah pindai. Sengaja
                                 // kecil: laju frame sudut yang benar-benar
                                 // sampai bisa jauh di bawah 200 Hz, dan pindai
                                 // punya ~40 langkah — menunggu 20 sampel tiap
                                 // langkah membuat 'J' berkali-kali lebih lama.
#define KAKI_RENTANG_WAJAR 8.0f  // rentang di atas ini tak bisa dijelaskan
                                 // galat gain servo (10% cuma ~1,5 mm)

#define EE_KOMPAS_ADDR  1792     // ditulis TES_IMU
#define EE_GERAK_ADDR   2048     // ditulis alat ini (peta EEPROM di CLAUDE.md)

// ================================================================ servo
Adafruit_PWMServoDriver drv0(ADDR_DRV0, BUS_DRV0);
Adafruit_PWMServoDriver drv1(ADDR_DRV1, BUS_DRV1);
ServoMap map_;

float curDeg[SM_SLOTS];
bool  driven = false;            // servo sudah pernah diberi pulsa?
bool  aktif  = false;            // motionTick() jalan?
bool  warnRange = false;

uint16_t writeHz = 50;           // laju tulis PWM
uint32_t lastWrite = 0, lastTick = 0;

// ================================================================ gerak
Motion mo;                       // yang menggerakkan robot
Motion sim;                      // kembaran untuk uji kering

enum Anim { A_NONE, A_DEMO };
Anim     anim   = A_NONE;
uint32_t animT0 = 0;

// ================================================================== IMU
uint32_t imuBaud = IMU_BAUD_DEF;
uint8_t  rxBuf[64]; uint8_t rxN = 0;
uint8_t  rxExtra[RX_EXTRA];

float roll = 0, pitch = 0, yaw = 0;
float gx = 0, gy = 0, gz = 0;
bool  punyaSudut = false;
uint32_t stOk = 0, stSumBad = 0, angFrames = 0;

// kompas arena (dibaca dari EEPROM yang ditulis TES_IMU)
static const char* const ARAH_NAMA[4] = { "UTARA", "TIMUR", "SELATAN", "BARAT" };
float headArah[4] = { -1, -1, -1, -1 };

// ------------------------------------------------------ hasil kalibrasi
float  degCCW    = 0;            // derajat yaw per siklus, perintah putar +1
float  degCW     = 0;            // ... perintah putar -1
float  mmMaju    = 0;            // mm per siklus saat maju penuh
int8_t pivotSign = +1;           // +1 kalau perintah putar + menaikkan yaw IMU
uint16_t odoCycles = 0;          // menunggu 'm<cm>'
float    odoStepL  = 0;          // stepLength SAAT berjalan, bukan saat 'm'

// ------------------------------------------------ kalibrasi rata badan
// Acuan "badan rata menurut IMU". Nol berarti IMU dianggap terpasang persis
// sejajar pelat badan. Kalau tidak (dan itu lumrah), rekam acuannya lewat 'Z'
// saat badan benar-benar rata menurut waterpass — kalau tidak, robot akan
// diratakan sampai IMU nol, yaitu MIRING sebesar kesalahan pemasangan IMU.
float refRoll = 0, refPitch = 0;

// jac[i][j] = d(sudut IMU ke-i) / d(perintah badan ke-j),
// i: 0 = roll IMU, 1 = pitch IMU;  j: 0 = trimRoll, 1 = trimPitch.
// Diukur, bukan diasumsikan — inilah yang membuat 'A' kebal terhadap sumbu
// IMU yang tertukar atau terbalik (pertanyaan STAB_SWAP_ROLL_PITCH di firmware).
float jac[2][2] = { { 0, 0 }, { 0, 0 } };
bool  punyaJac  = false;

// ============================================================ util kecil
static float wrap180(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}
static Adafruit_PWMServoDriver& drvOf(uint8_t d) { return d == 0 ? drv0 : drv1; }
static bool mapped(uint8_t i) { return map_.drv[i] >= 0 && map_.ch[i] >= 0; }
static uint16_t writeMs() { uint16_t m = 1000 / writeHz; return m < 3 ? 3 : m; }

// ========================================================== parser WIT
// Resinkronisasi buang-SATU-byte (bukan buang 11): satu byte hilang di kabel
// tidak boleh merusak seluruh frame sesudahnya. Sama dengan TES_IMU.
static void witFrame(const uint8_t* f) {
    int16_t v0 = (int16_t)(f[3] << 8 | f[2]);
    int16_t v1 = (int16_t)(f[5] << 8 | f[4]);
    int16_t v2 = (int16_t)(f[7] << 8 | f[6]);
    switch (f[1]) {
        case 0x52:
            gx = v0 / 32768.0f * 2000.0f;
            gy = v1 / 32768.0f * 2000.0f;
            gz = v2 / 32768.0f * 2000.0f;
            break;
        case 0x53: {
            roll  = v0 / 32768.0f * 180.0f;
            pitch = v1 / 32768.0f * 180.0f;
            float y = v2 / 32768.0f * 180.0f;
            if (y < 0) y += 360.0f;
            yaw = y; punyaSudut = true; angFrames++;
            break;
        }
        default: break;
    }
}

static void witPump() {
    while (IMU_SERIAL.available()) {
        if (rxN >= sizeof(rxBuf)) memmove(rxBuf, rxBuf + 1, --rxN);
        rxBuf[rxN++] = (uint8_t)IMU_SERIAL.read();
        while (rxN >= WIT_LEN) {
            if (rxBuf[0] != 0x55) { memmove(rxBuf, rxBuf + 1, --rxN); continue; }
            uint8_t sum = 0;
            for (uint8_t i = 0; i < WIT_LEN - 1; i++) sum += rxBuf[i];
            if (sum != rxBuf[WIT_LEN - 1]) {
                memmove(rxBuf, rxBuf + 1, --rxN); stSumBad++; continue;
            }
            witFrame(rxBuf); stOk++;
            rxN -= WIT_LEN;
            memmove(rxBuf, rxBuf + WIT_LEN, rxN);
        }
    }
}

// ============================================================ EEPROM
static uint8_t eeSum(const void* buf, size_t n) {
    const uint8_t* p = (const uint8_t*)buf;
    uint8_t acc = 0;
    for (size_t i = 0; i < n; i++) acc = (uint8_t)(acc + p[i] * 31 + 7);
    return acc;
}

struct KompasStore { uint8_t m0, m1, ver; float head[4]; uint8_t sum; };

static bool kompasMuat(bool cerewet) {
    KompasStore s;
    EEPROM.get(EE_KOMPAS_ADDR, s);
    if (s.m0 != 0xC0 || s.m1 != 0x3A || s.ver != 1 ||
        s.sum != eeSum(&s, offsetof(KompasStore, sum))) {
        if (cerewet) Serial.println(F("kompas arena belum ada di EEPROM (catat lewat TES_IMU 'c<n>' + 'e')."));
        return false;
    }
    for (uint8_t i = 0; i < 4; i++) headArah[i] = s.head[i];
    if (cerewet) Serial.println(F("kompas arena dimuat dari EEPROM (ditulis TES_IMU)."));
    return true;
}

// v1 = sebelum ada kalibrasi rata. Dibiarkan ada supaya EEPROM lama tetap
// terbaca (angka pivot/odometri mahal — sayang kalau hilang cuma karena
// strukturnya bertambah).
struct GerakStore1 { uint8_t m0, m1, ver; float ccw, cw, maju; int8_t sign; uint8_t sum; };

struct GerakStore {
    uint8_t m0, m1, ver;
    float   ccw, cw, maju;
    int8_t  sign;
    float   lvlR, lvlP;        // trim rata badan (derajat)
    float   refR, refP;        // pembacaan IMU saat badan dinyatakan RATA
    float   jac[4];            // d(imu roll,pitch)/d(perintah roll,pitch)
    float   zoff[6];           // offset tinggi telapak per kaki (mm)
    uint8_t sum;
};

static void gerakSimpan() {
    GerakStore s; memset(&s, 0, sizeof(s));
    s.m0 = 0x6E; s.m1 = 0x2C; s.ver = 2;
    s.ccw = degCCW; s.cw = degCW; s.maju = mmMaju; s.sign = pivotSign;
    s.lvlR = mo.trimRoll; s.lvlP = mo.trimPitch;
    s.refR = refRoll;     s.refP = refPitch;
    s.jac[0] = jac[0][0]; s.jac[1] = jac[0][1];
    s.jac[2] = jac[1][0]; s.jac[3] = jac[1][1];
    for (uint8_t i = 0; i < 6; i++) s.zoff[i] = mo.zOff[i];
    s.sum = eeSum(&s, offsetof(GerakStore, sum));
    EEPROM.put(EE_GERAK_ADDR, s);
    Serial.println(F("disimpan ke EEPROM 2048: pivot, odometri, rata badan, offset telapak."));
}

static bool gerakMuat(bool cerewet) {
    GerakStore s;
    EEPROM.get(EE_GERAK_ADDR, s);
    if (s.m0 == 0x6E && s.m1 == 0x2C && s.ver == 2 &&
        s.sum == eeSum(&s, offsetof(GerakStore, sum))) {
        degCCW = s.ccw; degCW = s.cw; mmMaju = s.maju; pivotSign = s.sign;
        mo.trimRoll = s.lvlR; mo.trimPitch = s.lvlP;
        refRoll = s.refR;     refPitch = s.refP;
        jac[0][0] = s.jac[0]; jac[0][1] = s.jac[1];
        jac[1][0] = s.jac[2]; jac[1][1] = s.jac[3];
        for (uint8_t i = 0; i < 6; i++) mo.zOff[i] = s.zoff[i];
        mo.computeHome();
        punyaJac = (fabsf(jac[0][0] * jac[1][1] - jac[0][1] * jac[1][0]) > LVL_DET_MIN);
        if (cerewet) Serial.println(F("kalibrasi dimuat dari EEPROM (v2: + rata badan + offset telapak)."));
        return true;
    }

    GerakStore1 s1;
    EEPROM.get(EE_GERAK_ADDR, s1);
    if (s1.m0 == 0x6E && s1.m1 == 0x2C && s1.ver == 1 &&
        s1.sum == eeSum(&s1, offsetof(GerakStore1, sum))) {
        degCCW = s1.ccw; degCW = s1.cw; mmMaju = s1.maju; pivotSign = s1.sign;
        if (cerewet) {
            Serial.println(F("kalibrasi gerak dimuat dari EEPROM (v1 — LAMA)."));
            Serial.println(F("  Rata badan belum ada di sana: jalankan 'A' lalu 'W'."));
        }
        return true;
    }

    if (cerewet) Serial.println(F("kalibrasi gerak belum ada di EEPROM."));
    return false;
}

// ============================================================ tulis servo
static void writeSlot(uint8_t i, float deg) {
    if (!mapped(i)) return;
    drvOf(map_.drv[i]).writeMicroseconds(map_.ch[i], smDegToUs(map_, i, deg));
    curDeg[i] = deg;
}

static void releaseAll() {
    for (uint8_t c = 0; c < 16; c++) { drv0.setPWM(c, 0, 0); drv1.setPWM(c, 0, 0); }
    aktif = false; driven = false; anim = A_NONE;
    Serial.println(F("semua PWM mati — servo bebas."));
}

static void rampKe(const float* t, uint16_t ms) {
    float from[SM_SLOTS];
    for (uint8_t i = 0; i < SM_SLOTS; i++) from[i] = curDeg[i];
    uint16_t n = ms / 20; if (n < 1) n = 1;
    for (uint16_t k = 1; k <= n; k++) {
        float a = (float)k / (float)n;
        for (uint8_t i = 0; i < 18; i++)
            if (mapped(i)) writeSlot(i, from[i] + (t[i] - from[i]) * a);
        delay(20);
    }
    driven = true;
}

static void staggerKe(const float* t) {
    Serial.println(F("posisi awal tak diketahui -> satu per satu. TOPANG ROBOT."));
    for (uint8_t i = 0; i < 18; i++) {
        if (!mapped(i)) continue;
        writeSlot(i, t[i]);
        delay(60);
    }
    driven = true;
}

static void goNetral() {
    float t[SM_SLOTS];
    for (uint8_t i = 0; i < SM_SLOTS; i++) t[i] = 90.0f;
    aktif = false; anim = A_NONE;
    Serial.println(F("-> NETRAL 90 der (18 sendi kaki; lengan tidak disentuh)"));
    if (driven) rampKe(t, 400); else staggerKe(t);
}

// Berdiri = pose home dengan pose badan yang berlaku sekarang.
static void goBerdiri() {
    anim = A_NONE;
    mo.setMove(0, 0, 0);
    mo.setHome();
    float t[SM_SLOTS];
    for (uint8_t i = 0; i < SM_SLOTS; i++) t[i] = 90.0f;
    if (!mo.solve(t)) Serial.println(F("!! pose berdiri di luar jangkauan IK — cek 'h'/'R'."));
    Serial.println(F("-> BERDIRI"));
    if (driven) rampKe(t, 400); else staggerKe(t);
    lastTick = millis(); lastWrite = millis();
    aktif = true; warnRange = false;
}

// ------------------------------------------------------- tick gerak
static void motionTick() {
    uint32_t now = millis();
    float dt = (now - lastTick) / 1000.0f;
    lastTick = now;

    if (anim == A_DEMO) {
        // demo body: 6 sumbu berurutan, tiap sumbu 1 putaran sinus 3 detik
        float t = (now - animT0) / 1000.0f;
        uint8_t fase = (uint8_t)(t / 3.0f);
        float  s = sinf(2.0f * PI_F * fmodf(t, 3.0f) / 3.0f);
        mo.bodyRoll = mo.bodyPitch = mo.bodyYaw = 0;
        mo.bodyTx = mo.bodyTy = mo.bodyTz = 0;
        switch (fase) {
            case 0: mo.bodyRoll  = 12.0f * s; break;
            case 1: mo.bodyPitch = 12.0f * s; break;
            case 2: mo.bodyYaw   = 15.0f * s; break;
            case 3: mo.bodyTx    = 25.0f * s; break;
            case 4: mo.bodyTy    = 25.0f * s; break;
            case 5: mo.bodyTz    = 20.0f * s; break;
            default:
                anim = A_NONE;
                Serial.println(F("demo body selesai — semua sumbu kembali nol."));
                break;
        }
    }

    mo.update(dt);
    float d[18];
    bool ok = mo.solve(d);
    for (uint8_t i = 0; i < 18; i++) writeSlot(i, d[i]);
    if (!ok && !warnRange) {
        warnRange = true;
        Serial.println(F("!! target di luar jangkauan kaki — sudah di-clamp."));
        Serial.println(F("   kecilkan langkah 'k', sudut badan, atau ubah tinggi 'h'."));
    }
}

// Jalan terus selama ms sambil tetap menggerakkan robot & membaca IMU.
// return false kalau dibatalkan lewat serial.
static bool tunggu(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        witPump();
        if (aktif && millis() - lastWrite >= writeMs()) { lastWrite = millis(); motionTick(); }
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            Serial.println(F("  ** dibatalkan **"));
            return false;
        }
    }
    return true;
}

// ====================================================== BODY KINEMATICS
static void printBody() {
    Serial.print(F("  body: roll ")); Serial.print(mo.bodyRoll, 1);
    Serial.print(F("  pitch ")); Serial.print(mo.bodyPitch, 1);
    Serial.print(F("  yaw ")); Serial.print(mo.bodyYaw, 1);
    Serial.print(F(" der | geser ")); Serial.print(mo.bodyTx, 0);
    Serial.print('/'); Serial.print(mo.bodyTy, 0);
    Serial.print('/'); Serial.print(mo.bodyTz, 0); Serial.println(F(" mm"));
    Serial.print(F("  trim rata: roll ")); Serial.print(mo.trimRoll, 2);
    Serial.print(F("  pitch ")); Serial.print(mo.trimPitch, 2);
    Serial.println(F(" der   (selalu berlaku, tidak dihapus 'b0' — 'a' rinci)"));
    Serial.print(F("  telapak :"));
    for (uint8_t i = 0; i < 6; i++) { Serial.print(' '); Serial.print(mo.zOff[i], 1); }
    Serial.println(F(" mm   ('j' rinci)"));
}

static void setBody(char sumbu, float v) {
    if (!aktif) { Serial.println(F("robot belum berdiri — 's' dulu.")); return; }
    anim = A_NONE;
    switch (sumbu) {
        case 'r': mo.bodyRoll  = constrain(v, -25.0f, 25.0f); break;
        case 'p': mo.bodyPitch = constrain(v, -25.0f, 25.0f); break;
        case 'w': mo.bodyYaw   = constrain(v, -30.0f, 30.0f); break;
        case 'x': mo.bodyTx    = constrain(v, -60.0f, 60.0f); break;
        case 'y': mo.bodyTy    = constrain(v, -60.0f, 60.0f); break;
        case 'z': mo.bodyTz    = constrain(v, -40.0f, 40.0f); break;
        case '0': mo.bodyRoll = mo.bodyPitch = mo.bodyYaw = 0;
                  mo.bodyTx = mo.bodyTy = mo.bodyTz = 0; break;
        default: Serial.println(F("sumbu: br bp bw bx by bz b0 bd bl")); return;
    }
    printBody();
    Serial.println(F("  PERIKSA ARAHNYA: roll+ = miring KANAN, pitch+ = MENDONGAK,"));
    Serial.println(F("  yaw+ = badan berputar ke KIRI, x+ = geser KANAN, y+ = MAJU, z+ = NAIK."));
    Serial.println(F("  Kalau ada yang terbalik, itu temuan — catat, jangan dibetulkan"));
    Serial.println(F("  dengan membalik tanda perintah."));
}

// ------------------------------------------------ uji batas pose (kering)
// Naikkan tiap sumbu sedikit demi sedikit sampai IK mentok. Servo tidak
// bergerak: ini murni hitungan, jadi aman dijalankan kapan saja.
static void ujiBatas() {
    sim = mo;                       // salin knob & profil yang berlaku
    sim.setMove(0, 0, 0);
    sim.setHome();

    const char* NAMA[6] = { "roll (+kanan)", "pitch (+dongak)", "yaw badan",
                            "geser X (kanan)", "geser Y (depan)", "geser Z (naik)" };
    const float LANGKAH[6] = { 1.0f, 1.0f, 1.0f, 2.0f, 2.0f, 2.0f };
    const float BATAS[6]   = { 40.0f, 40.0f, 45.0f, 90.0f, 90.0f, 60.0f };
    const char* SATUAN[6]  = { "der", "der", "der", "mm", "mm", "mm" };

    Serial.println(F("\n--- UJI BATAS POSE BADAN (kering, servo diam) ---"));
    Serial.print(F("  profil: tinggi ")); Serial.print(sim.prof.standH, 0);
    Serial.print(F(" mm, bentang kaki ")); Serial.print(sim.prof.standR, 0);
    Serial.println(F(" mm"));
    Serial.println(F("  sumbu               batas -      batas +"));

    float d[18];
    for (uint8_t a = 0; a < 6; a++) {
        float lim[2] = { 0, 0 };
        for (uint8_t arah = 0; arah < 2; arah++) {
            float sgn = arah ? 1.0f : -1.0f;
            float v = 0;
            while (fabsf(v) < BATAS[a]) {
                float nv = v + sgn * LANGKAH[a];
                sim.bodyRoll = sim.bodyPitch = sim.bodyYaw = 0;
                sim.bodyTx = sim.bodyTy = sim.bodyTz = 0;
                switch (a) {
                    case 0: sim.bodyRoll  = nv; break;
                    case 1: sim.bodyPitch = nv; break;
                    case 2: sim.bodyYaw   = nv; break;
                    case 3: sim.bodyTx    = nv; break;
                    case 4: sim.bodyTy    = nv; break;
                    case 5: sim.bodyTz    = nv; break;
                }
                if (!sim.solve(d)) break;
                bool sudutHabis = false;      // servo hanya 0..180
                for (uint8_t i = 0; i < 18; i++)
                    if (d[i] < 3.0f || d[i] > 177.0f) sudutHabis = true;
                if (sudutHabis) break;
                v = nv;
            }
            lim[arah] = v;
        }
        Serial.print(F("  ")); Serial.print(NAMA[a]);
        for (uint8_t k = strlen(NAMA[a]); k < 20; k++) Serial.print(' ');
        Serial.print(lim[0], 0); Serial.print(' '); Serial.print(SATUAN[a]);
        Serial.print(F("      +")); Serial.print(lim[1], 0);
        Serial.print(' '); Serial.println(SATUAN[a]);
    }
    Serial.println(F("  Batas ini dari IK + rentang servo 0..180, BUKAN dari"));
    Serial.println(F("  tabrakan mekanis. Uji statis pelan-pelan sebelum dipercaya."));
    sim.bodyRoll = sim.bodyPitch = sim.bodyYaw = 0;
    sim.bodyTx = sim.bodyTy = sim.bodyTz = 0;
}

// ============================================== UJI KERING GAIT ('S')
// Menjalankan mesin gait yang sama di RAM, secepat mungkin, lalu mengukur
// berapa cepat tiap sendi HARUS bergerak. Ini menjawab pertanyaan yang tidak
// bisa dijawab dengan melihat robot: "servo-nya sanggup tidak?"
// --- hasil simJalan(), global supaya tak perlu belasan parameter keluaran ---
float    simRate[3], simMn[3], simMx[3];
uint8_t  simLeg[3];
float    simBentangMin, simBentangMax, simZMax;
uint32_t simLuar, simN;

static void simJalan(float vx, float vy, float vyaw, uint8_t siklus) {
    sim = mo;
    sim.prof = mo.tgtProf; sim.tgtProf = mo.tgtProf;   // pakai profil target penuh
    sim.setHome();
    sim.setMove(vx, vy, vyaw);

    float dt = 1.0f / (float)writeHz;

    // ramp dulu sampai vektor gerak mencapai target (slew), baru diukur.
    for (uint16_t i = 0; i < (uint16_t)(3.0f / dt); i++) sim.update(dt);

    float prev[18], now18[18];
    sim.solve(prev);

    for (uint8_t j = 0; j < 3; j++) {
        simRate[j] = 0; simLeg[j] = 0; simMn[j] = 999; simMx[j] = -999;
    }
    simBentangMin = 1e9f; simBentangMax = -1e9f; simZMax = -1e9f;
    simLuar = 0; simN = 0;

    uint32_t langkah = (uint32_t)(siklus * sim.prof.cycleMs / 1000.0f / dt);
    for (uint32_t s = 0; s < langkah; s++) {
        sim.update(dt);
        if (!sim.solve(now18)) simLuar++;
        for (uint8_t i = 0; i < 18; i++) {
            uint8_t j = i % 3;
            float r = fabsf(now18[i] - prev[i]) / dt;
            if (r > simRate[j]) { simRate[j] = r; simLeg[j] = i / 3; }
            if (now18[i] < simMn[j]) simMn[j] = now18[i];
            if (now18[i] > simMx[j]) simMx[j] = now18[i];
            prev[i] = now18[i];
        }
        float b = sim.bentangX();
        if (b < simBentangMin) simBentangMin = b;
        if (b > simBentangMax) simBentangMax = b;
        for (uint8_t l = 0; l < 6; l++)
            if (sim.legTargets[l][2] > simZMax) simZMax = sim.legTargets[l][2];
        simN++;
    }
}

// Ringkasan 4 arah. Kasus terburuk BUKAN "maju": perintah maju dan putar
// saling menambah, dan gabungan itulah yang dikeluarkan wall-following
// setiap saat. Tanpa tabel ini gampang menyimpulkan "aman" dari satu arah.
static void ringkasArah() {
    static const char* const NM[4] = { "maju", "putar", "maju+putar", "geser" };
    static const float V[4][3] = { {0,1,0}, {0,0,1}, {0,0.8f,0.7f}, {1,0,0} };
    Serial.println(F("\n  laju sendi maks per arah (der/s):"));
    Serial.println(F("    arah          coxa  femur  tibia   bentang maks"));
    float terburuk = 0;
    for (uint8_t a = 0; a < 4; a++) {
        simJalan(V[a][0], V[a][1], V[a][2], 2);
        Serial.print(F("    ")); Serial.print(NM[a]);
        for (uint8_t k = strlen(NM[a]); k < 12; k++) Serial.print(' ');
        Serial.print(simRate[0], 0); Serial.print(F("    "));
        Serial.print(simRate[1], 0); Serial.print(F("    "));
        Serial.print(simRate[2], 0); Serial.print(F("     "));
        Serial.print(simBentangMax / 10.0f, 1); Serial.print(F(" cm"));
        float w = simRate[0];
        if (simRate[1] > w) w = simRate[1];
        if (simRate[2] > w) w = simRate[2];
        if (w > SERVO_MAX_DPS * 0.65f) Serial.print(F("   << lewat batas"));
        if (simLuar) Serial.print(F("   << IK mentok"));
        Serial.println();
        if (w > terburuk) terburuk = w;
    }
    Serial.print(F("    terburuk ")); Serial.print(terburuk, 0);
    Serial.print(F(" der/s dari ~")); Serial.print(SERVO_MAX_DPS * 0.65f, 0);
    Serial.println(F(" der/s yang sanggup dilayani servo berbeban."));
}

static void simGait(float vx, float vy, float vyaw, uint8_t siklus) {
    simJalan(vx, vy, vyaw, siklus);

    Serial.println(F("\n--- SIMULASI KERING GAIT (servo tidak bergerak) ---"));
    Serial.print(F("  vektor  : maju ")); Serial.print(vy, 2);
    Serial.print(F("  geser ")); Serial.print(vx, 2);
    Serial.print(F("  putar ")); Serial.println(vyaw, 2);
    Serial.print(F("  profil  : angkat ")); Serial.print(sim.prof.stepH, 0);
    Serial.print(F("  langkah ")); Serial.print(sim.prof.stepL, 0);
    Serial.print(F("  siklus ")); Serial.print(sim.prof.cycleMs, 0);
    Serial.print(F(" ms  tinggi ")); Serial.print(sim.prof.standH, 0);
    Serial.print(F("  bentang kaki ")); Serial.println(sim.prof.standR, 0);
    Serial.print(F("  tulis PWM ")); Serial.print(writeHz);
    Serial.print(F(" Hz -> ")); Serial.print(simN); Serial.print(F(" titik / "));
    Serial.print(siklus); Serial.println(F(" siklus"));

    const char* JN[3] = { "coxa ", "femur", "tibia" };
    Serial.println(F("\n  sendi   laju maks      sudut min..maks   kaki terparah"));
    bool terlaluCepat = false;
    for (uint8_t j = 0; j < 3; j++) {
        Serial.print(F("  ")); Serial.print(JN[j]);
        Serial.print(F("   ")); Serial.print(simRate[j], 0); Serial.print(F(" der/s"));
        if (simRate[j] < 100) Serial.print(' ');
        Serial.print(F("     ")); Serial.print(simMn[j], 0);
        Serial.print(F(" .. ")); Serial.print(simMx[j], 0);
        Serial.print(F("        K")); Serial.println(simLeg[j]);
        if (simRate[j] > SERVO_MAX_DPS * 0.65f) terlaluCepat = true;
    }

    Serial.print(F("\n  bentang badan  : ")); Serial.print(simBentangMin / 10.0f, 1);
    Serial.print(F(" .. ")); Serial.print(simBentangMax / 10.0f, 1); Serial.println(F(" cm"));
    Serial.print(F("  angkat kaki    : ")); Serial.print(simZMax + sim.prof.standH, 0);
    Serial.println(F(" mm dari tanah"));
    Serial.print(F("  laju teoretis  : ")); Serial.print(sim.lajuTeoretis() / 10.0f, 1);
    Serial.println(F(" cm/detik (sebelum slip)"));
    Serial.print(F("  titik di luar jangkauan IK: ")); Serial.print(simLuar);
    Serial.print('/'); Serial.println(simN);

    Serial.println(F("\n  VONIS:"));
    if (simLuar)
        Serial.println(F("  ! IK MENTOK di sebagian siklus — kaki akan 'menabrak' batasnya"));
    else
        Serial.println(F("  - seluruh siklus di dalam jangkauan kaki."));
    Serial.print(F("  - batas servo RDS3235 ")); Serial.print(SERVO_MAX_DPS, 0);
    Serial.print(F(" der/s tanpa beban (~")); Serial.print(SERVO_MAX_DPS * 0.65f, 0);
    Serial.println(F(" der/s berbeban)"));
    if (terlaluCepat) {
        Serial.println(F("  ! ADA SENDI YANG DIMINTA TERLALU CEPAT. Servo akan tertinggal"));
        Serial.println(F("    dari perintah: kaki mendarat di tempat yang salah dan robot"));
        Serial.println(F("    ngesot, bukan melangkah. Perbesar 'p' (siklus) atau kecilkan"));
        Serial.println(F("    'k' (panjang langkah)/'e' (tinggi angkat)."));
    } else {
        Serial.println(F("  - laju sendi masih di dalam kemampuan servo."));
    }
    ringkasArah();
}

// ------------------------------------------- tabel profil & celah sempit
static void tabelProfil() {
    Serial.println(F("\n--- PROFIL MEDAN ---"));
    Serial.println(F("  no  nama       angkat langkah siklus tinggi bentang  bentang badan"));
    for (uint8_t i = 0; i < 4; i++) {
        sim.setProfileNow(i);
        sim.setMove(0, 0, 0);
        sim.setHome();
        Serial.print(F("  ")); Serial.print(i); Serial.print(F("   "));
        Serial.print(PROFIL_NAMA[i]);
        for (uint8_t k = strlen(PROFIL_NAMA[i]); k < 11; k++) Serial.print(' ');
        Serial.print(PROFIL[i].stepH, 0);   Serial.print(F("     "));
        Serial.print(PROFIL[i].stepL, 0);   Serial.print(F("      "));
        Serial.print(PROFIL[i].cycleMs, 0); Serial.print(F("    "));
        Serial.print(PROFIL[i].standH, 0);  Serial.print(F("    "));
        Serial.print(PROFIL[i].standR, 0);  Serial.print(F("      "));
        Serial.print(sim.bentangX() / 10.0f, 1); Serial.println(F(" cm"));
    }
    Serial.println(F("\n  Bentang badan = jarak titik ujung kaki kiri-kanan saat DIAM;"));
    Serial.println(F("  lebar telapak & bagian yang menonjol BELUM termasuk."));
    Serial.println(F("  Maju dan BERBELOK tidak menambah bentang — kaki tengah, yang"));
    Serial.println(F("  paling lebar, bergerak searah badan saat berputar. Yang menambah"));
    Serial.println(F("  adalah MENGGESER MENYAMPING: profil SEMPIT 27,0 cm jadi 31,5 cm."));
    Serial.println(F("  Jadi di celah R11 koreksi heading boleh, strafe tidak."));
    Serial.println(F("  Lihat kolom 'bentang maks' per arah di 'S'."));
    Serial.println(F("  'N300' mencari bentang kaki yang muat celah 30 cm."));
}

// Cari standR terbesar yang masih muat celah selebar mm.
static void cariSempit(float celahMm) {
    if (celahMm < 150 || celahMm > 600) { Serial.println(F("celah 150..600 mm, mis. N300")); return; }
    // Margin menutup lebar telapak + bagian badan yang menonjol + galat
    // kemudi. bentangX() hanya menghitung TITIK ujung kaki.
    float margin = 30.0f;
    Serial.print(F("\n--- Cari bentang kaki untuk celah ")); Serial.print(celahMm / 10.0f, 0);
    Serial.println(F(" cm ---"));
    Serial.print(F("  margin keamanan ")); Serial.print(margin / 10.0f, 0); Serial.println(F(" cm"));

    float terbaik = -1;
    float d[18];
    for (float r = 80.0f; r >= 20.0f; r -= 1.0f) {
        sim = mo;
        sim.bodyRoll = sim.bodyPitch = sim.bodyYaw = 0;
        sim.bodyTx = sim.bodyTy = sim.bodyTz = 0;
        GaitProfile p = PROFIL[3]; p.standR = r;
        sim.tgtProf = p; sim.prof = p;
        sim.setMove(0, 0, 0);
        sim.setHome();
        if (!sim.solve(d)) continue;                      // IK tidak sanggup
        bool habis = false;
        for (uint8_t i = 0; i < 18; i++) if (d[i] < 3.0f || d[i] > 177.0f) habis = true;
        if (habis) continue;

        // ukur bentang MAKS selama berjalan, bukan saat diam
        sim.setMove(0, 1, 0);
        float dt = 1.0f / (float)writeHz;
        for (uint16_t i = 0; i < (uint16_t)(3.0f / dt); i++) sim.update(dt);
        float bmax = 0;
        for (uint32_t s = 0; s < (uint32_t)(2 * p.cycleMs / 1000.0f / dt); s++) {
            sim.update(dt);
            float b = sim.bentangX();
            if (b > bmax) bmax = b;
        }
        if (bmax + margin <= celahMm) { terbaik = r;
            Serial.print(F("  bentang kaki ")); Serial.print(r, 0);
            Serial.print(F(" mm -> badan ")); Serial.print(bmax / 10.0f, 1);
            Serial.println(F(" cm saat jalan   <-- MUAT"));
            break;
        }
    }
    if (terbaik < 0) {
        Serial.println(F("  TIDAK ADA bentang kaki yang muat sambil tetap bisa berjalan."));
        Serial.println(F("  Pilihan: kecilkan panjang langkah ('k') lalu ulangi, atau"));
        Serial.println(F("  robot harus melewati celah dengan gerakan lain (menyamping)."));
        return;
    }
    // Berapa lebar badan kalau robot MENGGESER MENYAMPING di dalam celah?
    // Berbelok tidak melebarkan (kaki tengah bergerak searah badan), strafe iya.
    sim.setMove(1, 0, 0);
    float dt2 = 1.0f / (float)writeHz;
    for (uint16_t i = 0; i < (uint16_t)(3.0f / dt2); i++) sim.update(dt2);
    float bGeser = 0;
    for (uint32_t s = 0; s < (uint32_t)(2 * PROFIL[3].cycleMs / 1000.0f / dt2); s++) {
        sim.update(dt2);
        if (sim.bentangX() > bGeser) bGeser = sim.bentangX();
    }
    Serial.print(F("  saat MENGGESER MENYAMPING badan jadi ")); Serial.print(bGeser / 10.0f, 1);
    Serial.println(bGeser + margin <= celahMm ? F(" cm — masih muat")
                                              : F(" cm — TIDAK muat"));
    Serial.println(F("  Berbelok TIDAK melebarkan badan, jadi mengoreksi heading di"));
    Serial.println(F("  dalam celah aman; strafe tidak. Navigation memang tak pernah"));
    Serial.println(F("  memakai strafe."));

    Serial.println(F("\n  Tempel ke motion.h (PROFIL[3] SEMPIT):"));
    Serial.print(F("    { ")); Serial.print(PROFIL[3].stepH, 1);
    Serial.print(F("f, ")); Serial.print(PROFIL[3].stepL, 1);
    Serial.print(F("f, ")); Serial.print(PROFIL[3].cycleMs, 1);
    Serial.print(F("f, ")); Serial.print(PROFIL[3].standH, 1);
    Serial.print(F("f, ")); Serial.print(terbaik, 1); Serial.println(F("f }   // SEMPIT"));
    Serial.println(F("  Di firmware profilnya ditulis RELATIF terhadap GAIT_* supaya"));
    Serial.println(F("  penyetelan GUI ikut terbawa — di Hexapod::profileNarrow() ubah"));
    Serial.print(F("  suku terakhir jadi  STAND_RADIUS - "));
    Serial.print(STAND_R_DEF - terbaik, 1); Serial.println(F("f"));
}

// ================================================== GAIT & PIVOT
static void mulaiGait(float vx, float vy, float vyaw, const __FlashStringHelper* nama) {
    if (!aktif) { Serial.println(F("robot belum berdiri — 's' dulu (taruh di lantai).")); return; }
    anim = A_NONE;
    mo.setMove(vx, vy, vyaw);
    Serial.print(F("-> ")); Serial.print(nama);
    Serial.println(F("   ('x' berhenti)"));
}

static void gaitPutar(float turn) { mo.setMove(0, 0, turn); }

// Akumulasi yaw yang TIDAK terpotong di 360. Dipanggil terus selama manuver.
static float yawAkum = 0, yawPrev = 0;
static void yawMulai() { yawAkum = 0; yawPrev = yaw; }
static void yawLanjut() { yawAkum += wrap180(yaw - yawPrev); yawPrev = yaw; }

// tunggu() + akumulasi yaw
static bool tungguYaw(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        witPump(); yawLanjut();
        if (aktif && millis() - lastWrite >= writeMs()) { lastWrite = millis(); motionTick(); }
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            Serial.println(F("  ** dibatalkan **"));
            return false;
        }
    }
    return true;
}

// ------------------------------------------------- pivot tertutup (PD)
static void pivotKe(float targetYaw) {
    if (!aktif) { Serial.println(F("robot belum berdiri — 's' dulu.")); return; }
    if (!punyaSudut) { Serial.println(F("Tidak ada data sudut dari IMU — pivot tertutup butuh yaw.")); return; }

    Serial.print(F("\n--- Pivot ke yaw ")); Serial.print(targetYaw, 1);
    Serial.println(F(" der ---"));
    Serial.print(F("  tanda pivot = ")); Serial.print(pivotSign);
    if (degCCW == 0 && degCW == 0)
        Serial.println(F("  (BELUM DIKALIBRASI — jalankan 'C3' dulu; kalau tandanya"
                         " salah robot berputar MENJAUHI sasaran)"));
    else Serial.println(F("  (dari kalibrasi)"));
    Serial.println(F("  Ketik apa saja = berhenti.\n"));

    uint32_t t0 = millis(), lapor = 0, masukSejak = 0;
    float errAwal = wrap180(targetYaw - yaw);
    bool  selesai = false;

    while (millis() - t0 < PIVOT_BATAS_MS) {
        witPump();
        if (aktif && millis() - lastWrite >= writeMs()) { lastWrite = millis(); motionTick(); }

        float err = wrap180(targetYaw - yaw);
        // Suku D dari gyro Z langsung: yaw berisik, mendiferensiasikannya
        // hanya memperbesar noise itu. Gyro juga kebal magnet servo.
        float turn = PIVOT_KP * err - PIVOT_KD * gz;
        if (turn >  1.0f) turn =  1.0f;
        if (turn < -1.0f) turn = -1.0f;
        // Di bawah PIVOT_MIN_CMD langkahnya terlalu pendek untuk mengalahkan
        // gesekan: robot bergetar di tempat dan error tak pernah mengecil.
        if (fabsf(err) > PIVOT_TOL_DEG && fabsf(turn) < PIVOT_MIN_CMD)
            turn = (turn >= 0 ? PIVOT_MIN_CMD : -PIVOT_MIN_CMD);

        gaitPutar(pivotSign * turn);

        if (fabsf(err) <= PIVOT_TOL_DEG) {
            if (!masukSejak) masukSejak = millis();
            if (millis() - masukSejak >= PIVOT_DIAM_MS) { selesai = true; break; }
        } else masukSejak = 0;

        if (millis() - lapor >= 250) {
            lapor = millis();
            Serial.print(F("  yaw ")); Serial.print(yaw, 1);
            Serial.print(F("  error ")); Serial.print(err, 1);
            Serial.print(F("  gyroZ ")); Serial.print(gz, 1);
            Serial.print(F("  -> putar ")); Serial.print(pivotSign * turn, 2);
            if (fabsf(err) <= PIVOT_TOL_DEG) Serial.print(F("  [dalam toleransi]"));
            Serial.println();
        }
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            Serial.println(F("  ** dihentikan **"));
            break;
        }
    }
    gaitPutar(0);
    tunggu(800);                                  // biar kaki settle ke home

    Serial.print(F("\n  hasil : "));
    if (selesai) { Serial.print(F("SAMPAI dalam ")); Serial.print((millis() - t0) / 1000.0f, 1);
                   Serial.println(F(" detik")); }
    else Serial.println(F("BELUM sampai dalam batas waktu"));
    Serial.print(F("  error : ")); Serial.print(errAwal, 1);
    Serial.print(F(" -> ")); Serial.print(wrap180(targetYaw - yaw), 1); Serial.println(F(" der"));
    if (!selesai)
        Serial.println(F("  Kalau error MEMBESAR: tanda pivot terbalik. Jalankan 'C3'."));
}

static void pivotRelatif(float der) {
    if (!punyaSudut) { Serial.println(F("Tidak ada data sudut dari IMU.")); return; }
    float t = yaw + der;
    while (t >= 360.0f) t -= 360.0f;
    while (t < 0)       t += 360.0f;
    pivotKe(t);
}

static void pivotKompas(uint8_t n) {
    if (headArah[n] < 0) {
        Serial.print(ARAH_NAMA[n]);
        Serial.println(F(" belum dicatat. Catat lewat TES_IMU ('c<n>' lalu 'e')."));
        return;
    }
    Serial.print(F("arah ")); Serial.println(ARAH_NAMA[n]);
    pivotKe(headArah[n]);
}

// ------------------------------------------- kalibrasi pivot ('C<siklus>')
// Menjawab: berapa derajat robot berputar per siklus gait, dan ke arah mana.
// Tanpa angka ini, misi hanya bisa "putar sampai kelihatan cukup".
static void kalibrasiPivot(uint8_t siklus) {
    if (!aktif) { Serial.println(F("robot belum berdiri — 's' dulu (taruh di lantai luas).")); return; }
    if (!punyaSudut) { Serial.println(F("Tidak ada data sudut dari IMU.")); return; }
    if (siklus < 1 || siklus > 20) { Serial.println(F("1..20 siklus, mis. C4")); return; }

    uint32_t lama = (uint32_t)(siklus * mo.prof.cycleMs);
    Serial.print(F("\n--- KALIBRASI PIVOT, ")); Serial.print(siklus);
    Serial.println(F(" siklus tiap arah ---"));
    Serial.println(F("  Robot akan BERPUTAR DI TEMPAT dua kali. Beri ruang."));
    Serial.println(F("  Ketik apa saja = batal.\n"));

    float hasil[2];
    for (uint8_t arah = 0; arah < 2; arah++) {
        float cmd = arah ? -1.0f : 1.0f;
        Serial.print(F("  perintah putar ")); Serial.print(cmd, 0); Serial.println(F(" ..."));
        gaitPutar(cmd);
        if (!tunggu(500)) { gaitPutar(0); return; }     // tunggu slew naik
        yawMulai();
        if (!tungguYaw(lama)) { gaitPutar(0); return; }
        gaitPutar(0);
        if (!tungguYaw(1000)) return;                   // ikut hitung sisa putaran
        hasil[arah] = yawAkum;
        Serial.print(F("    yaw berubah ")); Serial.print(yawAkum, 1);
        Serial.print(F(" der -> ")); Serial.print(yawAkum / siklus, 1);
        Serial.println(F(" der/siklus"));
    }

    degCCW = hasil[0] / siklus;
    degCW  = hasil[1] / siklus;
    pivotSign = (degCCW >= 0) ? +1 : -1;

    Serial.println(F("\n  ---------------- HASIL ----------------"));
    Serial.print(F("  perintah +1 : ")); Serial.print(degCCW, 2); Serial.println(F(" der/siklus"));
    Serial.print(F("  perintah -1 : ")); Serial.print(degCW, 2);  Serial.println(F(" der/siklus"));
    Serial.print(F("  tanda pivot : ")); Serial.print(pivotSign);
    Serial.println(pivotSign > 0 ? F("  (perintah + menaikkan yaw IMU)")
                                 : F("  (perintah + MENURUNKAN yaw IMU — dibalik otomatis)"));

    float a = fabsf(degCCW), b = fabsf(degCW);
    float beda = (a + b) > 0.1f ? fabsf(a - b) / ((a + b) / 2.0f) * 100.0f : 0;
    Serial.print(F("  asimetri    : ")); Serial.print(beda, 0); Serial.println(F(" %"));
    if (beda > 25.0f)
        Serial.println(F("    ! satu arah jauh lebih lemah — biasanya trim coxa belum rata"
                         " atau satu kaki menyeret. Cek di KALIBRASI."));
    if (a > 0.5f) {
        Serial.print(F("  untuk 90 der: ")); Serial.print(90.0f / a, 1);
        Serial.println(F(" siklus"));
        // Semua kaki berada di radius ~160 mm dari pusat, sedangkan vektor
        // rotasi dinormalkan terhadap 100 mm — jadi perintah putar di atas
        // 100/160 = 0,625 sudah mentok normalisasi dan TIDAK menambah cepat.
        Serial.println(F("  CATATAN: perintah putar di atas 0,63 tidak menambah"));
        Serial.println(F("  kecepatan (dibatasi normalisasi vektor langkah), jadi"));
        Serial.println(F("  PD pivot mentok pada error ~31 der ke atas. Itu wajar."));
    } else {
        Serial.println(F("  ! robot HAMPIR TIDAK BERPUTAR. Kaki selip (bentang terlalu lebar,"));
        Serial.println(F("    angkat terlalu rendah) atau lantainya licin."));
    }
    Serial.println(F("  'W' untuk menyimpan ke EEPROM."));
}

// -------------------------------------- kalibrasi odometri ('M<siklus>')
static void kalibrasiMaju(uint8_t siklus) {
    if (!aktif) { Serial.println(F("robot belum berdiri — 's' dulu.")); return; }
    if (siklus < 1 || siklus > 20) { Serial.println(F("1..20 siklus, mis. M5")); return; }

    Serial.print(F("\n--- ODOMETRI: jalan MAJU ")); Serial.print(siklus);
    Serial.println(F(" siklus ---"));
    Serial.println(F("  Tandai posisi robot sekarang. Ketik apa saja = batal."));
    mo.setMove(0, 1, 0);
    if (!tunggu(600)) { mo.setMove(0, 0, 0); return; }      // slew naik dulu
    if (!tunggu((uint32_t)(siklus * mo.prof.cycleMs))) { mo.setMove(0, 0, 0); return; }
    mo.setMove(0, 0, 0);
    tunggu(1000);

    odoCycles = siklus;
    odoStepL  = mo.prof.stepL;   // knob boleh berubah sebelum 'm<cm>' masuk
    float teoretis = odoStepL * siklus;
    Serial.print(F("  selesai. Jarak TEORETIS ")); Serial.print(teoretis / 10.0f, 1);
    Serial.println(F(" cm."));
    Serial.println(F("  Ukur jarak sebenarnya dengan mistar, lalu ketik: m<cm>  (mis. m24)"));
}

static void odoHasil(float cm) {
    if (!odoCycles) { Serial.println(F("jalankan 'M<siklus>' dulu.")); return; }
    mmMaju = cm * 10.0f / odoCycles;
    float teoretis = odoStepL;
    float slip = teoretis > 1 ? (1.0f - mmMaju / teoretis) * 100.0f : 0;
    Serial.print(F("\n  ")); Serial.print(cm, 1); Serial.print(F(" cm / "));
    Serial.print(odoCycles); Serial.print(F(" siklus = "));
    Serial.print(mmMaju, 1); Serial.println(F(" mm per siklus"));
    Serial.print(F("  teoretis ")); Serial.print(teoretis, 0);
    Serial.print(F(" mm -> SLIP ")); Serial.print(slip, 0); Serial.println(F(" %"));
    Serial.print(F("  laju nyata ")); Serial.print(mmMaju / (mo.prof.cycleMs / 1000.0f) / 10.0f, 1);
    Serial.println(F(" cm/detik"));
    if (slip > 40.0f)
        Serial.println(F("  ! slip besar. Kaki menyeret saat fase tumpu: naikkan 'e'"
                         " (tinggi angkat) atau perlambat 'p'."));
    else if (slip < -10.0f)
        Serial.println(F("  ? maju lebih jauh dari teoretis — cek jumlah siklus/pengukuran."));
    else
        Serial.println(F("  slip wajar. Pakai angka ini untuk odometri misi."));
    Serial.println(F("  'W' untuk menyimpan ke EEPROM."));
    odoCycles = 0;
}

// ================================================= KALIBRASI RATA BADAN
// Masalah yang dijawab: badan tetap miring kiri/kanan walau trim servo sudah
// disetel. Penyebabnya menumpuk dari banyak sumber kecil — gerigi horn servo
// (satu gerigi ~1,4 der di femur = ~2 mm di telapak), panjang kaki yang tak
// persis sama, rangka yang tidak simetris, dan sag servo yang berbeda-beda
// karena beban tiap kaki berbeda. Menyetelnya satu-satu dengan mata sulit
// karena mata tidak bisa membedakan "badan miring" dari "lantai miring".
//
// IMU bisa: ia mengukur arah GRAVITASI, bukan arah lantai. Jadi lingkarannya
// ditutup — miringkan badan sampai IMU membaca rata, lalu simpan sudut itu.
//
// Tiga hal yang membuat ini bukan sekadar "kurangi sudut IMU dari pose badan":
//
//  1. SUMBU IMU BELUM TENTU SUMBU BADAN. Bergantung orientasi pasang modul,
//     roll IMU bisa jadi pitch badan, dan tandanya bisa terbalik. Karena itu
//     hubungan keduanya DIUKUR dulu (matriks Jacobian 2x2) dengan memiringkan
//     badan +-LVL_PROBE ke tiap sumbu dan melihat apa yang berubah di IMU.
//     Hasilnya sekalian menjawab STAB_SWAP_ROLL_PITCH di config.h firmware.
//  2. IMU BELUM TENTU TERPASANG SEJAJAR PELAT BADAN. Itu diserap 'Z' (acuan).
//  3. LANTAI BELUM TENTU RATA. Itu diserap 'Q' (uji putar 180 der).

// Rata-rata roll & pitch IMU + sebarannya. Robot tetap ditahan pada posenya
// selama pengukuran (motionTick jalan terus), dan hanya frame sudut BARU yang
// dihitung — kalau tidak, satu frame yang sama akan tercacah berkali-kali dan
// sebarannya jadi bohong (terlihat mulus padahal datanya basi).
// Laju frame sudut TERUKUR (Hz), diisi tiap ukurRataT. Jangan percaya 200 Hz
// yang disetel di aplikasi WIT — yang berlaku adalah yang benar-benar sampai.
float rateHz = 0;

// Rata-rata roll & pitch IMU + sebarannya, TAHAN LIPAT ±180.
//
// Kenapa tahan lipat itu wajib, bukan kehati-hatian berlebihan: kalau IMU
// terpasang terbalik, roll diam di sekitar ±180 — dan di sana +179,9 dengan
// -179,9 itu BERTETANGGA (beda 0,2 der), bukan berjauhan. Merata-ratakan
// mentah-mentah memberi 0 (badan seolah tegak) dan mengurangkan mentah-mentah
// memberi 359,8 (badan seolah terjungkal). Karena itu semuanya dihitung
// sebagai SELISIH terhadap sampel pertama, lalu dilipat ke (-180, 180].
//
// Jumlah sampel, bukan lamanya waktu, yang menentukan kapan berhenti: laju
// frame sudut yang sebenarnya sampai bisa jauh di bawah 200 Hz, dan jendela
// tetap dalam milidetik membuat pengukuran gagal karena kekurangan sampel
// padahal IMU-nya sehat. Batas waktunya tetap ada supaya tidak menggantung.
static bool ukurRataT(uint32_t settleMs, uint32_t avgMs, uint8_t minN,
                      float& mr, float& mp, float& sdr, float& sdp) {
    if (!tunggu(settleMs)) return false;

    uint32_t t0 = millis(), lastFrame = angFrames, n = 0;
    uint32_t batas = avgMs * 4 + 500;          // jaring pengaman, bukan target
    float refR = 0, refP = 0;                  // sampel pertama = titik acuan
    double sr = 0, sp = 0, sr2 = 0, sp2 = 0;

    for (;;) {
        witPump();
        if (aktif && millis() - lastWrite >= writeMs()) { lastWrite = millis(); motionTick(); }
        if (angFrames != lastFrame) {
            lastFrame = angFrames;
            if (n == 0) { refR = roll; refP = pitch; }
            float dr = wrap180(roll  - refR);
            float dp = wrap180(pitch - refP);
            sr += dr; sp += dp; sr2 += (double)dr * dr; sp2 += (double)dp * dp;
            n++;
        }
        uint32_t lewat = millis() - t0;
        if (lewat >= avgMs && n >= minN) break;
        if (lewat >= batas) break;
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            Serial.println(F("  ** dibatalkan **"));
            return false;
        }
    }

    uint32_t lama = millis() - t0;
    rateHz = lama ? (float)n * 1000.0f / (float)lama : 0.0f;

    if (n < minN) {
        Serial.print(F("  ! cuma ")); Serial.print(n);
        Serial.print(F(" frame sudut dalam ")); Serial.print(lama);
        Serial.print(F(" ms (")); Serial.print(rateHz, 0);
        Serial.println(F(" Hz) — IMU tidak mengalir cukup cepat."));
        Serial.println(F("    Cek 'T' (frame harus bertambah) dan setelan output rate di"));
        Serial.println(F("    aplikasi WIT. Untuk sementara alat ini tetap jalan asal >20 Hz."));
        return false;
    }

    float dR = (float)(sr / n), dP = (float)(sp / n);
    mr = wrap180(refR + dR); mp = wrap180(refP + dP);
    double vr = sr2 / n - (double)dR * dR, vp = sp2 / n - (double)dP * dP;
    sdr = vr > 0 ? sqrtf((float)vr) : 0.0f;
    sdp = vp > 0 ? sqrtf((float)vp) : 0.0f;
    return true;
}

static bool ukurRata(float& mr, float& mp, float& sdr, float& sdp) {
    return ukurRataT(LVL_SETTLE_MS, LVL_AVG_MS, 20, mr, mp, sdr, sdp);
}

// Acuan "badan rata" yang masuk akal untuk pemasangan IMU apa pun.
// Modul selalu terpasang pada kelipatan 90 der (tegak, terbalik, miring ke
// samping) — pecahan derajatnya yang jadi urusan 'Z'. Jadi kalau pembacaan
// diam jauh dari acuan yang berlaku, acuannya yang salah, bukan badannya:
// robot yang sedang berdiri tidak mungkin benar-benar miring 45 der.
static bool acuanOtomatis(float mr, float mp) {
    if (fabsf(wrap180(mr - refRoll)) <= 45.0f && fabsf(wrap180(mp - refPitch)) <= 45.0f)
        return false;
    refRoll  = roundf(mr / 90.0f) * 90.0f;
    refPitch = roundf(mp / 90.0f) * 90.0f;
    Serial.print(F("  acuan disetel otomatis ke kelipatan 90 terdekat: roll "));
    Serial.print(refRoll, 0); Serial.print(F("  pitch ")); Serial.print(refPitch, 0);
    Serial.println(F(" der"));
    Serial.println(F("    (IMU terpasang tidak tegak — itu tidak masalah. Pakai 'Z' kalau"));
    Serial.println(F("     ingin acuan yang lebih halus daripada kelipatan 90.)"));
    return true;
}

static void setTrim(float r, float p) {
    mo.trimRoll  = constrain(r, -LVL_MAX_TRIM, LVL_MAX_TRIM);
    mo.trimPitch = constrain(p, -LVL_MAX_TRIM, LVL_MAX_TRIM);
}

// Ekuivalen trim rata sekarang dalam pergeseran tinggi TIAP TELAPAK (mm).
// Ini jembatan ke KALIBRASI: kalau satu kaki angkanya menonjol sendiri,
// yang salah kaki itu, bukan badannya.
static void cetakEkivalenKaki() {
    float pr = mo.trimPitch * DEG2RAD, rr = mo.trimRoll * DEG2RAD;
    Serial.println(F("  setara pergeseran tinggi telapak (mm, + = kaki ditarik NAIK):"));
    Serial.print(F("   "));
    for (uint8_t leg = 0; leg < 6; leg++) {
        float h[3], b[3];
        footHome(leg, mo.prof.standR, mo.prof.standH, h);
        moRotInv(h[0], h[1], h[2], pr, rr, 0.0f, b);
        Serial.print(F(" K")); Serial.print(leg); Serial.print(':');
        Serial.print(b[2] - h[2], 1);
    }
    Serial.println();
}

static void lvlStatus() {
    Serial.println(F("\nrata badan:"));
    Serial.print(F("  trim   roll ")); Serial.print(mo.trimRoll, 2);
    Serial.print(F("  pitch ")); Serial.print(mo.trimPitch, 2); Serial.println(F(" der"));
    Serial.print(F("  acuan  roll ")); Serial.print(refRoll, 2);
    Serial.print(F("  pitch ")); Serial.print(refPitch, 2);
    Serial.println((refRoll == 0 && refPitch == 0)
                   ? F(" der  (BAWAAN — IMU dianggap sejajar pelat badan)")
                   : F(" der  (direkam lewat 'Z')"));
    if (punyaJac) {
        Serial.println(F("  sumbu  (d sudut IMU / d perintah badan):"));
        Serial.print(F("    roll IMU  <- roll ")); Serial.print(jac[0][0], 2);
        Serial.print(F("  pitch ")); Serial.println(jac[0][1], 2);
        Serial.print(F("    pitch IMU <- roll ")); Serial.print(jac[1][0], 2);
        Serial.print(F("  pitch ")); Serial.println(jac[1][1], 2);
    } else Serial.println(F("  sumbu  belum dikenali — 'A' akan mengukurnya dulu."));
    if (mo.trimRoll != 0 || mo.trimPitch != 0) cetakEkivalenKaki();
    if (punyaSudut) {
        Serial.print(F("  IMU sekarang: roll ")); Serial.print(roll, 1);
        Serial.print(F("  pitch ")); Serial.print(pitch, 1);
        Serial.print(F("  -> sisa miring ")); Serial.print(wrap180(roll - refRoll), 1);
        Serial.print('/'); Serial.print(wrap180(pitch - refPitch), 1);
        Serial.println(F(" der"));
    }
}

// ---------------------------------------- identifikasi sumbu (Jacobian 2x2)
// Beda TENGAH (+probe dikurangi -probe), bukan beda maju dari titik nol:
// itu membuang hanyutan pelan IMU dan asimetri sag servo, yang dua-duanya
// muncul sebagai suku konstan dan hilang saat dikurangkan.
static bool identJac(float probe) {
    float baseR = mo.trimRoll, baseP = mo.trimPitch;
    float m[2][2][2];                       // [sumbu][arah -/+][roll/pitch IMU]

    Serial.print(F("  mengenali sumbu IMU dengan uji +-"));
    Serial.print(probe, 1); Serial.println(F(" der ..."));

    for (uint8_t ax = 0; ax < 2; ax++) {
        for (uint8_t s = 0; s < 2; s++) {
            float d = s ? probe : -probe;
            setTrim(baseR + (ax == 0 ? d : 0), baseP + (ax == 1 ? d : 0));
            float mr, mp, sdr, sdp;
            if (!ukurRata(mr, mp, sdr, sdp)) { setTrim(baseR, baseP); return false; }
            m[ax][s][0] = mr; m[ax][s][1] = mp;
            Serial.print(F("    ")); Serial.print(ax == 0 ? F("roll ") : F("pitch"));
            Serial.print(d > 0 ? F(" +") : F(" -")); Serial.print(probe, 1);
            Serial.print(F(" der -> IMU ")); Serial.print(mr, 2);
            Serial.print('/'); Serial.print(mp, 2);
            Serial.print(F("  (sebar ")); Serial.print(sdr, 2);
            Serial.print('/'); Serial.print(sdp, 2); Serial.println(F(")"));
        }
    }
    setTrim(baseR, baseP);

    // wrap180: pengurangan mentah salah kalau IMU diam di dekat +-180.
    for (uint8_t ax = 0; ax < 2; ax++) {
        jac[0][ax] = wrap180(m[ax][1][0] - m[ax][0][0]) / (2.0f * probe);
        jac[1][ax] = wrap180(m[ax][1][1] - m[ax][0][1]) / (2.0f * probe);
    }
    float det = jac[0][0] * jac[1][1] - jac[0][1] * jac[1][0];

    Serial.print(F("  Jacobian: [")); Serial.print(jac[0][0], 2); Serial.print(' ');
    Serial.print(jac[0][1], 2); Serial.print(F(" ; ")); Serial.print(jac[1][0], 2);
    Serial.print(' '); Serial.print(jac[1][1], 2);
    Serial.print(F("]  det ")); Serial.println(det, 2);

    if (fabsf(det) < LVL_DET_MIN) {
        Serial.println(F("  ! badan dimiringkan tapi IMU nyaris tidak bereaksi."));
        Serial.println(F("    Biasanya: kaki SELIP / robot masih ditahan tangan / ada kaki"));
        Serial.println(F("    yang menggantung, jadi badan tidak benar-benar ikut miring."));
        Serial.println(F("    Bisa juga IMU membeku (cek 'T': frame harus bertambah)."));
        punyaJac = false;
        return false;
    }

    // Tafsiran — sekaligus jawaban untuk STAB_SWAP_ROLL_PITCH di firmware.
    // Yang ditanya: perintah ROLL badan (KOLOM 0) muncul di sudut IMU yang mana?
    // Jadi yang dibandingkan jac[0][0] lawan jac[1][0], bukan sebaris.
    bool tukar = fabsf(jac[1][0]) > fabsf(jac[0][0]);
    Serial.print(F("  -> roll badan terbaca di "));
    Serial.print(tukar ? F("PITCH") : F("ROLL"));
    Serial.print(F(" IMU, tanda "));
    Serial.println((tukar ? jac[1][0] : jac[0][0]) > 0 ? F("+") : F("-"));
    Serial.print(F("     firmware: STAB_SWAP_ROLL_PITCH "));
    Serial.println(tukar ? F("1") : F("0"));
    punyaJac = true;
    return true;
}

// -------------------------------------------------- kalibrasi rata ('A')
// return true kalau selesai (bukan dibatalkan). sisa[] diisi galat akhir.
static bool ratakan(float probe, bool cetakJudul, float* sisaOut) {
    if (!aktif) { Serial.println(F("robot belum berdiri — 's' dulu.")); return false; }
    if (!punyaSudut) { Serial.println(F("Tidak ada data sudut dari IMU — kalibrasi rata butuh roll/pitch.")); return false; }

    anim = A_NONE;                 // demo body akan melawan pengukuran
    mo.setMove(0, 0, 0);           // harus DIAM: gait membuat badan bergoyang

    if (cetakJudul) {
        Serial.println(F("\n--- KALIBRASI RATA BADAN (otomatis, pakai IMU) ---"));
        Serial.println(F("  SYARAT: keenam kaki menapak, robot TIDAK dipegang,"));
        Serial.println(F("  di lantai yang datar. Badan akan dimiringkan sedikit"));
        Serial.println(F("  beberapa kali. Ketik apa saja = batal.\n"));

        bool adaZ = false;
        for (uint8_t i = 0; i < 6; i++) if (mo.zOff[i] != 0) adaZ = true;
        if (!adaZ) {
            Serial.println(F("  ! offset telapak masih nol semua. Kalau ada kaki yang tidak"));
            Serial.println(F("    menapak, 'A' TIDAK BISA memperbaikinya — IMU hanya melihat"));
            Serial.println(F("    bidang lewat kaki yang menapak. Jalankan 'J' (atau 'K') dulu.\n"));
        }
    }

    float mr, mp, sdr, sdp;
    if (!ukurRata(mr, mp, sdr, sdp)) return false;
    acuanOtomatis(mr, mp);
    float awalR = wrap180(mr - refRoll), awalP = wrap180(mp - refPitch);
    Serial.print(F("  miring awal: roll ")); Serial.print(awalR, 2);
    Serial.print(F("  pitch ")); Serial.print(awalP, 2);
    Serial.print(F(" der   (sebar ")); Serial.print(sdr, 2);
    Serial.print('/'); Serial.print(sdp, 2); Serial.println(F(" der)"));
    if (sdr > 1.0f || sdp > 1.0f)
        Serial.println(F("  ! sebar > 1 der: robot bergoyang. Hasil akan kasar."));

    if (!punyaJac && !identJac(probe)) return false;

    float det = jac[0][0] * jac[1][1] - jac[0][1] * jac[1][0];
    Serial.println(F("  meratakan ..."));

    float eR = awalR, eP = awalP;
    uint8_t it = 0;
    for (; it < LVL_MAX_ITER; it++) {
        if (fabsf(eR) <= LVL_TOL_DEG && fabsf(eP) <= LVL_TOL_DEG) break;
        // Newton: J * dCmd = e  ->  dCmd = J^-1 * e, lalu trim dikurangi dCmd.
        float dR = ( jac[1][1] * eR - jac[0][1] * eP) / det;
        float dP = (-jac[1][0] * eR + jac[0][0] * eP) / det;
        setTrim(mo.trimRoll - dR, mo.trimPitch - dP);
        if (!ukurRata(mr, mp, sdr, sdp)) return false;
        eR = wrap180(mr - refRoll); eP = wrap180(mp - refPitch);
        Serial.print(F("    #")); Serial.print(it + 1);
        Serial.print(F("  trim ")); Serial.print(mo.trimRoll, 2);
        Serial.print('/'); Serial.print(mo.trimPitch, 2);
        Serial.print(F(" der -> sisa ")); Serial.print(eR, 2);
        Serial.print('/'); Serial.print(eP, 2); Serial.println(F(" der"));
    }

    if (sisaOut) { sisaOut[0] = eR; sisaOut[1] = eP; }

    bool sampai = (fabsf(eR) <= LVL_TOL_DEG && fabsf(eP) <= LVL_TOL_DEG);
    Serial.println(F("\n  ---------------- HASIL ----------------"));
    Serial.print(F("  miring : ")); Serial.print(awalR, 2); Serial.print('/');
    Serial.print(awalP, 2); Serial.print(F(" -> ")); Serial.print(eR, 2);
    Serial.print('/'); Serial.print(eP, 2); Serial.print(F(" der  ("));
    Serial.print(sampai ? F("RATA") : F("belum masuk toleransi")); Serial.println(')');
    Serial.print(F("  trim   : roll ")); Serial.print(mo.trimRoll, 2);
    Serial.print(F("  pitch ")); Serial.print(mo.trimPitch, 2);
    Serial.print(F(" der   (")); Serial.print(it); Serial.println(F(" iterasi)"));

    // Angka yang bisa dibayangkan: berapa mm ujung kiri & ujung depan tadinya
    // terangkat. bentangX() 320 mm pada pose baku.
    float bx = mo.bentangX();
    Serial.print(F("  yaitu  : ")); Serial.print(fabsf(tanf(awalR * DEG2RAD) * bx), 0);
    Serial.print(F(" mm beda tinggi kiri-kanan sepanjang bentang "));
    Serial.print(bx / 10.0f, 0); Serial.println(F(" cm"));
    cetakEkivalenKaki();

    if (fabsf(mo.trimRoll) > 8.0f || fabsf(mo.trimPitch) > 8.0f) {
        Serial.println(F("  ! trim di atas 8 der bukan lagi soal setelan — periksa"));
        Serial.println(F("    kaki bengkok, horn servo meleset satu gerigi, atau rangka"));
        Serial.println(F("    yang tidak simetris. Memaksakannya memakan jangkauan IK."));
    }
    if (!sampai) {
        // Dua sebab yang beda obatnya, jadi dibedakan lewat MEMBESAR vs mandek.
        if (fabsf(eR) + fabsf(eP) > fabsf(awalR) + fabsf(awalP))
            Serial.println(F("  ! sisa MEMBESAR — sumbu IMU tersimpan sudah tidak cocok"
                             " (IMU dilepas/dipasang ulang?). Ketik 'aj' lalu 'A' lagi."));
        else
            Serial.println(F("  Sisa mandek biasanya berarti ada kaki yang MENGGANTUNG"
                             " (tidak menahan beban) — itu tak bisa diperbaiki dengan"
                             " memiringkan badan. Lihat angka mm per kaki di atas."));
    }
    Serial.println(F("  'W' menyimpan ke EEPROM. 'Q' memastikan ini badan, bukan lantai."));
    return true;
}

// ------------------------------------------- pisahkan badan vs lantai ('Q')
// Trim hasil 'A' = miring BADAN + miring LANTAI. Keduanya tak terbedakan dari
// satu posisi. Tapi kalau robot diputar 180 der di TITIK YANG SAMA, komponen
// lantai berbalik tanda di frame badan sedangkan komponen badan tidak:
//     t1 = -(badan + lantai)      t2 = -(badan - lantai)
// -> badan = -(t1+t2)/2   lantai = -(t1-t2)/2
// Rata-rata kedua trim itulah trim yang benar; selisihnya memberi tahu seberapa
// miring lantai tempat uji (berguna: kalau besar, semua uji gait di situ pincang).
static void ujiLantai(float probe) {
    if (!aktif) { Serial.println(F("robot belum berdiri — 's' dulu.")); return; }

    Serial.println(F("\n=== UJI BADAN vs LANTAI (2 posisi) ==="));
    Serial.println(F("  Tahap 1: ratakan di posisi sekarang."));
    float sisa[2];
    if (!ratakan(probe, false, sisa)) return;
    float t1R = mo.trimRoll, t1P = mo.trimPitch;

    Serial.println(F("\n  Tahap 2: ANGKAT robot, putar 180 der, taruh KEMBALI DI"));
    Serial.println(F("  TITIK YANG SAMA (kaki di petak lantai yang sama), lalu tekan Enter."));
    Serial.println(F("  ('x' + Enter = batal)"));
    while (Serial.available()) Serial.read();
    for (;;) {
        witPump();
        if (aktif && millis() - lastWrite >= writeMs()) { lastWrite = millis(); motionTick(); }
        if (!Serial.available()) continue;
        char c = Serial.read();
        while (Serial.available()) Serial.read();
        if (c == 'x' || c == 'X') { Serial.println(F("  ** dibatalkan **")); return; }
        break;
    }

    if (!ratakan(probe, false, sisa)) return;
    float t2R = mo.trimRoll, t2P = mo.trimPitch;

    float badanR = (t1R + t2R) / 2.0f, badanP = (t1P + t2P) / 2.0f;
    float lantaiR = (t1R - t2R) / 2.0f, lantaiP = (t1P - t2P) / 2.0f;

    Serial.println(F("\n  ---------------- PISAHAN ----------------"));
    Serial.print(F("  posisi 1 : ")); Serial.print(t1R, 2); Serial.print('/');
    Serial.print(t1P, 2); Serial.println(F(" der"));
    Serial.print(F("  posisi 2 : ")); Serial.print(t2R, 2); Serial.print('/');
    Serial.print(t2P, 2); Serial.println(F(" der"));
    Serial.print(F("  BADAN    : ")); Serial.print(badanR, 2); Serial.print('/');
    Serial.print(badanP, 2); Serial.println(F(" der   <- ini yang dipakai jadi trim"));
    Serial.print(F("  lantai   : ")); Serial.print(lantaiR, 2); Serial.print('/');
    Serial.print(lantaiP, 2); Serial.println(F(" der   <- sifat tempat uji, bukan robot"));

    setTrim(badanR, badanP);
    float bx = mo.bentangX();
    if (fabsf(lantaiR) > 1.0f || fabsf(lantaiP) > 1.0f) {
        Serial.print(F("  ! lantai miring setara "));
        Serial.print(fabsf(tanf(lantaiR * DEG2RAD) * bx), 0);
        Serial.println(F(" mm per bentang badan."));
        Serial.println(F("    Tanpa uji ini, kemiringan lantai akan ikut tersimpan"));
        Serial.println(F("    sebagai trim dan robot jadi miring di tempat lain."));
    } else {
        Serial.println(F("  Lantai praktis rata — hasil 'A' saja sudah sah."));
    }
    Serial.println(F("  Badan sekarang dipasang ke trim BADAN. 'W' untuk menyimpan."));
}

// ---------------------------------------------------- acuan IMU ('Z')
static void setAcuan() {
    if (!punyaSudut) { Serial.println(F("Tidak ada data sudut dari IMU.")); return; }
    Serial.println(F("\nMerekam ACUAN 'badan rata' dari pembacaan IMU sekarang."));
    Serial.println(F("  Ini hanya benar kalau badan MEMANG sedang rata menurut alat"));
    Serial.println(F("  di luar IMU (waterpass di pelat atas). Gunanya menyerap"));
    Serial.println(F("  kesalahan PEMASANGAN IMU, yang tidak bisa dibedakan dari"));
    Serial.println(F("  kemiringan badan oleh IMU itu sendiri."));
    float mr, mp, sdr, sdp;
    if (!ukurRata(mr, mp, sdr, sdp)) return;
    refRoll = mr; refPitch = mp;
    Serial.print(F("  acuan := roll ")); Serial.print(refRoll, 2);
    Serial.print(F("  pitch ")); Serial.print(refPitch, 2);
    Serial.print(F(" der  (sebar ")); Serial.print(sdr, 2);
    Serial.print('/'); Serial.print(sdp, 2); Serial.println(F(")"));
    Serial.println(F("  'W' untuk menyimpan. 'Z0' mengembalikan acuan ke nol."));
}

// ============================================== KALIBRASI TELAPAK PER KAKI
// Masalah yang dijawab: pada 90 der semua sendi terlihat benar, tapi pada pose
// BERDIRI ada kaki yang tidak menapak.
//
// Kenapa bisa begitu padahal 90 der sudah bagus: trim di servo_map.h satuannya
// MIKRODETIK dan ditambahkan setelah konversi derajat->pulse, jadi ia offset
// MURNI — kalau 90 der benar, semua sudut ikut benar SELAMA gain-nya benar.
// Yang tidak dijamin justru gain itu: 500..2500 us dianggap tepat 180 der
// (11,11 us/der), padahal tiap servo berbeda beberapa persen. Galatnya nol di
// 90 der dan tumbuh sebanding jaraknya dari 90 — dan pose berdiri memang jauh
// dari 90 (femur 79,4 = -10,6 der, tibia 82,0 = -8,0 der). Tambah lagi:
//   - panjang femur/tibia tiap kaki tidak persis 80/90 mm,
//   - sag servo saat MENAHAN BEBAN, yang tidak muncul saat 90 der tanpa beban.
// Ketiganya cuma bisa diukur PADA POSE BERDIRI DAN DALAM KEADAAN BERBEBAN —
// itulah yang dilakukan di sini, bukan menyetel ulang sudut netral.
//
// Hasilnya per kaki, bukan per servo, karena yang penting posisi TELAPAK:
// mo.zOff[6] dalam mm, negatif = kaki dipanjangkan.
//
// CARA OTOMATIS ('J') — memakai IMU sebagai sensor sentuh, tanpa hardware
// tambahan. Robot ditumpu SATU TRIPOD (3 kaki, statis tertentu), tiga kaki
// lain diangkat. Satu kaki uji diturunkan bertahap; begitu ia menyentuh lantai
// dan terus memanjang, ia MENGUNGKIT badan — dan itu terlihat jelas di
// roll/pitch IMU. Ketinggian saat ungkitan mulai terdeteksi = tinggi lantai
// di bawah kaki itu, diukur relatif terhadap badan.
//
// Kenapa harus TRIPOD, bukan 5 kaki menumpu: dengan 3 titik tumpu badan
// tertentu sepenuhnya, jadi kaki keempat yang menyentuh pasti memiringkan
// badan. Dengan 5 titik tumpu sistemnya berlebih — kaki keenam bisa menekan
// tanpa badan bergerak berarti, dan sentuhannya tak terdeteksi.

static const uint8_t TRIPOD[2][3] = { { 0, 2, 4 }, { 1, 3, 5 } };

static void kakiPasang(uint32_t settleMs) {
    mo.setHome();                    // snap, jangan menunggu low-pass settle
    tunggu(settleMs);
}

static void cetakTabelKaki(const float* v, const __FlashStringHelper* judul) {
    Serial.print(F("  ")); Serial.println(judul);
    Serial.println(F("    kaki   K0     K1     K2     K3     K4     K5"));
    Serial.print(F("    mm  "));
    for (uint8_t i = 0; i < 6; i++) {
        float x = v[i];
        Serial.print(x >= 0 ? F("  +") : F("  "));
        Serial.print(x, 1);
        if (fabsf(x) < 10.0f) Serial.print(' ');
    }
    Serial.println();
}

// Pindai satu kaki dari atas ke bawah sampai badan mulai terungkit.
// zSentuh = nilai zOff (mm) saat sentuhan terjadi.
// hasil: 0 = ketemu, 1 = tidak menyentuh sampai batas bawah,
//        2 = SUDAH menyentuh di batas atas (kaki kepanjangan, di luar jangkauan pindai),
//        3 = pengukuran gagal / dibatalkan  <- WAJIB dibedakan dari 1: melaporkan
//            IMU yang macet sebagai "kaki kependekan" mengirim orang membongkar
//            mekanik yang sebenarnya tidak apa-apa.
static uint8_t cariSentuh(uint8_t leg, float zAtas, float zBawah,
                          float baseR, float baseP, float ambang, float& zSentuh) {
    float zPrev = zAtas, dPrev = 0.0f;
    bool  ketemu = false;
    float zKasar = zAtas;
    bool  pertama = true;

    // --- tahap 1: langkah kasar, cari KURUNGAN ---
    for (float z = zAtas; z >= zBawah - 0.001f; z -= KAKI_KASAR_MM) {
        mo.zOff[leg] = z;
        kakiPasang(KAKI_SETTLE_MS);
        float mr, mp, sdr, sdp;
        if (!ukurRataT(0, KAKI_AVG_MS, KAKI_MIN_N, mr, mp, sdr, sdp)) return 3;
        float d = fabsf(wrap180(mr - baseR)) + fabsf(wrap180(mp - baseP));
        // Sudah mengungkit di langkah PERTAMA berarti sentuhan sebenarnya ada
        // di ATAS jangkauan pindai. Melaporkannya sebagai zAtas akan diam-diam
        // mengecilkan koreksi yang dibutuhkan — lebih baik mengaku tidak tahu.
        if (d > ambang && pertama) return 2;
        if (d > ambang) { ketemu = true; zKasar = z; break; }
        zPrev = z; dPrev = d; pertama = false;
    }
    if (!ketemu) return 1;

    // --- tahap 2: langkah halus di dalam kurungan, + interpolasi ---
    // Interpolasi linear pada perpotongan ambang: sudut ungkit tumbuh mulus
    // dengan kedalaman tekan, jadi titik potongnya berarti, bukan sekadar
    // pembulatan ke langkah terdekat.
    for (float z = zPrev - KAKI_HALUS_MM; z >= zKasar - 0.001f; z -= KAKI_HALUS_MM) {
        mo.zOff[leg] = z;
        kakiPasang(KAKI_SETTLE_MS);
        float mr, mp, sdr, sdp;
        if (!ukurRataT(0, KAKI_AVG_MS, KAKI_MIN_N, mr, mp, sdr, sdp)) return 3;
        float d = fabsf(wrap180(mr - baseR)) + fabsf(wrap180(mp - baseP));
        if (d > ambang) {
            float beda = d - dPrev;
            zSentuh = (beda > 1e-3f) ? zPrev + (z - zPrev) * (ambang - dPrev) / beda : z;
            return 0;
        }
        zPrev = z; dPrev = d;
    }
    zSentuh = zKasar;                // ambang terlewati di langkah kasar saja
    return 0;
}

// Buang komponen BIDANG dari offset (kuadrat terkecil pada basis {1, y, x}).
//
// Kenapa ini sah: menambahkan bidang apa pun ke keenam offset TIDAK mengubah
// kesebidangan telapak — bidang dikurangi bidang tetap bidang. Yang berubah
// hanya sikap badan, dan itu justru tugas 'A'. Jadi komponen bidang boleh
// dipindahkan ke 'A' secara cuma-cuma.
//
// Kenapa dilakukan: tanpa ini seluruh koreksi menumpuk di tiga kaki yang
// diukur, dan offset besar memakan jangkauan IK. Di robot ini offset terbesar
// turun dari 10,0 mm jadi 5,0 mm — persis setengahnya — tanpa mengubah
// satu pun posisi telapak terhadap lantai.
static void kakiBuangBidang(float o[6]) {
    float X[6], Y[6];
    for (uint8_t i = 0; i < 6; i++) {
        float h[3];
        footHome(i, mo.prof.standR, mo.prof.standH, h);
        X[i] = h[0]; Y[i] = h[1];
    }
    double A[3][4];
    for (uint8_t r = 0; r < 3; r++) for (uint8_t c = 0; c < 4; c++) A[r][c] = 0;
    for (uint8_t i = 0; i < 6; i++) {
        double b[3] = { 1.0, Y[i], X[i] };
        for (uint8_t r = 0; r < 3; r++) {
            for (uint8_t c = 0; c < 3; c++) A[r][c] += b[r] * b[c];
            A[r][3] += b[r] * o[i];
        }
    }
    for (uint8_t c = 0; c < 3; c++) {                       // eliminasi Gauss
        uint8_t p = c;
        for (uint8_t r = c + 1; r < 3; r++) if (fabs(A[r][c]) > fabs(A[p][c])) p = r;
        if (fabs(A[p][c]) < 1e-9) return;                   // tak terpecahkan: biarkan
        for (uint8_t k = 0; k < 4; k++) { double t = A[c][k]; A[c][k] = A[p][k]; A[p][k] = t; }
        for (uint8_t r = 0; r < 3; r++) {
            if (r == c) continue;
            double f = A[r][c] / A[c][c];
            for (uint8_t k = c; k < 4; k++) A[r][k] -= f * A[c][k];
        }
    }
    float c0 = (float)(A[0][3] / A[0][0]);
    float c1 = (float)(A[1][3] / A[1][1]);
    float c2 = (float)(A[2][3] / A[2][2]);
    for (uint8_t i = 0; i < 6; i++) o[i] -= c0 + c1 * Y[i] + c2 * X[i];
}

// Satu pengukuran: robot bertumpu pada tripod `tumpu`, tiga kaki lain diangkat
// dan diukur satu per satu. hasil[] masuk berisi offset yang berlaku sekarang;
// yang diubah HANYA ketiga kaki yang diangkat.
//
// Hanya SATU tripod yang jadi acuan, dan itu bukan penghematan waktu — itu
// syarat kebenaran. Tiga titik selalu tepat membentuk bidang, jadi galat kaki
// penumpu seluruhnya berupa kemiringan badan, yang memang tugas 'A'. Ketiga
// kaki yang diangkat lalu diukur TERHADAP bidang itu, dan hasilnya langsung
// membuat keenam telapak sebidang — eksak, sekali jalan, tanpa iterasi.
//
// Versi sebelumnya mengukur KEDUA tripod lalu memperbarui keenam offset
// serentak. Itu tidak konvergen: tiap tripod diukur memakai offset tripod
// lawannya, jadi memperbarui keduanya sekaligus membuat hasilnya berayun
// (terukur 9,80 -> 0 -> 9,80 -> 0 di uji bagian 11).
static bool kakiUkurTripod(uint8_t tumpu, float hasil[6]) {
    float asli[6];
    for (uint8_t i = 0; i < 6; i++) asli[i] = mo.zOff[i];
    const uint8_t* uji = TRIPOD[1 - tumpu];        // yang DIANGKAT & diukur
    bool ok = true;

    Serial.print(F("  tumpu tripod ")); Serial.print(tumpu == 0 ? F("A") : F("B"));
    Serial.print(F(" {"));
    for (uint8_t k = 0; k < 3; k++) { Serial.print('K'); Serial.print(TRIPOD[tumpu][k]); Serial.print(' '); }
    Serial.println(F("} — mengangkat & mengukur tiga kaki lain ..."));

    for (uint8_t k = 0; k < 3; k++) mo.zOff[uji[k]] = asli[uji[k]] + KAKI_ANGKAT_MM;
    mo.setHome();
    if (!tunggu(900)) {                            // badan turun ke tripod
        for (uint8_t i = 0; i < 6; i++) mo.zOff[i] = asli[i];
        mo.setHome();
        return false;
    }

    float bR, bP, sdr, sdp;
    if (!ukurRataT(200, 400, 20, bR, bP, sdr, sdp)) {
        for (uint8_t i = 0; i < 6; i++) mo.zOff[i] = asli[i];
        mo.setHome();
        return false;
    }
    float ambang = KAKI_AMBANG_X * (sdr + sdp);
    if (ambang < KAKI_AMBANG_DEG) ambang = KAKI_AMBANG_DEG;
    Serial.print(F("    acuan ")); Serial.print(bR, 2); Serial.print('/');
    Serial.print(bP, 2); Serial.print(F(" der, derau ")); Serial.print(sdr + sdp, 2);
    Serial.print(F(" -> ambang ungkit ")); Serial.print(ambang, 2);
    Serial.print(F(" der, IMU ")); Serial.print(rateHz, 0); Serial.println(F(" Hz"));
    if (rateHz < 20.0f) {
        Serial.println(F("    ! di bawah 20 Hz pindai jadi sangat lambat dan berisik."));
        Serial.println(F("      Naikkan output rate IMU di aplikasi WIT."));
    }

    {
        for (uint8_t k = 0; k < 3; k++) {
            uint8_t L = uji[k];
            float z;
            // Pindai SELURUH rentang sah (+-KAKI_MAX_OFF), bukan +-8 mm di
            // sekitar nilai kaki ini sekarang. Alasannya ketahuan di robot:
            // begitu tripod PENUMPU punya offset, badan ikut miring dan tinggi
            // sentuh kaki uji bergeser jauh — bisa keluar dari jendela yang
            // dipusatkan pada nilai lama, lalu dilaporkan "kaki kependekan"
            // padahal cuma jendelanya yang salah tempat. Rentang penuh membuat
            // hasilnya tidak bergantung pada tebakan awal sama sekali.
            uint8_t hs = cariSentuh(L, KAKI_PINDAI_MM, -KAKI_PINDAI_MM,
                                    bR, bP, ambang, z);
            if (hs == 0) {
                hasil[L] = z;
                Serial.print(F("    K")); Serial.print(L);
                Serial.print(F("  sentuh pada zOff ")); Serial.print(z, 2);
                Serial.println(F(" mm"));
            } else if (hs == 3) {
                Serial.print(F("    K")); Serial.print(L);
                Serial.println(F("  pengukuran GAGAL (IMU/dibatalkan) — bukan soal kaki ini."));
                ok = false;
            } else {
                Serial.print(F("    K")); Serial.print(L);
                Serial.println(hs == 2
                    ? F("  SUDAH mengungkit di +18 mm — kaki KEPANJANGAN di luar rentang pindai.")
                    : F("  TIDAK menyentuh sampai -18 mm — kaki KEPENDEKAN di luar rentang pindai."));
                ok = false;
            }
            mo.zOff[L] = asli[L] + KAKI_ANGKAT_MM;   // angkat lagi utk kaki berikutnya
            kakiPasang(200);
            if (!ok) break;
        }

    }

    for (uint8_t i = 0; i < 6; i++) mo.zOff[i] = asli[i];   // kembalikan dulu
    mo.setHome();
    tunggu(600);
    return ok;
}

static void kalibKakiOtomatis() {
    if (!aktif)      { Serial.println(F("robot belum berdiri — 's' dulu.")); return; }
    if (!punyaSudut) { Serial.println(F("Tidak ada data sudut dari IMU — 'J' memakai IMU sebagai sensor sentuh.")); return; }

    anim = A_NONE;
    mo.setMove(0, 0, 0);

    Serial.println(F("\n--- KALIBRASI TELAPAK OTOMATIS (IMU sbg sensor sentuh) ---"));
    Serial.println(F("  Robot akan berdiri di atas TIGA kaki bergantian dan"));
    Serial.println(F("  menurunkan kaki lain satu per satu sampai menyentuh lantai."));
    Serial.println(F("  Lantai HARUS keras & datar (jangan karpet/busa: telapak"));
    Serial.println(F("  tenggelam dan sentuhan tidak pernah mengungkit badan)."));
    Serial.println(F("  Jaga robot dengan tangan DEKAT tapi JANGAN menyentuh."));
    Serial.println(F("  Sekitar 1 menit. Ketik apa saja = batal.\n"));

    float sebelum[6];
    for (uint8_t i = 0; i < 6; i++) sebelum[i] = mo.zOff[i];

    // SATU tripod acuan, sekali ukur — eksak, tanpa iterasi (lihat komentar
    // kakiUkurTripod). Pass kedua memakai acuan yang SAMA, jadi ia benar-benar
    // uji ulang-ukur: kalau alatnya konsisten hasilnya harus sama.
    float h1[6];
    for (uint8_t i = 0; i < 6; i++) h1[i] = mo.zOff[i];
    Serial.println(F("\n  --- pengukuran ---"));
    if (!kakiUkurTripod(0, h1)) {
        for (uint8_t i = 0; i < 6; i++) mo.zOff[i] = sebelum[i];
        mo.setHome();
        Serial.println(F("\n  GAGAL / dibatalkan — offset dikembalikan seperti semula."));
        Serial.println(F("  Kalau ada kaki yang KEPANJANGAN/KEPENDEKAN di luar +-18 mm,"));
        Serial.println(F("  sebabnya mekanis, bukan setelan:"));
        Serial.println(F("    - horn servo meleset satu gerigi (spline 25T = 14,4 der,"));
        Serial.println(F("      di femur ~20 mm di telapak) — cek juga pose 90 der,"));
        Serial.println(F("    - femur/tibia terpasang terbalik atau panjangnya beda,"));
        Serial.println(F("    - lantai lunak sehingga sentuhan tak pernah mengungkit,"));
        Serial.println(F("    - robot tersangga sesuatu."));
        Serial.println(F("  Pakai 'K' (manual, uji kertas) untuk mendekatkan dulu, lalu 'J' lagi."));
        return;
    }
    for (uint8_t i = 0; i < 6; i++) mo.zOff[i] = h1[i];
    mo.setHome();
    cetakTabelKaki(h1, F("hasil ukur:"));

    // Ulang-ukur dengan acuan yang SAMA. Beda antara keduanya adalah
    // ketidakpastian NYATA alat ini di robot ini — angka yang jauh lebih
    // berguna daripada janji ketelitian di atas kertas.
    Serial.println(F("\n  --- ulang-ukur (uji konsistensi) ---"));
    float h2[6], ulang = -1.0f;
    for (uint8_t i = 0; i < 6; i++) h2[i] = mo.zOff[i];
    if (!kakiUkurTripod(0, h2)) {
        Serial.println(F("  ulang-ukur gagal/dibatalkan; hasil pengukuran pertama tetap dipakai."));
    } else {
        ulang = 0;
        for (uint8_t i = 0; i < 6; i++) {
            float d = fabsf(h2[i] - h1[i]);
            if (d > ulang) ulang = d;
            mo.zOff[i] = (h1[i] + h2[i]) * 0.5f;    // rerata dua pengukuran
        }
        mo.setHome();
        Serial.print(F("  beda terbesar antar dua pengukuran: ")); Serial.print(ulang, 2);
        Serial.println(ulang < 1.0f ? F(" mm — konsisten.")
                                    : F(" mm — besar; telapak licin, lantai lunak, atau servo bergetar."));
    }

    // Komponen bidang dipindahkan ke 'A'. Ini tidak menggeser satu pun telapak
    // terhadap lantai (bidang dikurangi bidang tetap bidang), tapi memangkas
    // offset terbesar — di robot uji dari 10,0 jadi 5,0 mm — sehingga jangkauan
    // IK tidak dimakan percuma.
    float sesudah[6];
    for (uint8_t i = 0; i < 6; i++) sesudah[i] = mo.zOff[i];
    float maksSebelumBidang = 0;
    for (uint8_t i = 0; i < 6; i++) if (fabsf(sesudah[i]) > maksSebelumBidang) maksSebelumBidang = fabsf(sesudah[i]);
    kakiBuangBidang(sesudah);
    float maksSesudahBidang = 0;
    for (uint8_t i = 0; i < 6; i++) if (fabsf(sesudah[i]) > maksSesudahBidang) maksSesudahBidang = fabsf(sesudah[i]);
    for (uint8_t i = 0; i < 6; i++) mo.zOff[i] = sesudah[i];
    mo.setHome();

    Serial.println(F("\n  ---------------- HASIL ----------------"));
    cetakTabelKaki(sebelum, F("sebelum:"));
    cetakTabelKaki(mo.zOff, F("sesudah:"));
    Serial.print(F("  komponen bidang dipindah ke trim badan: offset terbesar "));
    Serial.print(maksSebelumBidang, 1); Serial.print(F(" -> "));
    Serial.print(maksSesudahBidang, 1); Serial.println(F(" mm (posisi telapak tidak berubah)"));
    if (ulang >= 0) {
        Serial.print(F("  ulang-ukur ")); Serial.print(ulang, 2);
        Serial.println(F(" mm = ketidakpastian nyata metode ini di robot ini."));
    }

    float mn = 1e9f, mx = -1e9f;
    for (uint8_t i = 0; i < 6; i++) { if (mo.zOff[i] < mn) mn = mo.zOff[i]; if (mo.zOff[i] > mx) mx = mo.zOff[i]; }
    float rentang = mx - mn;
    Serial.print(F("  rentang ")); Serial.print(rentang, 1);
    Serial.println(F(" mm — beda tinggi telapak yang tadi membuat kaki menggantung."));

    // Rentang besar sudah dikoreksi, tapi jangan dibiarkan lewat tanpa
    // disebut: angkanya memberi tahu apakah penyebabnya masih dalam batas
    // yang bisa dijelaskan, atau sudah mekanis.
    if (rentang > KAKI_RENTANG_WAJAR) {
        Serial.println(F("  ! Rentang ini TERLALU BESAR untuk dijelaskan galat gain servo"));
        Serial.println(F("    (gain meleset 10% cuma ~1,5 mm di telapak) atau beda panjang"));
        Serial.println(F("    femur/tibia (~1 mm). Kalibrasi ini menutupinya, tapi kaki yang"));
        Serial.println(F("    offsetnya paling ekstrem layak diperiksa fisik: horn servo,"));
        Serial.println(F("    kekencangan sekrup, dan panjang link dibanding kaki lain."));
        Serial.println(F("    Offset besar memakan jangkauan IK dan menyisakan sedikit margin."));
    }
    for (uint8_t i = 0; i < 6; i++)
        if (fabsf(mo.zOff[i]) > KAKI_MAX_OFF - 0.1f) {
            Serial.print(F("  ! K")); Serial.print(i);
            Serial.println(F(" mentok di batas 12 mm — koreksinya kemungkinan terpotong."));
        }
    Serial.println(F("\n  Langkah berikutnya: 'A' (ratakan badan) lalu 'W' (simpan)."));
    Serial.println(F("  Urutan itu WAJIB — 'A' baru sah setelah keenam kaki menapak."));
}

// ---------------------------------------- kalibrasi telapak MANUAL ('K')
// Uji kertas: selipkan kertas di bawah telapak, tarik. Kalau lolos tanpa
// gesekan, kaki itu tidak menahan beban -> turunkan sampai kertas terjepit
// seperti kaki-kaki lain. Ini tidak butuh IMU dan tidak peduli lantai lunak,
// jadi ia jaring pengaman kalau 'J' gagal — dan cara mendekatkan kaki yang
// melesetnya di luar +-8 mm supaya 'J' bisa bekerja.
static void kalibKakiManual() {
    if (!aktif) { Serial.println(F("robot belum berdiri — 's' dulu.")); return; }
    anim = A_NONE;
    mo.setMove(0, 0, 0);

    uint8_t leg = 0;
    float langkah = 0.5f;

    Serial.println(F("\n--- KALIBRASI TELAPAK MANUAL (uji kertas) ---"));
    Serial.println(F("  Selipkan kertas di bawah tiap telapak lalu tarik:"));
    Serial.println(F("    lolos tanpa gesekan = kaki MENGGANTUNG, turunkan ('+')"));
    Serial.println(F("    tersangkut kuat      = kaki KEBERATAN, naikkan ('-')"));
    Serial.println(F("  Sasarannya: keenam telapak terasa sama."));
    Serial.println(F("  0..5 pilih kaki   + turunkan   - naikkan   s ganti langkah"));
    Serial.println(F("  n nolkan kaki ini   N nolkan semua   t tabel   x keluar"));
    Serial.println(F("  (boleh diketik beruntun, mis. '+++' lalu Enter)\n"));
    Serial.print(F("  kaki K0, langkah 0,5 mm\n"));

    while (Serial.available()) Serial.read();
    for (;;) {
        witPump();
        if (aktif && millis() - lastWrite >= writeMs()) { lastWrite = millis(); motionTick(); }
        if (!Serial.available()) continue;

        char c = Serial.read();
        if (c == '\r' || c == '\n' || c == ' ') continue;

        if (c >= '0' && c <= '5') {
            leg = c - '0';
            Serial.print(F("  -> kaki K")); Serial.print(leg);
            Serial.print(F("  zOff ")); Serial.print(mo.zOff[leg], 1); Serial.println(F(" mm"));
            continue;
        }
        switch (c) {
            case '+': case '=':
                mo.zOff[leg] -= langkah;           // telapak TURUN = kaki memanjang
                break;
            case '-': case '_':
                mo.zOff[leg] += langkah;
                break;
            case 's':
                langkah = (langkah > 0.9f) ? 0.2f : (langkah > 0.4f ? 1.0f : 0.5f);
                Serial.print(F("  langkah ")); Serial.print(langkah, 1); Serial.println(F(" mm"));
                continue;
            case 'n': mo.zOff[leg] = 0; break;
            case 'N': for (uint8_t i = 0; i < 6; i++) mo.zOff[i] = 0; break;
            case 't': cetakTabelKaki(mo.zOff, F("offset telapak sekarang:")); continue;
            case 'x':
                cetakTabelKaki(mo.zOff, F("offset telapak akhir:"));
                Serial.println(F("  keluar. 'A' untuk meratakan badan, 'W' untuk menyimpan."));
                return;
            default:
                Serial.println(F("  0..5 + - s n N t x"));
                continue;
        }
        mo.zOff[leg] = constrain(mo.zOff[leg], -KAKI_MAX_OFF, KAKI_MAX_OFF);
        mo.setHome();
        Serial.print(F("  K")); Serial.print(leg);
        Serial.print(F("  zOff ")); Serial.print(mo.zOff[leg], 1); Serial.println(F(" mm"));
    }
}

static void kakiStatus() {
    cetakTabelKaki(mo.zOff, F("offset telapak (negatif = kaki dipanjangkan):"));
    float mn = 1e9f, mx = -1e9f;
    bool ada = false;
    for (uint8_t i = 0; i < 6; i++) {
        if (mo.zOff[i] < mn) mn = mo.zOff[i];
        if (mo.zOff[i] > mx) mx = mo.zOff[i];
        if (mo.zOff[i] != 0) ada = true;
    }
    if (!ada) { Serial.println(F("  semua nol — belum dikalibrasi ('J' otomatis / 'K' manual).")); return; }
    Serial.print(F("  rentang ")); Serial.print(mx - mn, 1); Serial.println(F(" mm"));
    // Setara berapa derajat femur? Berguna untuk memutuskan apakah ini masih
    // wajar (galat gain servo) atau sudah kelewat (gerigi horn = 14,4 der).
    Serial.print(F("  setara ~")); Serial.print((mx - mn) / FEMUR_LEN * RAD2DEG, 1);
    Serial.println(F(" der di femur (1 gerigi spline 25T = 14,4 der)"));
}

// ============================================================ tampilan
static void showKnobs() {
    Serial.println(F("\nknob gerak:"));
    Serial.print(F("  h tinggi badan    ")); Serial.print(mo.tgtProf.standH, 0); Serial.println(F(" mm"));
    Serial.print(F("  R bentang kaki    ")); Serial.print(mo.tgtProf.standR, 0); Serial.println(F(" mm"));
    Serial.print(F("  k panjang langkah ")); Serial.print(mo.tgtProf.stepL, 0);  Serial.println(F(" mm"));
    Serial.print(F("  e tinggi angkat   ")); Serial.print(mo.tgtProf.stepH, 0);  Serial.println(F(" mm"));
    Serial.print(F("  p siklus          ")); Serial.print(mo.tgtProf.cycleMs, 0);Serial.println(F(" ms"));
    Serial.print(F("  d duty tumpu      ")); Serial.print(mo.duty * 100.0f, 0);  Serial.println(F(" %"));
    Serial.print(F("  l slew            ")); Serial.print(mo.slewRate, 1);       Serial.println(F(" /detik"));
    Serial.print(F("  w tulis PWM       ")); Serial.print(writeHz);              Serial.println(F(" Hz"));
    Serial.print(F("  -> laju teoretis  ")); Serial.print(mo.lajuTeoretis() / 10.0f, 1);
    Serial.println(F(" cm/detik"));
    printBody();
    Serial.println(F("kalibrasi:"));
    Serial.print(F("  pivot ")); Serial.print(degCCW, 2); Serial.print(F(" / "));
    Serial.print(degCW, 2); Serial.print(F(" der per siklus (tanda ")); Serial.print(pivotSign);
    Serial.println(')');
    Serial.print(F("  maju  ")); Serial.print(mmMaju, 1); Serial.println(F(" mm per siklus"));
}

static void telemetri() {
    Serial.print(F("IMU: "));
    if (!punyaSudut) { Serial.println(F("BELUM ADA FRAME SUDUT — cek TES_IMU dulu.")); }
    else {
        Serial.print(F("yaw ")); Serial.print(yaw, 1);
        Serial.print(F("  roll ")); Serial.print(roll, 1);
        Serial.print(F("  pitch ")); Serial.print(pitch, 1);
        Serial.print(F("  gyroZ ")); Serial.print(gz, 1);
        Serial.print(F("  frame ")); Serial.print(stOk);
        Serial.print(F(" gagal ")); Serial.println(stSumBad);
        // Laju frame SUDUT yang benar-benar sampai — bukan yang disetel di
        // aplikasi WIT. 'J' dan 'A' bergantung padanya, dan kalau terlalu
        // rendah gejalanya muncul sebagai "pengukuran gagal", bukan "lambat".
        float mr, mp, sdr, sdp;
        if (ukurRataT(0, 300, 1, mr, mp, sdr, sdp)) {
            Serial.print(F("     laju frame sudut TERUKUR ")); Serial.print(rateHz, 0);
            Serial.print(F(" Hz, sebar ")); Serial.print(sdr, 2);
            Serial.print('/'); Serial.print(sdp, 2); Serial.println(F(" der"));
            if (rateHz < 20.0f)
                Serial.println(F("     ! di bawah 20 Hz — naikkan output rate di aplikasi WIT."));
        }
    }
    Serial.print(F("gerak: vektor ")); Serial.print(mo.velX(), 2);
    Serial.print('/'); Serial.print(mo.velY(), 2);
    Serial.print('/'); Serial.print(mo.velYaw(), 2);
    Serial.print(F("  fase ")); Serial.print(mo.phase(), 2);
    Serial.print(mo.running() ? F("  JALAN") : F("  diam"));
    Serial.print(F("  bentang ")); Serial.print(mo.bentangX() / 10.0f, 1);
    Serial.println(F(" cm"));
}

static void printHelp() {
    Serial.println(F("\n=========== TES_GERAK — gait, body kinematics, pivot ==========="));
    Serial.println(F(" DASAR"));
    Serial.println(F("  n            netral 90 der (TOPANG ROBOT)"));
    Serial.println(F("  s            BERDIRI + mulai kendali gerak"));
    Serial.println(F("  x            STOP (kaki kembali ke pose berdiri)"));
    Serial.println(F("  r            lepas semua PWM     L  tabel pemetaan servo"));
    Serial.println(F(" BODY KINEMATICS  (6 kaki tetap menapak)"));
    Serial.println(F("  br<der>      roll  + = miring KANAN"));
    Serial.println(F("  bp<der>      pitch + = MENDONGAK"));
    Serial.println(F("  bw<der>      yaw badan + = ke KIRI (pivot halus tanpa melangkah)"));
    Serial.println(F("  bx/by/bz<mm> geser badan kanan+ / depan+ / naik+"));
    Serial.println(F("  b0           nolkan semua      bd  DEMO 6 sumbu berurutan"));
    Serial.println(F("  bl           UJI BATAS tiap sumbu (kering)"));
    Serial.println(F(" GAIT"));
    Serial.println(F("  g / G        MAJU / MUNDUR      [ / ]  geser KIRI / KANAN"));
    Serial.println(F("  y / Y        putar KIRI / KANAN di tempat"));
    Serial.println(F("  v<x> <y> <w> vektor bebas dalam persen, mis. v0 80 -40"));
    Serial.println(F("  f<0..3>      profil DATAR/TANGGA/MERUNDUK/SEMPIT"));
    Serial.println(F(" UJI KERING  (servo TIDAK bergerak)"));
    Serial.println(F("  S            simulasi gait + ringkasan 4 arah (maju/putar/gabungan/geser)"));
    Serial.println(F("  F            tabel profil + bentang badan"));
    Serial.println(F("  N<mm>        cari bentang kaki utk celah, mis. N300 (R11)"));
    Serial.println(F(" PIVOT & KALIBRASI  (butuh IMU, robot berjalan)"));
    Serial.println(F("  C<siklus>    KALIBRASI pivot: der/siklus + TANDA arah (mis. C4)"));
    Serial.println(F("  O<der>       pivot relatif tertutup, mis. O90 / O-90"));
    Serial.println(F("  o<n>         pivot ke arah kompas (0=U 1=T 2=S 3=B)"));
    Serial.println(F("  M<siklus>    jalan maju N siklus untuk diukur mistar"));
    Serial.println(F("  m<cm>        masukkan hasil ukur -> mm per siklus + slip"));
    Serial.println(F(" TELAPAK PER KAKI  (ada kaki tidak menapak saat berdiri) — LAKUKAN DULU"));
    Serial.println(F("  J            OTOMATIS: IMU jadi sensor sentuh, berdiri di 3 kaki"));
    Serial.println(F("  K            MANUAL: uji kertas, atur tinggi tiap telapak"));
    Serial.println(F("  j            tabel offset telapak      j0  nolkan"));
    Serial.println(F(" RATA BADAN  (badan miring walau keenam kaki sudah menapak) — SESUDAH 'J'"));
    Serial.println(F("  A[der]       RATAKAN otomatis pakai IMU (uji sumbu +-5 der)"));
    Serial.println(F("  Q            uji BADAN vs LANTAI: ratakan, putar 180, ratakan"));
    Serial.println(F("  a            status trim   a0 nolkan   aj lupakan sumbu IMU"));
    Serial.println(F("  Z            rekam acuan 'rata' (badan harus rata menurut waterpass)"));
    Serial.println(F("  Z0           acuan kembali ke nol"));
    Serial.println(F(" KNOB"));
    Serial.println(F("  h<mm> R<mm> k<mm> e<mm> p<ms> d<%> l<x10> w<hz>"));
    Serial.println(F("  ?  knob   T  telemetri   W/E  simpan/muat EEPROM   H  bantuan"));
    Serial.println(F("==============================================================\n"));
}

// ================================================================ setup
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) { }

    Serial.println(F("\n\nTES_GERAK — gait, body kinematics, pivot"));

    IMU_SERIAL.addMemoryForRead(rxExtra, sizeof(rxExtra));   // harus sebelum begin()
    IMU_SERIAL.begin(imuBaud);

    BUS_DRV0.begin(); BUS_DRV0.setClock(SERVO_I2C_CLOCK);
    BUS_DRV1.begin(); BUS_DRV1.setClock(SERVO_I2C_CLOCK);
    drv0.begin(); drv0.setPWMFreq(SERVO_PWM_FREQ);
    drv1.begin(); drv1.setPWMFreq(SERVO_PWM_FREQ);
    for (uint8_t c = 0; c < 16; c++) { drv0.setPWM(c, 0, 0); drv1.setPWM(c, 0, 0); }
    for (uint8_t i = 0; i < SM_SLOTS; i++) curDeg[i] = 90.0f;

    if (smLoad(map_)) Serial.println(F("pemetaan servo: dari EEPROM."));
    else { smDefaults(map_); Serial.println(F("pemetaan servo: bawaan program (EEPROM kosong).")); }

    mo.begin();
    sim.begin();
    kompasMuat(true);
    gerakMuat(true);

    Serial.println(F("PWM mati. 's' untuk berdiri (topang robot dulu)."));
    printHelp();
}

// ============================================================= perintah
static bool tigaArg(const char* s, int& a, int& b, int& c) {
    const char* p = s + 1;
    a = atoi(p);
    p = strchr(p, ' '); if (!p) return false;
    b = atoi(++p);
    p = strchr(p, ' '); if (!p) return false;
    c = atoi(++p);
    return true;
}

static void handleCmd(char* s) {
    while (*s == ' ') s++;
    char c = *s;
    bool hasNum = (s[1] >= '0' && s[1] <= '9') || s[1] == '-';
    int  v = atoi(s + 1);
    int  a, b, d;

    switch (c) {
        case 0: case 'H': printHelp(); break;
        case '?': showKnobs(); break;
        case 'T': telemetri(); break;
        case 'L': smPrint(map_); break;
        case 'r': releaseAll(); break;
        case 'n': goNetral(); break;
        case 's': goBerdiri(); break;
        case 'x':
            anim = A_NONE;
            mo.setMove(0, 0, 0);
            Serial.println(F("stop — kaki kembali ke pose berdiri."));
            break;

        // ---------------- body kinematics ----------------
        case 'b':
            if (s[1] == 'd') {
                if (!aktif) { Serial.println(F("robot belum berdiri — 's' dulu.")); break; }
                anim = A_DEMO; animT0 = millis();
                Serial.println(F("-> DEMO body: roll, pitch, yaw, geser X, Y, Z (3 detik tiap sumbu)"));
                Serial.println(F("   Perhatikan tiap sumbu; kaki HARUS tetap di tempatnya."));
            } else if (s[1] == 'l') {
                ujiBatas();
            } else if (s[1] == 0) {
                Serial.println(F("format: br/bp/bw/bx/by/bz<nilai>, b0, bd, bl"));
            } else {
                setBody(s[1], (float)atoi(s + 2));
            }
            break;

        // ---------------- gait ----------------
        case 'g': mulaiGait(0,  1, 0, F("MAJU")); break;
        case 'G': mulaiGait(0, -1, 0, F("MUNDUR")); break;
        case '[': mulaiGait(-1, 0, 0, F("geser KIRI")); break;
        case ']': mulaiGait( 1, 0, 0, F("geser KANAN")); break;
        case 'y': mulaiGait(0, 0,  1, F("putar KIRI (CCW)")); break;
        case 'Y': mulaiGait(0, 0, -1, F("putar KANAN (CW)")); break;
        case 'v':
            if (!tigaArg(s, a, b, d)) { Serial.println(F("format: v<x> <y> <putar> dalam persen, mis. v0 80 -40")); break; }
            mulaiGait(constrain(a, -100, 100) / 100.0f,
                      constrain(b, -100, 100) / 100.0f,
                      constrain(d, -100, 100) / 100.0f, F("vektor bebas"));
            break;
        case 'f':
            if (v < 0 || v > 3) { Serial.println(F("f0 DATAR f1 TANGGA f2 MERUNDUK f3 SEMPIT")); break; }
            mo.setProfile((uint8_t)v);
            Serial.print(F("profil -> ")); Serial.print(PROFIL_NAMA[v]);
            Serial.println(F(" (di-ramp, tidak melonjak)"));
            if (!aktif) Serial.println(F("  (robot belum berdiri; profil berlaku saat 's')"));
            break;

        // ---------------- uji kering ----------------
        case 'S': {
            float vx = mo.tgtX(), vy = mo.tgtY(), vw = mo.tgtYaw();
            if (fabsf(vx) + fabsf(vy) + fabsf(vw) < 0.01f) {
                vy = 1.0f;
                Serial.println(F("(robot diam -> disimulasikan MAJU penuh)"));
            }
            simGait(vx, vy, vw, 3);
            break;
        }
        case 'F': tabelProfil(); break;
        case 'N': cariSempit((float)v); break;

        // ---------------- pivot & kalibrasi ----------------
        case 'C': kalibrasiPivot((uint8_t)(hasNum ? v : 4)); break;
        case 'O':
            if (!hasNum) { Serial.println(F("format: O<derajat>, mis. O90 atau O-90")); break; }
            pivotRelatif((float)v);
            break;
        case 'o':
            if (v < 0 || v > 3) { Serial.println(F("o0=UTARA o1=TIMUR o2=SELATAN o3=BARAT")); break; }
            pivotKompas((uint8_t)v);
            break;
        case 'M': kalibrasiMaju((uint8_t)(hasNum ? v : 5)); break;
        case 'm': odoHasil((float)atof(s + 1)); break;

        // ---------------- rata badan ----------------
        case 'A': {
            float probe = hasNum ? constrain((float)v, 2.0f, 10.0f) : LVL_PROBE_DEF;
            if (hasNum) punyaJac = false;     // probe diubah = minta ukur ulang sumbu
            ratakan(probe, true, NULL);
            break;
        }
        case 'a':
            if (s[1] == '0') {
                setTrim(0, 0);
                Serial.println(F("trim rata dinolkan (acuan & sumbu IMU tetap)."));
            } else if (s[1] == 'j') {
                punyaJac = false;
                Serial.println(F("sumbu IMU dilupakan — 'A' akan mengukurnya lagi."));
            } else lvlStatus();
            break;
        case 'Q': ujiLantai(LVL_PROBE_DEF); break;

        // ---------------- telapak per kaki ----------------
        case 'J': kalibKakiOtomatis(); break;
        case 'K': kalibKakiManual(); break;
        case 'j':
            if (s[1] == '0') {
                for (uint8_t i = 0; i < 6; i++) mo.zOff[i] = 0;
                mo.setHome();
                Serial.println(F("offset telapak dinolkan."));
            } else kakiStatus();
            break;
        case 'Z':
            if (s[1] == '0') { refRoll = refPitch = 0;
                               Serial.println(F("acuan IMU kembali ke nol.")); }
            else setAcuan();
            break;

        // ---------------- knob ----------------
        case 'h': if (hasNum) mo.tgtProf.standH  = constrain(v, 50, 140); showKnobs(); break;
        case 'R': if (hasNum) mo.tgtProf.standR  = constrain(v, 30, 110); showKnobs(); break;
        case 'k': if (hasNum) mo.tgtProf.stepL   = constrain(v, 0, 150);  showKnobs(); break;
        case 'e': if (hasNum) mo.tgtProf.stepH   = constrain(v, 0, 120);  showKnobs(); break;
        case 'p': if (hasNum) mo.tgtProf.cycleMs = constrain(v, 300, 2000); showKnobs(); break;
        case 'd': if (hasNum) mo.duty     = constrain(v, 30, 70) / 100.0f; showKnobs(); break;
        case 'l': if (hasNum) mo.slewRate = constrain(v, 5, 100) / 10.0f;  showKnobs(); break;
        case 'w':
            if (hasNum) {
                writeHz = constrain(v, 20, 200);
                Serial.println(F("laju tulis diubah. Jalankan 'S' lagi: laju sendi maks"));
                Serial.println(F("ikut berubah karena jarak antar titik berubah."));
            }
            showKnobs();
            break;

        case 'W': gerakSimpan(); break;
        case 'E': if (gerakMuat(true)) showKnobs(); break;

        default: Serial.println(F("perintah tidak dikenal, ketik 'H'"));
    }
}

// ================================================================= loop
void loop() {
    static char buf[40];
    static uint8_t len = 0;

    witPump();

    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r') {
            buf[len] = 0;
            if (len) handleCmd(buf);
            else {                                   // Enter kosong = berhenti
                anim = A_NONE;
                mo.setMove(0, 0, 0);
                Serial.println(F("berhenti."));
            }
            len = 0;
        } else if (len < sizeof(buf) - 1) buf[len++] = ch;
    }

    if (!aktif) return;
    if (millis() - lastWrite < writeMs()) return;
    lastWrite = millis();
    motionTick();
}
