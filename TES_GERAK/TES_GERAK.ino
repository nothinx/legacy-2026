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

struct GerakStore { uint8_t m0, m1, ver; float ccw, cw, maju; int8_t sign; uint8_t sum; };

static void gerakSimpan() {
    GerakStore s; memset(&s, 0, sizeof(s));
    s.m0 = 0x6E; s.m1 = 0x2C; s.ver = 1;
    s.ccw = degCCW; s.cw = degCW; s.maju = mmMaju; s.sign = pivotSign;
    s.sum = eeSum(&s, offsetof(GerakStore, sum));
    EEPROM.put(EE_GERAK_ADDR, s);
    Serial.println(F("kalibrasi gerak disimpan ke EEPROM 2048."));
}

static bool gerakMuat(bool cerewet) {
    GerakStore s;
    EEPROM.get(EE_GERAK_ADDR, s);
    if (s.m0 != 0x6E || s.m1 != 0x2C || s.ver != 1 ||
        s.sum != eeSum(&s, offsetof(GerakStore, sum))) {
        if (cerewet) Serial.println(F("kalibrasi gerak belum ada di EEPROM."));
        return false;
    }
    degCCW = s.ccw; degCW = s.cw; mmMaju = s.maju; pivotSign = s.sign;
    if (cerewet) Serial.println(F("kalibrasi gerak dimuat dari EEPROM."));
    return true;
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
static void simGait(float vx, float vy, float vyaw, uint8_t siklus) {
    sim = mo;
    sim.prof = mo.tgtProf; sim.tgtProf = mo.tgtProf;   // pakai profil target penuh
    sim.setHome();
    sim.setMove(vx, vy, vyaw);

    float dt = 1.0f / (float)writeHz;

    // ramp dulu sampai vektor gerak mencapai target (slew), baru diukur.
    for (uint16_t i = 0; i < (uint16_t)(3.0f / dt); i++) sim.update(dt);

    float prev[18], now18[18];
    sim.solve(prev);

    float maxRate[3] = { 0, 0, 0 };     // coxa, femur, tibia
    uint8_t rateLeg[3] = { 0, 0, 0 };
    float mn[3] = { 999, 999, 999 }, mx[3] = { -999, -999, -999 };
    float bentangMin = 1e9f, bentangMax = -1e9f, zMax = -1e9f;
    uint32_t luarJangkauan = 0, n = 0;

    uint32_t langkah = (uint32_t)(siklus * sim.prof.cycleMs / 1000.0f / dt);
    for (uint32_t s = 0; s < langkah; s++) {
        sim.update(dt);
        if (!sim.solve(now18)) luarJangkauan++;
        for (uint8_t i = 0; i < 18; i++) {
            uint8_t j = i % 3;
            float r = fabsf(now18[i] - prev[i]) / dt;
            if (r > maxRate[j]) { maxRate[j] = r; rateLeg[j] = i / 3; }
            if (now18[i] < mn[j]) mn[j] = now18[i];
            if (now18[i] > mx[j]) mx[j] = now18[i];
            prev[i] = now18[i];
        }
        float b = sim.bentangX();
        if (b < bentangMin) bentangMin = b;
        if (b > bentangMax) bentangMax = b;
        for (uint8_t l = 0; l < 6; l++)
            if (sim.legTargets[l][2] > zMax) zMax = sim.legTargets[l][2];
        n++;
    }

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
    Serial.print(F(" Hz -> ")); Serial.print(n); Serial.print(F(" titik / "));
    Serial.print(siklus); Serial.println(F(" siklus"));

    const char* JN[3] = { "coxa ", "femur", "tibia" };
    Serial.println(F("\n  sendi   laju maks      sudut min..maks   kaki terparah"));
    bool terlaluCepat = false;
    for (uint8_t j = 0; j < 3; j++) {
        Serial.print(F("  ")); Serial.print(JN[j]);
        Serial.print(F("   ")); Serial.print(maxRate[j], 0); Serial.print(F(" der/s"));
        if (maxRate[j] < 100) Serial.print(' ');
        Serial.print(F("     ")); Serial.print(mn[j], 0);
        Serial.print(F(" .. ")); Serial.print(mx[j], 0);
        Serial.print(F("        K")); Serial.println(rateLeg[j]);
        if (maxRate[j] > SERVO_MAX_DPS * 0.65f) terlaluCepat = true;
    }

    Serial.print(F("\n  bentang badan  : ")); Serial.print(bentangMin / 10.0f, 1);
    Serial.print(F(" .. ")); Serial.print(bentangMax / 10.0f, 1); Serial.println(F(" cm"));
    Serial.print(F("  angkat kaki    : ")); Serial.print(zMax + sim.prof.standH, 0);
    Serial.println(F(" mm dari tanah"));
    Serial.print(F("  laju teoretis  : ")); Serial.print(sim.lajuTeoretis() / 10.0f, 1);
    Serial.println(F(" cm/detik (sebelum slip)"));
    Serial.print(F("  titik di luar jangkauan IK: ")); Serial.print(luarJangkauan);
    Serial.print('/'); Serial.println(n);

    Serial.println(F("\n  VONIS:"));
    if (luarJangkauan)
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
    Serial.println(F("\n  Bentang badan = jarak ujung telapak kiri-kanan saat DIAM."));
    Serial.println(F("  Saat BERJALAN bentangnya bertambah (kaki mengayun ke luar);"));
    Serial.println(F("  angka yang sah untuk R11 adalah bentang maks dari 'S'."));
    Serial.println(F("  R11 celah 30 cm -> pakai 'N300' untuk mencari bentang kaki"));
    Serial.println(F("  yang benar-benar muat."));
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
    Serial.println(F("\n  Tempel ke motion.h (profil 3 SEMPIT) dan ke firmware:"));
    Serial.print(F("    { 30.0f, 45.0f, 1000.0f, 100.0f, "));
    Serial.print(terbaik, 1); Serial.println(F("f }   // SEMPIT"));
    Serial.println(F("  CATATAN: GaitProfile firmware BELUM punya field standR"));
    Serial.println(F("  (STAND_RADIUS masih #define). Tanpa field itu profil SEMPIT"));
    Serial.println(F("  tidak mungkin dijalankan — lihat README."));
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
    float teoretis = mo.prof.stepL * siklus;
    Serial.print(F("  selesai. Jarak TEORETIS ")); Serial.print(teoretis / 10.0f, 1);
    Serial.println(F(" cm."));
    Serial.println(F("  Ukur jarak sebenarnya dengan mistar, lalu ketik: m<cm>  (mis. m24)"));
}

static void odoHasil(float cm) {
    if (!odoCycles) { Serial.println(F("jalankan 'M<siklus>' dulu.")); return; }
    mmMaju = cm * 10.0f / odoCycles;
    float teoretis = mo.prof.stepL;
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
    Serial.println(F("  S            simulasi gait: laju sendi vs batas servo, jangkauan IK"));
    Serial.println(F("  F            tabel profil + bentang badan"));
    Serial.println(F("  N<mm>        cari bentang kaki utk celah, mis. N300 (R11)"));
    Serial.println(F(" PIVOT & KALIBRASI  (butuh IMU, robot berjalan)"));
    Serial.println(F("  C<siklus>    KALIBRASI pivot: der/siklus + TANDA arah (mis. C4)"));
    Serial.println(F("  O<der>       pivot relatif tertutup, mis. O90 / O-90"));
    Serial.println(F("  o<n>         pivot ke arah kompas (0=U 1=T 2=S 3=B)"));
    Serial.println(F("  M<siklus>    jalan maju N siklus untuk diukur mistar"));
    Serial.println(F("  m<cm>        masukkan hasil ukur -> mm per siklus + slip"));
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
