/* =====================================================================
   TES_IMU — kelayakan IMU Yahboom 10-axis (protokol WIT) untuk heading
   Target : Teensy 4.1, IMU di Serial2 (RX2 pin 7 / TX2 pin 8) @ 921600,
            frame 0x55 11 byte
   Library: Adafruit PWM Servo Driver (untuk uji gangguan servo)

   PERTANYAAN YANG DIJAWAB ALAT INI
   Bukan "IMU-nya hidup atau tidak" — itu gampang. Yang menentukan nasib
   navigasi adalah: **yaw melenceng berapa derajat saat 18 servo bekerja?**

   Bedakan dua jenis gangguan, nasibnya jauh berbeda:
     - STATIS  (besi rangka, magnet servo diam) -> konstan terhadap badan
       robot, BISA dikoreksi dengan mencatat HEAD_UTARA/TIMUR/SELATAN/BARAT.
     - DINAMIS (arus 18 servo, magnet bergerak) -> berubah-ubah, TIDAK BISA
       dikoreksi. Kalau yang ini besar, kompas tak bisa dipakai heading-hold
       sambil berjalan, dan rencana harus pindah ke sudut dinding dari
       sepasang lidar samping (LEFT_FRONT - LEFT_REAR) yang kebal magnet.

   Gate IMU_MAX_YAW_JUMP 30 derajat di firmware TIDAK menolong di sini: ia
   menolak LONJAKAN, sedangkan kegagalan sebenarnya berupa PERGESERAN
   BERTAHAP — melenceng 40 derajat dalam 2 detik lolos mulus.

   KESELAMATAN: perintah 'g' dan 's2' MENGGERAKKAN SERVO.
   Taruh robot di atas dudukan/kardus sehingga kaki menggantung bebas.

   Serial Monitor 115200, line ending Newline. Ketik 'h' untuk bantuan.
   ===================================================================== */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include <stddef.h>      // offsetof — checksum EEPROM kompas
#include <Adafruit_PWMServoDriver.h>
#include "servo_map.h"

// ---------------------------------------------------------------- knob
// IMU di Serial2 = RX2 pin 7 / TX2 pin 8.
// CATATAN: config.h firmware masih menulis IMU_SERIAL Serial1 — harus diubah.
// Serial2 tadinya dicadangkan untuk Raspberry Pi; Pi harus pindah, dan TIDAK
// boleh ke Serial4 (pin 16/17 = Wire1) atau Serial6 (pin 25/24 = Wire2).
#define IMU_SERIAL      Serial2
#define WIT_LEN         11       // panjang frame WIT
#define RX_EXTRA        2048     // tambahan buffer RX Serial2 (lihat setup)
#define YAW_JUMP_DEG    30.0f    // sama dengan IMU_MAX_YAW_JUMP di config.h

// pose berdiri dari IK (config.h: STAND_RADIUS 70, STAND_HEIGHT 100)
#define HOME_COXA_DEG   90.00f
#define HOME_FEMUR_DEG  79.43f
#define HOME_TIBIA_DEG  82.02f

#define WIGGLE_HZ       0.7f     // frekuensi goyang saat uji gangguan
#define SERVO_RATE_MS   20       // 50 Hz penulisan servo

// 230400 = maksimum yang disediakan aplikasi WIT untuk modul ini.
// Bawaan pabrik 9600 — kalau modul belum disetel, jalankan 'B' untuk memindai.
uint32_t imuBaud   = 230400;
float    wiggleAmp = 10.0f;      // derajat, amplitudo goyang
uint16_t faseDetik = 8;          // lama tiap fase uji 'g'

// ------------------------------------------------------------- servo
Adafruit_PWMServoDriver drv0(ADDR_DRV0, BUS_DRV0);
Adafruit_PWMServoDriver drv1(ADDR_DRV1, BUS_DRV1);
ServoMap map_;

enum SvMode { SV_OFF, SV_HOLD, SV_WIGGLE };
SvMode   svMode      = SV_OFF;
uint32_t svLastWrite = 0;

// --------------------------------------------------------- data IMU
float ax = 0, ay = 0, az = 0;        // g
float gx = 0, gy = 0, gz = 0;        // derajat/detik
float roll = 0, pitch = 0, yaw = 0;  // derajat, yaw 0..360
float roll0 = 0, pitch0 = 0;         // tare
float mhx = 0, mhy = 0, mhz = 0;     // magnet mentah
float yawRef = -1;                   // referensi 'z', -1 = belum diset
bool  punyaSudut = false;

uint32_t angFrames = 0;              // pencacah frame 0x53 (sudut)

// ------------------------------------------------------ statistik frame
uint32_t stBytes = 0, stOk = 0, stSumBad = 0, stBuang = 0, stJumpYaw = 0;
uint32_t stTipe[8];                  // 0x50..0x57
uint32_t stMulai = 0;

uint8_t  rxBuf[64];
uint8_t  rxN = 0;
uint8_t  rxExtra[RX_EXTRA];

// ------------------------------------------------------- akumulator
// Fixed array, bukan struct: tipe buatan sendiri tidak boleh muncul di tanda
// tangan fungsi dalam file .ino (Arduino menyisipkan prototipe di atas badan
// sketsa, sebelum tipe itu dideklarasikan).
#define NACC 4                       // 0..2 = fase uji 'g', 3 = uji drift
bool     accHas[NACC];
float    accRef[NACC], accMn[NACC], accMx[NACC];
double   accSum[NACC], magSum[NACC];
uint32_t accN[NACC];

enum Mode { IDLE, TAMPIL };
Mode     mode      = IDLE;
uint32_t lastPrint = 0;

// ============================================================== util
static float wrap180(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static float magMagnitude() { return sqrtf(mhx * mhx + mhy * mhy + mhz * mhz); }

// ------------------------------------------------------- kompas arena
// Empat arah arena. BUKAN utara magnet — yang dicatat adalah yaw saat robot
// menghadap arah arena, jadi orientasi arena terhadap medan bumi tidak perlu
// diketahui sama sekali. Nilainya masuk ke HEAD_UTARA/TIMUR/SELATAN/BARAT
// di Calib.h. Dideklarasikan di sini karena printBaris() memakainya.
#define SEKTOR_MASUK    40.0f    // derajat; histeresis supaya sektor tidak berkedip
#define SEKTOR_KELUAR   50.0f
#define EE_KOMPAS_ADDR  1792     // peta EEPROM: 0 Calib, 1024 ServoMap, 1536 lidar

static const char* const ARAH_NAMA[4] = { "UTARA", "TIMUR", "SELATAN", "BARAT" };
float  headArah[4] = { -1, -1, -1, -1 };   // -1 = belum dicatat
int8_t sektorKini  = -1;

// Sektor dengan HISTERESIS. Tanpa ini, robot yang diam di dekat batas 45 derajat
// akan berkedip U<->T terus-menerus dan apa pun yang bercabang di atasnya ikut
// bergetar. Masuk sektor pada 40 derajat, baru lepas pada 50 derajat.
static int8_t hitungSektor() {
    int8_t best = -1; float bestErr = 999;
    for (uint8_t i = 0; i < 4; i++) {
        if (headArah[i] < 0) continue;
        float e = fabsf(wrap180(yaw - headArah[i]));
        if (e < bestErr) { bestErr = e; best = i; }
    }
    if (best < 0) return -1;
    if (sektorKini < 0) { if (bestErr <= SEKTOR_MASUK) sektorKini = best; return sektorKini; }
    if (best == sektorKini) return sektorKini;
    float errKini = fabsf(wrap180(yaw - headArah[sektorKini]));
    if (errKini > SEKTOR_KELUAR && bestErr <= SEKTOR_MASUK) sektorKini = best;
    return sektorKini;
}

// ---- akumulator sudut, aman terhadap lompatan 359->1 ----
// Semua dijumlahkan sebagai SELISIH terhadap sampel pertama, bukan sebagai
// nilai mutlak; kalau tidak, rata-rata 359 dan 1 akan keluar 180.
static void accReset(uint8_t i) {
    accHas[i] = false; accSum[i] = 0; accN[i] = 0;
    accMn[i] = 0; accMx[i] = 0; magSum[i] = 0;
}

static void accAdd(uint8_t i, float y, float m) {
    if (!accHas[i]) { accHas[i] = true; accRef[i] = y; }
    float d = wrap180(y - accRef[i]);
    accSum[i] += d;
    if (accN[i] == 0 || d < accMn[i]) accMn[i] = d;
    if (accN[i] == 0 || d > accMx[i]) accMx[i] = d;
    magSum[i] += m;
    accN[i]++;
}

static float accMean(uint8_t i) {
    if (!accN[i]) return 0;
    float m = accRef[i] + (float)(accSum[i] / accN[i]);
    if (m < 0)      m += 360.0f;
    if (m >= 360.0f) m -= 360.0f;
    return m;
}
static float accSpread(uint8_t i) { return accN[i] ? (accMx[i] - accMn[i]) : 0; }
static float accMagMean(uint8_t i) { return accN[i] ? (float)(magSum[i] / accN[i]) : 0; }

// ======================================================== parser WIT
static void witFrame(const uint8_t* f) {
    uint8_t t = f[1];
    if (t >= 0x50 && t <= 0x57) stTipe[t - 0x50]++;

    int16_t v0 = (int16_t)(f[3] << 8 | f[2]);
    int16_t v1 = (int16_t)(f[5] << 8 | f[4]);
    int16_t v2 = (int16_t)(f[7] << 8 | f[6]);

    switch (t) {
        case 0x51:                                   // percepatan, +-16 g
            ax = v0 / 32768.0f * 16.0f;
            ay = v1 / 32768.0f * 16.0f;
            az = v2 / 32768.0f * 16.0f;
            break;
        case 0x52:                                   // giro, +-2000 der/s
            gx = v0 / 32768.0f * 2000.0f;
            gy = v1 / 32768.0f * 2000.0f;
            gz = v2 / 32768.0f * 2000.0f;
            break;
        case 0x53: {                                 // sudut, +-180
            roll  = v0 / 32768.0f * 180.0f;
            pitch = v1 / 32768.0f * 180.0f;
            float y = v2 / 32768.0f * 180.0f;
            if (y < 0) y += 360.0f;
            if (punyaSudut && fabsf(wrap180(y - yaw)) > YAW_JUMP_DEG) stJumpYaw++;
            yaw = y;
            punyaSudut = true;
            angFrames++;
            break;
        }
        case 0x54:                                   // magnet, mentah
            mhx = v0; mhy = v1; mhz = v2;
            break;
        default: break;
    }
}

// Parser dengan resinkronisasi sejati: kalau checksum gagal, buang SATU byte
// lalu coba lagi — bukan buang seluruh 11 byte. Kalau sebuah byte hilang di
// kabel, cara "buang 11" akan terus salah bingkai dan seluruh frame berikutnya
// ikut hancur; itu terbaca sebagai "IMU rusak" padahal cuma salah sinkron.
static void witPump() {
    while (IMU_SERIAL.available()) {
        if (rxN >= sizeof(rxBuf)) {               // penuh: paksa geser
            memmove(rxBuf, rxBuf + 1, --rxN);
            stBuang++;
        }
        rxBuf[rxN++] = (uint8_t)IMU_SERIAL.read();
        stBytes++;

        while (rxN >= WIT_LEN) {
            if (rxBuf[0] != 0x55) {
                memmove(rxBuf, rxBuf + 1, --rxN);
                stBuang++;
                continue;
            }
            uint8_t sum = 0;
            for (uint8_t i = 0; i < WIT_LEN - 1; i++) sum += rxBuf[i];
            if (sum != rxBuf[WIT_LEN - 1]) {
                memmove(rxBuf, rxBuf + 1, --rxN);
                stSumBad++;
                continue;
            }
            witFrame(rxBuf);
            stOk++;
            rxN -= WIT_LEN;
            memmove(rxBuf, rxBuf + WIT_LEN, rxN);
        }
    }
}

static void statReset() {
    stBytes = stOk = stSumBad = stBuang = stJumpYaw = 0;
    memset(stTipe, 0, sizeof(stTipe));
    stMulai = millis();
}

// ============================================================== servo
static void servoWrite(uint8_t slot, float deg) {
    if (map_.drv[slot] < 0) return;
    Adafruit_PWMServoDriver& d = (map_.drv[slot] == 0) ? drv0 : drv1;
    d.writeMicroseconds(map_.ch[slot], smDegToUs(map_, slot, deg));
}

static void servoAllOff() {
    for (uint8_t c = 0; c < 16; c++) { drv0.setPWM(c, 0, 0); drv1.setPWM(c, 0, 0); }
}

static float homeDeg(uint8_t slot) {
    uint8_t j = slot % 3;                     // 0 coxa, 1 femur, 2 tibia
    return (j == 0) ? HOME_COXA_DEG : (j == 1) ? HOME_FEMUR_DEG : HOME_TIBIA_DEG;
}

static void servoSetMode(uint8_t m) {
    svMode = (SvMode)m;
    if (svMode == SV_OFF) { servoAllOff(); Serial.println(F("servo: MATI (tanpa pulsa)")); }
    else if (svMode == SV_HOLD) {
        for (uint8_t s = 0; s < 18; s++) servoWrite(s, homeDeg(s));
        Serial.println(F("servo: DIAM bertenaga di pose berdiri"));
    } else {
        Serial.print(F("servo: GOYANG +-")); Serial.print(wiggleAmp, 0);
        Serial.println(F(" derajat — pastikan kaki menggantung!"));
    }
}

static void servoUpdate() {
    if (svMode != SV_WIGGLE) return;
    if (millis() - svLastWrite < SERVO_RATE_MS) return;
    svLastWrite = millis();
    float t = millis() / 1000.0f;
    for (uint8_t s = 0; s < 18; s++) {
        // beda fase per kaki: arus servo tersebar merata sepanjang siklus,
        // bukan menghentak berbarengan
        float ph = (s / 3) * (float)(2.0 * PI / 6.0);
        servoWrite(s, homeDeg(s) + wiggleAmp * sinf(2.0f * (float)PI * WIGGLE_HZ * t + ph));
    }
}

// ========================================================= pengumpulan
// Kumpulkan sampel selama ms milidetik ke akumulator acc.
// Satu sampel per frame 0x53 BARU, bukan per putaran loop — kalau tidak, nilai
// yang sama tercatat berkali-kali dan sebarannya terlihat lebih kecil dari
// yang sebenarnya. Ketik apa saja untuk membatalkan.
static bool kumpul(uint8_t acc, uint32_t ms) {
    accReset(acc);
    uint32_t t0 = millis();
    uint32_t lastFrame = angFrames;
    while (millis() - t0 < ms) {
        witPump();
        servoUpdate();
        if (angFrames != lastFrame) {
            lastFrame = angFrames;
            accAdd(acc, yaw, magMagnitude());
        }
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            Serial.println(F("  ** dibatalkan **"));
            return false;
        }
    }
    return true;
}

static void barisFase(const char* nama, uint8_t i, uint8_t basis) {
    Serial.print(F("  ")); Serial.print(nama);
    for (uint8_t k = strlen(nama); k < 18; k++) Serial.print(' ');
    if (!accN[i]) { Serial.println(F("tidak ada data")); return; }
    Serial.print(accMean(i), 1);
    Serial.print(F("   ")); Serial.print(accSpread(i), 1);
    Serial.print(F("   ")); Serial.print(accMagMean(i), 0);
    if (i != basis && accN[basis]) {
        float geser = wrap180(accMean(i) - accMean(basis));
        Serial.print(F("   geser "));
        if (geser >= 0) Serial.print('+');
        Serial.print(geser, 1);
    }
    Serial.println();
}

// ============================================ uji gangguan servo ('g')
static void ujiGangguan() {
    if (!punyaSudut) { Serial.println(F("Belum ada data sudut dari IMU. Cek 'f'.")); return; }

    Serial.println(F("\n========== UJI GANGGUAN MAGNET DARI SERVO =========="));
    Serial.println(F("TARUH ROBOT DI ATAS DUDUKAN — kaki harus menggantung bebas."));
    Serial.println(F("Robot JANGAN digeser/diputar selama uji. Ketik apa saja = batal."));
    Serial.print  (F("Tiap fase ")); Serial.print(faseDetik); Serial.println(F(" detik.\n"));

    uint32_t ms = (uint32_t)faseDetik * 1000UL;

    Serial.println(F("  fase 1/3: servo MATI ..."));
    servoSetMode(SV_OFF);   delay(1500);
    if (!kumpul(0, ms)) { servoSetMode(SV_OFF); return; }

    Serial.println(F("  fase 2/3: servo BERTENAGA tapi DIAM ..."));
    servoSetMode(SV_HOLD);  delay(1500);
    if (!kumpul(1, ms)) { servoSetMode(SV_OFF); return; }

    Serial.println(F("  fase 3/3: servo BERGERAK ..."));
    servoSetMode(SV_WIGGLE); delay(1500);
    if (!kumpul(2, ms)) { servoSetMode(SV_OFF); return; }

    servoSetMode(SV_OFF);

    Serial.println(F("\n  fase                yaw     sebar   |mag|"));
    barisFase("1 servo MATI",   0, 0);
    barisFase("2 servo DIAM",   1, 0);
    barisFase("3 servo GERAK",  2, 0);

    float statis  = accN[1] && accN[0] ? fabsf(wrap180(accMean(1) - accMean(0))) : 0;
    float dinamis = accN[2] && accN[0] ? fabsf(wrap180(accMean(2) - accMean(0))) : 0;
    float sebar   = accSpread(2);
    float magNaik = accMagMean(0) > 1 ? (accMagMean(2) - accMagMean(0)) / accMagMean(0) * 100.0f : 0;

    Serial.println(F("\n  ---------------- VONIS ----------------"));
    Serial.print(F("  gangguan STATIS  (servo bertenaga): ")); Serial.print(statis, 1);
    Serial.println(F(" derajat"));
    Serial.println(F("    -> konstan terhadap badan robot, BISA dikoreksi lewat HEAD_*."));
    Serial.print(F("  gangguan DINAMIS (servo bergerak) : ")); Serial.print(dinamis, 1);
    Serial.print(F(" derajat, sebar ")); Serial.print(sebar, 1); Serial.println(F(" derajat"));
    Serial.print(F("  perubahan kuat medan |mag|        : ")); Serial.print(magNaik, 0);
    Serial.println(F(" %"));
    Serial.println(F("    -> |mag| ikut naik = buktinya memang magnet, bukan hanyutan giro."));

    float buruk = (dinamis > sebar) ? dinamis : sebar;
    Serial.print(F("\n  KESIMPULAN: "));
    if (buruk < 5.0f) {
        Serial.println(F("YAW LAYAK jadi acuan heading utama."));
        Serial.println(F("  Lanjut: catat HEAD_UTARA/TIMUR/SELATAN/BARAT, pakai PD holdHeading."));
    } else if (buruk < 15.0f) {
        Serial.println(F("YAW MERAGUKAN — bisa dipakai, tapi jangan sendirian."));
        Serial.println(F("  Silangkan dengan sudut dinding dari lidar samping"));
        Serial.println(F("  (LEFT_FRONT - LEFT_REAR), dan longgarkan HEADING_TOL_DEG."));
    } else {
        Serial.println(F("YAW TIDAK LAYAK untuk heading sambil berjalan."));
        Serial.println(F("  Servo mengacaukan kompas lebih besar dari toleransi navigasi."));
        Serial.println(F("  Pilihan: (a) jauhkan/perisai IMU dari servo & kabel dayanya,"));
        Serial.println(F("           (b) pindah ke sudut dinding lidar sebagai acuan utama,"));
        Serial.println(F("           (c) pakai yaw hanya saat DIAM (servo tidak bergerak)."));
    }
    Serial.println(F("  Uji ulang setelah memindahkan IMU untuk membandingkan angkanya."));
}

// ================================================ uji drift saat diam
static void ujiDrift(uint16_t detik) {
    if (!punyaSudut) { Serial.println(F("Belum ada data sudut dari IMU.")); return; }
    if (detik < 5 || detik > 600) { Serial.println(F("lama 5..600 detik, mis. d60")); return; }

    Serial.print(F("\n--- Uji hanyutan yaw saat DIAM, ")); Serial.print(detik);
    Serial.println(F(" detik ---"));
    Serial.println(F("Robot benar-benar diam, servo dimatikan. Ketik apa saja = batal."));
    servoSetMode(SV_OFF);
    delay(1000);

    float awal = yaw;
    uint32_t t0 = millis();
    uint32_t lastFrame = angFrames, lapor = millis();
    accReset(3);
    while (millis() - t0 < (uint32_t)detik * 1000UL) {
        witPump();
        if (angFrames != lastFrame) { lastFrame = angFrames; accAdd(3, yaw, magMagnitude()); }
        if (millis() - lapor >= 10000) {           // laporan antara tiap 10 detik
            lapor = millis();
            Serial.print(F("  t=")); Serial.print((millis() - t0) / 1000);
            Serial.print(F("s  hanyut ")); Serial.print(wrap180(yaw - awal), 1);
            Serial.println(F(" derajat"));
        }
        if (Serial.available()) { while (Serial.available()) Serial.read();
                                  Serial.println(F("  ** dibatalkan **")); return; }
    }
    float total = wrap180(yaw - awal);
    Serial.print(F("  hanyut total : ")); Serial.print(total, 1); Serial.println(F(" derajat"));
    Serial.print(F("  laju         : ")); Serial.print(total / (detik / 60.0f), 2);
    Serial.println(F(" derajat/menit"));
    Serial.print(F("  sebar        : ")); Serial.print(accSpread(3), 1); Serial.println(F(" derajat"));
    Serial.print(F("  VONIS: "));
    if (fabsf(total) < 2.0f)      Serial.println(F("stabil. Kompas terkunci ke magnet, bukan hanyut giro."));
    else if (fabsf(total) < 10.0f) Serial.println(F("hanyut sedang — masih bisa dipakai untuk misi pendek."));
    else                           Serial.println(F("HANYUT BESAR — fusi magnetnya tidak bekerja; cek kalibrasi IMU."));
}

// ============================================================ tampilan
static void printBaris() {
    float det = (millis() - stMulai) / 1000.0f;
    if (det < 0.001f) det = 0.001f;

    Serial.print(F("yaw ")); Serial.print(yaw, 1);
    if (yawRef >= 0) { Serial.print(F(" (rel ")); Serial.print(wrap180(yaw - yawRef), 1); Serial.print(')'); }
    int8_t sek = hitungSektor();
    if (sek >= 0) {
        Serial.print(F(" [")); Serial.print(ARAH_NAMA[sek]);
        Serial.print(' '); Serial.print(wrap180(yaw - headArah[sek]), 0); Serial.print(']');
    }
    Serial.print(F(" | rp ")); Serial.print(roll - roll0, 1);
    Serial.print('/'); Serial.print(pitch - pitch0, 1);
    Serial.print(F(" | gyro ")); Serial.print(gz, 1);
    Serial.print(F(" | acc ")); Serial.print(az, 2);
    Serial.print(F("g | |mag| ")); Serial.print(magMagnitude(), 0);
    Serial.print(F(" | ")); Serial.print(stOk / det, 0); Serial.print(F(" frame/s"));
    if (stOk + stSumBad > 0) {
        Serial.print(F(" gagal ")); Serial.print(100.0f * stSumBad / (stOk + stSumBad), 1);
        Serial.print('%');
    }
    Serial.println();
}

static void printStat() {
    float det = (millis() - stMulai) / 1000.0f;
    if (det < 0.001f) det = 0.001f;
    Serial.println(F("\n--- Statistik frame WIT ---"));
    Serial.print(F("  baud        : ")); Serial.println(imuBaud);
    Serial.print(F("  lama        : ")); Serial.print(det, 1); Serial.println(F(" detik"));
    Serial.print(F("  byte masuk  : ")); Serial.print(stBytes);
    Serial.print(F("  (")); Serial.print(stBytes / det, 0); Serial.println(F(" byte/s)"));
    Serial.print(F("  frame sah   : ")); Serial.print(stOk);
    Serial.print(F("  (")); Serial.print(stOk / det, 0); Serial.println(F(" frame/s)"));
    Serial.print(F("  checksum gagal : ")); Serial.print(stSumBad);
    if (stOk + stSumBad > 0) {
        Serial.print(F("  (")); Serial.print(100.0f * stSumBad / (stOk + stSumBad), 2);
        Serial.print(F(" %)"));
    }
    Serial.println();
    Serial.print(F("  byte dibuang saat resinkron : ")); Serial.println(stBuang);
    Serial.print(F("  lonjakan yaw > ")); Serial.print(YAW_JUMP_DEG, 0);
    Serial.print(F(" der : ")); Serial.println(stJumpYaw);

    Serial.print(F("  per tipe    :"));
    const char* NAMA[8] = { "waktu", "accel", "gyro", "sudut", "magnet", "port", "tekanan", "-" };
    for (uint8_t i = 0; i < 8; i++) {
        if (!stTipe[i]) continue;
        Serial.print(F("  0x5")); Serial.print(i, HEX);
        Serial.print('('); Serial.print(NAMA[i]); Serial.print(F(")="));
        Serial.print(stTipe[i] / det, 0); Serial.print(F("/s"));
    }
    Serial.println();

    // Laju paket SUDUT = laju kontrol heading. Ini angka yang penting untuk
    // navigasi, bukan byte/detik.
    float lajuSudut = stTipe[3] / det;
    Serial.print(F("  laju sudut (0x53) : ")); Serial.print(lajuSudut, 0);
    Serial.println(F(" Hz   <- laju kontrol heading"));

    // Berapa yang MUAT pada baud ini: tiap frame 11 byte, 10 bit per byte.
    // Kalau return rate IMU diset lebih tinggi dari ini, frame terpotong dan
    // muncul sebagai checksum gagal — bukan sebagai "lambat".
    uint8_t aktif = 0;
    for (uint8_t i = 0; i < 8; i++) if (stTipe[i]) aktif++;
    if (aktif) {
        float maks = imuBaud / 10.0f / (11.0f * aktif);
        Serial.print(F("  kapasitas baud    : ")); Serial.print(aktif);
        Serial.print(F(" jenis paket aktif -> maks ")); Serial.print(maks, 0);
        Serial.println(F(" Hz"));
        if (lajuSudut > maks * 0.9f) {
            Serial.println(F("    ! sudah mentok kapasitas baud. Naikkan baud, atau matikan"));
            Serial.println(F("      paket yang tak dipakai (accel/gyro) di aplikasi WIT."));
        }
    }

    Serial.print(F("  VONIS: "));
    if (stOk == 0)
        Serial.println(F("TIDAK ADA FRAME. Cek TX IMU -> RX2 (pin 7), GND tersambung, baud."));
    else if (stOk + stSumBad > 0 && 100.0f * stSumBad / (stOk + stSumBad) > 1.0f)
        Serial.println(F("BANYAK FRAME KORUP — kabel terlalu panjang untuk baud ini, "
                         "atau GND buruk. Turunkan baud di IMU dan di 'b'."));
    else if (stTipe[3] == 0)
        Serial.println(F("frame sudut (0x53) tidak dikirim — nyalakan output Angle di aplikasi WIT."));
    else
        Serial.println(F("jalur bersih."));
}

// ========================================================== KOMPAS
static uint8_t kompasSum(const void* buf, size_t n) {
    const uint8_t* p = (const uint8_t*)buf;
    uint8_t acc = 0;
    for (size_t i = 0; i < n; i++) acc = (uint8_t)(acc + p[i] * 31 + 7);
    return acc;
}

struct KompasStore { uint8_t m0, m1, ver; float head[4]; uint8_t sum; };

static void kompasSimpan() {
    KompasStore s;
    memset(&s, 0, sizeof(s));
    s.m0 = 0xC0; s.m1 = 0x3A; s.ver = 1;
    for (uint8_t i = 0; i < 4; i++) s.head[i] = headArah[i];
    s.sum = kompasSum(&s, offsetof(KompasStore, sum));
    EEPROM.put(EE_KOMPAS_ADDR, s);
    Serial.println(F("4 arah disimpan ke EEPROM."));
}

static bool kompasMuat(bool cerewet) {
    KompasStore s;
    EEPROM.get(EE_KOMPAS_ADDR, s);
    if (s.m0 != 0xC0 || s.m1 != 0x3A || s.ver != 1 ||
        s.sum != kompasSum(&s, offsetof(KompasStore, sum))) {
        if (cerewet) Serial.println(F("EEPROM kompas kosong — 4 arah belum dicatat."));
        return false;
    }
    for (uint8_t i = 0; i < 4; i++) headArah[i] = s.head[i];
    if (cerewet) Serial.println(F("4 arah dimuat dari EEPROM."));
    return true;
}

static void kompasCatat(uint8_t i) {
    if (!punyaSudut) { Serial.println(F("Belum ada data sudut dari IMU.")); return; }
    headArah[i] = yaw;
    Serial.print(F("  ")); Serial.print(ARAH_NAMA[i]);
    Serial.print(F(" = ")); Serial.print(yaw, 1); Serial.println(F(" derajat"));
    Serial.println(F("  (paling presisi: sikukan robot ke dinding pakai lidar samping"));
    Serial.println(F("   sampai LEFT_FRONT ~ LEFT_REAR, baru catat)"));
}

static void kompasTabel() {
    Serial.println(F("\n--- KOMPAS ARENA ---"));
    int8_t sek = hitungSektor();
    for (uint8_t i = 0; i < 4; i++) {
        Serial.print(F("  ")); Serial.print(i); Serial.print(F("  "));
        Serial.print(ARAH_NAMA[i]);
        for (uint8_t k = strlen(ARAH_NAMA[i]); k < 9; k++) Serial.print(' ');
        if (headArah[i] < 0) { Serial.println(F("belum dicatat")); continue; }
        Serial.print(headArah[i], 1); Serial.print(F(" der"));
        Serial.print(F("   error sekarang ")); Serial.print(wrap180(yaw - headArah[i]), 1);
        if (sek == (int8_t)i) Serial.print(F("   <== sektor sekarang"));
        Serial.println();
    }

    // Cek kelinieran: kalau kompas sehat, keempatnya berjarak ~90 derajat.
    // Menyimpang jauh = distorsi soft-iron, artinya sektor di antara keempat
    // titik itu tidak bisa dipercaya walau keempat titiknya sendiri benar.
    uint8_t lengkap = 0;
    for (uint8_t i = 0; i < 4; i++) if (headArah[i] >= 0) lengkap++;
    if (lengkap == 4) {
        Serial.println(F("  jarak antar arah (idealnya 90):"));
        float terburuk = 0;
        for (uint8_t i = 0; i < 4; i++) {
            float d = wrap180(headArah[(i + 1) % 4] - headArah[i]);
            if (d < 0) d += 360.0f;
            Serial.print(F("    ")); Serial.print(ARAH_NAMA[i]);
            Serial.print(F(" -> ")); Serial.print(ARAH_NAMA[(i + 1) % 4]);
            Serial.print(F(" = ")); Serial.print(d, 1); Serial.println(F(" der"));
            if (fabsf(d - 90.0f) > terburuk) terburuk = fabsf(d - 90.0f);
        }
        Serial.print(F("  simpangan terburuk dari 90: ")); Serial.print(terburuk, 1);
        Serial.println(F(" der"));
        if (terburuk <= 3.0f)
            Serial.println(F("  -> kompas LINIER. Cukup catat UTARA, sisanya bisa aritmetika."));
        else if (terburuk <= 10.0f)
            Serial.println(F("  -> sedikit melenceng. Pakai keempat nilai apa adanya."));
        else
            Serial.println(F("  -> DISTORSI SOFT-IRON besar. Keempat titik masih bisa dipakai,"
                             " tapi sudut DI ANTARA-nya tidak linier — jangan interpolasi."));
    } else {
        Serial.print(F("  baru ")); Serial.print(lengkap);
        Serial.println(F("/4 arah dicatat; cek kelinieran menunggu keempatnya."));
    }
}

// ============================================== PIVOT — KERANGKA
// Memutar robot di tempat sampai menghadap arah tersimpan.
//
// PENGGERAK GAIT-NYA BELUM ADA. gaitPutar() di bawah masih kosong, jadi robot
// TIDAK akan bergerak sendiri. Kerangkanya sengaja dibuat lebih dulu supaya
// lingkar kendalinya (yaw -> error -> perintah putar) bisa diuji dan disetel
// terpisah dari gait, dan supaya jelas apa persisnya yang perlu disambungkan:
// satu panggilan, satu argumen -1..+1.
//
// Sementara kosong, perintah 'o<n>' tetap berguna: putar robot DENGAN TANGAN
// ke arah sasaran dan lihat perintah putar bergerak menuju nol. Itu memverifikasi
// tanda (arah putar) dan perilaku PD sebelum gait disambungkan — kalau tandanya
// terbalik, robot nanti akan berputar menjauhi sasaran.
#define PIVOT_KP        0.020f   // perintah putar per derajat error
#define PIVOT_KD        0.004f   // per derajat/detik (dari gyro Z, bukan turunan yaw)
#define PIVOT_TOL_DEG   6.0f     // HEADING_TOL_DEG 3.0 di config.h terlalu ketat
#define PIVOT_DIAM_MS   500      // harus di dalam toleransi selama ini
#define PIVOT_BATAS_MS  20000

static void gaitPutar(float turn) {
    // TODO: sambungkan ke gait tripod.
    //   firmware  : HexaGait::setMoveVector(0, 0, turn)
    //   sementara : salin gait dari KALIBRASI/ ke sini
    // turn: -1 = putar penuh satu arah, +1 = arah sebaliknya, 0 = diam.
    (void)turn;
}

static void pivotKe(uint8_t arah) {
    if (headArah[arah] < 0) {
        Serial.print(ARAH_NAMA[arah]); Serial.println(F(" belum dicatat (pakai 'c<n>')."));
        return;
    }
    if (!punyaSudut) { Serial.println(F("Belum ada data sudut dari IMU.")); return; }

    Serial.print(F("\n--- Pivot ke ")); Serial.print(ARAH_NAMA[arah]);
    Serial.print(F(" (")); Serial.print(headArah[arah], 1); Serial.println(F(" der) ---"));
    Serial.println(F("  gaitPutar() masih kosong -> robot tidak bergerak sendiri."));
    Serial.println(F("  Putar robot DENGAN TANGAN; perintah putar harus menuju nol."));
    Serial.println(F("  Ketik apa saja = berhenti.\n"));

    uint32_t t0 = millis(), lapor = 0, masukSejak = 0;
    float    errAwal = wrap180(headArah[arah] - yaw);
    bool     selesai = false;

    while (millis() - t0 < PIVOT_BATAS_MS) {
        witPump();

        float err = wrap180(headArah[arah] - yaw);
        // Suku D dari gyro Z langsung, bukan dari turunan yaw: yaw berisik dan
        // mendiferensiasikannya memperbesar noise itu. gyro juga kebal magnet.
        float turn = PIVOT_KP * err - PIVOT_KD * gz;
        if (turn >  1.0f) turn =  1.0f;
        if (turn < -1.0f) turn = -1.0f;

        gaitPutar(turn);

        if (fabsf(err) <= PIVOT_TOL_DEG) {
            if (!masukSejak) masukSejak = millis();
            if (millis() - masukSejak >= PIVOT_DIAM_MS) { selesai = true; break; }
        } else {
            masukSejak = 0;
        }

        if (millis() - lapor >= 250) {
            lapor = millis();
            Serial.print(F("  yaw ")); Serial.print(yaw, 1);
            Serial.print(F("  error ")); Serial.print(err, 1);
            Serial.print(F("  gyroZ ")); Serial.print(gz, 1);
            Serial.print(F("  -> putar ")); Serial.print(turn, 3);
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

    Serial.print(F("\n  hasil: "));
    if (selesai) {
        Serial.print(F("SAMPAI dalam ")); Serial.print((millis() - t0) / 1000.0f, 1);
        Serial.println(F(" detik"));
    } else {
        Serial.println(F("belum sampai (wajar selama gaitPutar() kosong)"));
    }
    Serial.print(F("  error awal ")); Serial.print(errAwal, 1);
    Serial.print(F(" -> akhir ")); Serial.print(wrap180(headArah[arah] - yaw), 1);
    Serial.println(F(" derajat"));
    Serial.println(F("  Periksa TANDA: saat error positif, perintah putar harus positif juga."));
    Serial.println(F("  Kalau terbalik, robot nanti berputar menjauhi sasaran — balik tanda"));
    Serial.println(F("  di gaitPutar(), jangan di PIVOT_KP."));
}

// ======================================================== pindai baud
// Perangkat WIT keluar pabrik di 9600. Daripada menebak, coba semuanya dan
// lihat mana yang menghasilkan frame sah.
static const uint32_t BAUD_LIST[] = { 9600, 19200, 38400, 57600, 115200,
                                      230400, 460800, 921600 };
#define N_BAUD (sizeof(BAUD_LIST) / sizeof(BAUD_LIST[0]))

static void setBaud(uint32_t b) {
    imuBaud = b;
    IMU_SERIAL.end();
    IMU_SERIAL.addMemoryForRead(rxExtra, sizeof(rxExtra));   // harus sebelum begin()
    IMU_SERIAL.begin(imuBaud);
    rxN = 0;
    statReset();
}

static void pindaiBaud() {
    Serial.println(F("\n--- Pindai baud IMU ---"));
    Serial.println(F("  baud     frame sah   checksum gagal"));
    uint32_t terbaik = 0; uint32_t skorTerbaik = 0;

    for (uint8_t i = 0; i < N_BAUD; i++) {
        setBaud(BAUD_LIST[i]);
        delay(150);
        while (IMU_SERIAL.available()) IMU_SERIAL.read();   // buang sisa baud lama
        rxN = 0; statReset();

        uint32_t t0 = millis();
        while (millis() - t0 < 1200) witPump();

        Serial.print(F("  ")); Serial.print(BAUD_LIST[i]);
        for (uint8_t k = 0; k < 9 - (BAUD_LIST[i] >= 100000 ? 6 : 5); k++) Serial.print(' ');
        Serial.print(stOk);
        Serial.print(F("           ")); Serial.print(stSumBad);
        if (stOk > 0 && stOk > stSumBad) Serial.print(F("   <-- jalan"));
        Serial.println();

        if (stOk > skorTerbaik) { skorTerbaik = stOk; terbaik = BAUD_LIST[i]; }
    }

    if (!terbaik) {
        Serial.println(F("  TIDAK ADA baud yang menghasilkan frame."));
        Serial.println(F("  Cek TX IMU -> RX2 (pin 7) dan GND bersama. Tanpa GND,"));
        Serial.println(F("  gejalanya frame korup/kosong, bukan 'baud salah'."));
        setBaud(BAUD_LIST[0]);
        return;
    }
    Serial.print(F("  -> dipakai ")); Serial.println(terbaik);
    setBaud(terbaik);
}

// ============================================================ bantuan
static void printHelp() {
    Serial.println(F("\n============= TES_IMU — kelayakan yaw ============="));
    Serial.print  (F("IMU Serial2 (RX2 pin 7) @ ")); Serial.print(imuBaud);
    Serial.print  (F(" | servo: "));
    Serial.println(svMode == SV_OFF ? F("MATI") : svMode == SV_HOLD ? F("DIAM") : F("GOYANG"));
    Serial.println(F(" PENGUKURAN INTI"));
    Serial.println(F("  g        UJI GANGGUAN SERVO 3 fase (mati/diam/gerak) <- yang menentukan"));
    Serial.println(F("  d<detik> uji hanyutan yaw saat diam (mis. d60)"));
    Serial.println(F("  f        statistik frame WIT (checksum, laju, per tipe)"));
    Serial.println(F(" KOMPAS ARENA  (0=UTARA 1=TIMUR 2=SELATAN 3=BARAT)"));
    Serial.println(F("  c<n>     catat yaw sekarang sebagai arah n (mis. c0)"));
    Serial.println(F("  k        tabel 4 arah + cek kelinieran (jarak harus ~90 der)"));
    Serial.println(F("  o<n>     PIVOT ke arah n — gaitPutar() masih kosong, lihat README"));
    Serial.println(F("  e / E    simpan / muat 4 arah dari EEPROM"));
    Serial.println(F(" TAMPILAN"));
    Serial.println(F("  a        tampilkan data terus-menerus     x  berhenti"));
    Serial.println(F("  t        tare roll/pitch jadi nol"));
    Serial.println(F("  z        jadikan yaw sekarang sebagai referensi (tampil 'rel')"));
    Serial.println(F("  r        reset statistik frame"));
    Serial.println(F(" SERVO  (kaki harus menggantung!)"));
    Serial.println(F("  s0/s1/s2 servo mati / diam bertenaga / goyang"));
    Serial.println(F("  w<der>   amplitudo goyang, derajat (mis. w15)"));
    Serial.println(F("  p<detik> lama tiap fase uji 'g' (mis. p12)"));
    Serial.println(F(" LAIN"));
    Serial.println(F("  B        PINDAI baud — coba 9600..921600, pakai yang jalan"));
    Serial.println(F("  b<baud>  set baud langsung (mis. b9600, b115200)"));
    Serial.println(F("           IMU harus diset ke baud yang sama lewat aplikasi WIT"));
    Serial.println(F("  h        bantuan"));
    Serial.println(F("=================================================\n"));
}

// ============================================================== setup
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) { }

    Serial.println(F("\n\nTES_IMU — Yahboom 10-axis (WIT) di Serial2 (RX2 pin 7 / TX2 pin 8)"));

    // Buffer RX bawaan Teensy 64 byte = hanya 0,7 ms data pada 921600 baud.
    // Tanpa tambahan ini, jeda loop sedikit saja sudah membuat byte hilang dan
    // terbaca sebagai "IMU-nya berisik" padahal masalahnya di sisi Teensy.
    IMU_SERIAL.addMemoryForRead(rxExtra, sizeof(rxExtra));
    IMU_SERIAL.begin(imuBaud);

    BUS_DRV0.begin(); BUS_DRV0.setClock(SERVO_I2C_CLOCK);
    BUS_DRV1.begin(); BUS_DRV1.setClock(SERVO_I2C_CLOCK);
    drv0.begin(); drv0.setPWMFreq(SERVO_PWM_FREQ);
    drv1.begin(); drv1.setPWMFreq(SERVO_PWM_FREQ);
    servoAllOff();

    if (smLoad(map_)) Serial.println(F("Pemetaan servo dimuat dari EEPROM."));
    else { smDefaults(map_); Serial.println(F("EEPROM kosong — pakai pemetaan bawaan servo_map.h.")); }

    for (uint8_t i = 0; i < NACC; i++) accReset(i);
    statReset();
    kompasMuat(true);

    Serial.println(F("Servo dimatikan. 'g' = uji gangguan (kaki harus menggantung)."));
    printHelp();
}

// =========================================================== perintah
static void handleCmd(char* s) {
    while (*s == ' ') s++;
    uint8_t d1  = (uint8_t)(s[1] - '0');
    int     num = atoi(s + 1);

    switch (*s) {
        case 'h': case '?': printHelp(); break;
        case 'a': mode = TAMPIL; Serial.println(F("tampil (x + Enter untuk berhenti)")); break;
        case 'x': mode = IDLE; servoSetMode(SV_OFF); Serial.println(F("berhenti.")); break;
        case 'f': mode = IDLE; printStat(); break;
        case 'r': statReset(); Serial.println(F("statistik direset.")); break;
        case 't':
            roll0 = roll; pitch0 = pitch;
            Serial.println(F("roll/pitch di-tare ke nol."));
            break;
        case 'z':
            yawRef = yaw;
            Serial.print(F("referensi yaw = ")); Serial.print(yawRef, 1); Serial.println(F(" derajat"));
            break;

        case 'g': mode = IDLE; ujiGangguan(); break;
        case 'd': mode = IDLE; ujiDrift((uint16_t)num); break;

        case 'c':
            if (d1 > 3) { Serial.println(F("c0=UTARA c1=TIMUR c2=SELATAN c3=BARAT")); break; }
            kompasCatat(d1);
            break;
        case 'k': mode = IDLE; kompasTabel(); break;
        case 'o':
            if (d1 > 3) { Serial.println(F("o0=UTARA o1=TIMUR o2=SELATAN o3=BARAT")); break; }
            mode = IDLE; pivotKe(d1);
            break;
        case 'e': kompasSimpan(); break;
        case 'E': if (kompasMuat(true)) kompasTabel(); break;

        case 's':
            if (d1 > 2) { Serial.println(F("s0=mati s1=diam s2=goyang")); break; }
            servoSetMode(d1);
            break;
        case 'w':
            if (num < 2 || num > 30) { Serial.println(F("amplitudo 2..30 derajat")); break; }
            wiggleAmp = (float)num;
            Serial.print(F("amplitudo goyang = ")); Serial.print(wiggleAmp, 0); Serial.println(F(" derajat"));
            break;
        case 'p':
            if (num < 3 || num > 60) { Serial.println(F("lama fase 3..60 detik")); break; }
            faseDetik = (uint16_t)num;
            Serial.print(F("lama tiap fase = ")); Serial.print(faseDetik); Serial.println(F(" detik"));
            break;

        case 'b':
            if (num < 1200 || num > 921600) {
                Serial.println(F("tulis baud-nya langsung, mis. b9600 atau b115200"));
                break;
            }
            setBaud((uint32_t)num);
            Serial.print(F("baud Teensy = ")); Serial.println(imuBaud);
            Serial.println(F("IMU-nya juga harus diset ke baud ini, kalau tidak semua frame korup."));
            break;
        case 'B': mode = IDLE; pindaiBaud(); break;
        case 0: break;
        default: Serial.println(F("perintah tidak dikenal, ketik 'h'"));
    }
}

// ================================================================ loop
void loop() {
    static char    buf[16];
    static uint8_t len = 0;

    witPump();
    servoUpdate();

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            buf[len] = 0;
            if (len) handleCmd(buf);
            else if (mode != IDLE) { mode = IDLE; Serial.println(F("berhenti.")); }
            len = 0;
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        }
    }

    if (mode == TAMPIL && millis() - lastPrint >= 200) {
        lastPrint = millis();
        printBaris();
    }
}
