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

uint32_t imuBaud   = 9600;
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
