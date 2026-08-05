/* =====================================================================
   MAP_LIDAR — memetakan channel mux ke POSISI FISIK lidar, lalu mengukur
               offset jarak tiap unit.
   Target : Teensy 4.1 (Wire=18/19, Wire1=17/16, Wire2=25/24)
   Library: "VL53L0X" by Pololu

   Bedanya dengan TES_LIDAR: TES_LIDAR menjawab "sensornya hidup atau tidak".
   Sketsa ini menjawab "channel nomor berapa itu lidar yang mana", lalu
   "pembacaannya meleset berapa mm". Keluarannya blok kode siap tempel untuk
   lidar_map.h dan config.h.

   Tata letak: 6 lidar = DEPAN, BELAKANG, 2 KIRI, 2 KANAN. Yang samping
   dipasang di celah antar kaki. Lihat lidar_map.h untuk gambarnya.

   Serial Monitor 115200, line ending Newline. Ketik 'h' untuk bantuan.
   ===================================================================== */

#include <Wire.h>
#include <EEPROM.h>
#include <string.h>
#include <stddef.h>      // offsetof — dipakai untuk checksum EEPROM
#include <math.h>        // sqrtf, lroundf — statistik kalibrasi
#include <VL53L0X.h>
#include "lidar_map.h"

// ---------------------------------------------------------------- knob
#define MUX_ADDR        0x70
#define VL53_ADDR       0x29
#define MUX_SETTLE_MS   2
#define INIT_RETRY      3
#define IO_TIMEOUT_MS   200
#define NO_TARGET_MM    8000     // VL53L0X mengembalikan ~8190 bila tak melihat apa pun
#define SMOOTH_N        5        // panjang median berjalan untuk tampilan
#define STAB_SAMPLES    50       // sampel untuk uji kestabilan 'q'

// ambang deteksi lambaian tangan saat wizard
#define WAVE_NEAR_MM    400      // tangan harus lebih dekat dari ini
#define WAVE_DROP_MM    150      // dan turun sekian mm dari baseline
#define WAVE_HOLD       3        // sweep berturut-turut sebelum dianggap sah
#define WAVE_TIMEOUT_MS 30000UL

#define CAL_SAMPLES     30       // sampel per titik kalibrasi
#define CAL_LOG_MAX     16       // riwayat kalibrasi satu sesi

#define EE_ADDR         0        // alamat awal di EEPROM
#define EE_MAGIC0       0xA5
#define EE_MAGIC1       0x5C
#define EE_VER          1

uint32_t i2cClock = 400000;

// ------------------------------------------------------ profil pengukuran
// JANGAN campur preset ST. Long range (VCSEL 18/14 + limit 0,10) menyuruh
// sensor menerima pantulan lemah; kalau waktu integrasinya sekalian dipendekkan
// ke 20 ms (preset high speed), hasilnya meloncat-loncat. Naikkan budget dulu
// sebelum menurunkan rate limit.
struct Profil {
    const char* nama;
    uint32_t    budget;      // us
    float       rateLimit;   // MCPS
    uint8_t     vcselPre, vcselFinal;
    const char* catatan;
};
static const Profil PROFIL[3] = {
    { "CEPAT",    20000, 0.10f, 18, 14, "jangkauan ~2 m, PALING BERISIK (setelan lama)" },
    { "SEIMBANG", 50000, 0.25f, 18, 14, "jangkauan ~1,8 m, tenang  <- dipakai" },
    { "TENANG",  200000, 0.25f, 14, 10, "jangkauan ~1,2 m, paling stabil & akurat"      }
};
// BATAS PERCAYA — temuan lapangan 5 Agustus 2026, bukan angka datasheet.
// Di bawah 100 cm pembacaan terbukti stabil; di atas itu VL53L0X mengembalikan
// nilai yang ADA tapi meloncat-loncat. Bahayanya: itu bukan 8190 yang bersih,
// jadi gate ">= NO_TARGET_MM" meloloskannya dan sampah itu terlihat seperti
// jarak sungguhan. Segala yang melewati batas ini dilaporkan "jauh", bukan
// diberi angka. Ubah runtime dengan 't<cm>'.
int16_t  trustMm     = 1000;
uint8_t  profil      = 1;        // SEIMBANG
bool     modeKontinu = false;    // false = single-shot: hanya 1 sensor menembak
                                 // pada satu saat -> tidak ada crosstalk antar lidar
bool     smoothOn    = true;     // median berjalan untuk tampilan a/l

// ------------------------------------------------------------ bus & obj
// Satu objek per CHANNEL: Pololu VL53L0X menyimpan stop_variable milik sensor
// itu, jadi objek TIDAK boleh dipakai bergantian antar sensor.
VL53L0X  sens[NUM_MUX_CH];
bool     chReady[NUM_MUX_CH];

TwoWire*    BUS[3]     = { &Wire, &Wire1, &Wire2 };
const char* BUSNAME[3] = { "Wire  (SDA18/SCL19)",
                           "Wire1 (SDA17/SCL16)",
                           "Wire2 (SDA25/SCL24)" };
uint8_t  busIdx = 0;
TwoWire* bus    = &Wire;

// ------------------------------------------------------------- keadaan
int8_t   posCh [NUM_LIDAR];      // posisi -> channel mux, -1 = belum
int16_t  posOff[NUM_LIDAR];      // koreksi mm per posisi
uint8_t  focus = LID_FRONT;      // posisi sasaran perintah kalibrasi

enum Mode { IDLE, READ_DENAH, READ_LINE, READ_RAW };
Mode     mode      = IDLE;
uint8_t  modeCh    = 0;
uint32_t lastPrint = 0;

struct CalRow { int8_t pos; int16_t trueMm; int16_t meanMm; };
CalRow   calLog[CAL_LOG_MAX];
uint8_t  calLogN = 0;

// ============================================================== util I2C
static bool ping(uint8_t addr) {
    bus->beginTransmission(addr);
    return bus->endTransmission() == 0;
}

// ch > 7 = matikan semua channel
static bool muxSelect(uint8_t ch) {
    bus->beginTransmission(MUX_ADDR);
    bus->write(ch > 7 ? 0x00 : (uint8_t)(1 << ch));
    return bus->endTransmission() == 0;
}

static int rdReg8(uint8_t addr, uint8_t reg) {
    bus->beginTransmission(addr);
    bus->write(reg);
    if (bus->endTransmission(false) != 0) return -1;
    if (bus->requestFrom(addr, (uint8_t)1) != 1) return -1;
    return bus->read();
}

static bool isVL53L0X() {
    return rdReg8(VL53_ADDR, 0xC0) == 0xEE && rdReg8(VL53_ADDR, 0xC2) == 0x10;
}

// ============================================================ pemetaan
static int8_t chToPos(int8_t ch) {
    for (uint8_t p = 0; p < NUM_LIDAR; p++) if (posCh[p] == ch) return (int8_t)p;
    return -1;
}

// kosongkan channel saja; offset hasil kalibrasi dipertahankan
static void clearMapChannelsOnly() {
    for (uint8_t p = 0; p < NUM_LIDAR; p++) posCh[p] = -1;
}

static void loadDefaults() {
    for (uint8_t p = 0; p < NUM_LIDAR; p++) {
        posCh [p] = LIDAR_CH_DEFAULT[p];
        posOff[p] = LIDAR_OFFSET_DEFAULT[p];
    }
}

// pasang posisi p ke channel ch, lepas dulu pemilik lama channel itu
static void assign(uint8_t p, int8_t ch) {
    int8_t lama = chToPos(ch);
    if (lama >= 0 && lama != (int8_t)p) {
        Serial.print(F("  (ch")); Serial.print(ch);
        Serial.print(F(" sebelumnya dipakai ")); Serial.print(LIDAR_NAME[lama]);
        Serial.println(F(" -> dilepas)"));
        posCh[lama] = -1;
    }
    posCh[p] = ch;
}

// ============================================================== EEPROM
struct MapStore {
    uint8_t  magic0, magic1, ver;
    int8_t   ch [NUM_LIDAR];
    int16_t  off[NUM_LIDAR];
    uint8_t  sum;
};

// Sengaja memakai (void*, size_t), bukan (const MapStore&): Arduino IDE
// menyisipkan prototipe fungsi hasil generate DI ATAS badan sketsa, sebelum
// 'struct MapStore' sempat dideklarasikan -> "does not name a type".
// Tipe buatan sendiri tidak boleh muncul di tanda tangan fungsi dalam .ino.
static uint8_t storeSum(const void* buf, size_t n) {
    const uint8_t* p = (const uint8_t*)buf;
    uint8_t acc = 0;
    for (size_t i = 0; i < n; i++) acc = (uint8_t)(acc + p[i] * 31 + 7);
    return acc;
}

// Checksum dihitung sampai OFFSET field 'sum', bukan sizeof-1: kompiler
// menyisipkan padding (ch[6] ganjil, off[] butuh align 2), jadi 'sum' bukan
// byte terakhir. memset dulu supaya byte padding bernilai tetap — kalau tidak,
// isinya sampah stack dan checksum simpan != checksum muat.
static void eeSave() {
    MapStore s;
    memset(&s, 0, sizeof(s));
    s.magic0 = EE_MAGIC0; s.magic1 = EE_MAGIC1; s.ver = EE_VER;
    for (uint8_t p = 0; p < NUM_LIDAR; p++) { s.ch[p] = posCh[p]; s.off[p] = posOff[p]; }
    s.sum = storeSum(&s, offsetof(MapStore, sum));
    EEPROM.put(EE_ADDR, s);
    Serial.println(F("Pemetaan + offset disimpan ke EEPROM."));
}

static bool eeLoad(bool verbose) {
    MapStore s;
    EEPROM.get(EE_ADDR, s);
    if (s.magic0 != EE_MAGIC0 || s.magic1 != EE_MAGIC1 || s.ver != EE_VER ||
        s.sum != storeSum(&s, offsetof(MapStore, sum))) {
        if (verbose) Serial.println(F("EEPROM kosong / rusak — pakai tabel lidar_map.h."));
        return false;
    }
    for (uint8_t p = 0; p < NUM_LIDAR; p++) {
        posCh[p]  = (s.ch[p] >= 0 && s.ch[p] < NUM_MUX_CH) ? s.ch[p] : -1;
        posOff[p] = s.off[p];
    }
    if (verbose) Serial.println(F("Pemetaan dimuat dari EEPROM."));
    return true;
}

static void eeErase() {
    MapStore s;
    memset(&s, 0, sizeof(s));
    EEPROM.put(EE_ADDR, s);
    Serial.println(F("EEPROM dihapus. Reboot akan memakai tabel lidar_map.h."));
}

// ============================================================= membaca
// mm mentah; -1 = timeout / tak ada target / channel belum siap
static int16_t readMmRaw(int8_t ch) {
    if (ch < 0 || ch >= NUM_MUX_CH || !chReady[ch]) return -1;
    muxSelect((uint8_t)ch);
    delayMicroseconds(300);
    // Single-shot: sensor hanya menembak saat diminta, jadi lima lidar lainnya
    // diam dan tidak mencemari pengukuran ini. Kontinu lebih cepat tapi keenam
    // laser menyala bersamaan — mux memutus I2C, bukan lasernya.
    uint16_t mm = modeKontinu ? sens[ch].readRangeContinuousMillimeters()
                              : sens[ch].readRangeSingleMillimeters();
    if (sens[ch].timeoutOccurred() || mm == 0xFFFF || mm >= NO_TARGET_MM) return -1;
    return (int16_t)mm;
}

// median dari n nilai (mengabaikan -1). -1 bila yang sah kurang dari separuh.
static int16_t medianOf(const int16_t* v, uint8_t n) {
    int16_t s[SMOOTH_N > STAB_SAMPLES ? SMOOTH_N : STAB_SAMPLES];
    uint8_t m = 0;
    for (uint8_t i = 0; i < n; i++) if (v[i] >= 0) s[m++] = v[i];
    if (m == 0 || m * 2 < n) return -1;
    for (uint8_t i = 1; i < m; i++) {
        int16_t k = s[i]; int8_t j = i - 1;
        while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
        s[j + 1] = k;
    }
    return s[m / 2];
}

// ---- median berjalan untuk tampilan ----
// Satu sampel baru per refresh, median atas SMOOTH_N sampel terakhir. Sengaja
// begini, bukan ambil 5 sampel sekaligus tiap refresh: dalam single-shot satu
// bacaan makan ~budget penuh, 5x6 sensor jadi hampir 2 detik per denah.
int16_t  hist[NUM_LIDAR][SMOOTH_N];
uint8_t  histN[NUM_LIDAR];
uint8_t  histI[NUM_LIDAR];

static void histReset() {
    for (uint8_t p = 0; p < NUM_LIDAR; p++) { histN[p] = 0; histI[p] = 0; }
}

static void histPush(uint8_t p, int16_t mm) {
    hist[p][histI[p]] = mm;
    histI[p] = (uint8_t)((histI[p] + 1) % SMOOTH_N);
    if (histN[p] < SMOOTH_N) histN[p]++;
}

// Untuk TAMPILAN a/l saja: menyuntik satu sampel baru ke riwayat lalu
// mengembalikan median berjalan bila filter aktif. Kalibrasi dan uji kestabilan
// TIDAK lewat sini — keduanya harus melihat nilai mentah.
// -2 = posisi belum dipetakan, -1 = tak ada bacaan, selain itu mm terkoreksi
static int16_t readMmPos(uint8_t p) {
    if (posCh[p] < 0) return -2;
    int16_t mm = readMmRaw(posCh[p]);
    histPush(p, mm);
    if (smoothOn) mm = medianOf(hist[p], histN[p]);
    if (mm < 0) return -1;
    int32_t k = (int32_t)mm + posOff[p];
    if (k < 0) k = 0;
    if (k > trustMm) return -4;      // ada bacaan, tapi di luar batas percaya
    return (int16_t)k;
}

// tulis 3 karakter rata kanan ke buf (butuh 4 byte)
static void fmt3cm(int16_t mm, char* buf) {
    if (mm == -2)      { strcpy(buf, "---"); return; }   // belum dipetakan
    if (mm == -1)      { strcpy(buf, "..."); return; }   // tak ada target
    if (mm == -4)      { strcpy(buf, ">>>"); return; }   // di luar batas percaya
    int v = mm / 10;
    if (v > 999) v = 999;
    snprintf(buf, 4, "%3d", v);
}

// ========================================================== tampilan
static void printTabel() {
    Serial.println(F("\n--- PEMETAAN LIDAR ---"));
    Serial.println(F("  pos  nama          ch   offset   letak"));
    for (uint8_t p = 0; p < NUM_LIDAR; p++) {
        Serial.print(F("  "));
        Serial.print(p); Serial.print(p == focus ? F("* ") : F("  "));
        Serial.print(LIDAR_NAME[p]);
        for (uint8_t i = strlen(LIDAR_NAME[p]); i < 14; i++) Serial.print(' ');
        if (posCh[p] < 0) Serial.print(F("--"));
        else { Serial.print(F("ch")); Serial.print(posCh[p]); }
        Serial.print(F("   "));
        if (posOff[p] >= 0) Serial.print('+');
        Serial.print(posOff[p]); Serial.print(F("mm"));
        Serial.print(F("   ")); Serial.println(LIDAR_WHERE[p]);
    }
    // channel siap yang belum punya posisi
    bool ada = false;
    for (uint8_t c = 0; c < NUM_MUX_CH; c++) {
        if (chReady[c] && chToPos((int8_t)c) < 0) {
            if (!ada) { Serial.print(F("  channel siap tapi belum dipetakan:")); ada = true; }
            Serial.print(F(" ch")); Serial.print(c);
        }
    }
    if (ada) Serial.println();
    Serial.println(F("  (* = posisi fokus untuk perintah kalibrasi 'c')"));
}

static void printDenah() {
    char b[NUM_LIDAR][4];
    for (uint8_t p = 0; p < NUM_LIDAR; p++) fmt3cm(readMmPos(p), b[p]);
    muxSelect(8);

    Serial.println(F("\n            DEPAN            (cm)"));
    Serial.println(F("           +-------+"));
    Serial.print  (F("           | F ")); Serial.print(b[LID_FRONT]);       Serial.println(F(" |"));
    Serial.println(F(" +-------+ +-------+ +-------+"));
    Serial.print  (F(" | LF ")); Serial.print(b[LID_LEFT_FRONT]); Serial.print(F("|           |RF "));
    Serial.print  (b[LID_RIGHT_FRONT]); Serial.println(F(" |"));
    Serial.println(F(" +-------+  H E X A  +-------+"));
    Serial.print  (F(" | LR ")); Serial.print(b[LID_LEFT_REAR]);  Serial.print(F("|           |RR "));
    Serial.print  (b[LID_RIGHT_REAR]);  Serial.println(F(" |"));
    Serial.println(F(" +-------+ +-------+ +-------+"));
    Serial.print  (F("           | B ")); Serial.print(b[LID_BACK]);        Serial.println(F(" |"));
    Serial.println(F("           +-------+"));
    Serial.print  (F("           BELAKANG    >>> = lebih jauh dari batas percaya "));
    Serial.print(trustMm / 10); Serial.println(F(" cm"));
    Serial.println(F("                       ... = tak ada target, --- = belum dipetakan"));
}

static void printBaris() {
    Serial.print(F("  "));
    for (uint8_t p = 0; p < NUM_LIDAR; p++) {
        char b[4]; fmt3cm(readMmPos(p), b);
        Serial.print(LIDAR_SHORT[p]); Serial.print('='); Serial.print(b); Serial.print(F("cm  "));
    }
    muxSelect(8);
    Serial.println();
}

static void printRaw(uint8_t ch) {
    int16_t mm = readMmRaw((int8_t)ch);
    muxSelect(8);
    Serial.print(F("ch")); Serial.print(ch); Serial.print(F(" -> "));
    int8_t p = chToPos((int8_t)ch);
    if (p >= 0) { Serial.print(LIDAR_NAME[p]); Serial.print(F(" : ")); }
    else        { Serial.print(F("(belum dipetakan) : ")); }
    if (mm < 0) Serial.println(F("tak ada bacaan"));
    else { Serial.print(mm); Serial.print(F(" mm = ")); Serial.print(mm / 10); Serial.println(F(" cm")); }
}

// ============================================================ init bus
static void useBus(uint8_t i) {
    busIdx = i;
    bus    = BUS[i];
    bus->begin();
    bus->setClock(i2cClock);
    for (uint8_t c = 0; c < NUM_MUX_CH; c++) chReady[c] = false;
    mode = IDLE;
    Serial.print(F(">> Bus lidar: ")); Serial.println(BUSNAME[i]);
}

// hitung sensor VL53L0X di balik mux pada bus aktif
static uint8_t hitungSensor() {
    bus->beginTransmission(MUX_ADDR);
    bus->write((uint8_t)0x00);
    if (bus->endTransmission() != 0) return 0;      // tak ada mux di bus ini
    uint8_t n = 0;
    for (uint8_t c = 0; c < NUM_MUX_CH; c++) {
        if (!muxSelect(c)) continue;
        delay(MUX_SETTLE_MS);
        if (ping(VL53_ADDR) && isVL53L0X()) n++;
    }
    muxSelect(8);
    return n;
}

static void autoDetect() {
    Serial.println(F("\n--- Cari bus lidar ---"));
    int best = -1; uint8_t bestN = 0;
    for (uint8_t i = 0; i < 3; i++) {
        busIdx = i; bus = BUS[i];
        bus->begin(); bus->setClock(i2cClock);
        uint8_t n = hitungSensor();
        Serial.print(F("  ")); Serial.print(BUSNAME[i]);
        Serial.print(F(" : ")); Serial.print(n); Serial.println(F(" VL53L0X di balik mux"));
        if (n > bestN) { bestN = n; best = i; }
    }
    if (best < 0) {
        Serial.println(F("  TIDAK ADA lidar ditemukan. Jalankan TES_LIDAR ('d') untuk diagnosa."));
        useBus(0);
    } else {
        useBus((uint8_t)best);
    }
}

static void initAll() {
    const Profil& pr = PROFIL[profil];
    Serial.println(F("\n--- Init lidar ---"));
    Serial.print(F("  profil ")); Serial.print(profil); Serial.print(F(" = ")); Serial.print(pr.nama);
    Serial.print(F(" | budget ")); Serial.print(pr.budget / 1000); Serial.print(F(" ms"));
    Serial.print(F(" | limit ")); Serial.print(pr.rateLimit, 2);
    Serial.print(F(" | VCSEL ")); Serial.print(pr.vcselPre); Serial.print('/'); Serial.print(pr.vcselFinal);
    Serial.print(F(" | ")); Serial.println(modeKontinu ? F("KONTINU") : F("single-shot"));

    // Hentikan mode kontinu sisa init sebelumnya. Mux hanya memutus I2C, bukan
    // laser — sensor yang masih free-running akan mencemari pengukuran lain.
    for (uint8_t c = 0; c < NUM_MUX_CH; c++) {
        if (!chReady[c]) continue;
        if (muxSelect(c)) { delay(MUX_SETTLE_MS); sens[c].stopContinuous(); }
    }

    uint8_t ok = 0;
    for (uint8_t c = 0; c < NUM_MUX_CH; c++) {
        chReady[c] = false;
        if (!muxSelect(c)) continue;
        delay(MUX_SETTLE_MS);
        if (!ping(VL53_ADDR) || !isVL53L0X()) continue;

        sens[c].setBus(bus);
        // harus di ATAS timing budget, kalau tidak profil TENANG (200 ms)
        // selalu timeout sebelum pengukurannya sendiri selesai
        sens[c].setTimeout(pr.budget / 1000 + IO_TIMEOUT_MS);
        bool good = false;
        for (uint8_t r = 0; r < INIT_RETRY && !good; r++) good = sens[c].init();

        Serial.print(F("  ch")); Serial.print(c); Serial.print(F(": "));
        if (!good) { Serial.println(F("GAGAL init (coba 'k1' lalu 'i')")); continue; }

        // Urutan penting: rate limit & VCSEL dulu, budget belakangan —
        // mengubah VCSEL period ikut menghitung ulang budget di dalam library.
        sens[c].setSignalRateLimit(pr.rateLimit);
        sens[c].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,   pr.vcselPre);
        sens[c].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, pr.vcselFinal);
        sens[c].setMeasurementTimingBudget(pr.budget);
        if (modeKontinu) sens[c].startContinuous(0);
        chReady[c] = true; ok++;

        int8_t p = chToPos((int8_t)c);
        if (p >= 0) { Serial.print(F("OK -> ")); Serial.println(LIDAR_NAME[p]); }
        else          Serial.println(F("OK (belum dipetakan)"));
    }
    muxSelect(8);
    histReset();
    Serial.print(F("  siap: ")); Serial.print(ok); Serial.print('/');
    Serial.print(NUM_LIDAR); Serial.println(F(" lidar"));
    if (ok < NUM_LIDAR)
        Serial.println(F("  kurang dari 6 — cek kabel channel yang hilang sebelum memetakan."));
}

// ============================================================== wizard
// baseline per channel: median 5 bacaan, dipakai sebagai pembanding lambaian
static void ambilBaseline(int16_t* base) {
    for (uint8_t c = 0; c < NUM_MUX_CH; c++) {
        base[c] = -1;
        if (!chReady[c]) continue;
        int16_t s[5];
        for (uint8_t i = 0; i < 5; i++) {
            int16_t v = readMmRaw((int8_t)c);
            s[i] = (v < 0) ? 8190 : v;          // "tak ada target" = sangat jauh
        }
        for (uint8_t i = 1; i < 5; i++) {       // insertion sort
            int16_t k = s[i]; int8_t j = i - 1;
            while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
            s[j + 1] = k;
        }
        base[c] = s[2];
    }
    muxSelect(8);
}

// true bila operator mengetik sesuatu (pembatal wizard).
// CR/LF/spasi diabaikan: sisa baris perintah pemanggil ("w\r\n" pada mode
// "Both NL & CR") masih ada di buffer saat wizard mulai — kalau ikut dihitung,
// wizard batal sendiri sebelum sempat jalan.
static bool dibatalkan() {
    bool batal = false;
    while (Serial.available()) {
        char c = Serial.read();
        if (c != '\r' && c != '\n' && c != ' ') batal = true;
    }
    if (batal) Serial.println(F("  ** dibatalkan **"));
    return batal;
}

// tunggu semua channel kembali ke baseline (tangan ditarik)
static bool tungguLepas(const int16_t* base) {
    uint32_t t0 = millis();
    while (millis() - t0 < 5000) {
        bool bersih = true;
        for (uint8_t c = 0; c < NUM_MUX_CH; c++) {
            if (!chReady[c]) continue;
            int16_t v = readMmRaw((int8_t)c);
            if (v >= 0 && v < WAVE_NEAR_MM && base[c] - v >= WAVE_DROP_MM / 2) bersih = false;
        }
        muxSelect(8);
        if (bersih) return true;
        if (dibatalkan()) return false;
    }
    return true;   // menyerah menunggu, lanjut saja
}

// petakan SATU posisi lewat lambaian tangan. true = berhasil.
static bool wizardSatu(uint8_t p, const int16_t* base) {
    Serial.println();
    Serial.print(F("  >> ")); Serial.print(LIDAR_NAME[p]);
    Serial.print(F("  (")); Serial.print(LIDAR_WHERE[p]); Serial.println(F(")"));
    Serial.println(F("     Dekatkan telapak tangan 10-20 cm di depan lidar itu."));
    Serial.println(F("     (Enter/karakter apa pun = batal)"));

    uint8_t  hold[NUM_MUX_CH]; memset(hold, 0, sizeof(hold));
    uint32_t t0 = millis();

    while (millis() - t0 < WAVE_TIMEOUT_MS) {
        if (dibatalkan()) return false;

        int8_t  kandidat = -1;
        uint8_t nAktif   = 0;
        for (uint8_t c = 0; c < NUM_MUX_CH; c++) {
            if (!chReady[c]) continue;
            if (chToPos((int8_t)c) >= 0 && chToPos((int8_t)c) != (int8_t)p) continue;  // sudah dipakai posisi lain

            int16_t v = readMmRaw((int8_t)c);
            bool trig = (v >= 0) && (v < WAVE_NEAR_MM) && (base[c] - v >= WAVE_DROP_MM);
            if (trig) { if (hold[c] < 250) hold[c]++; } else hold[c] = 0;

            if (hold[c] >= WAVE_HOLD) { nAktif++; kandidat = (int8_t)c; }
        }
        muxSelect(8);

        if (nAktif > 1) {
            Serial.println(F("     ! dua sensor sekaligus melihat tangan — mundurkan tangan,"));
            Serial.println(F("       lalu dekatkan lebih rapat ke SATU lidar saja."));
            memset(hold, 0, sizeof(hold));
            delay(600);
            continue;
        }
        if (nAktif == 1) {
            assign(p, kandidat);
            Serial.print(F("     OK: ")); Serial.print(LIDAR_NAME[p]);
            Serial.print(F(" = ch")); Serial.println(kandidat);
            tungguLepas(base);
            return true;
        }
    }
    Serial.println(F("     (waktu habis — posisi ini dilewati)"));
    return false;
}

static void wizardSemua() {
    Serial.println(F("\n========== WIZARD PEMETAAN LIDAR =========="));
    Serial.println(F("Robot didiamkan di tempat lapang. Program mencatat jarak diam tiap"));
    Serial.println(F("sensor dulu, lalu Anda mendekatkan tangan ke satu lidar saat diminta."));

    uint8_t siap = 0;
    for (uint8_t c = 0; c < NUM_MUX_CH; c++) if (chReady[c]) siap++;
    if (siap == 0) { Serial.println(F("Tidak ada lidar siap. Jalankan 'i' dulu.")); return; }
    if (siap < NUM_LIDAR) {
        Serial.print(F("PERINGATAN: hanya ")); Serial.print(siap);
        Serial.println(F(" lidar siap — sebagian posisi akan kosong."));
    }

    clearMapChannelsOnly();
    int16_t base[NUM_MUX_CH];
    Serial.println(F("\nMengambil baseline (jangan ada tangan/benda dekat sensor)..."));
    delay(1200);
    ambilBaseline(base);
    Serial.print(F("  baseline cm:"));
    for (uint8_t c = 0; c < NUM_MUX_CH; c++) {
        if (!chReady[c]) continue;
        Serial.print(F("  ch")); Serial.print(c); Serial.print('=');
        Serial.print(base[c] / 10);
    }
    Serial.println();

    uint8_t ok = 0;
    for (uint8_t p = 0; p < NUM_LIDAR; p++) {
        if (!wizardSatu(p, base)) {
            Serial.println(F("  wizard berhenti."));
            break;
        }
        ok++;
    }
    Serial.print(F("\n")); Serial.print(ok); Serial.print('/');
    Serial.print(NUM_LIDAR); Serial.println(F(" posisi terpetakan."));
    printTabel();
    Serial.println(F("Simpan dengan 'e', cetak kode dengan 'g'."));
}

// ========================================================= kalibrasi
static void kalibrasi(int trueCm) {
    if (posCh[focus] < 0) {
        Serial.print(LIDAR_NAME[focus]); Serial.println(F(" belum dipetakan."));
        return;
    }
    if (trueCm <= 0 || trueCm > 200) {
        Serial.println(F("jarak harus 1..200 cm, mis. 'c50'"));
        return;
    }
    Serial.print(F("\n--- Kalibrasi ")); Serial.print(LIDAR_NAME[focus]);
    Serial.print(F(" (ch")); Serial.print(posCh[focus]);
    Serial.print(F(") pada ")); Serial.print(trueCm); Serial.println(F(" cm ---"));
    Serial.println(F("  Target rata & tegak lurus, diukur dari PERMUKAAN LENSA."));

    int32_t jml = 0; int16_t mn = 32767, mx = -1;
    int16_t s[CAL_SAMPLES]; uint8_t n = 0;
    for (uint8_t i = 0; i < CAL_SAMPLES; i++) {
        int16_t v = readMmRaw(posCh[focus]);      // MENTAH, tanpa offset
        if (v < 0) continue;
        s[n++] = v; jml += v;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    muxSelect(8);

    if (n < CAL_SAMPLES / 2) {
        Serial.print(F("  hanya ")); Serial.print(n);
        Serial.println(F(" sampel sah — sensor tidak melihat target. Batal."));
        return;
    }
    float mean = (float)jml / n;
    float var  = 0;
    for (uint8_t i = 0; i < n; i++) { float d = s[i] - mean; var += d * d; }
    float sd = sqrtf(var / n);

    int16_t trueMm = (int16_t)(trueCm * 10);
    int16_t off    = (int16_t)lroundf(trueMm - mean);

    Serial.print(F("  sampel sah : ")); Serial.print(n); Serial.print('/'); Serial.println(CAL_SAMPLES);
    Serial.print(F("  rata-rata  : ")); Serial.print(mean, 1); Serial.println(F(" mm"));
    Serial.print(F("  min / maks : ")); Serial.print(mn); Serial.print(F(" / ")); Serial.print(mx);
    Serial.print(F(" mm  (sebar ")); Serial.print(mx - mn); Serial.println(F(" mm)"));
    Serial.print(F("  simpangan  : ")); Serial.print(sd, 1); Serial.println(F(" mm"));
    Serial.print(F("  sebenarnya : ")); Serial.print(trueMm); Serial.println(F(" mm"));
    Serial.print(F("  MELESET    : ")); Serial.print(mean - trueMm, 1);
    Serial.println(F(" mm  (positif = pembacaan kejauhan)"));

    posOff[focus] = off;
    Serial.print(F("  offset ")); Serial.print(LIDAR_NAME[focus]);
    Serial.print(F(" diset ke ")); if (off >= 0) Serial.print('+');
    Serial.print(off); Serial.println(F(" mm"));

    if (sd > 15) Serial.println(F("  ! berisik (>15 mm) — target terlalu gelap/miring, atau I2C tidak stabil."));

    if (calLogN < CAL_LOG_MAX) {
        calLog[calLogN].pos    = (int8_t)focus;
        calLog[calLogN].trueMm = trueMm;
        calLog[calLogN].meanMm = (int16_t)lroundf(mean);
        calLogN++;
    }
    Serial.println(F("  ulangi di jarak lain (10/20/50/100 cm) lalu 'T' untuk tabel."));
}

// Uji kestabilan: 50 sampel MENTAH pada posisi fokus, target diam.
// Menjawab "berisiknya berapa mm", supaya perbandingan antar profil terukur.
static void ujiStabil() {
    if (posCh[focus] < 0) {
        Serial.print(LIDAR_NAME[focus]); Serial.println(F(" belum dipetakan."));
        return;
    }
    const Profil& pr = PROFIL[profil];
    Serial.print(F("\n--- Uji kestabilan ")); Serial.print(LIDAR_NAME[focus]);
    Serial.print(F(" (ch")); Serial.print(posCh[focus]); Serial.println(F(") ---"));
    Serial.print(F("  profil ")); Serial.print(pr.nama);
    Serial.print(F(", ")); Serial.print(modeKontinu ? F("kontinu") : F("single-shot"));
    Serial.println(F(" — target JANGAN digerakkan."));

    int16_t s[STAB_SAMPLES];
    int32_t jml = 0; int16_t mn = 32767, mx = -1; uint8_t n = 0, gagal = 0, jauh = 0;
    for (uint8_t i = 0; i < STAB_SAMPLES; i++) {
        int16_t v = readMmRaw(posCh[focus]);      // MENTAH, batas percaya tidak dipakai di sini
        if (v < 0) { gagal++; continue; }
        if (v > trustMm) jauh++;
        s[n++] = v; jml += v;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    muxSelect(8);

    if (n < 5) { Serial.println(F("  hampir semua bacaan gagal — sensor tidak melihat target.")); return; }

    float mean = (float)jml / n, var = 0;
    for (uint8_t i = 0; i < n; i++) { float d = s[i] - mean; var += d * d; }
    float sd = sqrtf(var / n);

    Serial.print(F("  sah/gagal  : ")); Serial.print(n); Serial.print('/'); Serial.println(gagal);
    Serial.print(F("  median     : ")); Serial.print(medianOf(s, n)); Serial.println(F(" mm"));
    Serial.print(F("  rata-rata  : ")); Serial.print(mean, 1); Serial.println(F(" mm"));
    Serial.print(F("  min / maks : ")); Serial.print(mn); Serial.print(F(" / ")); Serial.println(mx);
    Serial.print(F("  SEBAR      : ")); Serial.print(mx - mn); Serial.println(F(" mm  <- ini yang terlihat 'jumping'"));
    Serial.print(F("  simpangan  : ")); Serial.print(sd, 1); Serial.println(F(" mm"));

    Serial.print(F("  VONIS: "));
    if (jauh > n / 2) {
        Serial.print(F("target di luar batas percaya ")); Serial.print(trustMm / 10);
        Serial.println(F(" cm."));
        Serial.println(F("  Sebaran besar di sini WAJAR dan bukan kerusakan — di jarak segini"));
        Serial.println(F("  VL53L0X memang mengarang. Dekatkan target, atau naikkan batas"));
        Serial.println(F("  dengan 't<cm>' HANYA kalau di jarak itu sebarannya terbukti kecil."));
        return;
    }
    if      (gagal > STAB_SAMPLES / 4) Serial.println(F("BANYAK GAGAL — target terlalu jauh/gelap, atau I2C ('k1')."));
    else if (sd < 5)                   Serial.println(F("tenang. Aman dipakai."));
    else if (sd < 15)                  Serial.println(F("wajar untuk VL53L0X. Median-3 di firmware cukup."));
    else if (sd < 40)                  Serial.println(F("BERISIK — naikkan profil ('P2'), atau target terlalu miring/gelap."));
    else                               Serial.println(F("SANGAT BERISIK — curigai crosstalk antar lidar, daya 3V3, atau cahaya matahari."));
    if (!modeKontinu && sd >= 15)
        Serial.println(F("  Sudah single-shot, jadi bukan crosstalk antar lidar. Lihat permukaan target & cahaya sekitar."));
}

static void printCalLog() {
    if (!calLogN) { Serial.println(F("Belum ada titik kalibrasi sesi ini.")); return; }
    Serial.println(F("\n--- Titik kalibrasi sesi ini ---"));
    Serial.println(F("  lidar         asli(cm)  baca(cm)  meleset(mm)"));
    for (uint8_t i = 0; i < calLogN; i++) {
        const CalRow& r = calLog[i];
        Serial.print(F("  ")); Serial.print(LIDAR_NAME[r.pos]);
        for (uint8_t k = strlen(LIDAR_NAME[r.pos]); k < 14; k++) Serial.print(' ');
        Serial.print(r.trueMm / 10); Serial.print(F("        "));
        Serial.print(r.meanMm / 10); Serial.print(F("        "));
        Serial.println(r.meanMm - r.trueMm);
    }
    Serial.println(F("  Kalau 'meleset' hampir sama di semua jarak -> offset tetap (dipakai)."));
    Serial.println(F("  Kalau membesar seiring jarak -> butuh skala, bukan offset. Catat manual."));
}

// ======================================================== cetak kode
static void cetakKode() {
    Serial.println(F("\n// ===== tempel ke MAP_LIDAR/lidar_map.h ====="));
    Serial.println(F("static const int8_t LIDAR_CH_DEFAULT[NUM_LIDAR] = {"));
    for (uint8_t p = 0; p < NUM_LIDAR; p++) {
        Serial.print(F("    ")); Serial.print(posCh[p]);
        Serial.print(p < NUM_LIDAR - 1 ? F(",   // LID_") : F("    // LID_"));
        Serial.println(LIDAR_NAME[p]);
    }
    Serial.println(F("};"));
    Serial.println(F("static const int16_t LIDAR_OFFSET_DEFAULT[NUM_LIDAR] = {"));
    for (uint8_t p = 0; p < NUM_LIDAR; p++) {
        Serial.print(F("    ")); Serial.print(posOff[p]);
        Serial.print(p < NUM_LIDAR - 1 ? F(",   // LID_") : F("    // LID_"));
        Serial.println(LIDAR_NAME[p]);
    }
    Serial.println(F("};"));

    Serial.println(F("\n// ===== tempel ke HEXAPOD_KRSRI_2026/config.h ====="));
    Serial.println(F("// Indeks lidar -> arti (searah jarum jam dari depan)"));
    for (uint8_t p = 0; p < NUM_LIDAR; p++) {
        Serial.print(F("#define LIDAR_")); Serial.print(LIDAR_NAME[p]);
        for (uint8_t i = strlen(LIDAR_NAME[p]); i < 14; i++) Serial.print(' ');
        Serial.println(p);
    }
    Serial.println(F("// indeks lidar -> channel TCA9548A (JANGAN diasumsikan sama)"));
    Serial.print  (F("#define LIDAR_CH_MAP     { "));
    for (uint8_t p = 0; p < NUM_LIDAR; p++) {
        Serial.print(posCh[p]); if (p < NUM_LIDAR - 1) Serial.print(F(", "));
    }
    Serial.println(F(" }"));
    Serial.print  (F("#define LIDAR_OFFSET_MM  { "));
    for (uint8_t p = 0; p < NUM_LIDAR; p++) {
        Serial.print(posOff[p]); if (p < NUM_LIDAR - 1) Serial.print(F(", "));
    }
    Serial.println(F(" }"));
    Serial.print  (F("#define LIDAR_MAX_CM     ")); Serial.print(trustMm / 10);
    Serial.println(F("    // batas TERUKUR, bukan spek datasheet"));
    Serial.println(F("// Di atas batas ini VL53L0X mengembalikan nilai yang meloncat-loncat"));
    Serial.println(F("// dan BUKAN 8190, jadi gate 'nilai >= 8000' saja tidak cukup:"));
    Serial.println(F("//     if (mm >= 8000 || mm > LIDAR_MAX_CM*10) -> anggap tak ada target"));
    Serial.println(F("// catatan: tidak ada lidar diagonal depan. Halangan serong hanya"));
    Serial.println(F("// terbaca lewat FRONT + sepasang sensor samping."));
    Serial.println();
}

// ============================================================ bantuan
static void printHelp() {
    Serial.println(F("\n========== MAP_LIDAR — pemetaan & kalibrasi ==========" ));
    Serial.print  (F("Bus: ")); Serial.print(BUSNAME[busIdx]);
    Serial.print  (F(" | clock ")); Serial.print(i2cClock / 1000); Serial.print(F(" kHz"));
    Serial.print  (F(" | percaya <= ")); Serial.print(trustMm / 10); Serial.print(F(" cm"));
    Serial.print  (F(" | profil ")); Serial.print(PROFIL[profil].nama);
    Serial.print  (F(" | ")); Serial.print(modeKontinu ? F("kontinu") : F("single-shot"));
    Serial.println(smoothOn ? F(" | filter ON") : F(" | filter OFF"));
    Serial.println(F(" PEMETAAN"));
    Serial.println(F("  w        wizard: petakan 6 posisi lewat lambaian tangan"));
    Serial.println(F("  n<p>     ulang wizard untuk SATU posisi (mis. n3)"));
    Serial.println(F("  m<p><c>  set manual posisi p ke channel c (mis. m05)"));
    Serial.println(F("  u<p>     lepas pemetaan posisi p"));
    Serial.println(F("  s        tabel pemetaan sekarang"));
    Serial.println(F(" MEMBACA"));
    Serial.println(F("  a        denah robot + jarak, terus-menerus"));
    Serial.println(F("  l        satu baris per sweep, terus-menerus"));
    Serial.println(F("  r<n>     baca satu CHANNEL mentah terus-menerus"));
    Serial.println(F("  x        berhenti membaca"));
    Serial.println(F(" KESTABILAN  (obati dulu kalau angkanya meloncat)"));
    Serial.println(F("  q        uji kestabilan 50 sampel di posisi fokus (ukur sebarannya)"));
    Serial.println(F("  t<cm>    batas percaya; di atasnya ditampilkan '>>>' (mis. t100)"));
    Serial.println(F("  P0/P1/P2 profil: CEPAT / SEIMBANG / TENANG (langsung init ulang)"));
    Serial.println(F("  o        ganti single-shot <-> kontinu (single-shot = tanpa crosstalk)"));
    Serial.println(F("  F        filter median berjalan untuk tampilan a/l on-off"));
    Serial.println(F(" KALIBRASI JARAK"));
    Serial.println(F("  f<p>     pilih posisi fokus (mis. f0)"));
    Serial.println(F("  c<cm>    ukur pada jarak sebenarnya <cm>, set offset (mis. c50)"));
    Serial.println(F("  T        tabel titik kalibrasi sesi ini"));
    Serial.println(F("  z        nolkan semua offset"));
    Serial.println(F(" LAIN"));
    Serial.println(F("  d        cari bus lidar + init"));
    Serial.println(F("  i        init ulang lidar"));
    Serial.println(F("  k1/k4    clock I2C 100/400 kHz (lalu 'i')"));
    Serial.println(F("  g        cetak kode untuk lidar_map.h + config.h"));
    Serial.println(F("  e/E/X    simpan / muat / hapus EEPROM"));
    Serial.println(F("  h        bantuan"));
    Serial.println(F(" posisi p: 0=FRONT 1=RIGHT_FRONT 2=RIGHT_REAR 3=BACK 4=LEFT_REAR 5=LEFT_FRONT"));
    Serial.println(F("======================================================\n"));
}

// ============================================================== setup
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) { }

    for (uint8_t c = 0; c < NUM_MUX_CH; c++) chReady[c] = false;
    loadDefaults();

    Serial.println(F("\n\nMAP_LIDAR — pemetaan posisi 6x VL53L0X (Teensy 4.1)"));
    Serial.println(F("Tata letak: DEPAN, BELAKANG, 2 KIRI, 2 KANAN (samping di celah kaki)."));

    eeLoad(true);
    autoDetect();
    initAll();
    printTabel();
    printHelp();
}

// =========================================================== perintah
static void handleCmd(char* s) {
    while (*s == ' ') s++;
    char     c1 = *s;
    uint8_t  d1 = (uint8_t)(s[1] - '0');
    uint8_t  d2 = (uint8_t)(s[2] - '0');
    int      num = atoi(s + 1);

    switch (c1) {
        case 'h': case '?': printHelp(); break;
        case 'd': mode = IDLE; autoDetect(); initAll(); printTabel(); break;
        case 'i': mode = IDLE; initAll(); break;
        case 's': mode = IDLE; printTabel(); break;
        case 'g': mode = IDLE; cetakKode(); break;
        case 'x': mode = IDLE; muxSelect(8); Serial.println(F("berhenti.")); break;

        case 'a': mode = READ_DENAH; Serial.println(F("denah (x + Enter untuk berhenti)")); break;
        case 'l': mode = READ_LINE;  Serial.println(F("baris (x + Enter untuk berhenti)"));  break;
        case 'r':
            if (d1 < NUM_MUX_CH) { modeCh = d1; mode = READ_RAW; }
            else Serial.println(F("channel 0..7"));
            break;

        case 'w': mode = IDLE; wizardSemua(); break;
        case 'n': {
            mode = IDLE;
            if (d1 >= NUM_LIDAR) { Serial.println(F("posisi 0..5")); break; }
            int16_t base[NUM_MUX_CH];
            Serial.println(F("Ambil baseline (jangan ada benda dekat sensor)..."));
            delay(1000);
            ambilBaseline(base);
            posCh[d1] = -1;
            wizardSatu(d1, base);
            printTabel();
            break;
        }
        case 'm':
            mode = IDLE;
            if (d1 >= NUM_LIDAR || d2 >= NUM_MUX_CH) { Serial.println(F("format: m<posisi 0-5><channel 0-7>, mis. m05")); break; }
            assign(d1, (int8_t)d2);
            printTabel();
            break;
        case 'u':
            mode = IDLE;
            if (d1 >= NUM_LIDAR) { Serial.println(F("posisi 0..5")); break; }
            posCh[d1] = -1;
            printTabel();
            break;

        case 'f':
            if (d1 >= NUM_LIDAR) { Serial.println(F("posisi 0..5")); break; }
            focus = d1;
            Serial.print(F("fokus = ")); Serial.println(LIDAR_NAME[focus]);
            break;
        case 'c': mode = IDLE; kalibrasi(num); break;
        case 'T': mode = IDLE; printCalLog(); break;
        case 'z':
            for (uint8_t p = 0; p < NUM_LIDAR; p++) posOff[p] = 0;
            Serial.println(F("semua offset dinolkan."));
            break;

        case 'q': mode = IDLE; ujiStabil(); break;
        case 't':
            if (num < 10 || num > 200) { Serial.println(F("batas percaya 10..200 cm, mis. t100")); break; }
            trustMm = (int16_t)(num * 10);
            histReset();
            Serial.print(F("batas percaya = ")); Serial.print(num);
            Serial.println(F(" cm. Di atas itu ditampilkan '>>>', bukan angka."));
            Serial.println(F("Naikkan HANYA kalau 'q' di jarak itu memang menunjukkan sebaran kecil."));
            break;
        case 'P':
            if (d1 > 2) { Serial.println(F("profil 0=CEPAT 1=SEIMBANG 2=TENANG")); break; }
            profil = d1;
            Serial.print(F("profil = ")); Serial.print(PROFIL[profil].nama);
            Serial.print(F(" — ")); Serial.println(PROFIL[profil].catatan);
            mode = IDLE; initAll();
            break;
        case 'o':
            modeKontinu = !modeKontinu;
            Serial.println(modeKontinu
                ? F("mode KONTINU — cepat, tapi 6 laser menyala bersamaan (crosstalk).")
                : F("mode SINGLE-SHOT — hanya 1 lidar menembak, tanpa crosstalk."));
            mode = IDLE; initAll();
            break;
        case 'F':
            smoothOn = !smoothOn;
            histReset();
            Serial.println(smoothOn ? F("filter median berjalan: AKTIF (tampilan a/l)")
                                    : F("filter: MATI — tampilan a/l jadi nilai mentah"));
            break;

        case 'k':
            i2cClock = (d1 == 1) ? 100000UL : 400000UL;
            bus->setClock(i2cClock);
            Serial.print(F("clock I2C = ")); Serial.print(i2cClock / 1000);
            Serial.println(F(" kHz — jalankan 'i'."));
            break;

        case 'e': eeSave(); break;
        case 'E': if (eeLoad(true)) printTabel(); break;
        case 'X': eeErase(); break;

        case 0: break;
        default: Serial.println(F("perintah tidak dikenal, ketik 'h'"));
    }
}

// ================================================================ loop
void loop() {
    static char    buf[16];
    static uint8_t len = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            buf[len] = 0;
            if (len) handleCmd(buf);
            else if (mode != IDLE) { mode = IDLE; muxSelect(8); Serial.println(F("berhenti.")); }
            len = 0;
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        }
    }

    if (mode != IDLE && millis() - lastPrint >= 250) {
        lastPrint = millis();
        if      (mode == READ_DENAH) printDenah();
        else if (mode == READ_LINE)  printBaris();
        else                         printRaw(modeCh);
    }
}
