/* =====================================================================
   TES_LIDAR — alat diagnosa 6x TOF200C (VL53L0X) di belakang mux TCA9548A
   Target: Teensy 4.1 (Wire=18/19, Wire1=17/16, Wire2=25/24)
   Library: "VL53L0X" by Pololu  (Library Manager)

   CATATAN PENTING
   Modul TOF200C memakai chip **VL53L0X**, bukan VL53L1X. Keduanya sama-sama
   ber-alamat 0x29 sehingga scanner I2C tidak bisa membedakannya — itulah sebab
   init dengan library VL53L1X gagal (kode 1) walau 0x29 menjawab ACK.
   Sketsa ini memverifikasi chip lewat register ID:
     VL53L0X : reg 8-bit  0xC0/0xC1/0xC2 = 0xEE / 0xAA / 0x10
     VL53L1X : reg 16-bit 0x010F/0x0110  = 0xEA / 0xCC
   Jangkauan VL53L0X maksimum ~200 cm (long range), bukan 400 cm.

   Serial Monitor 115200, line ending Newline. Ketik 'h' untuk bantuan.
   ===================================================================== */

#include <Wire.h>
#include <VL53L0X.h>

// ---------------------------------------------------------------- knob
#define MUX_ADDR        0x70      // alamat TCA9548A
#define VL53_ADDR       0x29      // alamat default VL53L0X/VL53L1X
#define NUM_CH          8         // channel mux
#define MUX_SETTLE_MS   2         // jeda setelah ganti channel (mode scan)
#define INIT_RETRY      3         // percobaan init per sensor
#define TIMING_BUDGET   20000     // us, 20 ms (default ST = 33 ms)
#define IO_TIMEOUT_MS   200       // batas tunggu I/O library
#define MAX_VALID_MM    2000      // di atas ini dianggap tak terpakai (VL53L0X ~2 m)

uint32_t i2cClock  = 400000;      // ubah runtime lewat perintah 'k'
bool     longRange = true;        // TOF200C = mode long range (~2 m)

// Arti tiap channel (samakan dengan config.h firmware bila sudah fix)
const char* CH_NAME[NUM_CH] = {
    "FRONT", "FRONT_R", "RIGHT", "BACK", "LEFT", "FRONT_L", "-", "-"
};

// ------------------------------------------------------------ bus & obj
// Satu objek per CHANNEL: Pololu VL53L0X menyimpan stop_variable hasil init
// milik sensor itu, jadi objek tidak boleh dipakai bergantian antar sensor.
VL53L0X     sens[NUM_CH];
bool        chReady[NUM_CH];

TwoWire*    BUS[3]     = { &Wire, &Wire1, &Wire2 };
const char* BUSNAME[3] = { "Wire  (SDA18/SCL19)",
                           "Wire1 (SDA17/SCL16)",
                           "Wire2 (SDA25/SCL24)" };
uint8_t     busIdx = 0;
TwoWire*    bus    = &Wire;

enum Mode { IDLE, READ_ALL, READ_ONE };
Mode     mode      = IDLE;
uint8_t  modeCh    = 0;
uint32_t lastPrint = 0;

// ------------------------------------------------------------ util I2C
static void printHex2(int v) {
    if (v < 0) { Serial.print(F("--")); return; }
    if (v < 16) Serial.print('0');
    Serial.print(v, HEX);
}

static bool ping(uint8_t addr) {
    bus->beginTransmission(addr);
    return bus->endTransmission() == 0;
}

// pilih channel mux; ch > 7 = matikan SEMUA channel
static bool muxSelect(uint8_t ch) {
    bus->beginTransmission(MUX_ADDR);
    bus->write(ch > 7 ? 0x00 : (uint8_t)(1 << ch));
    return bus->endTransmission() == 0;
}

// TCA9548A bisa dibaca balik -> bukti chip asli, bukan alamat hantu
static int muxReadback() {
    if (bus->requestFrom((uint8_t)MUX_ADDR, (uint8_t)1) != 1) return -1;
    return bus->read();
}

// register dengan alamat 8-bit (gaya VL53L0X)
static int rdReg8(uint8_t addr, uint8_t reg) {
    bus->beginTransmission(addr);
    bus->write(reg);
    if (bus->endTransmission(false) != 0) return -1;
    if (bus->requestFrom(addr, (uint8_t)1) != 1) return -1;
    return bus->read();
}

// register dengan alamat 16-bit (gaya VL53L1X)
static int rdReg16a(uint8_t addr, uint16_t reg) {
    bus->beginTransmission(addr);
    bus->write((uint8_t)(reg >> 8));
    bus->write((uint8_t)(reg & 0xFF));
    if (bus->endTransmission(false) != 0) return -1;
    if (bus->requestFrom(addr, (uint8_t)1) != 1) return -1;
    return bus->read();
}

// 0 = tak dikenal, 1 = VL53L0X, 2 = VL53L1X
static int8_t identify() {
    if (rdReg8(VL53_ADDR, 0xC0) == 0xEE && rdReg8(VL53_ADDR, 0xC2) == 0x10) return 1;
    if (rdReg16a(VL53_ADDR, 0x010F) == 0xEA && rdReg16a(VL53_ADDR, 0x0110) == 0xCC) return 2;
    return 0;
}

static void useBus(uint8_t i) {
    busIdx = i;
    bus    = BUS[i];
    bus->begin();
    bus->setClock(i2cClock);
    for (uint8_t c = 0; c < NUM_CH; c++) chReady[c] = false;
    mode = IDLE;
    Serial.print(F("\n>> Bus aktif: ")); Serial.println(BUSNAME[i]);
    Serial.println(F("   (channel mux di-reset, lidar perlu 'i' lagi)"));
}

// ------------------------------------------------------------ diagnosa
static void scanBus(bool verbose = true) {
    if (verbose) { Serial.print(F("\n--- Scan alamat ")); Serial.print(BUSNAME[busIdx]); Serial.println(F(" ---")); }
    uint8_t n = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (ping(a)) {
            n++;
            if (verbose) {
                Serial.print(F("  0x")); printHex2(a);
                if (a == MUX_ADDR)  Serial.print(F("  <- kandidat TCA9548A"));
                if (a == VL53_ADDR) Serial.print(F("  <- ToF (LANGSUNG di bus, TIDAK lewat mux!)"));
                if (a == 0x40 || a == 0x41) Serial.print(F("  <- PCA9685 (driver servo)"));
                Serial.println();
            }
        }
    }
    if (verbose) { Serial.print(F("  total: ")); Serial.println(n); }
}

// Uji apakah 0x70 benar TCA9548A: tulis pola, baca balik.
static bool testMux(bool verbose = true) {
    if (verbose) Serial.println(F("\n--- Uji mux TCA9548A @0x70 ---"));
    if (!ping(MUX_ADDR)) {
        if (verbose) Serial.println(F("  TIDAK ada device di 0x70 pada bus ini."));
        return false;
    }
    const uint8_t pola[] = { 0x00, 0x01, 0x08, 0x20 };
    bool ok = true;
    for (uint8_t i = 0; i < 4; i++) {
        bus->beginTransmission(MUX_ADDR);
        bus->write(pola[i]);
        uint8_t err = bus->endTransmission();
        int rb = muxReadback();
        if (verbose) {
            Serial.print(F("  tulis 0x")); printHex2(pola[i]);
            Serial.print(F(" -> err=")); Serial.print(err);
            Serial.print(F(" baca=0x")); printHex2(rb); Serial.println();
        }
        if (err != 0 || rb != (int)pola[i]) ok = false;
    }
    muxSelect(8);
    if (verbose) {
        Serial.println(ok ? F("  HASIL: TCA9548A ASLI (tulis-baca cocok).")
                          : F("  HASIL: BUKAN TCA9548A / kabel bermasalah "
                              "(baca-balik tidak cocok -> alamat hantu)."));
    }
    return ok;
}

// Scan tiap channel + identifikasi chip. Sekaligus deteksi "hantu".
static uint8_t scanChannels(bool verbose = true) {
    if (verbose) Serial.println(F("\n--- Scan channel mux ---"));

    muxSelect(8); delay(MUX_SETTLE_MS);
    if (ping(VL53_ADDR) && verbose)
        Serial.println(F("  !! 0x29 terlihat saat SEMUA channel OFF -> "
                         "ada sensor tersambung langsung ke bus utama."));

    uint8_t found = 0;
    for (uint8_t c = 0; c < NUM_CH; c++) {
        if (!muxSelect(c)) {
            if (verbose) { Serial.print(F("  ch")); Serial.print(c); Serial.println(F(": gagal pilih channel")); }
            continue;
        }
        delay(MUX_SETTLE_MS);
        bool ada = ping(VL53_ADDR);
        if (verbose) {
            Serial.print(F("  ch")); Serial.print(c);
            Serial.print(F(" [")); Serial.print(CH_NAME[c]); Serial.print(F("] "));
        }
        if (ada) {
            found++;
            int8_t id = identify();
            if (verbose) {
                Serial.print(F("0x29 ADA  ID: C0=0x")); printHex2(rdReg8(VL53_ADDR, 0xC0));
                Serial.print(F(" C2=0x")); printHex2(rdReg8(VL53_ADDR, 0xC2));
                if      (id == 1) Serial.println(F("  -> VL53L0X (TOF200C) OK"));
                else if (id == 2) Serial.println(F("  -> VL53L1X (chip berbeda!)"));
                else              Serial.println(F("  -> ID tak dikenal"));
            }
        } else if (verbose) {
            Serial.println(F("kosong"));
        }
        if (verbose && ada) {   // device lain yang nyasar di channel ini
            for (uint8_t a = 0x08; a < 0x78; a++) {
                if (a == VL53_ADDR || a == MUX_ADDR) continue;
                if (ping(a)) { Serial.print(F("        + device lain 0x")); printHex2(a); Serial.println(); }
            }
        }
    }
    muxSelect(8);
    if (verbose) { Serial.print(F("  total sensor terdeteksi: ")); Serial.println(found); }
    return found;
}

static void autoDetect() {
    Serial.println(F("\n========== DETEKSI OTOMATIS 3 BUS =========="));
    uint8_t simpan = busIdx;
    int best = -1, bestN = 0;
    for (uint8_t i = 0; i < 3; i++) {
        busIdx = i; bus = BUS[i];
        bus->begin(); bus->setClock(i2cClock);
        Serial.print(F("\n[")); Serial.print(BUSNAME[i]); Serial.println(F("]"));
        scanBus(true);
        bool muxOk = testMux(true);
        uint8_t n = muxOk ? scanChannels(true) : 0;
        if (muxOk && n > bestN) { bestN = n; best = i; }
    }
    Serial.println(F("\n---------- KESIMPULAN ----------"));
    if (best < 0) {
        Serial.println(F("Tidak ada bus dengan TCA9548A asli. Cek SDA/SCL, pull-up, "
                         "pin A0/A1/A2 dan RESET mux (harus HIGH)."));
    } else {
        Serial.print(F("Bus lidar = ")); Serial.print(BUSNAME[best]);
        Serial.print(F("  dengan ")); Serial.print(bestN); Serial.println(F(" sensor."));
    }
    useBus(best < 0 ? simpan : (uint8_t)best);
}

// Probe mendalam satu channel — dipakai saat init gagal.
static void probe(uint8_t ch) {
    Serial.print(F("\n--- Probe ch")); Serial.print(ch); Serial.println(F(" ---"));
    if (!muxSelect(ch)) { Serial.println(F("  gagal pilih channel mux")); return; }
    delay(MUX_SETTLE_MS);
    if (!ping(VL53_ADDR)) { Serial.println(F("  0x29 tidak menjawab")); muxSelect(8); return; }
    Serial.println(F("  0x29 menjawab (ACK)"));

    Serial.print(F("  reg8  0xC0/0xC1/0xC2 = 0x")); printHex2(rdReg8(VL53_ADDR, 0xC0));
    Serial.print(F(" 0x")); printHex2(rdReg8(VL53_ADDR, 0xC1));
    Serial.print(F(" 0x")); printHex2(rdReg8(VL53_ADDR, 0xC2));
    Serial.println(F("   (VL53L0X = EE AA 10)"));

    Serial.print(F("  reg16 0x010F/0x0110  = 0x")); printHex2(rdReg16a(VL53_ADDR, 0x010F));
    Serial.print(F(" 0x")); printHex2(rdReg16a(VL53_ADDR, 0x0110));
    Serial.println(F("        (VL53L1X = EA CC)"));

    int8_t id = identify();
    Serial.println(id == 1 ? F("  VONIS: VL53L0X — pakai library Pololu VL53L0X (sketsa ini).")
                 : id == 2 ? F("  VONIS: VL53L1X — sensor beda dari yang lain!")
                           : F("  VONIS: ID tak dikenal — komunikasi kacau, turunkan clock ('k1')."));

    // uji tulis 1 byte ke register tidak berbahaya (SYSRANGE_START = 0x00)
    bus->beginTransmission(VL53_ADDR);
    bus->write((uint8_t)0x00); bus->write((uint8_t)0x00);
    uint8_t err = bus->endTransmission();
    Serial.print(F("  uji tulis reg 0x00 -> err=")); Serial.print(err);
    Serial.println(err == 0 ? F(" (ok)") : F(" (GAGAL: 2=NACK alamat, 3=NACK data, 4=lain)"));
    muxSelect(8);
}

// ---------------------------------------------------------------- init
static void initAll() {
    Serial.println(F("\n--- Init semua lidar (VL53L0X) ---"));
    Serial.print(F("  clock=")); Serial.print(i2cClock / 1000); Serial.print(F(" kHz, mode="));
    Serial.println(longRange ? F("LONG RANGE (~2 m)") : F("standar (~1,2 m)"));

    uint8_t ok = 0;
    for (uint8_t c = 0; c < NUM_CH; c++) {
        chReady[c] = false;
        if (!muxSelect(c)) continue;
        delay(MUX_SETTLE_MS);
        if (!ping(VL53_ADDR)) continue;

        Serial.print(F("  ch")); Serial.print(c);
        Serial.print(F(" [")); Serial.print(CH_NAME[c]); Serial.print(F("] "));

        int8_t id = identify();
        if (id != 1) {
            Serial.println(id == 2 ? F("VL53L1X — dilewati (sketsa ini untuk VL53L0X)")
                                   : F("ID tak dikenal — dilewati (jalankan 'p' untuk detail)"));
            continue;
        }

        sens[c].setBus(bus);
        sens[c].setTimeout(IO_TIMEOUT_MS);
        bool good = false;
        for (uint8_t r = 0; r < INIT_RETRY && !good; r++) good = sens[c].init();

        if (!good) {
            Serial.println(F("GAGAL init (I2C tidak stabil? coba 'k1' lalu 'i')"));
            continue;
        }
        if (longRange) {              // konfigurasi long range TOF200C
            sens[c].setSignalRateLimit(0.1);
            sens[c].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,   18);
            sens[c].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
        }
        sens[c].setMeasurementTimingBudget(TIMING_BUDGET);
        sens[c].startContinuous(0);   // 0 = secepat mungkin
        chReady[c] = true; ok++;
        Serial.println(F("OK"));
    }
    muxSelect(8);
    Serial.print(F("  siap: ")); Serial.print(ok); Serial.println(F(" lidar"));
    if (ok == 0) Serial.println(F("  (coba 'd' untuk cari bus, atau 'p<n>' untuk probe satu channel)"));
}

// -------------------------------------------------------------- membaca
// return mm; 0xFFFF = timeout. >= 8000 = tak ada target / di luar jangkauan.
static uint16_t measure(uint8_t ch) {
    muxSelect(ch);
    delayMicroseconds(300);
    return sens[ch].readRangeContinuousMillimeters();
}

static void readOneVerbose(uint8_t ch) {
    if (!chReady[ch]) {
        Serial.print(F("ch")); Serial.print(ch);
        Serial.println(F(" belum di-init — jalankan 'i', atau 'p<n>' untuk probe."));
        return;
    }
    uint32_t t0 = micros();
    uint16_t mm = measure(ch);
    uint32_t dt = micros() - t0;
    bool to = sens[ch].timeoutOccurred();

    Serial.print(F("ch")); Serial.print(ch);
    Serial.print(F(" [")); Serial.print(CH_NAME[ch]); Serial.print(F("] "));
    if (to || mm == 0xFFFF)      Serial.print(F("TIMEOUT (sensor diam / kabel)"));
    else if (mm >= 8000)         Serial.print(F("tak ada target (di luar jangkauan)"));
    else {
        Serial.print(mm); Serial.print(F(" mm = "));
        Serial.print(mm / 10); Serial.print(F(" cm"));
        if (mm > MAX_VALID_MM) Serial.print(F("  (> 2 m, ragukan)"));
    }
    Serial.print(F(" | ")); Serial.print(dt / 1000.0f, 1); Serial.println(F(" ms"));
    muxSelect(8);
}

static void sweepAll() {
    uint32_t t0 = millis();
    Serial.print(F("  "));
    for (uint8_t c = 0; c < NUM_CH; c++) {
        if (!chReady[c]) continue;
        uint16_t mm = measure(c);
        Serial.print(CH_NAME[c]); Serial.print('=');
        if (sens[c].timeoutOccurred() || mm == 0xFFFF) Serial.print(F("  TO"));
        else if (mm >= 8000)                           Serial.print(F("  --"));
        else                                           Serial.print(mm / 10);
        Serial.print(F("cm  "));
    }
    muxSelect(8);
    Serial.print(F("(sweep ")); Serial.print(millis() - t0); Serial.println(F(" ms)"));
}

// -------------------------------------------------------------- bantuan
static void printHelp() {
    Serial.println(F("\n============ TES LIDAR VL53L0X / TOF200C ============"));
    Serial.print  (F("Bus: ")); Serial.print(BUSNAME[busIdx]);
    Serial.print  (F(" | clock ")); Serial.print(i2cClock / 1000); Serial.print(F(" kHz | "));
    Serial.println(longRange ? F("long range") : F("standar"));
    Serial.println(F("  d      deteksi otomatis 3 bus (scan + uji mux + ID chip)"));
    Serial.println(F("  b0/1/2 pilih bus Wire / Wire1 / Wire2"));
    Serial.println(F("  s      scan alamat I2C bus aktif"));
    Serial.println(F("  m      uji mux 0x70 + scan & identifikasi tiap channel"));
    Serial.println(F("  i      init semua lidar"));
    Serial.println(F("  a      baca SEMUA lidar terus-menerus"));
    Serial.println(F("  r<n>   baca SATU channel terus-menerus (mis. r2)"));
    Serial.println(F("  t<n>   baca SATU channel sekali, detail (mis. t0)"));
    Serial.println(F("  p<n>   probe mendalam satu channel (dump register ID)"));
    Serial.println(F("  c<n>   pilih channel mux manual, lalu 's'"));
    Serial.println(F("  k1/k4  clock I2C 100 kHz / 400 kHz (lalu 'i' lagi)"));
    Serial.println(F("  l      ganti long range <-> standar (lalu 'i' lagi)"));
    Serial.println(F("  x      berhenti membaca      h  bantuan"));
    Serial.println(F("=====================================================\n"));
}

// ---------------------------------------------------------------- setup
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) { }
    for (uint8_t c = 0; c < NUM_CH; c++) chReady[c] = false;

    Serial.println(F("\n\nTES_LIDAR — TOF200C / VL53L0X x6 @ TCA9548A (Teensy 4.1)"));
    useBus(0);
    autoDetect();
    initAll();
    printHelp();
}

// -------------------------------------------------------------- perintah
static void handleCmd(char* s) {
    while (*s == ' ') s++;
    uint8_t n = (uint8_t)(s[1] - '0');
    switch (*s) {
        case 'h': case '?': printHelp(); break;
        case 'd': mode = IDLE; autoDetect(); initAll(); break;
        case 's': mode = IDLE; scanBus(); break;
        case 'm': mode = IDLE; testMux(); scanChannels(); break;
        case 'i': mode = IDLE; initAll(); break;
        case 'x': mode = IDLE; muxSelect(8); Serial.println(F("berhenti.")); break;
        case 'a': mode = READ_ALL; Serial.println(F("baca semua (x + Enter untuk berhenti)")); break;
        case 'l':
            longRange = !longRange;
            Serial.print(F("mode: ")); Serial.println(longRange ? F("LONG RANGE") : F("standar"));
            Serial.println(F("jalankan 'i' untuk menerapkan."));
            break;
        case 'k':
            i2cClock = (n == 1) ? 100000UL : 400000UL;
            bus->setClock(i2cClock);
            Serial.print(F("clock I2C = ")); Serial.print(i2cClock / 1000); Serial.println(F(" kHz"));
            Serial.println(F("jalankan 'i' untuk init ulang."));
            break;
        case 'b': if (n < 3) useBus(n); else Serial.println(F("pakai b0/b1/b2")); break;
        case 'r': if (n < NUM_CH) { modeCh = n; mode = READ_ONE; } else Serial.println(F("channel 0..7")); break;
        case 't': mode = IDLE; if (n < NUM_CH) readOneVerbose(n); else Serial.println(F("channel 0..7")); break;
        case 'p': mode = IDLE; if (n < NUM_CH) probe(n); else Serial.println(F("channel 0..7")); break;
        case 'c':
            mode = IDLE;
            if (n < NUM_CH) {
                Serial.print(F("mux -> ch")); Serial.print(n);
                Serial.println(muxSelect(n) ? F(" (ok)") : F(" (GAGAL)"));
                Serial.print(F("baca balik reg mux = 0x")); printHex2(muxReadback()); Serial.println();
            } else { muxSelect(8); Serial.println(F("semua channel OFF")); }
            break;
        case 0: break;
        default: Serial.println(F("perintah tidak dikenal, ketik 'h'"));
    }
}

// ----------------------------------------------------------------- loop
void loop() {
    static char buf[16];
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

    if (mode != IDLE && millis() - lastPrint >= 200) {
        lastPrint = millis();
        if (mode == READ_ALL) sweepAll();
        else                  readOneVerbose(modeCh);
    }
}
