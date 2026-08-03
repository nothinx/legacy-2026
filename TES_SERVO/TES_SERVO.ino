/* =====================================================================
   TES_SERVO — pemetaan servo satu per satu (Teensy 4.1 + 2x PCA9685)

   Alur pakai:
     1. 'V'      pastikan kedua driver terbaca (bukan alamat hantu)
     2. 'c0'     pilih channel, 'W' goyangkan -> lihat servo MANA yang gerak
     3. '=5'     kaitkan channel itu ke slot logis (mis. 5 = K1_TIBIA)
     4. 'n'      lanjut channel berikutnya, ulangi sampai 24 slot terisi
     5. 'S'      simpan ke EEPROM, 'D' cetak kode untuk config.h

   Aman: hanya SATU channel yang diberi sinyal pada satu waktu; channel lain
   dimatikan (PWM off) sehingga servo bebas dan tidak saling melawan.
   ===================================================================== */

#include <Adafruit_PWMServoDriver.h>
#include "servo_map.h"

// ------------------------------------------------------------ knob
#define WIGGLE_US     120     // amplitudo goyang (us). 120us ~ 11 derajat
#define WIGGLE_MS     260     // periode setengah goyang
#define NUDGE_US      25      // langkah '+' / '-'
#define HOME_STAGGER  120     // ms jeda antar servo saat 'H' (batasi arus)

// PCA9685 register mentah (untuk verifikasi & ALL-CALL)
#define PCA_MODE1     0x00
#define PCA_PRESCALE  0xFE

Adafruit_PWMServoDriver drv0(ADDR_DRV0, BUS_DRV0);
Adafruit_PWMServoDriver drv1(ADDR_DRV1, BUS_DRV1);

ServoMap  map_;
uint8_t   curDrv  = 0;
uint8_t   curCh   = 0;
uint16_t  curUs   = PULSE_MID;
int8_t    prevDrv = -1, prevCh = -1;
bool      wiggling = false;

// ------------------------------------------------------------ util
static TwoWire* busOf(uint8_t d) { return d == 0 ? &BUS_DRV0 : &BUS_DRV1; }
static uint8_t  addrOf(uint8_t d) { return d == 0 ? ADDR_DRV0 : ADDR_DRV1; }
static Adafruit_PWMServoDriver& drvOf(uint8_t d) { return d == 0 ? drv0 : drv1; }

static int pcaRead(uint8_t d, uint8_t reg) {
    TwoWire* b = busOf(d);
    b->beginTransmission(addrOf(d));
    b->write(reg);
    if (b->endTransmission(false) != 0) return -1;
    if (b->requestFrom(addrOf(d), (uint8_t)1) != 1) return -1;
    return b->read();
}

static bool pcaWrite(uint8_t d, uint8_t reg, uint8_t val) {
    TwoWire* b = busOf(d);
    b->beginTransmission(addrOf(d));
    b->write(reg); b->write(val);
    return b->endTransmission() == 0;
}

// matikan output satu channel (servo jadi bebas)
static void release(int8_t d, int8_t ch) {
    if (d < 0 || ch < 0) return;
    drvOf(d).setPWM(ch, 0, 0);
}

static void releaseAll() {
    for (uint8_t c = 0; c < 16; c++) { drv0.setPWM(c, 0, 0); drv1.setPWM(c, 0, 0); }
    prevDrv = prevCh = -1;
    Serial.println(F("semua output PWM dimatikan (servo bebas)."));
}

// slot logis yang saat ini terkait ke (curDrv,curCh); -1 bila belum ada
static int8_t slotOfCurrent() {
    for (uint8_t i = 0; i < SM_SLOTS; i++)
        if (map_.drv[i] == (int8_t)curDrv && map_.ch[i] == (int8_t)curCh) return i;
    return -1;
}

static void showCurrent() {
    Serial.print(F("\n>> driver ")); Serial.print(curDrv);
    Serial.print(F(" (0x")); Serial.print(addrOf(curDrv), HEX);
    Serial.print(F(" @ ")); Serial.print(curDrv == 0 ? F("Wire2") : F("Wire1"));
    Serial.print(F(")  channel ")); Serial.print(curCh);
    Serial.print(F("  pulse ")); Serial.print(curUs); Serial.print(F(" us"));
    int8_t s = slotOfCurrent();
    if (s >= 0) { Serial.print(F("  -> slot ")); Serial.print(s);
                  Serial.print(' '); Serial.print(SLOT_NAME[s]); }
    else Serial.print(F("  -> (belum di-assign)"));
    Serial.println();
}

static void driveCurrent(uint16_t us) {
    if (us < PULSE_MIN) us = PULSE_MIN;
    if (us > PULSE_MAX) us = PULSE_MAX;
    curUs = us;
    drvOf(curDrv).writeMicroseconds(curCh, curUs);
}

static void selectCh(uint8_t d, uint8_t ch) {
    if (prevDrv != (int8_t)d || prevCh != (int8_t)ch) release(prevDrv, prevCh);
    curDrv = d; curCh = ch;
    prevDrv = d; prevCh = ch;
    curUs = PULSE_MID;
    driveCurrent(curUs);
    showCurrent();
}

// ------------------------------------------------------------ aksi
static void wiggle(bool terus) {
    Serial.println(terus ? F("goyang terus — tekan Enter untuk berhenti")
                         : F("goyang 6x..."));
    uint16_t base = curUs;
    uint8_t  n = 0;
    while (true) {
        driveCurrent(base + WIGGLE_US); delay(WIGGLE_MS);
        driveCurrent(base - WIGGLE_US); delay(WIGGLE_MS);
        n++;
        if (!terus && n >= 3) break;
        if (Serial.available()) { while (Serial.available()) Serial.read(); break; }
    }
    driveCurrent(base);
    Serial.println(F("selesai."));
}

static void assign(uint8_t slot) {
    if (slot >= SM_SLOTS) { Serial.println(F("slot 0..23")); return; }
    // lepaskan slot lain yang memakai pin yang sama (cegah duplikat)
    for (uint8_t i = 0; i < SM_SLOTS; i++)
        if (i != slot && map_.drv[i] == (int8_t)curDrv && map_.ch[i] == (int8_t)curCh) {
            Serial.print(F("  (slot ")); Serial.print(i); Serial.print(' ');
            Serial.print(SLOT_NAME[i]); Serial.println(F(" dilepas — pin dipakai ulang)"));
            map_.drv[i] = -1; map_.ch[i] = -1;
        }
    map_.drv[slot] = curDrv;
    map_.ch[slot]  = curCh;
    Serial.print(F("slot ")); Serial.print(slot); Serial.print(' ');
    Serial.print(SLOT_NAME[slot]);
    Serial.print(F(" <- driver ")); Serial.print(curDrv);
    Serial.print(F(" ch ")); Serial.println(curCh);
}

// tukar seluruh isi dua slot (pin + invert + trim)
static void swapSlot(uint8_t ia, uint8_t ib) {
    int8_t  td = map_.drv[ia];    map_.drv[ia]    = map_.drv[ib];    map_.drv[ib]    = td;
    int8_t  tc = map_.ch[ia];     map_.ch[ia]     = map_.ch[ib];     map_.ch[ib]     = tc;
    uint8_t ti = map_.invert[ia]; map_.invert[ia] = map_.invert[ib]; map_.invert[ib] = ti;
    int16_t tt = map_.trim[ia];   map_.trim[ia]   = map_.trim[ib];   map_.trim[ib]   = tt;
}

static void swapLegs(uint8_t a, uint8_t b) {
    if (a >= SM_LEGS || b >= SM_LEGS || a == b) { Serial.println(F("kaki 0..5, harus beda")); return; }
    for (uint8_t j = 0; j < 3; j++) swapSlot(a * 3 + j, b * 3 + j);
    Serial.print(F("kaki ")); Serial.print(a); Serial.print(F(" <-> ")); Serial.println(b);
}

// Kaki ternyata terbalik kiri-kanan: K0<->K5, K1<->K4, K2<->K3, lengan R<->L.
static void mirrorMap() {
    Serial.println(F("\ntukar sisi kiri <-> kanan:"));
    swapLegs(0, 5); swapLegs(1, 4); swapLegs(2, 3);
    for (uint8_t j = 0; j < 3; j++) swapSlot(18 + j, 21 + j);
    Serial.println(F("lengan kanan <-> kiri"));
    Serial.println(F("jangan lupa 'S' untuk menyimpan."));
}

static void listSlots() {
    Serial.println(F("\nslot logis:"));
    for (uint8_t i = 0; i < SM_SLOTS; i++) {
        Serial.print(' '); if (i < 10) Serial.print(' ');
        Serial.print(i); Serial.print(' ');
        Serial.print(SLOT_NAME[i]);
        Serial.print((i % 3 == 2) ? '\n' : '\t');
    }
    Serial.println(F("kaki: 0=Ka-Depan 1=Ka-Tengah 2=Ka-Belakang "
                     "3=Ki-Belakang 4=Ki-Tengah 5=Ki-Depan"));
}

static void verifyDrivers() {
    Serial.println(F("\n--- Verifikasi PCA9685 ---"));
    for (uint8_t d = 0; d < 2; d++) {
        TwoWire* b = busOf(d);
        Serial.print(F("driver ")); Serial.print(d);
        Serial.print(F(" 0x")); Serial.print(addrOf(d), HEX);
        Serial.print(d == 0 ? F(" @Wire2: ") : F(" @Wire1: "));
        b->beginTransmission(addrOf(d));
        if (b->endTransmission() != 0) { Serial.println(F("TIDAK menjawab")); continue; }
        int mode1 = pcaRead(d, PCA_MODE1);
        int pre   = pcaRead(d, PCA_PRESCALE);
        Serial.print(F("ACK  MODE1=0x")); Serial.print(mode1, HEX);
        Serial.print(F(" PRESCALE=")); Serial.print(pre);
        // prescale 50 Hz = round(25e6/(4096*50))-1 = 121
        Serial.println((pre >= 118 && pre <= 124) ? F("  (50 Hz, chip asli)")
                                                  : F("  (prescale tak sesuai 50 Hz?)"));
        if (mode1 >= 0 && (mode1 & 0x01))
            Serial.println(F("       ALLCALL aktif -> driver ini juga menjawab di 0x70 "
                             "(inilah 0x70 di scan Anda, bukan mux)"));
    }
}

static void disableAllCall() {
    Serial.println(F("\n--- Matikan ALL-CALL (0x70) di kedua PCA9685 ---"));
    for (uint8_t d = 0; d < 2; d++) {
        int m1 = pcaRead(d, PCA_MODE1);
        if (m1 < 0) { Serial.print(F("driver ")); Serial.print(d); Serial.println(F(": gagal baca")); continue; }
        pcaWrite(d, PCA_MODE1, (uint8_t)(m1 & ~0x01));
        Serial.print(F("driver ")); Serial.print(d);
        Serial.print(F(": MODE1 0x")); Serial.print(m1, HEX);
        Serial.print(F(" -> 0x")); Serial.println(pcaRead(d, PCA_MODE1), HEX);
    }
    Serial.println(F("Catatan: TIDAK permanen — hilang saat power cycle. Perlu hanya bila "
                     "PCA9685 disatukan sebus dengan TCA9548A (bentrok 0x70)."));
}

static void homeMapped() {
    Serial.println(F("\nsemua slot termapping -> 90 der (bertahap, tahan badan robot!)"));
    release(prevDrv, prevCh); prevDrv = prevCh = -1;
    for (uint8_t i = 0; i < SM_SLOTS; i++) {
        if (map_.drv[i] < 0) continue;
        uint16_t us = smDegToUs(map_, i, 90.0f);
        drvOf(map_.drv[i]).writeMicroseconds(map_.ch[i], us);
        Serial.print(F("  ")); Serial.print(SLOT_NAME[i]);
        Serial.print(F(" -> ")); Serial.print(us); Serial.println(F(" us"));
        delay(HOME_STAGGER);
    }
    Serial.println(F("selesai. (SET_HOME punya versi lebih lengkap)"));
}

// ------------------------------------------------------------ bantuan
static void printHelp() {
    Serial.println(F("\n=============== TES SERVO — PEMETAAN ==============="));
    Serial.println(F("  V        verifikasi kedua PCA9685 (+ cek ALL-CALL 0x70)"));
    Serial.println(F("  d0 / d1  pilih driver 0 (0x40@Wire2) / 1 (0x41@Wire1)"));
    Serial.println(F("  c<n>     pilih channel 0..15 (channel lain dimatikan)"));
    Serial.println(F("  n / b    channel berikutnya / sebelumnya"));
    Serial.println(F("  w / W    goyang 6x / goyang terus  <- cari servo mana yang gerak"));
    Serial.println(F("  + / -    geser pulse 25us       u<us>  set pulse (mis. u1200)"));
    Serial.println(F("  9        kembali ke 1500us (90 der)"));
    Serial.println(F("  =<slot>  kaitkan channel ini ke slot logis (mis. =5)"));
    Serial.println(F("  ?        daftar nama slot"));
    Serial.println(F("  i<slot>  balik arah (invert) slot"));
    Serial.println(F("  t<us>    set trim slot yang sedang terpilih (mis. t-40)"));
    Serial.println(F("  M        tukar sisi kiri<->kanan (K0<->K5, K1<->K4, K2<->K3)"));
    Serial.println(F("  T<a> <b> tukar dua kaki, mis. T0 5"));
    Serial.println(F("  L        tabel mapping        D  cetak kode untuk config.h"));
    Serial.println(F("  S / O    simpan / muat EEPROM       X  kosongkan mapping"));
    Serial.println(F("  F        isi mapping dengan dugaan lama (config.h) sbg titik awal"));
    Serial.println(F("  H        semua slot termapping -> 90 der (bertahap)"));
    Serial.println(F("  R        matikan SEMUA PWM (servo bebas)"));
    Serial.println(F("  A        matikan ALL-CALL 0x70 pada PCA9685"));
    Serial.println(F("  h        bantuan"));
    Serial.println(F("===================================================\n"));
}

// ------------------------------------------------------------ setup
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) { }

    BUS_DRV0.begin(); BUS_DRV0.setClock(SERVO_I2C_CLOCK);
    BUS_DRV1.begin(); BUS_DRV1.setClock(SERVO_I2C_CLOCK);

    drv0.begin(); drv0.setPWMFreq(SERVO_PWM_FREQ);
    drv1.begin(); drv1.setPWMFreq(SERVO_PWM_FREQ);
    for (uint8_t c = 0; c < 16; c++) { drv0.setPWM(c, 0, 0); drv1.setPWM(c, 0, 0); }

    Serial.println(F("\n\nTES_SERVO — pemetaan servo (Teensy 4.1, 2x PCA9685)"));
    if (smLoad(map_)) Serial.println(F("mapping dimuat dari EEPROM."));
    else { smClear(map_); Serial.println(F("EEPROM kosong -> mapping KOSONG "
                                           "('F' untuk mulai dari dugaan lama).")); }
    verifyDrivers();
    printHelp();
    showCurrent();
    Serial.println(F("(belum ada servo yang diberi sinyal — tekan 'c0' untuk mulai)"));
}

// ---------------------------------------------------------- perintah
static void handleCmd(char* s) {
    while (*s == ' ') s++;
    char  c = *s;
    int   v = atoi(s + 1);
    switch (c) {
        case 'h': case '\0': printHelp(); break;
        case '?': listSlots(); break;
        case 'V': verifyDrivers(); break;
        case 'A': disableAllCall(); break;
        case 'L': smPrint(map_); break;
        case 'D': smDumpC(map_); break;
        case 'S': smSave(map_); Serial.println(F("mapping disimpan ke EEPROM.")); break;
        case 'O': if (smLoad(map_)) Serial.println(F("mapping dimuat dari EEPROM."));
                  else              Serial.println(F("EEPROM tidak valid.")); break;
        case 'X': smClear(map_);   Serial.println(F("mapping dikosongkan.")); break;
        case 'M': mirrorMap(); smPrint(map_); break;
        case 'T': {
            char* sp = strchr(s, ' ');
            if (!sp) { Serial.println(F("format: T<a> <b>, mis. T0 5")); break; }
            swapLegs((uint8_t)atoi(s + 1), (uint8_t)atoi(sp + 1));
            break;
        }
        case 'F': smDefaults(map_); Serial.println(F("mapping diisi dugaan lama (config.h). "
                                                    "Tetap verifikasi satu per satu!")); break;
        case 'R': releaseAll(); break;
        case 'H': homeMapped(); break;
        case 'd': if (v == 0 || v == 1) selectCh((uint8_t)v, curCh);
                  else Serial.println(F("pakai d0 / d1")); break;
        case 'c': if (v >= 0 && v < 16) selectCh(curDrv, (uint8_t)v);
                  else Serial.println(F("channel 0..15")); break;
        case 'n': selectCh(curDrv, (curCh + 1) & 0x0F); break;
        case 'b': selectCh(curDrv, (curCh + 15) & 0x0F); break;
        case 'w': wiggle(false); break;
        case 'W': wiggle(true);  break;
        case '9': driveCurrent(PULSE_MID); showCurrent(); break;
        case '+': driveCurrent(curUs + NUDGE_US); showCurrent(); break;
        case '-': driveCurrent(curUs - NUDGE_US); showCurrent(); break;
        case 'u': driveCurrent((uint16_t)v); showCurrent(); break;
        case '=': assign((uint8_t)v); break;
        case 'i':
            if (v >= 0 && v < SM_SLOTS) {
                map_.invert[v] = !map_.invert[v];
                Serial.print(SLOT_NAME[v]); Serial.print(F(" invert = "));
                Serial.println(map_.invert[v]);
            } else Serial.println(F("slot 0..23"));
            break;
        case 't': {
            int8_t sl = slotOfCurrent();
            if (sl < 0) { Serial.println(F("channel ini belum di-assign ke slot")); break; }
            map_.trim[sl] = (int16_t)v;
            Serial.print(SLOT_NAME[sl]); Serial.print(F(" trim = "));
            Serial.print(map_.trim[sl]); Serial.println(F(" us"));
            driveCurrent(smDegToUs(map_, sl, 90.0f));
            break;
        }
        default: Serial.println(F("perintah tidak dikenal, ketik 'h'"));
    }
}

void loop() {
    static char buf[24];
    static uint8_t len = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') { buf[len] = 0; handleCmd(buf); len = 0; }
        else if (len < sizeof(buf) - 1) buf[len++] = c;
    }
}
