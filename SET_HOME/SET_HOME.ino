/* =====================================================================
   SET_HOME — menaruh semua servo ke posisi HOME (Teensy 4.1 + 2x PCA9685)

   BISA LANGSUNG DIPAKAI. Pemetaan servo sudah tertanam di servo_map.h
   (hasil TES_SERVO Agustus 2026), jadi tidak perlu menjalankan TES_SERVO dulu.
   Kalau ada mapping tersimpan di EEPROM, itu yang dipakai (lebih baru).

   Saat boot sketsa menghitung mundur 4 detik lalu OTOMATIS ke NETRAL 90 der,
   supaya bisa langsung memasang horn/kaki/body. Tekan apa saja + Enter untuk
   membatalkan, atau set AUTO_HOME_ON_BOOT 0 di bawah.

   Dua arti "home", keduanya disediakan:
     'n' NETRAL  — semua sendi 90 der (1500 us). Ini yang dipakai saat
                   memasang horn/kaki supaya semua lurus & simetris.
     's' BERDIRI — pose siap jalan (coxa/femur/tibia sesuai knob di bawah).

   KESELAMATAN
   - Topang badan robot / gantung kakinya saat pertama kali menjalankan 'n'.
     Servo bisa melompat jauh dari posisi mekanik sekarang ke 90 der.
   - 'n' sengaja menggerakkan servo SATU PER SATU (jeda) supaya arus puncak
     tidak menumpuk. 's' bergerak halus (ramp) karena posisi awal sudah tahu.
   - 'r' kapan saja untuk mematikan semua PWM (servo bebas).
   ===================================================================== */

#include <Adafruit_PWMServoDriver.h>
#include "servo_map.h"

// ------------------------------------------------------------ knob pose
// Pose BERDIRI diturunkan dari IK firmware, BUKAN dikira-kira. Dengan
// COXA=20 FEMUR=80 TIBIA=90, STAND_RADIUS=70, STAND_HEIGHT=100 (config.h),
// LegIK::solve(70, 0, -100) menghasilkan sudut geometris
//     coxa = 0.00   femur = -10.57 (dari horizontal)   knee = 82.02
// dan Hexapod::angleToPulse memakai baseline 90 (tibia dipusatkan di 90):
//     servo coxa = 90 + 0      = 90.00  -> 1500 us
//     servo femur= 90 + (-10.57)= 79.43 -> 1383 us
//     servo tibia= 90 + (82.02-90) = 82.02 -> 1411 us
// Jadi angka di bawah SAMA dengan yang akan dikirim firmware saat diam.
// Ubah STAND_HEIGHT/STAND_RADIUS di config.h -> hitung ulang angka ini.
#define HOME_COXA_DEG    90.00f   // kaki lurus ke arah hadap netralnya
#define HOME_FEMUR_DEG   79.43f   // femur sedikit turun dari horizontal
#define HOME_TIBIA_DEG   82.02f   // sudut lutut (interior)
#define ARM_BASE_DEG     90.0f    // pose parkir lengan (sama dgn Calib firmware)
#define ARM_SHOULDER_DEG 30.0f
#define ARM_GRIP_DEG     20.0f

// Goyang saat verifikasi identitas kaki ('v')
#define VERIFY_DELTA_DEG 15.0f
#define VERIFY_MS        350

#define STAGGER_MS      150       // jeda antar servo saat 'n'
#define RAMP_MS         1500      // durasi gerak halus 's'
#define RAMP_STEP_MS    20        // periode langkah ramp

// Ke NETRAL otomatis saat boot (untuk perakitan). Set 0 bila robot sudah
// berdiri dan Anda tidak mau ia bergerak sendiri setiap kali di-reset.
#define AUTO_HOME_ON_BOOT   1
#define AUTO_HOME_DELAY_S   4

Adafruit_PWMServoDriver drv0(ADDR_DRV0, BUS_DRV0);
Adafruit_PWMServoDriver drv1(ADDR_DRV1, BUS_DRV1);

ServoMap map_;
bool  mapOk  = false;             // mapping valid dari EEPROM?
bool  driven = false;             // sudah pernah menggerakkan (posisi diketahui)?
float curDeg[SM_SLOTS];           // sudut yang terakhir diperintahkan

static Adafruit_PWMServoDriver& drvOf(uint8_t d) { return d == 0 ? drv0 : drv1; }

static bool mapped(uint8_t i) { return map_.drv[i] >= 0 && map_.ch[i] >= 0; }

static void writeSlot(uint8_t i, float deg) {
    if (!mapped(i)) return;
    drvOf(map_.drv[i]).writeMicroseconds(map_.ch[i], smDegToUs(map_, i, deg));
    curDeg[i] = deg;
}

static void releaseAll() {
    for (uint8_t c = 0; c < 16; c++) { drv0.setPWM(c, 0, 0); drv1.setPWM(c, 0, 0); }
    driven = false;
    Serial.println(F("semua PWM dimatikan — servo bebas. "
                     "(posisi tidak lagi diketahui; 'n' akan bertahap lagi)"));
}

// ------------------------------------------------------- pose builder
// isi target[] untuk semua slot; slot yang tak dipakai diberi NAN
static void poseNeutral(float* t) {
    for (uint8_t i = 0; i < SM_SLOTS; i++) t[i] = 90.0f;
}

static void poseStand(float* t) {
    for (uint8_t i = 0; i < 18; i++) {
        switch (i % 3) {
            case 0: t[i] = HOME_COXA_DEG;  break;
            case 1: t[i] = HOME_FEMUR_DEG; break;
            default:t[i] = HOME_TIBIA_DEG; break;
        }
    }
    for (uint8_t i = 18; i < SM_SLOTS; i++) {
        switch ((i - 18) % 3) {
            case 0: t[i] = ARM_BASE_DEG;     break;
            case 1: t[i] = ARM_SHOULDER_DEG; break;
            default:t[i] = ARM_GRIP_DEG;     break;
        }
    }
}

// ------------------------------------------------------------ gerak
// Bertahap satu per satu — untuk perpindahan pertama (posisi awal tak diketahui)
static void applyStaggered(const float* t, int8_t onlyLeg) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < SM_SLOTS; i++) {
        if (!mapped(i)) continue;
        if (onlyLeg >= 0 && (i / 3) != (uint8_t)onlyLeg) continue;
        writeSlot(i, t[i]);
        Serial.print(F("  ")); Serial.print(SLOT_NAME[i]);
        Serial.print(F(" -> ")); Serial.print(t[i], 0); Serial.print(F(" der / "));
        Serial.print(smDegToUs(map_, i, t[i])); Serial.println(F(" us"));
        n++;
        delay(STAGGER_MS);
    }
    driven = true;
    Serial.print(F("selesai, ")); Serial.print(n); Serial.println(F(" servo."));
}

// Halus (ramp) — hanya sah bila posisi sekarang sudah diketahui
static void applyRamp(const float* t, int8_t onlyLeg) {
    float from[SM_SLOTS];
    for (uint8_t i = 0; i < SM_SLOTS; i++) from[i] = curDeg[i];

    uint16_t steps = RAMP_MS / RAMP_STEP_MS;
    for (uint16_t k = 1; k <= steps; k++) {
        float a = (float)k / (float)steps;
        for (uint8_t i = 0; i < SM_SLOTS; i++) {
            if (!mapped(i)) continue;
            if (onlyLeg >= 0 && (i / 3) != (uint8_t)onlyLeg) continue;
            writeSlot(i, from[i] + (t[i] - from[i]) * a);
        }
        delay(RAMP_STEP_MS);
    }
    driven = true;
    Serial.println(F("selesai (gerak halus)."));
}

static void goPose(const float* t, int8_t onlyLeg, const __FlashStringHelper* nama) {
    if (!mapOk) { Serial.println(F("mapping tidak tersedia.")); return; }
    Serial.print(F("\n-> pose ")); Serial.println(nama);
    if (!driven) {
        Serial.println(F("posisi awal belum diketahui -> gerak BERTAHAP. "
                         "TOPANG BADAN ROBOT."));
        applyStaggered(t, onlyLeg);
    } else {
        applyRamp(t, onlyLeg);
    }
}

// ------------------------------------------------------------ tampilan
static void showPose() {
    if (!driven) { Serial.println(F("belum ada perintah gerak (PWM mati).")); return; }
    Serial.println(F("\nslot  nama            sudut   pulse"));
    for (uint8_t i = 0; i < SM_SLOTS; i++) {
        if (!mapped(i)) continue;
        Serial.print(' '); if (i < 10) Serial.print(' ');
        Serial.print(i); Serial.print(F("   "));
        Serial.print(SLOT_NAME[i]);
        for (uint8_t k = strlen(SLOT_NAME[i]); k < 16; k++) Serial.print(' ');
        Serial.print(curDeg[i], 1); Serial.print(F("    "));
        Serial.println(smDegToUs(map_, i, curDeg[i]));
    }
}

// Goyangkan femur kaki n supaya terlihat kaki MANA yang sebenarnya itu.
static void verifyLeg(uint8_t leg) {
    uint8_t f = leg * 3 + 1;                 // slot femur kaki ini
    if (!mapped(f)) { Serial.println(F("femur kaki itu belum dipetakan")); return; }
    float base = driven ? curDeg[f] : HOME_FEMUR_DEG;
    Serial.print(F("goyang femur kaki ")); Serial.print(leg);
    Serial.print(F(" (")); Serial.print(SLOT_NAME[f]);
    Serial.println(F(") — perhatikan kaki mana yang bergerak"));
    for (uint8_t k = 0; k < 3; k++) {
        writeSlot(f, base + VERIFY_DELTA_DEG); delay(VERIFY_MS);
        writeSlot(f, base - VERIFY_DELTA_DEG); delay(VERIFY_MS);
    }
    writeSlot(f, base);
    driven = true;
    Serial.println(F("selesai."));
}

// Goyangkan keenam kaki berurutan sambil menyebut namanya.
static void verifyAll() {
    Serial.println(F("\nverifikasi penomoran kaki — cocokkan dengan diagram di README:"));
    Serial.println(F("  K0 ka-depan  K1 ka-tengah  K2 ka-belakang"));
    Serial.println(F("  K3 ki-belakang  K4 ki-tengah  K5 ki-depan"));
    for (uint8_t leg = 0; leg < SM_LEGS; leg++) { verifyLeg(leg); delay(500); }
    Serial.println(F("Kalau ada yang tidak cocok: perbaiki di TES_SERVO "
                     "('M' tukar sisi, 'T<a> <b>' tukar 2 kaki), lalu 'S'."));
}

// Balik arah satu slot. Kalau servo sedang aktif, langsung diterapkan supaya
// efeknya kelihatan seketika. Pose NETRAL tidak terpengaruh: invert mencerminkan
// sudut terhadap 90 der, jadi 90 tetap 90 (pemasangan horn tidak perlu diulang).
static void toggleInvert(uint8_t slot) {
    if (slot >= SM_SLOTS) { Serial.println(F("slot 0..23")); return; }
    map_.invert[slot] = !map_.invert[slot];
    Serial.print(SLOT_NAME[slot]); Serial.print(F(" invert = "));
    Serial.print(map_.invert[slot]);
    if (driven && mapped(slot)) { writeSlot(slot, curDeg[slot]); Serial.print(F("  (diterapkan)")); }
    Serial.println();
}

static void toggleInvertLeg(uint8_t leg) {
    if (leg >= SM_LEGS) { Serial.println(F("kaki 0..5")); return; }
    for (uint8_t j = 0; j < 3; j++) toggleInvert(leg * 3 + j);
}

// Uji arah satu sendi. Menyebutkan DULU apa yang seharusnya terjadi, baru
// menggerakkan — supaya penilaiannya tidak tergantung tafsir.
// Konvensi firmware (Hexapod::angleToPulse, baseline 90):
//   coxa  > 90 = berputar CCW terhadap badan (dilihat dari atas)
//   femur > 90 = femur naik  -> ujung kaki NAIK
//   tibia > 90 = sudut lutut membesar -> lutut MEMBUKA, kaki lebih lurus
static void testDir(uint8_t slot) {
    if (slot >= SM_SLOTS)  { Serial.println(F("slot 0..23")); return; }
    if (!mapped(slot))     { Serial.println(F("slot itu belum dipetakan")); return; }
    uint8_t j = slot % 3;
    Serial.print(F("\nuji arah slot ")); Serial.print(slot);
    Serial.print(' '); Serial.println(SLOT_NAME[slot]);
    Serial.println(F("  kaki harus MENGGANTUNG BEBAS."));
    Serial.print(F("  90 -> 120 der, yang BENAR: "));
    if (j == 0) Serial.println(F("kaki berayun BERLAWANAN JARUM JAM dilihat "
                                 "dari atas (kaki kanan MAJU, kaki kiri MUNDUR)"));
    else if (j == 1) Serial.println(F("ujung kaki NAIK"));
    else Serial.println(F("lutut MEMBUKA (kaki jadi lebih lurus)"));

    writeSlot(slot, 90.0f);  driven = true; delay(700);
    writeSlot(slot, 120.0f); delay(1200);
    writeSlot(slot, 90.0f);  delay(700);

    Serial.print(F("  kalau KEBALIKAN, ketik: i")); Serial.println(slot);
}

static void testDirLeg(uint8_t leg) {
    if (leg >= SM_LEGS) { Serial.println(F("kaki 0..5")); return; }
    for (uint8_t j = 0; j < 3; j++) { testDir(leg * 3 + j); delay(400); }
}

static void printHelp() {
    Serial.println(F("\n================= SET HOME ================="));
    Serial.println(F("  n        NETRAL: semua sendi 90 der (pasang horn/kaki/body)"));
    Serial.println(F("  s        BERDIRI: pose siap jalan (dari IK firmware)"));
    Serial.println(F("  v<n>     goyang kaki n — pastikan penomoran kaki benar"));
    Serial.println(F("  V        goyang keenam kaki berurutan"));
    Serial.println(F("  k<n>     hanya kaki n (0..5) ke pose BERDIRI"));
    Serial.println(F("  a        kedua lengan ke pose parkir"));
    Serial.println(F("  g<slot> <deg>   satu slot ke sudut tertentu (mis. g4 120)"));
    Serial.println(F("  p        tampilkan pose sekarang"));
    Serial.println(F("  L        tabel mapping aktif"));
    Serial.println(F("  u<slot>  UJI ARAH 1 sendi       U<kaki>  uji arah 1 kaki"));
    Serial.println(F("  i<slot>  balik arah 1 sendi     I<kaki>  balik arah 1 kaki"));
    Serial.println(F("  F        muat mapping bawaan program (abaikan EEPROM)"));
    Serial.println(F("  W        simpan mapping aktif ke EEPROM"));
    Serial.println(F("  r        matikan semua PWM (servo bebas)"));
    Serial.println(F("  h        bantuan"));
    Serial.println(F("Knob pose ada di bagian atas SET_HOME.ino "
                     "(HOME_FEMUR_DEG, HOME_TIBIA_DEG, ...)"));
    Serial.println(F("============================================\n"));
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

    for (uint8_t i = 0; i < SM_SLOTS; i++) curDeg[i] = 90.0f;

    Serial.println(F("\n\nSET_HOME — posisi home servo hexapod"));
    if (smLoad(map_)) {
        Serial.println(F("mapping: dari EEPROM."));
        Serial.println(F("Kalau tabel di bawah tidak cocok, 'F' lalu 'W' untuk "
                         "memakai & menyimpan bawaan program."));
    } else {
        smDefaults(map_);
        Serial.println(F("mapping: BAWAAN program (EEPROM kosong). 'W' untuk menyimpan."));
    }
    mapOk = true;
    smPrint(map_);

#if AUTO_HOME_ON_BOOT
    Serial.print(F("\nAUTO-HOME ke NETRAL 90 der dalam "));
    Serial.print(AUTO_HOME_DELAY_S); Serial.println(F(" detik."));
    Serial.println(F("TOPANG BADAN ROBOT. Kirim apa saja untuk MEMBATALKAN."));
    bool batal = false;
    for (int i = AUTO_HOME_DELAY_S; i > 0 && !batal; i--) {
        Serial.print(i); Serial.println(F("..."));
        uint32_t t = millis();
        while (millis() - t < 1000) { if (Serial.available()) { batal = true; break; } }
    }
    while (Serial.available()) Serial.read();
    if (batal) {
        Serial.println(F("dibatalkan — PWM tetap mati."));
    } else {
        float t0d[SM_SLOTS];
        poseNeutral(t0d);
        goPose(t0d, -1, F("NETRAL 90 der"));
    }
#else
    Serial.println(F("PWM mati. Topang badan robot sebelum menekan 'n'."));
#endif
    printHelp();
}

// ---------------------------------------------------------- perintah
static void handleCmd(char* s) {
    while (*s == ' ') s++;
    char c = *s;
    int  v = atoi(s + 1);
    float t[SM_SLOTS];

    switch (c) {
        case 'h': case '\0': printHelp(); break;
        case 'L': smPrint(map_); break;
        case 'p': showPose(); break;
        case 'r': releaseAll(); break;
        case 'n': poseNeutral(t); goPose(t, -1, F("NETRAL 90 der")); break;
        case 's': poseStand(t);   goPose(t, -1, F("BERDIRI")); break;
        case 'k':
            if (v >= 0 && v < SM_LEGS) { poseStand(t); goPose(t, (int8_t)v, F("BERDIRI (1 kaki)")); }
            else Serial.println(F("kaki 0..5"));
            break;
        case 'v':
            if (v >= 0 && v < SM_LEGS) verifyLeg((uint8_t)v);
            else Serial.println(F("kaki 0..5"));
            break;
        case 'V': verifyAll(); break;
        case 'u': testDir((uint8_t)v); break;
        case 'U': testDirLeg((uint8_t)v); break;
        case 'i': toggleInvert((uint8_t)v); break;
        case 'I': toggleInvertLeg((uint8_t)v); break;
        case 'F': smDefaults(map_); mapOk = true;
                  Serial.println(F("mapping bawaan program dimuat (EEPROM diabaikan). "
                                   "'W' untuk menyimpannya.")); break;
        case 'W': smSave(map_); Serial.println(F("mapping aktif disimpan ke EEPROM.")); break;
        case 'a': {
            poseStand(t);
            if (!mapOk) { Serial.println(F("DITOLAK: mapping belum ada.")); break; }
            Serial.println(F("\n-> lengan ke pose parkir"));
            for (uint8_t i = 18; i < SM_SLOTS; i++) {
                if (!mapped(i)) continue;
                writeSlot(i, t[i]);
                delay(STAGGER_MS);
            }
            driven = true;
            Serial.println(F("selesai."));
            break;
        }
        case 'g': {
            char* sp = strchr(s, ' ');
            if (!sp) { Serial.println(F("format: g<slot> <derajat>, mis. g4 120")); break; }
            int slot = atoi(s + 1);
            int deg  = atoi(sp + 1);
            if (slot < 0 || slot >= SM_SLOTS) { Serial.println(F("slot 0..23")); break; }
            if (!mapped(slot)) { Serial.println(F("slot itu belum dipetakan")); break; }
            if (deg < 0 || deg > 180) { Serial.println(F("sudut 0..180")); break; }
            writeSlot(slot, (float)deg);
            driven = true;
            Serial.print(SLOT_NAME[slot]); Serial.print(F(" -> ")); Serial.print(deg);
            Serial.print(F(" der / ")); Serial.print(smDegToUs(map_, slot, deg));
            Serial.println(F(" us"));
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
