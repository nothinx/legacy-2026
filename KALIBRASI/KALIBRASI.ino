/* =====================================================================
   KALIBRASI — trim servo, uji IK per kaki, dan uji jalan (Teensy 4.1)

   Tiga tahap, semuanya di sketsa ini:
     1. TRIM   — ratakan netral tiap servo ('n' lalu 't<slot> <us>')
     2. IK     — uji kemulusan gerak per kaki ('c<kaki>', 'o<kaki>', 'b')
     3. JALAN  — gait tripod dengan kecepatan yang bisa disetel ('g','G','y')

   Kinematika di kinematics.h SALINAN PERSIS firmware (LegIK + solvePose),
   jadi apa yang mulus di sini akan mulus juga di firmware.

   KECEPATAN — semua knob bisa diubah runtime:
     v<ms> transisi pose | p<ms> siklus gait | k<mm> panjang langkah
     e<mm> tinggi angkat | f<hz> refresh PWM
   Batas nyata ada di servo (RDS3235 ~0,15 s/60 der), bukan di I2C:
   18 servo @400 kHz hanya ~2,9 ms per refresh.
   ===================================================================== */

#include <Adafruit_PWMServoDriver.h>
#include "servo_map.h"
#include "kinematics.h"

// ------------------------------------------------------------ default knob
#define RAMP_MS_DEF      400      // transisi pose (dulu 1500 — terlalu lambat)
#define STAGGER_MS_DEF    60      // jeda antar servo saat netral pertama
#define PWM_HZ_DEF        50      // refresh PCA9685
#define CYCLE_MS_DEF     900      // 1 siklus gait penuh
#define STEP_LEN_DEF      60.0f   // panjang langkah (mm)
#define STEP_H_DEF        40.0f   // tinggi angkat kaki (mm)
#define SWEEP_DEG_DEF     25.0f   // amplitudo ayun coxa saat uji 'o'
#define CIRCLE_R_DEF      25.0f   // jari-jari lingkaran uji 'c' (mm)
#define BODY_AMP_DEF      20.0f   // amplitudo naik-turun badan uji 'b' (mm)
#define TEST_PERIOD_MS  2000      // periode 1 putaran uji c/o/b

Adafruit_PWMServoDriver drv0(ADDR_DRV0, BUS_DRV0);
Adafruit_PWMServoDriver drv1(ADDR_DRV1, BUS_DRV1);

ServoMap map_;
float    curDeg[SM_SLOTS];
bool     driven = false;

// knob runtime
uint16_t rampMs    = RAMP_MS_DEF;
uint16_t staggerMs = STAGGER_MS_DEF;
uint16_t pwmHz     = PWM_HZ_DEF;
uint16_t cycleMs   = CYCLE_MS_DEF;
float    stepLen   = STEP_LEN_DEF;
float    stepH     = STEP_H_DEF;
float    standH    = STAND_H_DEF;
float    standR    = STAND_R_DEF;

enum Mode { M_IDLE, M_GAIT_F, M_GAIT_B, M_GAIT_Y, M_CIRCLE, M_SWEEP, M_BODY };
Mode     mode    = M_IDLE;
uint8_t  testLeg = 0;
uint32_t modeT0  = 0;
uint32_t lastCommit = 0;
bool     warnedRange = false;

static Adafruit_PWMServoDriver& drvOf(uint8_t d) { return d == 0 ? drv0 : drv1; }
static bool mapped(uint8_t i) { return map_.drv[i] >= 0 && map_.ch[i] >= 0; }
static uint16_t commitMs() { uint16_t m = 1000 / pwmHz; return m < 5 ? 5 : m; }

// ------------------------------------------------------------ tulis servo
static void writeSlot(uint8_t i, float deg) {
    if (!mapped(i)) return;
    drvOf(map_.drv[i]).writeMicroseconds(map_.ch[i], smDegToUs(map_, i, deg));
    curDeg[i] = deg;
}

static void releaseAll() {
    for (uint8_t c = 0; c < 16; c++) { drv0.setPWM(c, 0, 0); drv1.setPWM(c, 0, 0); }
    mode = M_IDLE; driven = false;
    Serial.println(F("semua PWM mati — servo bebas."));
}

// Kirim satu set titik kaki (frame badan) ke servo lewat IK.
static void applyFeet(const float foot[6][3]) {
    bool bad = false;
    for (uint8_t leg = 0; leg < 6; leg++) {
        float d[3];
        if (!footToServo(leg, foot[leg][0], foot[leg][1], foot[leg][2], d)) bad = true;
        for (uint8_t j = 0; j < 3; j++) writeSlot(leg * 3 + j, d[j]);
    }
    driven = true;
    if (bad && !warnedRange) {
        Serial.println(F("!! target di luar jangkauan kaki — sudah di-clamp. "
                         "Kecilkan langkah ('k') atau ubah tinggi ('h')."));
        warnedRange = true;
    }
}

// ------------------------------------------------------------ pose
static void poseNeutralAngles(float* t) { for (uint8_t i = 0; i < SM_SLOTS; i++) t[i] = 90.0f; }

static void poseStandAngles(float* t) {
    poseNeutralAngles(t);
    for (uint8_t leg = 0; leg < 6; leg++) {
        float h[3], d[3];
        footHome(leg, standR, standH, h);
        footToServo(leg, h[0], h[1], h[2], d);
        for (uint8_t j = 0; j < 3; j++) t[leg * 3 + j] = d[j];
    }
}

static void rampToAngles(const float* t) {
    float from[SM_SLOTS];
    for (uint8_t i = 0; i < SM_SLOTS; i++) from[i] = curDeg[i];
    uint16_t steps = rampMs / 20; if (steps < 1) steps = 1;
    for (uint16_t k = 1; k <= steps; k++) {
        float a = (float)k / (float)steps;
        for (uint8_t i = 0; i < SM_SLOTS; i++)
            if (mapped(i)) writeSlot(i, from[i] + (t[i] - from[i]) * a);
        delay(20);
    }
    driven = true;
}

static void staggerToAngles(const float* t) {
    Serial.println(F("posisi awal tak diketahui -> satu per satu. TOPANG ROBOT."));
    for (uint8_t i = 0; i < SM_SLOTS; i++) {
        if (!mapped(i)) continue;
        writeSlot(i, t[i]);
        delay(staggerMs);
    }
    driven = true;
}

static void goAngles(const float* t, const __FlashStringHelper* nama) {
    mode = M_IDLE; warnedRange = false;
    Serial.print(F("-> ")); Serial.println(nama);
    if (driven) rampToAngles(t); else staggerToAngles(t);
    Serial.println(F("selesai."));
}

static void goNeutral() { float t[SM_SLOTS]; poseNeutralAngles(t); goAngles(t, F("NETRAL 90 der")); }
static void goStand()   { float t[SM_SLOTS]; poseStandAngles(t);   goAngles(t, F("BERDIRI")); }

// ------------------------------------------------------------ mode gerak
// Gait tripod: grup A = kaki 0,2,4 ; grup B = kaki 1,3,5 (beda fase 1/2).
static void gaitTick(float dirY, float yawAmt) {
    float ph = fmodf((millis() - modeT0) / (float)cycleMs, 1.0f);
    float foot[6][3];
    for (uint8_t leg = 0; leg < 6; leg++) {
        bool grupA = (leg == 0 || leg == 2 || leg == 4);
        float p = grupA ? ph : fmodf(ph + 0.5f, 1.0f);
        bool  swing = (p < 0.5f);
        float s = swing ? (p / 0.5f) : ((p - 0.5f) / 0.5f);

        // ayun: -L/2 -> +L/2 (maju di udara). tumpu: +L/2 -> -L/2 (dorong badan).
        float along = swing ? stepLen * (s - 0.5f) : stepLen * (0.5f - s);
        float lift  = swing ? stepH * sinf(PI_F * s) : 0.0f;

        float h[3];
        footHome(leg, standR, standH, h);
        if (yawAmt != 0.0f) {                     // putar titik kaki mengelilingi badan
            float ang = yawAmt * along * DEG2RAD * 0.5f;
            float ca = cosf(ang), sa = sinf(ang);
            foot[leg][0] = h[0] * ca - h[1] * sa;
            foot[leg][1] = h[0] * sa + h[1] * ca;
        } else {
            foot[leg][0] = h[0];
            foot[leg][1] = h[1] + dirY * along;
        }
        foot[leg][2] = h[2] + lift;
    }
    applyFeet(foot);
}

// Lingkaran di bidang VERTIKAL searah hadap kaki -> menguji femur+tibia.
static void circleTick() {
    float a = 2.0f * PI_F * fmodf((millis() - modeT0) / (float)TEST_PERIOD_MS, 1.0f);
    float foot[6][3];
    for (uint8_t leg = 0; leg < 6; leg++) footHome(leg, standR, standH, foot[leg]);
    float la = LEG_ANGLE[testLeg] * DEG2RAD;
    float dr = CIRCLE_R_DEF * cosf(a);
    foot[testLeg][0] += dr * cosf(la);
    foot[testLeg][1] += dr * sinf(la);
    foot[testLeg][2] += CIRCLE_R_DEF * sinf(a) + CIRCLE_R_DEF;   // selalu di atas tanah
    applyFeet(foot);
}

// Ayun mendatar -> menguji coxa.
static void sweepTick() {
    float a = 2.0f * PI_F * fmodf((millis() - modeT0) / (float)TEST_PERIOD_MS, 1.0f);
    float foot[6][3];
    for (uint8_t leg = 0; leg < 6; leg++) footHome(leg, standR, standH, foot[leg]);
    float la = (LEG_ANGLE[testLeg] + SWEEP_DEG_DEF * sinf(a)) * DEG2RAD;
    foot[testLeg][0] = LEG_ORIGIN[testLeg][0] + standR * cosf(la);
    foot[testLeg][1] = LEG_ORIGIN[testLeg][1] + standR * sinf(la);
    foot[testLeg][2] = -standH + stepH * 0.5f;                   // diangkat sedikit
    applyFeet(foot);
}

// Badan naik-turun, keenam kaki tetap di tanah -> uji beban & kemulusan bersama.
static void bodyTick() {
    float a = 2.0f * PI_F * fmodf((millis() - modeT0) / (float)TEST_PERIOD_MS, 1.0f);
    float h = standH + BODY_AMP_DEF * sinf(a);
    float foot[6][3];
    for (uint8_t leg = 0; leg < 6; leg++) footHome(leg, standR, h, foot[leg]);
    applyFeet(foot);
}

static void startMode(Mode m, const __FlashStringHelper* nama) {
    mode = m; modeT0 = millis(); warnedRange = false;
    Serial.print(F("-> ")); Serial.print(nama);
    Serial.println(F("   ('x' untuk berhenti)"));
}

// ------------------------------------------------------------ tampilan
static void showKnobs() {
    Serial.println(F("\nknob kecepatan & pose:"));
    Serial.print(F("  v transisi pose   ")); Serial.print(rampMs);   Serial.println(F(" ms"));
    Serial.print(F("  q jeda stagger    ")); Serial.print(staggerMs);Serial.println(F(" ms"));
    Serial.print(F("  f refresh PWM     ")); Serial.print(pwmHz);    Serial.println(F(" Hz"));
    Serial.print(F("  p siklus gait     ")); Serial.print(cycleMs);  Serial.println(F(" ms"));
    Serial.print(F("  k panjang langkah ")); Serial.print(stepLen,0);Serial.println(F(" mm"));
    Serial.print(F("  e tinggi angkat   ")); Serial.print(stepH,0);  Serial.println(F(" mm"));
    Serial.print(F("  h tinggi badan    ")); Serial.print(standH,0); Serial.println(F(" mm"));
    Serial.print(F("  R bentang kaki    ")); Serial.print(standR,0); Serial.println(F(" mm"));
    float v = (stepLen * 2.0f) / (cycleMs / 1000.0f) / 10.0f;   // 2 langkah per siklus
    Serial.print(F("  -> laju jalan teoretis ~")); Serial.print(v, 1);
    Serial.println(F(" cm/detik"));
}

static void printHelp() {
    Serial.println(F("\n=============== KALIBRASI HEXAPOD ==============="));
    Serial.println(F("-- trim & netral --"));
    Serial.println(F("  n            NETRAL 90 der semua"));
    Serial.println(F("  L            tabel mapping + trim"));
    Serial.println(F("  t<slot> <us> set trim, mis. t4 -40   (netral geser -40us)"));
    Serial.println(F("  j<slot> <deg> jog satu sendi, mis. j4 110"));
    Serial.println(F("  i<slot>      balik arah sendi"));
    Serial.println(F("  W / F        simpan EEPROM / muat bawaan program"));
    Serial.println(F("-- uji IK per kaki --"));
    Serial.println(F("  s            BERDIRI (pose home IK)"));
    Serial.println(F("  c<kaki>      kaki menggambar LINGKARAN (uji femur+tibia)"));
    Serial.println(F("  o<kaki>      kaki AYUN mendatar (uji coxa)"));
    Serial.println(F("  b            badan naik-turun, 6 kaki di tanah"));
    Serial.println(F("-- jalan --"));
    Serial.println(F("  g / G        gait tripod MAJU / MUNDUR"));
    Serial.println(F("  y            gait PUTAR di tempat"));
    Serial.println(F("  x            STOP, kaki kembali ke pose berdiri"));
    Serial.println(F("-- kecepatan (angka tanpa nilai = tampilkan semua) --"));
    Serial.println(F("  v<ms> q<ms> f<hz> p<ms> k<mm> e<mm> h<mm> R<mm>"));
    Serial.println(F("  ?            tampilkan knob      r  lepas semua servo"));
    Serial.println(F("================================================\n"));
}

// ------------------------------------------------------------ setup
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) { }

    BUS_DRV0.begin(); BUS_DRV0.setClock(SERVO_I2C_CLOCK);
    BUS_DRV1.begin(); BUS_DRV1.setClock(SERVO_I2C_CLOCK);
    drv0.begin(); drv0.setPWMFreq(pwmHz);
    drv1.begin(); drv1.setPWMFreq(pwmHz);
    for (uint8_t c = 0; c < 16; c++) { drv0.setPWM(c, 0, 0); drv1.setPWM(c, 0, 0); }
    for (uint8_t i = 0; i < SM_SLOTS; i++) curDeg[i] = 90.0f;

    Serial.println(F("\n\nKALIBRASI — trim, uji IK, uji jalan"));
    if (smLoad(map_)) {
        Serial.println(F("mapping: dari EEPROM."));
        Serial.println(F("Periksa tabel di bawah. Kalau tidak cocok, "
                         "'F' untuk muat bawaan program lalu 'W' untuk menimpanya."));
    } else {
        smDefaults(map_);
        Serial.println(F("mapping: bawaan program (EEPROM kosong). 'W' untuk menyimpan."));
    }
    smPrint(map_);
    Serial.println(F("\nPWM mati. 'n' untuk mulai (topang robot)."));
    printHelp();
    showKnobs();
}

// ------------------------------------------------------------ perintah
static bool twoArgs(char* s, int& a, int& b) {
    char* sp = strchr(s, ' ');
    if (!sp) return false;
    a = atoi(s + 1); b = atoi(sp + 1);
    return true;
}

static void handleCmd(char* s) {
    while (*s == ' ') s++;
    char c = *s;
    bool hasNum = (s[1] >= '0' && s[1] <= '9') || s[1] == '-';
    int  v = atoi(s + 1);
    int  a, b;

    switch (c) {
        case 0: case 'H': printHelp(); break;
        case '?': showKnobs(); break;
        case 'L': smPrint(map_); break;
        case 'r': releaseAll(); break;
        case 'n': goNeutral(); break;
        case 's': goStand();   break;
        case 'x':
            if (mode != M_IDLE) { Serial.println(F("stop.")); goStand(); }
            else Serial.println(F("sudah diam."));
            break;
        case 'W': smSave(map_); Serial.println(F("mapping+trim disimpan ke EEPROM.")); break;
        case 'F': smDefaults(map_); Serial.println(F("mapping bawaan dimuat ('W' utk simpan).")); break;

        case 't':
            if (!twoArgs(s, a, b)) { Serial.println(F("format: t<slot> <us>, mis. t4 -40")); break; }
            if (a < 0 || a >= SM_SLOTS) { Serial.println(F("slot 0..23")); break; }
            map_.trim[a] = (int16_t)b;
            Serial.print(SLOT_NAME[a]); Serial.print(F(" trim = ")); Serial.print(b);
            Serial.println(F(" us"));
            if (driven) writeSlot(a, curDeg[a]);
            break;

        case 'j':
            if (!twoArgs(s, a, b)) { Serial.println(F("format: j<slot> <deg>, mis. j4 110")); break; }
            if (a < 0 || a >= SM_SLOTS || !mapped(a)) { Serial.println(F("slot tidak valid")); break; }
            mode = M_IDLE;
            writeSlot(a, (float)b); driven = true;
            Serial.print(SLOT_NAME[a]); Serial.print(F(" -> ")); Serial.print(b);
            Serial.print(F(" der / ")); Serial.print(smDegToUs(map_, a, b)); Serial.println(F(" us"));
            break;

        case 'i':
            if (v < 0 || v >= SM_SLOTS) { Serial.println(F("slot 0..23")); break; }
            map_.invert[v] = !map_.invert[v];
            Serial.print(SLOT_NAME[v]); Serial.print(F(" invert = ")); Serial.println(map_.invert[v]);
            if (driven) writeSlot(v, curDeg[v]);
            break;

        case 'c': if (v >= 0 && v < 6) { testLeg = v; startMode(M_CIRCLE, F("lingkaran kaki")); }
                  else Serial.println(F("kaki 0..5")); break;
        case 'o': if (v >= 0 && v < 6) { testLeg = v; startMode(M_SWEEP, F("ayun mendatar kaki")); }
                  else Serial.println(F("kaki 0..5")); break;
        case 'b': startMode(M_BODY,   F("badan naik-turun")); break;
        case 'g': startMode(M_GAIT_F, F("jalan MAJU"));  break;
        case 'G': startMode(M_GAIT_B, F("jalan MUNDUR")); break;
        case 'y': startMode(M_GAIT_Y, F("putar di tempat")); break;

        case 'v': if (hasNum) rampMs    = constrain(v, 40, 5000);  showKnobs(); break;
        case 'q': if (hasNum) staggerMs = constrain(v, 0, 500);    showKnobs(); break;
        case 'p': if (hasNum) cycleMs   = constrain(v, 200, 4000); showKnobs(); break;
        case 'k': if (hasNum) stepLen   = constrain(v, 0, 120);    showKnobs(); break;
        case 'e': if (hasNum) stepH     = constrain(v, 0, 80);     showKnobs(); break;
        case 'h': if (hasNum) { standH  = constrain(v, 50, 140); if (mode == M_IDLE) goStand(); }
                  showKnobs(); break;
        case 'R': if (hasNum) { standR  = constrain(v, 40, 110); if (mode == M_IDLE) goStand(); }
                  showKnobs(); break;
        case 'f':
            if (hasNum) {
                pwmHz = constrain(v, 50, 333);
                drv0.setPWMFreq(pwmHz); drv1.setPWMFreq(pwmHz);
                Serial.println(F("PWM diubah. Kalau servo berdengung/panas, "
                                 "turunkan lagi — tidak semua servo tahan >50 Hz."));
            }
            showKnobs();
            break;

        default: Serial.println(F("perintah tidak dikenal, ketik 'H'"));
    }
}

// ------------------------------------------------------------ loop
void loop() {
    static char buf[32];
    static uint8_t len = 0;
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r') { buf[len] = 0; handleCmd(buf); len = 0; }
        else if (len < sizeof(buf) - 1) buf[len++] = ch;
    }

    if (mode == M_IDLE) return;
    if (millis() - lastCommit < commitMs()) return;
    lastCommit = millis();

    switch (mode) {
        case M_GAIT_F: gaitTick( 1.0f, 0.0f); break;
        case M_GAIT_B: gaitTick(-1.0f, 0.0f); break;
        case M_GAIT_Y: gaitTick( 0.0f, 1.0f); break;
        case M_CIRCLE: circleTick(); break;
        case M_SWEEP:  sweepTick();  break;
        case M_BODY:   bodyTick();   break;
        default: break;
    }
}
