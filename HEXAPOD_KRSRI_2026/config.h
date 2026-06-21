#ifndef CONFIG_H
#define CONFIG_H

// =====================================================================
//  KONFIGURASI PUSAT HEXAPOD KRSRI 2026
//  Semua "knob" kalibrasi ada di sini. Ubah angka, JANGAN ubah logika.
//  Konvensi sumbu (body frame):
//     +X = kanan, +Y = depan, +Z = atas. Origin = pusat badan.
//  File ini murni konstanta (stdint saja) -> aman di-include host & Teensy.
// =====================================================================
#include <stdint.h>

#define NUM_SERVOS 18   // 6 kaki x 3 sendi (lengan terpisah, lihat ARM_*)

// ---------------------------------------------------------------------
//  DIMENSI KAKI (mm) -- ukur fisik robot Anda
// ---------------------------------------------------------------------
#define COXA_LENGTH   20.0f
#define FEMUR_LENGTH  80.0f
#define TIBIA_LENGTH  90.0f

// ---------------------------------------------------------------------
//  GEOMETRI BADAN
//  BODY_LEG_ORIGINS = posisi pangkal coxa tiap kaki relatif pusat (mm).
//  BODY_LEG_ANGLE   = arah hadap kaki (derajat, dari +X, CCW positif).
//                     Dipakai untuk transform titik kaki -> frame kaki
//                     supaya IK benar walau kaki terpasang menyudut.
//  Urutan kaki: 0=Ka-Depan 1=Ka-Tengah 2=Ka-Belakang
//               3=Ki-Belakang 4=Ki-Tengah 5=Ki-Depan
// ---------------------------------------------------------------------
const float BODY_LEG_ORIGINS[6][3] = {
    { 45.0f,  78.0f, 0.0f},  // 0
    { 90.0f,   0.0f, 0.0f},  // 1
    { 45.0f, -78.0f, 0.0f},  // 2
    {-45.0f, -78.0f, 0.0f},  // 3
    {-90.0f,   0.0f, 0.0f},  // 4
    {-45.0f,  78.0f, 0.0f}   // 5
};
const float BODY_LEG_ANGLE[6] = {
     60.0f,    // 0 Ka-Depan
      0.0f,    // 1 Ka-Tengah
    -60.0f,    // 2 Ka-Belakang
   -120.0f,    // 3 Ki-Belakang
    180.0f,    // 4 Ki-Tengah
    120.0f     // 5 Ki-Depan
};

// ---------------------------------------------------------------------
//  POSISI BERDIRI (HOME) ujung kaki, di frame badan (mm)
//  STAND_HEIGHT positif = tinggi badan dari tanah; foot z = -STAND_HEIGHT.
//  STAND_RADIUS = seberapa jauh kaki melebar dari origin coxa.
// ---------------------------------------------------------------------
#define STAND_HEIGHT  100.0f
#define STAND_RADIUS   70.0f   // jarak ujung kaki ke pangkal coxa (arah hadap kaki)

// ---------------------------------------------------------------------
//  PETA PIN SERVO KAKI  {driver_index(0/1), channel(0..15)}
//  driver 0 = PCA9685 @0x40, driver 1 = @0x41
// ---------------------------------------------------------------------
const uint8_t SERVO_PIN_MAP[NUM_SERVOS][2] = {
    {0, 8},  {0, 9},  {0, 10}, // Kaki 0 (coxa,femur,tibia)
    {0, 4},  {0, 5},  {0, 6},  // Kaki 1
    {0, 0},  {0, 1},  {0, 2},  // Kaki 2
    {1, 8},  {1, 9},  {1, 10}, // Kaki 3
    {1, 4},  {1, 5},  {1, 6},  // Kaki 4
    {1, 0},  {1, 1},  {1, 2}   // Kaki 5
};

// ---------------------------------------------------------------------
//  KALIBRASI SERVO -> RUNTIME (lihat Calib.h/.cpp)
//  SERVO_PULSE_MIN/MAX, SERVO_INVERT/OFFSET/TRIM_US kini disetel via GUI tuner
//  & disimpan di EEPROM. Nilai default ada di tabel PARAM_DEFS / applyDefaults
//  (Calib.cpp). Nama simbol lama tetap dipakai konsumen (macro redirect di Calib.h).
// ---------------------------------------------------------------------

// Peta pin gabungan untuk JOG tuner: 0..17 kaki, 18..20 lengan kanan, 21..23 kiri.
#define NUM_TUNE_SERVOS 24
const uint8_t TUNE_PIN_MAP[NUM_TUNE_SERVOS][2] = {
    {0, 8},  {0, 9},  {0, 10}, // Kaki 0
    {0, 4},  {0, 5},  {0, 6},  // Kaki 1
    {0, 0},  {0, 1},  {0, 2},  // Kaki 2
    {1, 8},  {1, 9},  {1, 10}, // Kaki 3
    {1, 4},  {1, 5},  {1, 6},  // Kaki 4
    {1, 0},  {1, 1},  {1, 2},  // Kaki 5
    {0, 12}, {0, 13}, {0, 14}, // Lengan kanan (base,shoulder,gripper)
    {1, 12}, {1, 13}, {1, 14}  // Lengan kiri
};

// ---------------------------------------------------------------------
//  GAIT -> RUNTIME (Calib.h): GAIT_STEP_HEIGHT/LENGTH/CYCLE_TIME/DUTY,
//  GAIT_SLEW_RATE/PROFILE_TAU/SETTLE_TAU disetel via GUI. Default di Calib.cpp.
//  Profil medan (tangga/crouch) tetap diatur runtime via setProfile (Hexapod.cpp).
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
//  LENGAN / GRIPPER (untuk ambil korban)
//  Default 3 servo pada driver 0 channel 12,13,14: base, shoulder, gripper.
//  Pose dalam derajat 0..180 (sudut servo). KALIBRASI sesuai mekanik.
// ---------------------------------------------------------------------
#define ARM_NUM_SERVOS 3
// Dua lengan: kanan di driver 0, kiri di driver 1 (base, shoulder, gripper).
const uint8_t ARM_PIN_MAP_R[ARM_NUM_SERVOS][2] = { {0, 12}, {0, 13}, {0, 14} };
const uint8_t ARM_PIN_MAP_L[ARM_NUM_SERVOS][2] = { {1, 12}, {1, 13}, {1, 14} };
// Pose lengan {base, shoulder, gripper} -> RUNTIME (Calib.h). Default di Calib.cpp.
// ARM_POSE_* kini pointer ke gParam (macro redirect); disetel via GUI tuner.

// ---------------------------------------------------------------------
//  BUS I2C — SATU jalur (sesuai PCB: 1x SDA/SCL untuk semua device).
//  Semua di Wire (SDA 18 / SCL 19 Teensy 4.1): 2x PCA9685 + mux + 6 VL53L1X.
//  Clock dibatasi 400 kHz karena VL53L1X & TCA9548A maksimum Fast Mode (400k);
//  walau PCA9685 bisa 1 MHz, di bus bersama clock ikut device terlambat.
//  (Pisah ke Wire1 untuk servo @1MHz HANYA bila PCB di-rework — lihat README.)
// ---------------------------------------------------------------------
#define SERVO_I2C_BUS    Wire
#define SERVO_I2C_CLOCK  400000
#define LIDAR_I2C_BUS    Wire
#define LIDAR_I2C_CLOCK  400000

// ---------------------------------------------------------------------
//  SENSOR
// ---------------------------------------------------------------------
#define NUM_LIDAR        6
#define I2C_MUX_ADDR     0x70
#define LIDAR_EMA_ALPHA  0.4f    // bobot sampel baru (median dulu, lalu EMA)
#define LIDAR_TIMEOUT_MS 300     // sensor dianggap mati bila tak ada data valid
#define LIDAR_MAX_CM     400     // di atas ini dianggap tak valid

// IMU
#define IMU_MAX_YAW_JUMP 30.0f   // derajat/sample; lonjakan > ini ditolak (gangguan magnet)
// Indeks lidar -> arti (sesuaikan pemasangan fisik)
#define LIDAR_FRONT      0
#define LIDAR_FRONT_R    1
#define LIDAR_RIGHT      2
#define LIDAR_BACK       3
#define LIDAR_LEFT       4
#define LIDAR_FRONT_L    5

#define IMU_SERIAL       Serial1
#define IMU_BAUD         921600   // Yahboom 10-axis (protokol WIT, frame 0x55)

// ---------------------------------------------------------------------
//  NAVIGASI (closed-loop)
// ---------------------------------------------------------------------
// HEADING_KP/KD, WALL_KP/KD, WALL_SETPOINT_CM, HEAD_* -> RUNTIME (Calib.h, GUI).
#define HEADING_TOL_DEG   3.0f     // toleransi heading dianggap "lurus" (const)
#define FRONT_STOP_CM     20       // berhenti/belok bila depan < ini (const)
#define NAV_FWD_SPEED     0.8f     // kecepatan maju normal (0..1) (const)

// ---------------------------------------------------------------------
//  PIN LAIN
// ---------------------------------------------------------------------
#define PIN_BUTTON_START  30   // tombol mulai (INPUT_PULLUP)
#define PIN_LED_FOUND     13   // LED tanda korban ditemukan (cek aturan lomba)

// Stabilisasi badan (IMU). STAB_TAU & tanda koreksi (STAB_SIGN_*) -> RUNTIME (Calib.h).
#define STAB_MAX_DEG      15.0f   // clamp koreksi roll/pitch (const)
#define STAB_DEADBAND_DEG 1.0f    // abaikan getaran kecil (const)

// Refresh servo (knob hardware). RDS3235 digital sering sanggup > 50 Hz.
// Naikkan untuk micro-motion lebih halus; turunkan bila servo panas/getar.
#define SERVO_PWM_FREQ    50      // Hz, frekuensi sinyal PCA9685
#define SERVO_COMMIT_MS   20      // ms, periode kirim 18 pulse (20=50Hz, 10=100Hz)

#endif
