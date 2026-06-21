// =====================================================================
//  HEXAPOD KRSRI 2026 - Main
//  Teensy 4.1 + 2x PCA9685 + 6x VL53L1X (mux) + IMU WT + lengan gripper.
//
//  Alur: tahan tombol START -> jalankan state machine misi (Mission).
//  Semua kerumitan disembunyikan di balik class Hexapod & Mission.
// =====================================================================
#include "Hexapod.h"
#include "Imu.h"
#include "LidarArray.h"
#include "Mission.h"
#include "SerialTuner.h"

Hexapod    robot;
Imu        imu;
LidarArray lidar;
Mission    mission(&robot, &imu, &lidar);
SerialTuner tuner(&robot);

bool started = false;

// Penjadwal laju-tetap + profiling loop.
static const uint32_t TICK_US = 1000000UL / CONTROL_HZ;
elapsedMicros tickTimer;
static uint32_t profMaxUs = 0, profSumUs = 0, profCount = 0;
elapsedMillis  profReport;

static void profileTick(uint32_t workUs) {
#if PROFILE_LOOP
    profSumUs += workUs; profCount++;
    if (workUs > profMaxUs) profMaxUs = workUs;
    if (profReport >= 1000) {                  // laporan tiap 1 detik
        if (!tuner.tuneActive() && profCount) {
            uint32_t avg = profSumUs / profCount;
            // util = beban / jendela tick (persen). > 100% = loop tak sanggup laju-tetap.
            Serial.printf("PROF avg=%luus max=%luus util=%lu%% n=%lu tick=%luus\n",
                          avg, profMaxUs, (avg * 100) / TICK_US, profCount, TICK_US);
        }
        profSumUs = profMaxUs = profCount = 0; profReport = 0;
    }
#endif
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_BUTTON_START, INPUT_PULLUP);
    pinMode(PIN_LED_FOUND, OUTPUT);

    Serial.println("Booting Hexapod KRSRI 2026...");
    Calib::begin();            // muat kalibrasi EEPROM SEBELUM servo dipakai
    robot.begin();
    imu.begin();
    if (!lidar.begin()) Serial.println("WARNING: ada sensor lidar gagal init");

    robot.profileFlat();
    delay(1500);              // beri waktu servo ke pose home

    // Tare IMU: robot harus DATAR & DIAM saat ini (baca ~500ms lalu nol-kan roll/pitch).
    uint32_t t0 = millis();
    while (millis() - t0 < 500) imu.update();
    if (imu.hasData()) { imu.tare(); Serial.println("IMU tared."); }
    else Serial.println("WARNING: IMU tak ada data (cek baud/kabel)");

    Serial.println("Siap. Tekan tombol START.");
}

void loop() {
    // Tick laju-tetap: jalankan kontrol tepat tiap TICK_US -> dt deterministik.
    // Reset di AWAL tick -> period = TICK_US bila kerja < tick; bila kerja > tick, fire
    // berturut (degradasi mulus, util > 100% terlihat di PROF).
    if (tickTimer < TICK_US) return;
    tickTimer = 0;
    uint32_t t0 = micros();

    // 1) Sensor (non-blocking)
    imu.update();
    lidar.update();

    // 1b) Mode tuning (GUI). Bila aktif: misi/stabilisasi disuspend, tuner pegang servo.
    if (!tuner.update()) {
        // 2) Stabilisasi badan (lawan kemiringan). Tanda koreksi = knob runtime (STAB_SIGN_*).
        robot.setStabilization(gParam[K_STAB_SIGN_ROLL]  * imu.rollDeg(),
                               gParam[K_STAB_SIGN_PITCH] * imu.pitchDeg());

        // 3) Trigger misi
        if (!started && digitalRead(PIN_BUTTON_START) == LOW) {
            Serial.println("START!");
            mission.start();
            started = true;
        }

        // 4) Otak misi (closed-loop, non-blocking)
        if (started) mission.update();

        // 5) Eksekusi gerak (gait -> IK -> servo)
        robot.update();
    }

    profileTick(micros() - t0);
}
