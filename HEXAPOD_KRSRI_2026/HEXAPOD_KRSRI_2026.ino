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

Hexapod    robot;
Imu        imu;
LidarArray lidar;
Mission    mission(&robot, &imu, &lidar);

bool started = false;

void setup() {
    Serial.begin(115200);
    pinMode(PIN_BUTTON_START, INPUT_PULLUP);
    pinMode(PIN_LED_FOUND, OUTPUT);

    Serial.println("Booting Hexapod KRSRI 2026...");
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
    // 1) Sensor (non-blocking)
    imu.update();
    lidar.update();

    // 2) Stabilisasi badan (lawan kemiringan). Sign bisa dibalik saat tuning.
    robot.setStabilization(-imu.rollDeg(), -imu.pitchDeg());

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
