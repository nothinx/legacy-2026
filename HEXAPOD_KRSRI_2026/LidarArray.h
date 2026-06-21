#ifndef LIDARARRAY_H
#define LIDARARRAY_H

// 6x VL53L1X via mux TCA9548A, NON-BLOCKING (state machine round-robin).
// update() memajukan SATU sensor per panggilan tanpa busy-wait -> loop tetap 50Hz.
// Filter: gate RangeStatus -> median-3 (buang outlier) -> EMA. getDistance() = nilai
// terakhir valid, atau -1 bila status buruk / sensor mati (timeout) -> fail-safe.
#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include "config.h"

class LidarArray {
public:
    LidarArray();
    bool begin();
    void update();                 // non-blocking
    int  getDistance(uint8_t id);  // cm (terakhir valid), -1 jika error

private:
    SFEVL53L1X _sensor;
    float _dist[NUM_LIDAR];           // hasil EMA (cm)
    int  _hist[NUM_LIDAR][3];         // 3 sampel terakhir (untuk median)
    uint8_t _histN[NUM_LIDAR];        // jumlah sampel terkumpul (<=3)
    uint32_t _lastOk[NUM_LIDAR];      // waktu data valid terakhir
    uint8_t _cur;        // sensor yang sedang diproses
    bool _ranging;       // sudah startRanging?
    void selectMux(uint8_t ch);
};

#endif
