#include "LidarArray.h"

LidarArray::LidarArray() {
    for (int i = 0; i < NUM_LIDAR; i++) { _dist[i] = -1; _histN[i] = 0; _lastOk[i] = 0; }
    _cur = 0;
    _ranging = false;
}

void LidarArray::selectMux(uint8_t ch) {
    LIDAR_I2C_BUS.beginTransmission(I2C_MUX_ADDR);
    LIDAR_I2C_BUS.write(1 << ch);
    LIDAR_I2C_BUS.endTransmission();
}

bool LidarArray::begin() {
    LIDAR_I2C_BUS.begin();
    LIDAR_I2C_BUS.setClock(LIDAR_I2C_CLOCK);
    bool ok = true;
    for (uint8_t i = 0; i < NUM_LIDAR; i++) {
        selectMux(i);
        if (_sensor.begin() != 0) {
            Serial.print("VL53L1X gagal channel "); Serial.println(i);
            ok = false;
        } else {
            _sensor.setDistanceModeShort();   // < ~1.3m, cepat & akurat
            _sensor.setTimingBudgetInMs(20);
        }
    }
    return ok;
}

// median 3 nilai
static int median3(int a, int b, int c) {
    if (a > b) { int t=a; a=b; b=t; }
    if (b > c) { int t=b; b=c; c=t; }
    if (a > b) { int t=a; a=b; b=t; }
    return b;
}

// State machine: tiap panggilan -> mulai ranging sensor _cur, atau ambil hasil bila siap.
void LidarArray::update() {
    selectMux(_cur);
    if (!_ranging) {
        _sensor.startRanging();
        _ranging = true;
        return;            // beri waktu sensor mengukur (cek lagi panggilan berikut)
    }
    if (_sensor.checkForDataReady()) {
        int8_t status = _sensor.getRangeStatus();   // 0 = valid
        int mm = _sensor.getDistance();
        _sensor.clearInterrupt();
        _sensor.stopRanging();
        _ranging = false;

        int cm = mm / 10;
        if (status == 0 && cm >= 0 && cm <= LIDAR_MAX_CM) {
            // 1) histori 3-tap untuk median (buang outlier/spike)
            _hist[_cur][2] = _hist[_cur][1];
            _hist[_cur][1] = _hist[_cur][0];
            _hist[_cur][0] = cm;
            if (_histN[_cur] < 3) _histN[_cur]++;
            int m = (_histN[_cur] < 3) ? cm : median3(_hist[_cur][0], _hist[_cur][1], _hist[_cur][2]);
            // 2) EMA untuk haluskan
            _dist[_cur] = (_dist[_cur] < 0) ? m
                          : (1.0f - LIDAR_EMA_ALPHA) * _dist[_cur] + LIDAR_EMA_ALPHA * m;
            _lastOk[_cur] = millis();
        }
        // status buruk -> abaikan sampel ini (jangan racuni filter)
        _cur = (_cur + 1) % NUM_LIDAR;   // lanjut sensor berikut
    }
    // jika belum siap: keluar, coba lagi loop berikutnya (tanpa blocking)
}

int LidarArray::getDistance(uint8_t id) {
    if (id >= NUM_LIDAR) return -1;
    if (_dist[id] < 0) return -1;
    if (millis() - _lastOk[id] > LIDAR_TIMEOUT_MS) return -1;  // sensor mati/diam -> fail-safe
    return (int)(_dist[id] + 0.5f);
}
