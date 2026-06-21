#ifndef HEXASERVOS_H
#define HEXASERVOS_H

// Lapis "otot": pemilik 2x PCA9685. Kaki di-update non-blocking 50Hz.
// Lengan/aux pakai writeRaw() (langsung) lewat driver yang sama.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Calib.h"   // SERVO_PULSE_MIN/MAX runtime

#define PCA9685_0_ADDR 0x40
#define PCA9685_1_ADDR 0x41
#define SERVO_FREQ SERVO_PWM_FREQ   // dari config.h (knob)

class HexaServos {
public:
    HexaServos();
    void begin();

    // Set target pulse (us) satu servo kaki (0..NUM_SERVOS-1). Disimpan, dikirim saat commit().
    void setLegPulse(uint8_t legServoID, uint16_t pulseUs);

    // Kirim 18 pulse kaki bila sudah waktunya (tiap 20ms). Panggil di loop.
    // return true bila benar-benar mengirim siklus ini.
    bool commit();

    // Tulis langsung ke channel mana pun (untuk lengan/aux). Bukan via timer.
    void writeRaw(uint8_t driver, uint8_t channel, uint16_t pulseUs);

private:
    Adafruit_PWMServoDriver _kit0 = Adafruit_PWMServoDriver(PCA9685_0_ADDR, SERVO_I2C_BUS);
    Adafruit_PWMServoDriver _kit1 = Adafruit_PWMServoDriver(PCA9685_1_ADDR, SERVO_I2C_BUS);
    uint16_t _target[NUM_SERVOS];
    unsigned long _lastUpdate;
    static const unsigned int UPDATE_MS = SERVO_COMMIT_MS;
};

#endif
