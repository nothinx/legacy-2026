# CLAUDE.md — Handoff

Repo: `legacy-2026`. Robot hexapod KRSRI, Teensy 4.1.
Lomba **18–19 Sept 2026** (Robot SAR UNLIMITED/UNDIP, Semarang).

## STATUS (4 Agustus 2026)

**Robot sudah berjalan.** Gait tripod maju/mundur/putar terbukti jalan lewat
`KALIBRASI/`. Pemetaan 24 servo terverifikasi fisik, 6 lidar terbaca semua.

Yang BELUM: firmware utama `HEXAPOD_KRSRI_2026/` **tidak bisa jalan** dengan
hardware sekarang — masih memakai library dan wiring yang salah (lihat bawah).
Alat bringup di `TES_*`/`SET_HOME`/`KALIBRASI` yang dipakai sejauh ini.

## PERANGKAT KERAS — fakta terukur, bukan asumsi

Tiga temuan yang membatalkan asumsi lama. Jangan percaya `config.h` untuk hal-hal ini.

**1. Tiga bus I2C terpisah**, bukan satu:

| Bus | Pin | Isi |
|---|---|---|
| `Wire` | SDA 18 / SCL 19 | TCA9548A `0x70` → 6× lidar |
| `Wire1` | SDA 17 / SCL 16 | PCA9685 `0x41` = driver 1 = **KANAN** |
| `Wire2` | SDA 25 / SCL 24 | PCA9685 `0x40` = driver 0 = **KIRI** |

**2. `0x70` muncul di ketiga bus dan itu normal.** Di `Wire` itu mux; di
`Wire1`/`Wire2` itu **ALL-CALL bawaan PCA9685** (`ALLCALLADR`=`0xE0` 8-bit).
Konsekuensi: PCA9685 **tidak boleh** sebus dengan TCA9548A.

**3. Lidar = VL53L0X (modul TOF200C), bukan VL53L1X.** Keduanya beralamat `0x29`
sehingga scanner tak bisa membedakan. Library: **Pololu VL53L0X**. Jangkauan
**~200 cm**, bukan 400.

**Pemetaan servo** (di `servo_map.h`, sudah jadi bawaan program):

```
drv1 0x41@Wire1 KANAN : ch8-10 K0, ch4-6 K1, ch0-2 K2, ch12-14 lengan kanan
drv0 0x40@Wire2 KIRI  : ch8-10 K3, ch4-6 K4, ch0-2 K5, ch12-14 lengan kiri
invert: coxa dibalik di KEENAM kaki; femur hanya sisi kanan; tibia hanya kiri
trim  : masih 0 semua — belum disetel
```

Pose berdiri dari IK: coxa **90.00°**, femur **79.43°**, tibia **82.02°**
(STAND_RADIUS 70, STAND_HEIGHT 100).

## ALAT BRINGUP (masing-masing punya README)

| Folder | Isi |
|---|---|
| `TES_LIDAR/` | deteksi bus, uji keaslian mux, identifikasi chip, baca semua/satu-satu |
| `TES_SERVO/` | pemetaan servo satu per satu, invert, trim, cetak kode `config.h` |
| `SET_HOME/` | netral 90° & pose berdiri, auto-home saat boot, uji arah per sendi |
| `KALIBRASI/` | trim, uji kemulusan IK per kaki, gait tripod, knob kecepatan |

`servo_map.h` ada di TES_SERVO, SET_HOME, KALIBRASI — **kalau diubah, salin ke
ketiganya** (Arduino IDE tidak bisa berbagi file antar folder sketsa).

---

# BESOK: verifikasi semua sensor

Fokus sensor saja. **Belum menyentuh algoritma** (navigasi, FSM misi, vision).
Akhiri dengan commit + push.

### 1. IMU Yahboom 10-axis — PRIORITAS UTAMA, belum pernah diuji sama sekali

Satu-satunya subsistem yang sama sekali belum disentuh hardware-nya.
Protokol WIT, frame `0x55`, `Serial1`, **921600 baud**.

Buat `TES_IMU/`:
- baca frame mentah, hitung **rasio checksum gagal** dan **frame drop/detik**
  (921600 di kabel panjang rawan; kalau rusak, ini yang menunjukkannya)
- tampilkan roll/pitch/yaw + accel + gyro + mag
- tare roll/pitch; uji gate lonjakan yaw (`IMU_MAX_YAW_JUMP` 30°/sampel)
- **uji gangguan magnet**: dekatkan servo/motor saat berjalan, lihat yaw meleset
  berapa derajat. Ini menentukan apakah yaw bisa dipercaya untuk heading arena.

### 2. Arah lidar — index → arah fisik belum dicocokkan

`config.h` menyebut ch0=FRONT, 1=FRONT_R, 2=RIGHT, 3=BACK, 4=LEFT, 5=FRONT_L,
tapi itu warisan, belum pernah diverifikasi.

Pakai `TES_LIDAR` yang sudah ada: `r<n>` per channel, halangi satu sisi dengan
tangan, catat channel mana yang berubah. Hasilnya masuk ke `config.h`.

Sekalian: ukur jarak sebenarnya vs pembacaan pada 10/20/50/100 cm — cari offset
sistematis dan jarak minimum yang masih waras.

### 3. Tombol START + LED

`PIN_BUTTON_START` 30 (INPUT_PULLUP), `PIN_LED_FOUND` 13. Sepele tapi belum
pernah dites. Perlu debounce — pastikan tidak memicu ganda.

### 4. Monitor baterai — BUTUH KEPUTUSAN ANDA

Belum ada hardware-nya. Sebelum bisa dikerjakan, tiga hal harus ditentukan:

- **jumlah sel LiPo** (3S/4S?) dan tegangan minimum aman per sel
- **rasio pembagi tegangan** (Teensy 4.1 ADC **maks 3,3 V — tidak toleran 5 V**)
- **pin ADC** mana yang dipakai

Ini fail-safe paling penting yang belum ada: tanpa itu, tegangan drop saat 18
servo bergerak serentak bisa me-reset Teensy di tengah lomba tanpa peringatan.

### 5. UART ke Raspberry Pi 5 (`Serial2`)

Uji jalur fisiknya saja, bukan vision-nya: kirim/terima frame uji, ukur baud
yang stabil, pastikan GND tersambung. Parser `VIC ...` menyusul bersama vision.

### 6. Commit + push

---

# SETELAH ITU (bukan besok)

Port firmware ke hardware nyata — ini yang membuat `HEXAPOD_KRSRI_2026/` bisa jalan:

- `LidarArray` → Pololu VL53L0X, **satu objek per channel** (Pololu menyimpan
  `stop_variable` milik sensor itu; objek tidak boleh dipakai bergantian — jebakan halus)
- `config.h`: 3 bus terpisah, `SERVO_PIN_MAP`/`TUNE_PIN_MAP`/`ARM_PIN_MAP_*` baru,
  `LIDAR_MAX_CM` 400 → **200**
- `HexaServos`: bus per-driver, bukan satu `SERVO_I2C_BUS`
- `Calib::applyDefaults`: tabel invert & trim hasil kalibrasi
- gate `getRangeStatus()==0` diganti: buang nilai ≥ 8000 & `timeoutOccurred()`

Baru sesudah itu: algoritma (nav PD sudah ada, FSM misi ↔ arena R1–R11,
profil gait NARROW untuk R11, watchdog, Mission ↔ Vision).

**R11 celah 30 cm**: berdiri normal robot 32 cm — belum muat. Butuh
`profileNarrow()`, target bentang ≤ 28–29 cm.

## CATATAN

- Firmware **belum pernah lolos compiler** sejak refaktor — tidak ada
  Teensyduino di sesi-sesi ini. Compile di Arduino IDE sebelum percaya.
- Preferensi user: **JANGAN pakai Co-Authored-By Claude** di commit message.
- Library: Teensyduino, Adafruit PWM Servo Driver, **VL53L0X by Pololu**.
- Batas kecepatan servo: RDS3235 ~0,15 s/60°. Bottleneck bukan I2C
  (18 servo @400 kHz ≈ 2,9 ms/refresh). Detail di `KALIBRASI/README.md`.
