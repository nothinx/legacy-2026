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
sehingga scanner tak bisa membedakan. Library: **Pololu VL53L0X**.

**Jangkauan pakai yang TERUKUR: ~100 cm, bukan 200 dan jelas bukan 400.**
Di bawah 100 cm pembacaan terbukti stabil (5 Agustus 2026). Di atas itu sensor
mengembalikan angka yang **meloncat-loncat tapi bukan `8190`** — inilah
jebakannya: gate `nilai >= 8000` meloloskan sampah itu dan ia terlihat seperti
jarak sungguhan. Firmware wajib membuang **dua-duanya**:

```c
if (mm >= 8000 || mm > LIDAR_MAX_CM * 10) -> anggap tak ada target
```

Konsekuensi ke navigasi: dinding arena baru "terlihat" pada ~1 m. Perencanaan
belok/berhenti harus muat dalam jarak itu.

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
| `MAP_LIDAR/` | pemetaan channel mux → posisi fisik lidar, kalibrasi offset jarak, cetak kode config.h |
| `TES_IMU/` | frame WIT + statistik, hanyutan saat diam, **uji gangguan magnet servo 3 fase** |

**PETA EEPROM** (Teensy 4.1, 4284 byte) — tiap alat menulis di alamat berbeda,
jangan ada yang bertabrakan:

| Alamat | Isi | Ditulis oleh |
|---|---|---|
| 0 .. ~280 | blok `Calib` firmware | `Calib.cpp` (`CALIB_ADDR 0`) |
| 1024 .. ~1150 | `ServoMap` | `servo_map.h` (`SM_EE_ADDR 1024`) |
| 1536 .. ~1560 | pemetaan & offset lidar | `MAP_LIDAR` (`EE_ADDR 1536`) |

`MAP_LIDAR` semula memakai alamat 0 → perintah `e` menimpa kalibrasi firmware.
Sudah diperbaiki. Kalau Teensy pernah dipakai MAP_LIDAR versi lama, blok `Calib`
akan gagal CRC dan otomatis kembali ke default (tidak fatal, tapi trim hilang).

`servo_map.h` ada di TES_SERVO, SET_HOME, KALIBRASI — **kalau diubah, salin ke
ketiganya** (Arduino IDE tidak bisa berbagi file antar folder sketsa).

---

# BESOK: verifikasi semua sensor

Fokus sensor saja. **Belum menyentuh algoritma** (navigasi, FSM misi, vision).
Akhiri dengan commit + push.

### 1. IMU Yahboom 10-axis — `TES_IMU/` SUDAH DIBUAT, belum dijalankan

Protokol WIT, frame `0x55`, `Serial1`, **921600 baud**.

**Angka penentu: pergeseran yaw saat servo bergerak** (`g`, 3 fase — servo
mati / bertenaga diam / bergerak). Bedakan:

- **statis** (rangka besi, magnet servo diam) → konstan, **bisa** dikoreksi
  lewat `HEAD_*`;
- **dinamis** (arus 18 servo, magnet bergerak) → **tidak bisa** dikoreksi.

Ambang: < 5° yaw layak jadi acuan utama · 5–15° perlu disilangkan dengan sudut
dinding lidar · > 15° kompas tidak layak dipakai sambil berjalan.

Gate `IMU_MAX_YAW_JUMP` 30° **tidak** melindungi dari ini — ia menolak lonjakan,
sedangkan kegagalannya berupa pergeseran bertahap (40° dalam 2 detik lolos).

Dua hal di `TES_IMU` sengaja beda dari `Imu.cpp` dan harus diadopsi firmware
kalau terbukti perlu: **resinkronisasi buang-1-byte** (bukan buang 11, yang
membuat satu byte hilang merusak seluruh frame berikutnya) dan **buffer RX
+2 KB** (bawaan 64 byte = 0,7 ms data pada 921600).

### 2. Arah lidar — pakai `MAP_LIDAR/` (sudah dibuat)

**Tata letak sebenarnya** (dikonfirmasi user, 5 Agustus 2026): 6 lidar =
**depan, belakang, 2 kiri, 2 kanan**; yang samping dipasang **di celah antar
kaki** (satu di celah kaki depan–tengah, satu di celah tengah–belakang per sisi).

**Tidak ada lidar diagonal depan.** Jadi `config.h` (`FRONT_R`, `FRONT_L`,
`RIGHT`, `LEFT`) salah, bukan cuma belum terverifikasi. Indeks baru searah jarum
jam dari depan: 0 FRONT, 1 RIGHT_FRONT, 2 RIGHT_REAR, 3 BACK, 4 LEFT_REAR,
5 LEFT_FRONT. Nomor channel mux **tidak** diasumsikan sama dengan nomor posisi.

`MAP_LIDAR/` melakukan pemetaan channel→posisi lewat lambaian tangan (`w`),
verifikasi lewat denah (`a`), kalibrasi offset jarak (`f<p>` + `c<cm>` di
10/20/50/100 cm), simpan ke EEPROM (`e`), lalu cetak kode untuk `config.h` (`g`).

**Hasil pemetaan (5 Agustus 2026)** — sudah masuk `MAP_LIDAR/lidar_map.h`:

| posisi | 0 FRONT | 1 RIGHT_FRONT | 2 RIGHT_REAR | 3 BACK | 4 LEFT_REAR | 5 LEFT_FRONT |
|---|---|---|---|---|---|---|
| **channel mux** | 5 | 4 | 3 | 2 | 1 | 0 |

Yaitu `channel = 5 − posisi`; kabel dicolok berurutan mengelilingi robot
berlawanan arah jarum jam. Offset jarak **belum** dikalibrasi (masih 0 semua).

Untung dari sepasang sensor per sisi: selisih `LEFT_FRONT − LEFT_REAR` memberi
**sudut robot terhadap dinding** langsung — wall-following tidak perlu yaw IMU.

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
  `LIDAR_MAX_CM` 400 → **100** (terukur), indeks lidar baru + `LIDAR_CH_MAP`/`LIDAR_OFFSET_MM`
  dari `MAP_LIDAR` (`LIDAR_FRONT_R`/`FRONT_L` dihapus — sensornya memang tidak ada,
  jadi kode navigasi yang membacanya harus ditulis ulang)
- `HexaServos`: bus per-driver, bukan satu `SERVO_I2C_BUS`
- `Calib::applyDefaults`: tabel invert & trim hasil kalibrasi
- gate `getRangeStatus()==0` diganti: buang nilai ≥ 8000, **nilai > `LIDAR_MAX_CM`**,
  & `timeoutOccurred()` — yang tengah paling gampang terlupa

Baru sesudah itu: algoritma (nav PD sudah ada, FSM misi ↔ arena R1–R11,
profil gait NARROW untuk R11, watchdog, Mission ↔ Vision).

**R11 celah 30 cm**: berdiri normal robot 32 cm — belum muat. Butuh
`profileNarrow()`, target bentang ≤ 28–29 cm.

## CATATAN

- Firmware **belum pernah lolos compiler** sejak refaktor — tidak ada
  Teensyduino di sesi-sesi ini. Compile di Arduino IDE sebelum percaya.
- **Jebakan `.ino`**: Arduino IDE menyisipkan prototipe fungsi hasil generate di
  atas badan sketsa, sebelum `struct`/`enum` buatan sendiri dideklarasikan. Tipe
  buatan sendiri **tidak boleh muncul di tanda tangan fungsi** dalam file `.ino`
  (`'X' does not name a type`) — taruh tipenya di file `.h`, atau ganti parameter
  jadi `(const void*, size_t)`. Kena di `MAP_LIDAR` (`storeSum`).
- Preferensi user: **JANGAN pakai Co-Authored-By Claude** di commit message.
- Library: Teensyduino, Adafruit PWM Servo Driver, **VL53L0X by Pololu**.
- Batas kecepatan servo: RDS3235 ~0,15 s/60°. Bottleneck bukan I2C
  (18 servo @400 kHz ≈ 2,9 ms/refresh). Detail di `KALIBRASI/README.md`.
