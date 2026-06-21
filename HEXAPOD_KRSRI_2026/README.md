# Hexapod KRSRI 2026 — Firmware Refaktor

Firmware Teensy 4.1 untuk robot hexapod Kontes Robot SAR Indonesia (KRSRI).
Ini **refaktor menyeluruh** dari 4 generasi kode lama (lihat `../program lama/ANALISIS.md`):
mengambil **fondasi gerak terbaik** dari LEGACY2026 dan **mengembalikan semua kemampuan
yang sempat hilang** (lengan ambil korban, navigasi closed-loop, urutan misi,
multi-profil medan), plus perbaikan algoritma yang dulu hard-coded.

> Ditulis untuk mudah dikalibrasi: semua "knob" ada di `config.h`. Ubah angka, jangan
> ubah logika. Tiap penyederhanaan ditandai komentar `ponytail:`.

---

## 1. Apa yang berubah vs kode lama (dan kenapa)

| Masalah lama | Di firmware ini |
|---|---|
| **Belok (yaw) tidak ada** (`HexaGait` `// TODO`) | Yaw diimplementasi penuh: tiap kaki ikut komponen rotasi (cross-product) di `HexaGait.cpp` |
| **Trajektori slip** (stance pakai `cos`) | Stance = garis lurus kecepatan konstan; swing = angkat `sin`. Tidak menyeret. |
| **Lidar blocking ~120ms** → servo patah-patah | `LidarArray` non-blocking (state-machine round-robin), loop tetap 50Hz |
| **Heading/kompas hilang** | `Imu::yawDeg()` 0..360 + `Nav::holdHeading` PID |
| **Navigasi dead-reckoning** (`while{passData×N}`) | `Mission` FSM closed-loop + wall-follow proporsional + timeout/recovery |
| **Tidak bisa ambil korban** (lengan hilang) | `HexaArm` (pose REACH/GRIP/LIFT/DROP), terintegrasi di FSM |
| **Tidak ada multi-profil medan** | `profileFlat/Stairs/Crouch()` ubah tinggi/panjang langkah & tinggi badan |
| **IK pakai hack** (`if(x<0&&y<0)…`) | IK bersih + sudut pasang kaki (`BODY_LEG_ANGLE`) + clamp anti-NaN |
| **CMSIS-DSP matriks 4x4** (rumit) | Rotasi titik langsung (`rotatePoint`) — lebih sederhana, cukup cepat di FPU |
| **Kalibrasi servo seragam** | Per-servo `INVERT` + `OFFSET` + `TRIM_US` |

---

## 2. Arsitektur

```
HEXAPOD_KRSRI_2026.ino        <- main: baca sensor -> Mission -> robot.update()
│
├─ Mission        (FSM misi: korban1->ambil->SZ1->...->tangga)
│    └─ Navigation (header-only: holdHeading, followWall, goStraight)
│
├─ Hexapod        (FASAD: walk/stop/pose/profil + arm())
│    ├─ HexaGait   (tripod gait: stance/swing + yaw + profil medan)
│    ├─ HexaArm    (lengan gripper: pose preset interpolasi)
│    ├─ HexaServos (2x PCA9685, kaki non-blocking 50Hz, aux langsung)
│    └─ LegIK      (IK 3-DOF murni — teruji di PC)
│
├─ Imu            (WT IMU UART: roll/pitch stabil, yaw kompas)
├─ LidarArray     (6x VL53L1X via mux, non-blocking)
├─ config.h       (SEMUA kalibrasi)
└─ types.h        (Vec3 + math helper + rotatePoint, murni)
```

Konvensi sumbu **body frame**: `+X = kanan, +Y = depan, +Z = atas`, origin pusat badan.
`walk(forward, strafe, turn)` → maju(+)/mundur(−), geser kanan(+)/kiri(−), putar CCW(+).

---

## 3. Hardware & Wiring

- **MCU**: Teensy 4.1 (FPU + cepat; cocok untuk IK 6 kaki @50Hz).
- **Bus I2C — SATU jalur** (sesuai PCB): semua di **Wire** (SDA 18 / SCL 19) @**400 kHz**:
  2× PCA9685 + mux TCA9548A + 6× VL53L1X. Clock dibatasi 400 kHz oleh device terlambat
  (VL53L1X & TCA9548A Fast Mode). Cukup untuk 50 Hz — lihat "Anggaran bus I2C" di bawah.
  *(Opsi pisah bus servo→Wire1 @1 MHz hanya bila PCB di-rework; tidak perlu sekarang.)*
- **Driver servo**: 2× PCA9685 — `0x40` (kaki kanan + lengan kanan) & `0x41` (kaki kiri + lengan kiri).
  - Kaki: lihat `SERVO_PIN_MAP` di `config.h`.
  - Lengan: `ARM_PIN_MAP_R` (driver0 ch 12/13/14) & `ARM_PIN_MAP_L` (driver1 ch 12/13/14) = base/shoulder/gripper.
- **Lidar**: 6× VL53L1X di belakang mux I2C **TCA9548A** (`0x70`). Indeks → arti di `config.h`
  (`LIDAR_FRONT`, `LIDAR_RIGHT`, dst) — **samakan dengan pemasangan fisik**. Pasang < 10 cm (dinding arena 10 cm).
- **Pull-up I2C**: 1 bus dengan banyak device + trace PCB → pakai pull-up 2.2k–4.7k untuk integritas sinyal di 400 kHz.

### Anggaran bus I2C (kenapa 1 bus @400 kHz cukup untuk 50 Hz)
1 byte I2C @400 kHz ≈ 22,5 µs. `writeMicroseconds` per servo ≈ 6 byte ≈ ~160 µs → 18 servo ≈ **~2,9 ms**
per refresh, hanya ~15% dari jendela 20 ms (50 Hz). Pembacaan lidar non-blocking (1 sensor/loop) hanya
beberapa transaksi pendek → sisa headroom besar. **Bottleneck lama bukan jumlah bus, tapi lidar blocking
(~120 ms) yang sudah dihapus** (lihat `LidarArray`). Jika kelak refresh perlu > 50 Hz atau device bertambah,
optimasi berikutnya adalah **batch tulis 16 channel PCA9685 dalam 1 transaksi** (register auto-increment) —
belum diperlukan sekarang.
- **IMU**: **Yahboom 10-axis** (protokol WIT, frame 0x55), UART **921600** → `Serial1` (pin 0/1 Teensy).
- **Tombol START**: `PIN_BUTTON_START` (30) ke GND (INPUT_PULLUP).
- **Power**: servo butuh suplai terpisah yang kuat (RDS3235 bisa tarik banyak arus).
  GND servo & Teensy harus tersambung. **Beri kapasitor & pantau tegangan** (lihat §8).

---

## 4. Library yang harus diinstal (Arduino IDE)

1. **Teensyduino** (board Teensy 4.1).
2. **Adafruit PWM Servo Driver Library** (PCA9685).
3. **SparkFun VL53L1X 4m Laser Distance Sensor** (`SparkFun_VL53L1X.h`).

Buka folder `HEXAPOD_KRSRI_2026/` di Arduino IDE (semua `.h/.cpp` + `.ino` satu folder).
Pilih board *Teensy 4.1*, CPU 600MHz, lalu Upload.

---

## 5. Prosedur Kalibrasi (WAJIB sebelum lomba)

Lakukan berurutan. Ini bagian terpenting — robot rapi/oleng ditentukan di sini.

### 5.0 GUI Tuner (laptop) — setel tanpa recompile
Kalibrasi servo + knob gerak kini **runtime**, disetel dari laptop & disimpan di **EEPROM**
(robot ingat saat hari-H, laptop boleh dicabut). Default ada di tabel `PARAM_DEFS` (`Calib.cpp`).

1. Sambungkan Teensy via USB. Buka **`hexapod_tuner.html`** di **Chrome/Edge** (Web Serial;
   browser lain tak didukung). Tak perlu instal apa pun.
2. Klik **Connect** → pilih port Teensy. GUI otomatis kirim `MODE TUNE` + `DUMP` dan
   menampilkan slider: jog 24 servo, kalibrasi 18 servo kaki, dan semua knob gerak/pose lengan.
3. Selama mode TUNE: misi & gait disuspend; robot berdiri di pose HOME (kecuali saat jog).
4. **Pasang horn**: klik **Netral 1500** pada servo terkait → pasang horn lurus → lepas.
5. Setel **offset/trim/invert** tiap sendi sampai HOME rapi (klik **HOME** untuk melihat efek live).
6. Setel knob wall/PID/gait/heading/pose lengan via slider (berlaku live).
7. Klik **SAVE → EEPROM**. Klik **MODE RUN (keluar)** untuk lanjut misi.
   Tombol **LOAD** (batalkan perubahan belum disimpan), **DEFAULTS** (kembali ke pabrik).

Sub-bab di bawah menjelaskan APA yang disetel; lakukan semuanya lewat GUI di atas.

### 5.1 Dimensi & geometri
Ukur `COXA/FEMUR/TIBIA_LENGTH`, `BODY_LEG_ORIGINS`, `BODY_LEG_ANGLE`, `STAND_HEIGHT`,
`STAND_RADIUS` di `config.h` sesuai robot fisik.

### 5.2 Servo netral (paling penting) — via GUI §5.0
1. Mode TUNE → robot di pose HOME. (`pulse.min/max` sesuaikan datasheet servo dulu.)
2. Untuk tiap servo tak lurus, geser **offset** (derajat) sampai sendi pas netral mekanik;
   haluskan dengan **trim** (µs).
3. Jika sendi bergerak berlawanan, centang **invert**.
4. `pulse.min/max`: banyak digital servo 500–2500µs = ~180°. Salah di sini = sudut tak akurat.
5. **SAVE** setelah rapi.

### 5.3 IK
Sudah diverifikasi matematis (`test/`). Jika kaki "terbalik" arah tekuk, itu soal
`SERVO_INVERT`/`OFFSET`, **bukan** IK.

### 5.4 IMU / stabilisasi
- Letakkan robot datar, pastikan `imu.rollDeg()/pitchDeg()` ≈ 0.
- Jika badan justru makin miring saat distabilkan, **balik tanda** via GUI: knob
  `stab.sign_roll` / `stab.sign_pitch` (default −1, ubah ke +1). `stab.tau` = low-pass.
- `STAB_DEADBAND_DEG`, `STAB_MAX_DEG` tetap di `config.h` (jarang diubah).

### 5.5 Kompas (yaw)
Catat `imu.yawDeg()` saat robot menghadap tiap arah arena, isi knob GUI
`head.utara/timur/selatan/barat`. (Arah arena belum tentu = mata angin asli.)

### 5.6 Lidar & wall-follow
Setel knob GUI `wall.setpoint` (jarak ideal ke dinding), `wall.kp`/`wall.kd`,
`heading.kp`/`heading.kd`. Mulai gain kecil, naikkan sampai responsif tanpa berosilasi;
`*.kd` meredam zig-zag (nav sudah PD). `FRONT_STOP_CM` tetap di `config.h`.

---

## 6. Gait — cara kerja singkat

Tripod: grup kaki `{0,2,4}` dan `{1,3,5}` bergantian.
- **Stance** (`legPhase < GAIT_DUTY`): kaki menapak, geser **lurus** dari +½ ke −½
  langkah → mendorong badan maju dengan kecepatan tetap (tidak slip).
- **Swing**: kaki kembali −½ → +½ sambil diangkat `sin` setinggi `stepHeight`.
- **Yaw**: tiap kaki diberi komponen langkah `(-yaw·ry, +yaw·rx)` → berputar di tempat
  atau sambil maju.
- **Profil medan**: `profileStairs()` (langkah tinggi+panjang, lebih lambat),
  `profileCrouch()` (badan rendah). Ubah preset di `Hexapod.cpp`.

Tuning gait: `GAIT_STEP_HEIGHT/LENGTH/CYCLE_TIME/DUTY` di `config.h`.

---

## 7. Misi (FSM) — cara menyesuaikan ke arena

`Mission.cpp` mengikuti blueprint kontes 2024:
```
M_TO_VICTIM1 → M_PICK_VICTIM1 → M_TO_SZ1 → M_DROP_SZ1 →
M_TO_VICTIM2 → M_PICK_VICTIM2 → M_TO_SZ2 → M_TO_STAIRS → M_CLIMB_STAIRS → M_DONE
```
Tiap state navigasi = wall-follow closed-loop + heading target, keluar saat lidar depan
menyentuh ambang, dengan **timeout/recovery** (mundur bila tersangkut) — pengganti
dead-reckoning lama yang rapuh.

**Yang harus Anda tune** (ditandai `TUNE` di `Mission.cpp`):
- `VICTIM_REACH_CM`, `SZ_REACH_CM`, `STAIR_REACH_CM`, `STATE_TIMEOUT`.
- Heading target tiap state (`HEAD_*`) sesuai denah arena.
- Sisi dinding (`followWallRight` ↔ `followWallLeft`) sesuai jalur.

Pose lengan ambil/taruh korban: `ARM_POSE_*` di `config.h`.
Pakai `Serial` print `mission.state()` untuk debug di lapangan.

> Catatan: deteksi korban via kamera (Raspberry Pi → Serial) belum disambung di sini —
> lihat §9. Untuk awal, FSM mendeteksi "sampai korban" dari jarak lidar depan.

---

## 8. Keandalan (jangan diremehkan — banyak tim kalah di sini)

- **Proteksi servo**: IK sudah clamp jangkauan (anti-NaN). Hindari pose mustahil.
- **Tegangan**: pantau baterai; servo brownout = robot ambruk. Tambah kapasitor besar
  di rel servo. (Hook ADC tegangan → turunkan `NAV_FWD_SPEED` saat drop — TODO opsional.)
- **Soft-start**: `delay(1500)` di `setup()` memberi servo waktu ke home sebelum jalan.

---

## 9. Roadmap / yang bisa ditambah

- [ ] **Deteksi korban kamera**: ditangani **Raspberry Pi 5 (YOLOv8n → ONNX)** —
  lihat `../RASPI_VISION_KRSRI/README.md`. Pi mengirim posisi korban via UART
  (`VIC <state> <x_norm> <area> <conf>\n`, 115200). Tambahkan parser di `Serial2` +
  state `M_ALIGN_VICTIM` di FSM: luruskan badan pakai `x_norm`, dekati sampai `area`
  ambang, lalu masuk `M_PICK_VICTIM`.
- [ ] **Feedback gripper** (limit switch/arus) agar korban tidak lolos.
- [ ] **Monitor tegangan** + derating kecepatan.
- [ ] **IK lengan** bila perlu jangkau koordinat sembarang (sekarang pose preset saja).
- [ ] **LED tanda korban** (`PIN_LED_FOUND`) dinyalakan saat `M_PICK_*` sesuai aturan lomba.

---

## 10. Testing

`test/test_kinematics.cpp` memverifikasi IK (round-trip ke FK). Lihat `test/run_test.md`.
Jalankan ulang setiap kali mengubah `LegIK.cpp` sebelum upload.
`test/test_pid.cpp` menguji kontroler PD nav; `test/test_calib.cpp` menguji kalibrasi runtime
(tabel default, CRC, round-trip EEPROM, clamp). EEPROM di-stub otomatis saat dikompilasi di PC.

---

## 11. Peta file

| File | Isi |
|---|---|
| `HEXAPOD_KRSRI_2026.ino` | main loop |
| `config.h` | konstanta tetap (geometri, pin, sensor) |
| `Calib.{h,cpp}` | kalibrasi runtime + EEPROM; redireksi nama lama → tunable |
| `SerialTuner.{h,cpp}` | parser serial mode TUNE (untuk GUI) |
| `hexapod_tuner.html` | GUI laptop (Web Serial) setel servo + knob, simpan EEPROM |
| `types.h` | Vec3, math helper, rotatePoint, Pid (murni) |
| `LegIK.{h,cpp}` | inverse kinematics 1 kaki (murni, teruji) |
| `HexaServos.{h,cpp}` | driver PCA9685, kaki non-blocking 50Hz |
| `HexaGait.{h,cpp}` | tripod gait + yaw + profil medan |
| `HexaArm.{h,cpp}` | lengan gripper (pose preset) |
| `Hexapod.{h,cpp}` | fasad: gait→body transform→IK→servo |
| `Imu.{h,cpp}` | IMU WT (roll/pitch/yaw) |
| `LidarArray.{h,cpp}` | 6× VL53L1X non-blocking |
| `Navigation.h` | helper navigasi closed-loop |
| `Mission.{h,cpp}` | state machine misi KRSRI |
| `test/` | uji IK di PC |
