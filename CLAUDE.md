# CLAUDE.md — Handoff / Kekurangan (lanjut besok pagi)

Repo: `legacy-2026` (remote sudah ada). Firmware utama: `HEXAPOD_KRSRI_2026/`.
Robot: hexapod Teensy 4.1, 2× PCA9685 (0x40/0x41), 6× VL53L1X via mux TCA9548A (0x70),
IMU **Yahboom 10-axis** (WIT, UART Serial1 @921600), **2 lengan** gripper, kamera+RasPi5 (YOLOv8n→ONNX).
I2C **1 bus** (Wire @400kHz) sesuai PCB. Konvensi sumbu: +X kanan, +Y depan, +Z atas.

## STATUS terakhir
SUDAH selesai & terverifikasi (math): gait sikloid, slew-rate vektor gerak, ramp profil medan,
settle dt-based, stabilisasi low-pass dt-based, refresh servo jadi knob, **filter Lidar**
(gate RangeStatus + median-3 + EMA + timeout fail-safe), **IMU** (checksum + gate lonjakan yaw + tare roll/pitch).

⚠️ BELUM di-commit. Langkah pertama besok: `git add -A && git commit && git push` (JANGAN pakai Co-Authored-By Claude — preferensi user).
⚠️ BELUM diverifikasi compiler (tak ada Teensyduino di sesi ini). Compile di Arduino IDE dulu.

## YANG MASIH KURANG (urut prioritas)

### 1. Nav P→PD ✅ SELESAI (math diverifikasi via Python)
`Pid{kp,kd,prev,has; step(err,dt); reset()}` di `types.h` (pure, host-test).
`Mission` punya `_headPid`/`_wallPid` + hitung `dt` dari millis; reset PD tiap `enter()` (cegah lonjakan D saat target lompat). `followWall*` reset PD saat depan mentok. Knob baru: `HEADING_KD=0.004`, `WALL_KD=0.010` (TUNE). Test: `test/test_pid.cpp` (butuh g++).
⚠️ BELUM diverifikasi compiler Teensyduino — compile dulu di Arduino IDE.

### 2. Fail-safe daya + watchdog (TINGGI)
- **Battery monitor**: butuh spesifik hardware (jumlah sel LiPo, rasio divider tegangan, pin ADC). Tanya user. Lalu: baca ADC→low-pass→histeresis; bila < ambang → `robot.stop()` + LED + print. Knob: `PIN_BATT`, `BATT_DIVIDER`, `BATT_MIN_V`.
- **Watchdog**: Teensy 4.1 pakai lib `Watchdog_t4` (WDT_T4). `WDT_timings_t`; feed di loop. Reset otomatis bila hang.

### 3. IMU lanjutan (SEDANG)
- Rate-limit perintah sudut badan (cegah snap saat spike pitch sesaat).
- Pertimbangkan quaternion bila roll/pitch besar (sekarang Euler, cukup utk sudut kecil).

### 4. Mission ↔ Vision (SEDANG) — lihat `RASPI_VISION_KRSRI/README.md`
- Tambah parser UART di `Serial2`: `VIC <state> <class> <x_norm> <area> <conf>`.
- State `M_ALIGN_VICTIM`: pakai `x_norm` luruskan badan, dekati sampai `area` ambang, pilih `armLeft()/armRight()` sesuai sisi (`runPick` sudah parametrik lengan), hanya `class=asli`.
- Petakan state FSM ke arena nyata (R1–R11, K1–K5, SZ1–SZ5) — lihat `berkas_lomba/ANALISIS_BERKAS.md`.

### 5. Profil gait NARROW untuk R11 (TINGGI utk lolos lintasan)
R11 hanya 30 cm; lebar robot 36,9 cm → **tidak muat** stance normal. Tambah `profileNarrow()`
(kecilkan `STAND_RADIUS` efektif / rapatkan kaki) + state khusus R11. Verifikasi bentang kaki ≤ 28–29 cm.

### 6. Pematangan lain (RENDAH)
- ✅ EEPROM + tuning runtime: `Calib.h/.cpp` + GUI `hexapod_tuner.html` (Web Serial) + `SerialTuner`.
- ✅ Scheduler laju-tetap: tick `CONTROL_HZ`=100 via `elapsedMicros` + profiling `PROF` (di `.ino`).
- ✅ Arena cermin: knob `arena.mirror` (0/1) → mirror wall-side+heading+lengan di `Mission.cpp`.
- Telemetri serial terstruktur (state/jarak/pose/tegangan) ter-throttle (PROF sudah ada, perluas).
- Lidar 2 lapis (R5/R10 medan berat): adaptasi pijakan via kontak kaki (arus servo/limit switch).

## KALIBRASI WAJIB sebelum lomba (lihat HEXAPOD_KRSRI_2026/README.md §5)
Servo netral (`SERVO_OFFSET/INVERT/TRIM`), `SERVO_PULSE_MIN/MAX`, tare IMU, `HEAD_*` (baca yaw tiap arah arena),
`WALL_SETPOINT_CM`/`WALL_KP`, pemasangan lidar < 10 cm (dinding arena 10 cm).
Knob hardware: `SERVO_I2C_CLOCK` (turunkan ke 400000 bila tak stabil), `SERVO_PWM_FREQ/COMMIT_MS`,
`GAIT_SLEW_RATE`, `STAB_TAU`, `IMU_MAX_YAW_JUMP`, `LIDAR_*`.

## CATATAN
- `_ref_yahboom_imu/` (clone vendor) di-gitignore, sempat terkunci proses; boleh hapus manual.
- Test IK host: `HEXAPOD_KRSRI_2026/test/` (butuh g++; sudah diverifikasi via Python, round-trip error 0).
- Deadline proposal **22 Juni 2026** (hari ini) — berkas di `berkas_lomba/` (`Draf_Isi_Borang.docx`, `PROPOSAL_DRAFT.md`).
- Lomba 18–19 Sept 2026 (Robot SAR UNLIMITED/UNDIP, Semarang).
