# legacy-2026 — Hexapod KRSRI

Robot hexapod untuk Kontes Robot SAR Indonesia (KRSRI). Repo ini berisi firmware
refaktor terbaru, subsistem vision, analisis lintas-generasi, dan arsip kode lama.

## Struktur

| Folder | Isi |
|---|---|
| [`HEXAPOD_KRSRI_2026/`](HEXAPOD_KRSRI_2026/) | **Firmware Teensy 4.1 (refaktor menyeluruh)** — gait+yaw, IK teruji, lidar non-blocking, lengan gripper, navigasi closed-loop, FSM misi. Lihat README-nya. |
| [`TES_LIDAR/`](TES_LIDAR/) | Alat diagnosa 6× VL53L0X (TOF200C) + mux TCA9548A: deteksi bus, uji keaslian mux, identifikasi chip, baca semua / satu per satu. |
| [`TES_SERVO/`](TES_SERVO/) | Alat pemetaan servo **satu per satu** (channel PCA9685 → sendi), simpan ke EEPROM, cetak kode untuk `config.h`. |
| [`SET_HOME/`](SET_HOME/) | **Mulai dari sini saat merakit.** Menaruh semua servo ke home (netral 90° / pose berdiri); mapping kaki sudah tertanam, auto-home saat boot. |
| [`KALIBRASI/`](KALIBRASI/) | Trim servo, uji kemulusan IK per kaki, dan uji jalan (gait tripod) dengan knob kecepatan runtime. IK-nya salinan persis firmware. |
| [`RASPI_VISION_KRSRI/`](RASPI_VISION_KRSRI/) | **Deteksi korban Raspberry Pi 5** (YOLOv8n → ONNX) + protokol UART ke Teensy. README lengkap. |
| [`program lama/ANALISIS.md`](program%20lama/ANALISIS.md) | Analisis mendalam 4 generasi kode + roadmap menang KRSRI. |
| `program lama/` | Arsip kode generasi lama (LEGACY2026, 2024, raspi vision). |

## Wiring I2C saat ini (hasil scan, Agustus 2026)

| Bus | Pin Teensy 4.1 | Isi |
|---|---|---|
| `Wire`  | SDA 18 / SCL 19 | TCA9548A `0x70` → 6× **VL53L0X** (modul TOF200C, ~2 m) |
| `Wire1` | SDA 17 / SCL 16 | PCA9685 `0x41` → driver servo **1** |
| `Wire2` | SDA 25 / SCL 24 | PCA9685 `0x40` → driver servo **0** |

`0x70` ikut muncul di `Wire1`/`Wire2` karena **ALL-CALL bawaan PCA9685**, bukan mux
kedua — jadi PCA9685 tidak boleh satu bus dengan TCA9548A. Firmware
(`config.h`, `HexaServos`, `LidarArray`) **belum** disesuaikan dengan wiring ini
dan masih memakai library VL53L1X; lihat `TES_LIDAR/README.md`.

## Mulai dari mana

1. Baca [`program lama/ANALISIS.md`](program%20lama/ANALISIS.md) — apa yang salah di kode lama & kenapa.
2. [`HEXAPOD_KRSRI_2026/README.md`](HEXAPOD_KRSRI_2026/README.md) — arsitektur, wiring, kalibrasi, build.
3. [`RASPI_VISION_KRSRI/README.md`](RASPI_VISION_KRSRI/README.md) — sisi penglihatan.

Arsitektur: **Teensy** (real-time gerak 50Hz) + **Raspberry Pi 5** (vision) ↔ UART.
