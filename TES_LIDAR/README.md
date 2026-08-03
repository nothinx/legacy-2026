# TES_LIDAR — diagnosa 6× TOF200C (VL53L0X) + TCA9548A (Teensy 4.1)

Sketsa berdiri sendiri (tidak menyentuh `HEXAPOD_KRSRI_2026/`). Buka
`TES_LIDAR.ino` di Arduino IDE → Board **Teensy 4.1** → Serial Monitor **115200**,
line ending **Newline**.

Library yang dibutuhkan: **VL53L0X by Pololu** (Library Manager).
*Bukan* SparkFun VL53L1X — lihat di bawah.

## Konfigurasi bus I2C saat ini

| Bus | Pin Teensy 4.1 | Isi |
|---|---|---|
| `Wire`  | SDA 18 / SCL 19 | **TCA9548A `0x70`** → 6× VL53L0X (TOF200C) di channel mux |
| `Wire1` | SDA 17 / SCL 16 | **PCA9685 `0x41`** (driver servo 1) |
| `Wire2` | SDA 25 / SCL 24 | **PCA9685 `0x40`** (driver servo 0) |

Tiga bus terpisah — **berbeda dari `config.h` firmware** yang masih menganggap
semuanya satu bus di `Wire`.

## Temuan sejauh ini

**1. Bus lidar = `Wire` (SDA 18 / SCL 19), mux TCA9548A-nya asli.**
Terbukti dari perilaku, bukan sekadar ACK: channel 0/6/7 kosong sementara
channel 1–5 berisi device. Pemilihan channel benar-benar mengubah isi bus,
jadi mux itu nyata dan berfungsi.

**1b. `0x70` di `Wire1`/`Wire2` bukan hantu — itu ALL-CALL PCA9685.**
(Koreksi atas dugaan awal.) PCA9685 dari pabrik punya register `ALLCALLADR`
= `0xE0` (8-bit) = **`0x70` 7-bit**, dan bit ALLCALL di `MODE1` aktif secara
default. Jadi setiap PCA9685 menjawab di dua alamat: alamatnya sendiri **dan**
`0x70`. Itu persis pola scan Anda — `Wire1` = 0x41 + 0x70, `Wire2` = 0x40 + 0x70.

Konsekuensinya penting: **PCA9685 dan TCA9548A tidak boleh satu bus** tanpa
mematikan ALL-CALL, karena keduanya menjawab `0x70` → tabrakan. Wiring sekarang
aman karena sudah beda bus. `TES_SERVO` punya perintah `A` untuk mematikan
ALL-CALL bila suatu saat bus disatukan (tidak permanen, hilang saat power cycle).

**2. Sensornya VL53L0X (TOF200C), bukan VL53L1X.** Inilah sebab `GAGAL init
(kode 1)` padahal `0x29` menjawab ACK: kedua chip memakai alamat 0x29 yang
sama, sehingga scanner I2C tidak bisa membedakannya, tetapi urutan register
inisialisasinya berbeda total. Library VL53L1X menulis konfigurasi ke register
yang tidak ada di VL53L0X → gagal. Konsekuensi teknis:

| | VL53L1X (diasumsikan) | VL53L0X (kenyataan) |
|---|---|---|
| Jangkauan maks | 400 cm | **~200 cm** (mode long range) |
| Library | SparkFun VL53L1X | **Pololu VL53L0X** |
| Mode jarak | `setDistanceModeShort()` | `setSignalRateLimit()` + `setVcselPulsePeriod()` |
| Status hasil | `getRangeStatus()` (0 = valid) | tidak ada; deteksi lewat timeout & nilai ≥ 8000 |

**3. Baru 5 sensor terdeteksi (ch1–ch5), ch0 kosong.** Kalau memang 6 lidar
terpasang di ch0–ch5, berarti satu unit di ch0 mati / kabelnya lepas / salah
channel. Jalankan `m` untuk memastikan pemetaan channel sesungguhnya.

## Perintah

| Perintah | Fungsi |
|---|---|
| `d` | Deteksi otomatis: scan + uji keaslian mux + identifikasi chip, di ketiga bus |
| `b0` `b1` `b2` | Pilih bus manual (Wire / Wire1 / Wire2) |
| `s` | Scan alamat I2C bus aktif |
| `m` | Uji mux 0x70 + scan tiap channel **dan identifikasi chip tiap sensor** |
| `i` | Init semua lidar |
| `a` | **Baca semua** lidar terus-menerus (satu baris per sweep + waktu sweep) |
| `r<n>` | Baca **satu** channel terus-menerus, mis. `r2` |
| `t<n>` | Baca **satu** channel sekali, detail |
| `p<n>` | **Probe mendalam** satu channel: dump register ID + uji tulis |
| `c<n>` | Pilih channel mux manual lalu `s` — melacak kabel per channel |
| `k1` / `k4` | Turunkan / naikkan clock I2C ke 100 kHz / 400 kHz, lalu `i` lagi |
| `l` | Ganti long range ↔ standar, lalu `i` lagi |
| `x` | Berhenti membaca |
| `h` | Bantuan |

Saat boot langsung dijalankan `d` + `i`.

## Membaca hasilnya

- **Identifikasi chip** (perintah `m` / `p`):
  `C0=0xEE C2=0x10` → VL53L0X. `reg16 0x010F/0x0110 = 0xEA 0xCC` → VL53L1X.
  Kalau keduanya sampah, komunikasi yang bermasalah — coba `k1` (100 kHz).
- **Uji mux**: `tulis 0x08 -> err=0 baca=0x08` di semua pola = TCA9548A asli.
- **Hantu**: bila `0x29` terlihat saat semua channel OFF, ada sensor tersambung
  langsung ke bus utama — akan menabrak yang lain karena alamatnya sama.
- **Pembacaan**: `TO` = timeout (sensor diam), `--` = tak ada target /
  di luar jangkauan. VL53L0X mengembalikan ~8190 mm bila tidak melihat apa pun.
- **Waktu sweep** `a`: sensor dijalankan mode kontinu, jadi sweep 5–6 sensor
  umumnya 30–140 ms tergantung fase pengukuran. Ini sketsa tes yang blocking;
  jangan pakai angka ini untuk menilai loop rate firmware.

## Bila masih gagal init

1. `k1` → 100 kHz. Kabel dupont panjang sering gagal di 400 kHz saat burst
   init (VL53L0X menulis puluhan register berurutan).
2. `p<n>` pada channel yang gagal — lihat apakah register ID terbaca bersih.
3. Pull-up: mux **tidak** meneruskan pull-up; tiap channel butuh pull-up
   sendiri. Modul TOF200C umumnya sudah membawa pull-up di board-nya.
4. Daya: VL53L0X menyalakan VCSEL saat init. 6 modul + kabel panjang di rail
   3V3 Teensy bisa drop. Ukur 3V3 di modul terjauh saat init.
5. Pin **RESET** TCA9548A harus HIGH, bukan floating.

## Dampak ke firmware (belum dikerjakan)

`HEXAPOD_KRSRI_2026/LidarArray.h/.cpp` masih memakai `SparkFun_VL53L1X`,
`config.h` memakai `LIDAR_MAX_CM 400`, dan seluruh device masih diasumsikan
satu bus `Wire`. Semuanya harus disesuaikan setelah tes ini beres:

- `SERVO_I2C_BUS` tidak bisa satu makro lagi — driver 0 di `Wire2`, driver 1
  di `Wire1`, lidar di `Wire`. `HexaServos` perlu bus per-driver;

- ganti `SFEVL53L1X` → `VL53L0X` (Pololu), satu objek **per channel**
  (Pololu menyimpan `stop_variable` hasil init milik sensor tsb — objek tidak
  boleh dipakai bergantian antar sensor, ini jebakan halus);
- `LIDAR_MAX_CM` 400 → **200**;
- gate `getRangeStatus()==0` diganti: buang nilai ≥ 8000 dan `timeoutOccurred()`;
- state machine non-blocking: pakai `startContinuous()` + polling
  `RESULT_INTERRUPT_STATUS` agar tidak busy-wait.

Filter median-3 + EMA + timeout fail-safe yang sudah ada tetap dipakai apa adanya.
