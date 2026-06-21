# Elektrikal — Wiring & Perbaikan Hardware

Dokumentasi kelistrikan Hexapod KRSRI 2026 (Teensy 4.1, 2× PCA9685, 24 servo,
6× VL53L1X via mux, IMU Yahboom UART). Pelengkap firmware (`../HEXAPOD_KRSRI_2026/README.md`).

Dibagi dua:
- **Bagian A — Perbaikan wiring (rework PCB diizinkan).** Perubahan arsitektur yang menaikkan keandalan.
- **Bagian B — Kekurangan elektrikal per prioritas.** Checklist daya, proteksi, integritas sinyal.

> Konteks: robot **~5 kg** ditumpu **3 kaki** (tripod) → torsi & arus servo besar; ada riwayat
> **"servo rusak"**; belum ada **monitor tegangan**. Setelah R5 **tak ada retry** → satu brownout = gagal.
> **⚠️ Nilai bertanda `ISI:` wajib diukur/dilengkapi sesuai hardware nyata.**

---

# Bagian A — Perbaikan Wiring (rework)

## A.0 Peta pin Teensy 4.1

| Fungsi | Pin (sekarang) | Pin (rekomendasi rework) | Catatan |
|---|---|---|---|
| I2C **sensor** (mux+lidar) | `Wire` SDA 18 / SCL 19 | `Wire` SDA 18 / SCL 19 | tetap, @400 kHz |
| I2C **servo** (2× PCA9685) | `Wire` (gabung) | **`Wire1` SDA 17 / SCL 16** | **pisah**, @1 MHz |
| IMU UART | `Serial1` RX 0 / TX 1 | `Serial1` RX 0 / TX 1 | 921600 |
| Tombol START | 30 (INPUT_PULLUP) | 30 | ke GND |
| LED korban | 13 | pindah ≠ 13 bila perlu | pin 13 = LED onboard |
| **PCA9685 OE** (baru) | — (di-GND) | **GPIO mis. 2** | aktif-LOW, lihat A2 |
| **ADC baterai** (baru) | — | **A0 (pin 14)** mis. | lewat pembagi tegangan, ≤3.3 V |

Teensy 4.1 punya 3 bus I2C: `Wire`(18/19), `Wire1`(17/16), `Wire2`(25/24) — manfaatkan.

## A.1 Pisahkan bus I2C — perbaikan utama
**Sekarang:** satu `Wire` @400 kHz dipakai bersama 2× PCA9685 (servo) **dan** mux+6× VL53L1X (sensor).
Clock dipaksa turun ke 400 kHz mengikuti device terlambat (VL53L1X/TCA9548A Fast-Mode), dan tulis
servo berebut bus dengan baca lidar.

**Rework:**
- **`Wire` (18/19) @400 kHz** → khusus **sensor**: TCA9548A (`0x70`) + 6× VL53L1X di belakangnya.
- **`Wire1` (16/17) @1 MHz** → khusus **servo**: 2× PCA9685 (`0x40`/`0x41`). PCA9685 sanggup
  Fast-Mode-Plus 1 MHz.

**Manfaat:** tulis servo **~2,5× lebih cepat** (1 MHz vs 400 kHz) **dan** tak lagi berebut dengan
lidar → jitter turun, headroom untuk refresh > 50 Hz. Lihat juga A4 (batch write).

**Perubahan `config.h` (lakukan SETELAH PCB jadi — jangan sekarang, board lama masih 1 bus):**
```c
#define SERVO_I2C_BUS    Wire1     // dari Wire
#define SERVO_I2C_CLOCK  1000000   // dari 400000
// LIDAR_I2C_BUS tetap Wire @400000
```
Pull-up **terpisah per bus**. Wire1 @1 MHz butuh pull-up lebih kuat (**2.2 kΩ**); Wire @400 kHz cukup 4.7 kΩ.

## A.2 Kendali pin OE PCA9685 — boot & e-stop aman
**Sekarang:** OE (Output Enable, aktif-LOW) kemungkinan di-GND → output **selalu hidup**, termasuk
saat power-on sebelum firmware siap → servo bisa **menyentak ke pose acak** (bahaya + arus inrush).

**Rework:** sambungkan **OE kedua PCA9685** ke satu **GPIO Teensy** (mis. pin 2), dengan **pull-up ke 3.3 V**
(default disabled saat reset).
- Boot: tahan OE HIGH (output mati) → muat kalibrasi + hitung pose HOME → baru OE LOW (aktif).
- **E-stop / brownout**: tarik OE HIGH → drive servo **putus seketika** tanpa tunggu firmware.

Hook firmware: `PIN_SERVO_OE`; set HIGH di awal `setup()`, LOW setelah `robot.begin()`.

## A.3 Address strap & wiring PCA9685
- A0 di-strap: board pertama `0x40` (A0 terbuka), kedua `0x41` (A0 ke VCC). Verifikasi solder jumper.
- Logic I2C **3.3 V** (lihat B.4). V+ (rail servo) **terpisah** dari VCC logic — jangan dijembatani.

## A.4 (Opsional) Batch write 16 channel
Setelah bus servo @1 MHz, optimasi lanjutan: tulis 16 channel PCA9685 dalam **1 transaksi** (register
auto-increment dari `LED0_ON_L`) alih-alih `writeMicroseconds` per servo. Perlu sedikit ubah `HexaServos`.
Hanya bila butuh refresh > 50 Hz — ukur dulu (lihat firmware README, profiling loop).

---

# Bagian B — Kekurangan Elektrikal (per prioritas)

## Tier 1 — Kritis (bisa menggagalkan lomba)

### B.1 Monitor tegangan baterai (anti-brownout)
LiPo drop di bawah beban → Teensy/servo reset → robot diam.
- Pembagi tegangan V-batt → pin ADC (≤ **3.3 V**). Firmware: baca ADC → low-pass → histeresis →
  `robot.stop()` + LED bila < ambang (hook: `PIN_BATT`, `BATT_DIVIDER`, `BATT_MIN_V`).

| Knob | Nilai | Catatan |
|---|---|---|
| Jumlah sel LiPo | `ISI:` (2S/3S/4S) | menentukan V penuh/kosong |
| `PIN_BATT` (ADC) | `ISI:` (mis. A0) | pin analog bebas |
| Rasio divider | `ISI:` | V-batt-maks × rasio ≤ 3.3 V (beri margin) |
| `BATT_MIN_V` | `ISI:` | mis. 3.5 V/sel di bawah beban |

### B.2 Anggaran torsi & arus servo
24 servo bisa tarik **> 20 A puncak**; BEC kurang kuat → tegangan ambruk → B.1.
- Hitung torsi puncak per sendi vs rating servo (RDS3235 ≈ 35 kg·cm @ `ISI:` V).
- Rail servo **terpisah & kuat**; UBEC/regulator > arus puncak terukur. **Jangan** ambil daya servo dari 5 V Teensy.
- Batasi percepatan (slew gait sudah ada) → inrush turun.

| Knob | Nilai | Catatan |
|---|---|---|
| Tegangan rail servo | `ISI:` (mis. 6.0–7.4 V) | dalam rating servo |
| Arus puncak terukur | `ISI: A` | clamp meter saat berdiri + bergerak |
| Rating BEC/UBEC | `ISI: A kontinu / puncak` | harus > arus puncak |

### B.3 Distribusi daya & proteksi
- [ ] **Ground bersama** logic ↔ rail servo (wajib).
- [ ] Kabel daya AWG cukup (`ISI: AWG`) untuk arus puncak; jalur pendek.
- [ ] **Sekering** jalur baterai (`ISI: A`).
- [ ] **Saklar utama** + **proteksi polaritas terbalik** (MOSFET/dioda).
- [ ] **E-stop** memutus daya servo (padukan dengan OE, A.2).

## Tier 2 — Keandalan

### B.4 Decoupling & level logic
- [ ] Elektrolit besar (mis. **1000 µF**, `ISI:`) di rail servo tiap PCA9685.
- [ ] Ceramic **0.1 µF** dekat tiap VCC IC (PCA9685, mux, sensor).
- [ ] Bulk cap di rail **3.3 V** logic agar Teensy tak ikut brownout.
- [ ] **Teensy 4.1 TIDAK 5 V-tolerant** → semua SDA/SCL & I/O **maks 3.3 V**. Jalankan logic PCA9685 di 3.3 V;
      VL53L1X 3.3 V. Tak boleh ada device menarik SDA/SCL ke 5 V tanpa level-shift.

### B.5 Integritas bus I2C
- [ ] Pull-up per bus (A.1): 2.2 kΩ untuk Wire1 @1 MHz, 4.7 kΩ untuk Wire @400 kHz.
- [ ] Bus panjang/kapasitif → pertimbangkan buffer (PCA9517) atau turunkan clock.
- [ ] Retry/timeout I2C di firmware bila device hang (lidar fail-safe sudah ada).

### B.6 Watchdog & IMU
- [ ] **Watchdog** Teensy (`Watchdog_t4`/WDT_T4) → reset otomatis bila hang.
- [ ] Boot pasca-reset aman (OE menahan servo, A.2).
- [ ] Kabel UART IMU **pendek & terpilin**, jauh dari kabel arus (noise).
- [ ] Yaw (kompas) terganggu logam/medan → **jauhkan IMU** dari kabel daya & rangka besi
      (gate `IMU_MAX_YAW_JUMP` sudah ada, tapi hilangkan sumber gangguan lebih baik).

## Tier 3 — Pematangan

- [ ] **Konektor servo**: crimp kuat + strain relief; getaran kaki melonggarkan Dupont.
- [ ] **LED status/korban** (`PIN_LED_FOUND`) + resistor seri; cek aturan lomba.
- [ ] **Pendinginan** PCA9685/regulator bila panas setelah jalan lama.
- [ ] **Pelindung lidar**: < 10 cm dari lantai + tutup dari debu koral/lumpur (R5/R7).
- [ ] **Manajemen kabel**: ikat rapi, tak tersangkut rintangan / terinjak kaki.

---

## Checklist rework PCB
- [ ] Bus servo dipindah ke `Wire1` (16/17), pull-up 2.2 kΩ; bus sensor tetap `Wire` (18/19), 4.7 kΩ.
- [ ] OE kedua PCA9685 ke 1 GPIO (mis. 2) + pull-up 3.3 V.
- [ ] Pembagi tegangan baterai → pin ADC (≤ 3.3 V).
- [ ] Rail servo terpisah + caps (B.4); ground bersama; sekering + saklar + proteksi polaritas.
- [ ] Semua logic I2C/I/O 3.3 V (B.4).

## Checklist verifikasi sebelum lomba
- [ ] Ukur arus puncak (clamp) berdiri + berjalan vs rating BEC.
- [ ] Ukur tegangan rail servo saat semua servo bergerak (tak ambruk).
- [ ] Uji monitor baterai: turunkan V → robot berhenti aman sebelum brownout.
- [ ] Uji E-stop & OE: drive servo benar-benar putus.
- [ ] Goyang tiap konektor saat menahan beban → tak ada kedip/reset.
- [ ] Jalan 5 menit penuh (durasi lomba) tanpa reset/overheat.

> Isi semua `ISI:` setelah mengukur. Tanpa angka nyata, B.1–B.2 belum tuntas.
> Perubahan `config.h` di A.1 **hanya** setelah PCB rework selesai.
