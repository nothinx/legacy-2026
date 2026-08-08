# CLAUDE.md — Handoff

Repo: `legacy-2026`. Robot hexapod KRSRI, Teensy 4.1.
Lomba **18–19 Sept 2026** (Robot SAR UNLIMITED/UNDIP, Semarang).

## STATUS (9 Agustus 2026)

**Robot sudah berjalan.** Gait tripod maju/mundur/putar terbukti jalan lewat
`KALIBRASI/`. Pemetaan 24 servo terverifikasi fisik, 6 lidar terbaca semua.

**Kalibrasi kaki & badan sudah ada dan sudah dipakai di robot** — `TES_GERAK`
`J` (telapak per kaki) lalu `A` (rata badan). Panduan pakainya di
[`PANDUAN_KALIBRASI.md`](PANDUAN_KALIBRASI.md); rincian teknis di bagian
"TELAPAK PER KAKI" dan "RATA BADAN OTOMATIS" di bawah.

Yang BELUM:

- firmware utama `HEXAPOD_KRSRI_2026/` **tidak bisa jalan** dengan hardware
  sekarang — masih memakai library dan wiring yang salah (lihat bawah);
- **belum ada yang lolos compiler** sejak refaktor — tidak pernah ada
  Teensyduino di sesi mana pun;
- **rentang offset telapak 11,9 mm** di robot ini, jauh di atas 8 mm yang bisa
  dijelaskan galat gain servo. K1 paling menonjol. Kalibrasi menutupinya, tapi
  ada yang mekanis dan perlu diperiksa fisik.

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
| `TES_GERAK/` | gait (port firmware), body kinematics, pivot tertutup + kalibrasi der/siklus & mm/siklus, simulasi kering laju sendi, **kalibrasi telapak per kaki + rata badan otomatis pakai IMU** |

**PETA EEPROM** (Teensy 4.1, 4284 byte) — tiap alat menulis di alamat berbeda,
jangan ada yang bertabrakan:

| Alamat | Isi | Ditulis oleh |
|---|---|---|
| 0 .. ~280 | blok `Calib` firmware | `Calib.cpp` (`CALIB_ADDR 0`) |
| 1024 .. ~1150 | `ServoMap` | `servo_map.h` (`SM_EE_ADDR 1024`) |
| 1536 .. ~1560 | pemetaan & offset lidar | `MAP_LIDAR` (`EE_ADDR 1536`) |
| 1792 .. ~1812 | 4 arah kompas arena | `TES_IMU` (`EE_KOMPAS_ADDR 1792`) |
| 2048 .. ~2128 | der/siklus pivot, mm/siklus maju, tanda pivot, trim rata badan + acuan IMU + Jacobian sumbu, offset telapak 6 kaki | `TES_GERAK` (`EE_GERAK_ADDR 2048`) |

`MAP_LIDAR` semula memakai alamat 0 → perintah `e` menimpa kalibrasi firmware.
Sudah diperbaiki. Kalau Teensy pernah dipakai MAP_LIDAR versi lama, blok `Calib`
akan gagal CRC dan otomatis kembali ke default (tidak fatal, tapi trim hilang).

`servo_map.h` ada di TES_SERVO, SET_HOME, KALIBRASI, TES_IMU, TES_GERAK —
**kalau diubah, salin ke semuanya** (Arduino IDE tidak bisa berbagi file antar
folder sketsa). Sama untuk `kinematics.h` di KALIBRASI dan TES_GERAK.

---

# VERIFIKASI SENSOR (sebagian sudah selesai)

Fokus sensor saja. **Belum menyentuh algoritma** (navigasi, FSM misi, vision).

Selesai: IMU (§1) dan arah lidar (§2). Belum: tombol START + LED (§3), monitor
baterai (§4, butuh keputusan hardware), UART ke Raspberry Pi (§5).

### 1. IMU Yahboom 10-axis — SUDAH DIUJI, YAW LAYAK

**Hasil uji gangguan servo (`g`, 5 Agustus 2026)** — robot di dudukan, kaki
menggantung:

| Fase | yaw | sebar | \|mag\| |
|---|---|---|---|
| 1 servo mati | 40,9° | 0,2° | 7268 |
| 2 servo bertenaga diam | 38,9° | 2,2° | 7826 |
| 3 servo bergerak | 38,6° | 1,4° | 7941 |

Gangguan **statis 2,0°**, **dinamis 2,2°** dengan sebar 1,4°, `|mag|` naik 9%.
Semuanya jauh di bawah ambang 5° → **yaw layak jadi acuan heading utama**.
Rencana cadangan berbasis sudut dinding lidar tidak jadi diperlukan (tapi
sepasang lidar samping tetap berguna untuk menyikukan robot saat mencatat arah).

`TES_IMU` sudah punya kompas 4 arah (`c<n>` catat, `k` tabel + cek kelinieran,
`o<n>` pivot, EEPROM 1792) dengan sektor berhisteresis 40°/50°.
`gaitPutar()` di TES_IMU masih placeholder — `o<n>` di sana hanya untuk memutar
robot manual dan memeriksa tanda. **Pivot yang sebenarnya sudah ada di
`TES_GERAK/` (`C` kalibrasi, `O`/`o` pivot tertutup)**, membaca 4 arah kompas
dari EEPROM 1792 yang sama.


Protokol WIT, frame `0x55`, **`Serial2` = RX2 pin 7 / TX2 pin 8**.

**Setelan IMU (aplikasi WIT, 5 Agustus 2026):** baud **230400** (maks modul ini;
bawaan pabrik 9600, jadi `IMU_BAUD 921600` di `config.h` salah), output rate
**200 Hz**, content **4**: Euler `0x53` + angular velocity `0x52` + magnetism
`0x54` + acceleration `0x51`. Terpakai 38% kapasitas.

Rate 200 Hz dipilih karena `CONTROL_HZ` = 100 Hz: IMU 100 Hz yang tak tersinkron
membuat sebagian tick kontrol dapat nol sampel segar. Oversampling 2× menjamin
selalu ada yang baru.

**Angular velocity wajib aktif** — gyro Z jadi suku D PD heading (tanpa
mendiferensiasi yaw yang berisik) sekaligus **cadangan belok** kalau uji `g`
menyatakan yaw tak layak: integrasi gyro Z cukup akurat untuk manuver 90° pendek.

Kalau return rate melebihi kapasitas baud, frame terpotong → muncul sebagai
checksum gagal, bukan "lambat". `TES_IMU` punya `B` (pindai baud) dan `f`
(hitung kapasitas + laju sudut sebenarnya).

**Kalibrasi magnetometer harus dilakukan saat IMU SUDAH TERPASANG di robot**
(servo tanpa tenaga, putar seluruh robot). Kalibrasi di meja hanya membuang
medan meja. Ini menghilangkan komponen **statis**, sehingga uji `g` benar-benar
mengukur komponen **dinamis** — satu-satunya yang tak bisa diperbaiki.

**`config.h` masih menulis `IMU_SERIAL Serial1` — salah, ubah ke `Serial2`.**
Akibatnya `Serial2` tidak lagi tersedia untuk Raspberry Pi. Pi harus pindah,
dan **tidak boleh** ke `Serial4` (pin 16/17 = `Wire1`) atau `Serial6`
(pin 25/24 = `Wire2`) — pinnya bentrok dengan I2C. Yang bebas: `Serial1` (0/1),
`Serial3` (15/14), `Serial5` (21/20), `Serial7` (28/29), `Serial8` (34/35).

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

### 5. UART ke Raspberry Pi 5 — PORTNYA HARUS DIPILIH ULANG

`Serial2` sekarang dipakai IMU. Pi belum punya port; pilih dari `Serial1` (0/1),
`Serial3` (15/14), `Serial5` (21/20), `Serial7` (28/29), `Serial8` (34/35).
`Serial4` dan `Serial6` **tidak boleh** — pinnya bentrok `Wire1`/`Wire2`.

Uji jalur fisiknya saja, bukan vision-nya: kirim/terima frame uji, ukur baud
yang stabil, pastikan GND tersambung. Parser `VIC ...` menyusul bersama vision.

### 6. Kalibrasi kaki & badan — SUDAH ADA, sudah dipakai di robot

`TES_GERAK` `J` lalu `A`, lihat [`PANDUAN_KALIBRASI.md`](PANDUAN_KALIBRASI.md).
Lakukan sebelum uji berjalan mana pun: kaki menggantung atau badan miring
membuat beban tiap kaki tidak rata, dan itu mencemari kalibrasi pivot (`C`) dan
odometri (`M`).

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
- `Hexapod`: **trim rata badan** (`_levelX/_levelY`, ditambahkan ke `_rotX/_rotY`
  di `solvePose`, TIDAK ikut dinolkan saat pose direset) + **offset telapak per
  kaki** (`_zOff[6]`, ditambahkan ke titik netral tiap kaki di `HexaGait` supaya
  ikut terbawa saat berjalan) — 8 param di `Calib`, diisi dari hasil `J` + `A`
  di `TES_GERAK`
- gate `getRangeStatus()==0` diganti: buang nilai ≥ 8000, **nilai > `LIDAR_MAX_CM`**,
  & `timeoutOccurred()` — yang tengah paling gampang terlupa

Baru sesudah itu: algoritma (nav PD sudah ada, FSM misi ↔ arena R1–R11,
watchdog, Mission ↔ Vision).

## GERAK — SUDAH DIPERBAIKI (7 Agustus 2026, dihitung, belum diuji di robot)

Empat kesalahan ditemukan lewat `TES_GERAK/` dan **sudah diperbaiki di firmware
juga**. Detail & angkanya di `TES_GERAK/README.md`; semuanya diverifikasi
`TES_GERAK/test_motion.py`. **Belum ada yang lolos compiler.**

1. **`solvePose` memakai invers palsu.** `rotatePoint(p, -roll, -pitch, -yaw)`
   itu fungsi maju dengan sudut dinegatifkan, bukan invers `Rz*Ry*Rx`. Tepat
   kalau cuma satu sumbu aktif; begitu roll+pitch bersamaan (stabilisasi IMU di
   medan miring, `STAB_MAX_DEG` 15) ujung kaki melenceng ~10 mm.
   → `rotatePointInv()` di `types.h`, urutan rotasi dibalik.
   Ikutannya: nama `roll(X)/pitch(Y)` di `types.h` terbalik untuk frame ini
   (+X kanan, +Y depan ⇒ rotasi-X = pitch). Sekarang `rotX/rotY/rotZ`, dan
   `Hexapod` menyimpan `_rotX/_rotY/_rotZ`. Sisanya soal pemasangan IMU:
   **`STAB_SWAP_ROLL_PITCH` di `config.h` masih 0 dan belum diverifikasi fisik**
   — cara mengujinya ada di komentarnya (2 menit, tanpa servo). Sekarang ada cara
   kedua yang otomatis: `A` di `TES_GERAK` mengukur Jacobian sumbu IMU dan
   **mencetak nilai yang benar** untuknya sebagai efek samping.
2. **Fase gait `fmod(elapsed, cycleTime)` melompat saat profil di-ramp.** Karena
   `elapsed` terus tumbuh, mengubah `cycleTime` setelah berjalan 30 detik
   menggeser fase sampai **0,48** — kedua tripod bertukar peran seketika dan
   robot menjatuhkan badan, tepat saat masuk tangga.
   → fase diakumulasi `dt/cycleTime`; `_cycleStart` dihapus.
3. **`GaitProfile` butuh `standRadius`.** `STAND_RADIUS` dulu `#define`, jadi
   profil tak bisa mengecilkan bentang kaki dan `profileNarrow()` mustahil.
   → jadi field profil, ikut di-ramp; `Hexapod::profileNarrow()` sudah ada.
4. **Vektor langkah tidak dinormalkan — yang terparah.** Perintah maju dan putar
   saling menambah: "maju 0,8 + putar 0,7" (persis keluaran `followWall()`
   setiap saat) membuat kaki terluar melangkah **115 mm** padahal `stepLength`
   60 mm, dan coxa diminta **417°/s**; "maju 0,8 + putar 1,0" bahkan 520°/s.
   Jadi kondisi terberat justru kondisi normal.
   → keenam vektor dibagi SATU faktor skala supaya yang terpanjang pas
   `stepLength`. 520 → **219°/s** tanpa mengubah bentuk gerak. Efek samping yang
   perlu diingat: perintah putar > **0,63** tidak menambah cepat (mentok
   normalisasi), jadi PD pivot jenuh pada error ≳31° — itu wajar.

**Profil medan disetel ulang** supaya laju sendi < ~260°/s (RDS3235 berbeban):
TANGGA 1200→**1800 ms** (femur 388→252), MERUNDUK 900→**1100 ms** (303→248).
DATAR sengaja tetap 900 ms (256°/s, tepat di batas) — memperlambatnya berarti
kehilangan 10% kecepatan lomba; naikkan ke 1000 hanya kalau servo terlihat
tertinggal di lantai. Menggeser menyamping (strafe) masih 270–277°/s dan
dibiarkan: `Navigation` tidak pernah mengisi `NavCmd.strafe`.

**R11 celah 30 cm**: berdiri normal 32 cm. `N300` menemukan **bentang kaki 45 mm
→ badan 27,0 cm** (48 mm tidak muat); sudah jadi profil SEMPIT & `profileNarrow()`.
Yang melebarkan badan ternyata **strafe** (31,5 cm), **bukan berbelok** (tetap
27,0 cm) — kaki tengah bergerak searah badan saat berputar. Jadi mengoreksi
heading di dalam celah aman, dan itu memang yang dilakukan navigasi.

## TELAPAK PER KAKI (9 Agustus 2026, `TES_GERAK` `J`/`K`/`j`)

Gejala nyata di robot: **pada 90° semua sendi bagus, tapi pada pose berdiri ada
kaki yang tidak menapak.** Ini bukan soal offset netral.

`trim[]` di `servo_map.h` satuannya **µs** dan ditambahkan setelah konversi
derajat→pulse — offset murni, jadi kalau 90° benar semua sudut ikut benar
**selama gain-nya benar**. Gain itu yang tidak dijamin: 500–2500 µs dianggap
tepat 180° (11,11 µs/°), tiap servo beda beberapa persen. Galatnya **nol tepat
di 90°** dan tumbuh sebanding jaraknya dari 90 — dan berdiri memang jauh dari 90
(femur −10,6°, tibia −8,0°). Angka (`test_motion.py` bagian 9): 1 mm telapak =
0,72° femur = 8 µs; gain meleset 5% → 0,7 mm; 10% → 1,5 mm. Bandingkan
**1 gerigi spline 25T = 14,4° = 20 mm** — itu pasti terlihat di 90°, jadi gerigi
horn **bukan** penyebabnya. Sisanya: gain, panjang femur/tibia tak persis 80/90,
dan **sag servo saat berbeban** (tak muncul di 90° tanpa beban).

Kesimpulan yang menentukan desain: koreksi harus **diukur pada pose berdiri dan
dalam keadaan berbeban**, dan bentuknya **per kaki** (`Motion::zOff[6]` mm,
negatif = kaki dipanjangkan), bukan per servo.

**`J` otomatis** — IMU jadi sensor sentuh tanpa hardware tambahan. Robot ditumpu
**satu tripod**, tiga kaki lain diangkat; kaki uji diturunkan bertahap sampai ia
mengungkit badan (terlihat di roll/pitch). Harus tripod, bukan 5 kaki: dengan 3
titik tumpu badan tertentu penuh sehingga kaki keempat yang menyentuh **pasti**
memiringkan badan; dengan 5 titik sistemnya berlebih dan sentuhan tak terdeteksi.
Pindai kasar 1,5 mm → halus 0,3 mm → interpolasi di perpotongan ambang; ambang
`max(0,6°, 5× sebar derau)`. Butuh lantai keras (karpet/busa: telapak tenggelam,
tak pernah mengungkit).

**Hanya SATU tripod yang jadi acuan — itu syarat kebenaran.** Tiga titik selalu
*tepat* membentuk bidang, jadi galat kaki penumpu seluruhnya berupa kemiringan
badan (tugas `A`); ketiga kaki yang diangkat diukur terhadap bidang itu, dan
hasilnya langsung membuat keenam telapak sebidang — **eksak, sekali ukur, tanpa
iterasi**. Versi pertama mengukur **kedua** tripod lalu memperbarui keenam offset
serentak: **tidak konvergen, berayun `9,80 → 0 → 9,80 → 0`** (tiap tripod diukur
memakai offset tripod lawannya). Sekarang jadi uji regresi. Karena acuan tidak
berubah, pengukuran kedua memberi angka identik — itulah yang membuatnya sah
sebagai uji konsistensi.

**Komponen bidang dibuang di akhir** (kuadrat terkecil pada `{1, y, x}`).
Menambahkan bidang ke keenam offset tidak mengubah kesebidangan telapak, hanya
sikap badan — jadi boleh dipindahkan ke `A` cuma-cuma, dan offset terbesar turun
**setengahnya** (10,0 → 5,0 mm) sehingga jangkauan IK tidak dimakan percuma.
Karena itu jendela pindai **±18 mm lebih lebar** dari batas offset akhir ±12 mm:
pengukuran mentah ~2× hasil akhir. `J` idempoten (dijalankan ulang, geser 9e-16 mm).

**`K` manual** — uji kertas, tanpa IMU. Jaring pengaman kalau `J` gagal, dan
cara mendekatkan kaki yang melesetnya di luar ±8 mm.

**Urutan wajib `J` lalu `A`.** Keduanya orthogonal: `zOff` membuat keenam telapak
**sebidang** (IMU tidak bisa melihat ini), `trimRoll/Pitch` memiringkan badan
supaya **rata** terhadap gravitasi (`zOff` tidak tahu bidangnya miring). Terbalik,
`A` meratakan bidang yang salah karena IMU hanya melihat bidang lewat kaki yang
menapak — `A` sekarang memperingatkan kalau `zOff` masih nol semua.

`zOff` ditambahkan di `computeHome()`, jadi ikut terbawa saat **berjalan** juga,
bukan cuma berdiri — kalau tidak, kaki menggantung lagi begitu melangkah.
±12 mm berseling masih muat di IK pada keempat profil.

## RATA BADAN OTOMATIS (9 Agustus 2026, `TES_GERAK` `A`/`Q`/`Z`/`a`)

Keluhan: badan tetap miring kiri/kanan walau trim servo sudah disetel tangan.
Penyebabnya menumpuk (galat gain servo, panjang kaki tak sama, rangka, sag servo
berbeda per kaki) dan **mata tidak bisa membedakan badan miring dari lantai
miring**. IMU bisa — ia mengukur gravitasi.

`A` menutup lingkarannya: miringkan badan sampai IMU rata, simpan sudutnya
sebagai `Motion::trimRoll/trimPitch` yang berlaku di **setiap** `solve()`,
termasuk saat berjalan, dan sengaja terpisah dari `bodyRoll/bodyPitch` supaya
`b0` tidak menghapusnya. Rinciannya di `TES_GERAK/README.md`; tiga hal yang
membuatnya bukan sekadar pengurangan sudut:

1. **Sumbu IMU diukur, tidak diasumsikan.** Badan dimiringkan ±5° tiap sumbu →
   Jacobian 2×2 → koreksi Newton `J⁻¹·galat`. Kebal terhadap roll/pitch yang
   tertukar atau terbalik, dan **mencetak nilai `STAB_SWAP_ROLL_PITCH` yang
   benar**. Determinan < 0,25 = ditolak (kaki selip / robot dipegang / IMU beku).
2. **`Z`** merekam acuan "rata" untuk menyerap IMU yang tidak sejajar pelat badan
   (butuh waterpass; tanpa itu robot diratakan sampai IMU nol = miring sebesar
   kesalahan pasang). Untuk pemasangan yang jauh dari tegak acuannya otomatis:
   **IMU di robot ini terpasang TERBALIK — roll diam di −179,98°**. Modul selalu
   dipasang pada kelipatan 90°, jadi kalau simpangan > 45° acuan dibulatkan ke
   kelipatan 90° terdekat.
3. **`Q`** memisahkan badan dari lantai lewat putar 180° di titik yang sama:
   `badan = -(t1+t2)/2`, `lantai = -(t1-t2)/2`. Tanpa ini kemiringan lantai
   tempat uji ikut tersimpan sebagai trim.

**Batasnya jelas dan harus diingat:** satu IMU = 2 DOF, galat tinggi 6 kaki =
6 DOF. Yang bisa dikoreksi hanya **bidang** melalui keenam telapak. Satu kaki
yang pendek sendiri sampai **menggantung** tidak terlihat IMU dan tidak bisa
diperbaiki dengan memiringkan badan — karena itu `A`/`a` mencetak ekuivalen trim
dalam **mm per telapak**; kaki yang angkanya menonjol dibetulkan di `KALIBRASI/`.

Ongkos trim saat berjalan (DATAR/maju, dari `test_motion.py`): 3° → +4°/s,
5° → +6°/s, 8° → +10°/s laju sendi, dan tidak pernah keluar jangkauan IK.
Trim > 8° diperingatkan: itu kerusakan mekanis, bukan setelan.

**Dua jebakan yang sudah menggigit di robot nyata (9 Agustus 2026), berlaku juga
untuk firmware `Imu.cpp`/`Hexapod`:**

1. **Sudut IMU harus dihitung tahan lipat ±180.** Roll di robot ini diam di
   −179,98°, dan di sana +179,9 dengan −179,9 **bertetangga** (0,2°), bukan
   berjauhan (359,8°). Merata-ratakan mentah memberi **0°** — badan terbalik
   dilaporkan tegak — dan mengurangkan mentah melewati ambang apa pun. Rerata
   dihitung sebagai selisih terhadap sampel pertama; tiap pengurangan lewat
   `wrap180()`. Uji: `test_motion.py` bagian 10.
2. **Jangan mematok jendela pengukuran dalam milidetik.** Laju frame sudut yang
   benar-benar sampai **terukur ~50 Hz**, bukan 200 Hz seperti yang disetel di
   aplikasi WIT. Berhenti berdasarkan **jumlah sampel** dengan batas waktu
   sebagai jaring pengaman. `T` sekarang menampilkan laju terukur.

Ikutannya soal pelaporan: kegagalan pengukuran **wajib** dibedakan dari temuan
mekanis. Versi pertama `J` melaporkan IMU yang macet sebagai "kaki kependekan >8
mm" — dan itu mengirim orang membongkar mekanik yang tidak apa-apa.

Blok EEPROM 2048 naik ke **versi 2**; versi 1 tetap terbaca (pivot & odometri
tidak hilang, trim rata nol).

## CATATAN

- Firmware **belum pernah lolos compiler** sejak refaktor — tidak ada
  Teensyduino di sesi-sesi ini. Compile di Arduino IDE sebelum percaya.
- **Jebakan `.ino`**: Arduino IDE menyisipkan prototipe fungsi hasil generate di
  atas badan sketsa, sebelum `struct`/`enum` buatan sendiri dideklarasikan. Tipe
  buatan sendiri **tidak boleh muncul di tanda tangan fungsi** dalam file `.ino`
  (`'X' does not name a type`) — taruh tipenya di file `.h`, atau ganti parameter
  jadi `(const void*, size_t)`. Kena di `MAP_LIDAR` (`storeSum`).
- `TES_GERAK/test_motion.py` — uji matematika gerak **tanpa hardware**, replika
  Python dari `motion.h` (tak butuh g++, yang memang belum terpasang). Jalankan
  ulang tiap kali `motion.h` diubah.
- Preferensi user: **JANGAN pakai Co-Authored-By Claude** di commit message.
- Library: Teensyduino, Adafruit PWM Servo Driver, **VL53L0X by Pololu**.
- Batas kecepatan servo: RDS3235 ~0,15 s/60°. Bottleneck bukan I2C
  (18 servo @400 kHz ≈ 2,9 ms/refresh). Detail di `KALIBRASI/README.md`.
