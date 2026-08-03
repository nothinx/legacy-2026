# TES_SERVO & SET_HOME — pemetaan servo dan posisi home

Dua sketsa berdiri sendiri untuk Teensy 4.1. Library: **Adafruit PWM Servo
Driver Library**. Serial Monitor **115200**, line ending **Newline**.

- `TES_SERVO/` — cari tahu channel PCA9685 mana milik sendi mana, **satu per satu**.
- `SET_HOME/` — taruh semua servo ke posisi home memakai hasil pemetaan itu.

Keduanya berbagi `servo_map.h`. **Bila file itu diubah, salin ke kedua folder** —
Arduino IDE tidak bisa berbagi file antar folder sketsa.

## Wiring yang dipakai

| Bus | Pin Teensy 4.1 | Isi |
|---|---|---|
| `Wire`  | SDA 18 / SCL 19 | TCA9548A `0x70` → 6× VL53L0X |
| `Wire1` | SDA 17 / SCL 16 | PCA9685 `0x41` → **driver 1** |
| `Wire2` | SDA 25 / SCL 24 | PCA9685 `0x40` → **driver 0** |

Perhatikan urutannya: **driver 0 (`0x40`) ada di `Wire2`**, driver 1 (`0x41`) di
`Wire1` — kebalikan dari tebakan intuitif. Diatur di `servo_map.h` (`BUS_DRV0`,
`BUS_DRV1`).

`0x70` yang ikut muncul di `Wire1`/`Wire2` saat scan adalah **ALL-CALL address
bawaan PCA9685** (`ALLCALLADR` = `0xE0` 8-bit = `0x70` 7-bit, aktif dari pabrik),
bukan mux kedua. Karena itu PCA9685 **tidak boleh** satu bus dengan TCA9548A
kecuali ALL-CALL dimatikan — perintah `A` di TES_SERVO melakukannya (tidak
permanen, hilang saat power cycle).

## Slot logis & penomoran kaki

24 slot: `0..17` = 6 kaki × (coxa, femur, tibia), `18..20` = lengan kanan
(base, shoulder, gripper), `21..23` = lengan kiri.

Penomoran kaki mengikuti `config.h` firmware (`BODY_LEG_ORIGINS` / `BODY_LEG_ANGLE`).
Dilihat **dari atas**, depan robot di atas, `+X` = kanan, `+Y` = depan:

```
                       DEPAN (+Y)
              K5 \                    / K0
        (ki-depan) \________________/ (ka-depan)
                   |                |
        K4 --------|     BADAN      |-------- K1
      (ki-tengah)  |                |     (ka-tengah)
                   |________________|
              K3 /                    \ K2
      (ki-blkg) /                       \ (ka-blkg)
                       BELAKANG (-Y)
     KIRI (-X)                             KANAN (+X)
```

| Kaki | Posisi | Pangkal coxa (x, y) mm | Arah hadap |
|---|---|---|---|
| K0 | kanan-depan | (45, 78) | 60° |
| K1 | kanan-tengah | (90, 0) | 0° |
| K2 | kanan-belakang | (45, −78) | −60° |
| K3 | **kiri-belakang** | (−45, −78) | −120° |
| K4 | kiri-tengah | (−90, 0) | 180° |
| K5 | **kiri-depan** | (−45, 78) | 120° |

⚠️ Perhatikan K3 = **kiri-BELAKANG** dan K5 = **kiri-DEPAN**, bukan sebaliknya.
Penomoran berputar berlawanan arah jarum jam dari kanan-depan. Ini kesalahan yang
paling sering terjadi saat memetakan — verifikasi dengan `v<n>` di SET_HOME.

Jadi slot `4` = `K1_FEMUR` (femur kaki kanan-tengah), slot `17` = `K5_TIBIA`.

## Hasil pemetaan (Agustus 2026)

Setelah dikoreksi lewat pengecekan fisik — **driver 0 ternyata sisi KIRI**:

| Driver | Bus | Alamat | Sisi | ch 0–2 | ch 4–6 | ch 8–10 |
|---|---|---|---|---|---|---|
| 0 | `Wire2` | `0x40` | **kiri** | **K5** ki-depan | **K4** ki-tengah | **K3** ki-belakang |
| 1 | `Wire1` | `0x41` | **kanan** | **K2** ka-belakang | **K1** ka-tengah | **K0** ka-depan |

Tiap kaki memakai 3 channel berurutan: coxa, femur, tibia. Channel 3, 7, 11–15
kosong. Lengan (slot 18–23) belum dipetakan — dikerjakan terakhir; SET_HOME
melewati slot yang belum dipetakan.

Pada kedua driver, nomor channel naik searah **belakang → depan pada driver 1**
dan **depan → belakang pada driver 0** — konsisten dengan dua board PCA9685 yang
dipasang saling bercermin.

Dibanding `SERVO_PIN_MAP` lama di `config.h`: urutan channel per driver ternyata
**sudah benar**, yang salah adalah **driver mana untuk sisi mana** — tabel lama
menaruh kaki kanan di driver 0, padahal driver 0 (`0x40`) sekarang di `Wire2`
dan melayani sisi kiri. Tetap ganti dengan keluaran `D`.

### Kalau sisi tertukar

Jangan petakan ulang 18 servo. Di TES_SERVO:

- **`M`** — tukar sisi kiri↔kanan sekaligus (K0↔K5, K1↔K4, K2↔K3, dan lengan
  kanan↔kiri). Pin, `invert`, dan `trim` ikut berpindah.
- **`T<a> <b>`** — tukar dua kaki saja, mis. `T0 5`.

Lalu `L` untuk memeriksa, `S` untuk menyimpan.

---

# TES_SERVO — memetakan satu per satu

## Alur kerja

1. **`V`** — pastikan kedua PCA9685 menjawab dan `PRESCALE` ≈ 121 (bukti chip
   asli pada 50 Hz, bukan sekadar ACK kosong).
2. **`c0`** — pilih driver 0 channel 0. Hanya channel ini yang diberi sinyal;
   semua channel lain dimatikan, jadi servo lain bebas dan tidak melawan.
3. **`W`** — servo tersebut bergoyang terus. Jalan keliling robot, lihat **servo
   mana yang bergerak**. Tekan Enter untuk berhenti.
4. **`=<slot>`** — kaitkan channel ini ke slot itu. Contoh: kalau yang bergerak
   ternyata tibia kaki kanan-tengah, ketik `=5`. (`?` menampilkan daftar slot.)
5. **`n`** — lanjut ke channel berikutnya. Ulangi 3–4.
6. Setelah driver 0 selesai, **`d1`** lalu ulangi.
7. **`L`** — periksa tabel; pastikan tidak ada yang "BELUM".
8. **`S`** — simpan ke EEPROM. **`D`** — cetak kode siap tempel ke `config.h`.

Kalau satu pin di-assign ke dua slot, assign yang lama otomatis dilepas dan
diberitahukan — jadi tidak mungkin ada duplikat diam-diam.

## Perintah

| | |
|---|---|
| `V` | verifikasi kedua PCA9685 (+ status ALL-CALL) |
| `d0` `d1` | pilih driver 0 (`0x40`@Wire2) / 1 (`0x41`@Wire1) |
| `c<n>` | pilih channel 0–15 (channel lain dimatikan) |
| `n` / `b` | channel berikutnya / sebelumnya |
| `w` / `W` | goyang 6× / goyang terus — **cari servo mana yang bergerak** |
| `+` `-` | geser pulse 25 µs;  `u<us>` set pulse langsung (mis. `u1200`) |
| `9` | kembali ke 1500 µs (90°) |
| `=<slot>` | kaitkan channel ini ke slot logis (mis. `=5`) |
| `?` | daftar nama slot |
| `i<slot>` | balik arah (invert) slot |
| `t<us>` | trim netral slot yang sedang terpilih (mis. `t-40`) |
| `M` | tukar sisi kiri↔kanan (K0↔K5, K1↔K4, K2↔K3, lengan R↔L) |
| `T<a> <b>` | tukar dua kaki, mis. `T0 5` |
| `L` / `D` | tabel mapping / cetak kode untuk `config.h` |
| `S` / `O` / `X` | simpan EEPROM / muat EEPROM / kosongkan |
| `F` | isi dengan dugaan lama dari `config.h` sebagai titik awal |
| `H` | semua slot termapping → 90° (bertahap) |
| `R` | matikan semua PWM (servo bebas) |
| `A` | matikan ALL-CALL `0x70` pada kedua PCA9685 |

## Menentukan `invert` dan `trim`

Sesudah pemetaan selesai, untuk tiap slot:

- **invert** — di `9` (90°), lalu `+`. Kalau sendi bergerak ke arah yang
  berlawanan dari yang Anda anggap "positif", tekan `i<slot>`.
- **trim** — di `9`, kalau sendi tidak benar-benar lurus/simetris, geser dengan
  `+`/`-` sampai lurus, lalu catat selisihnya dari 1500 dan masukkan lewat
  `t<us>`. Contoh: lurus di 1460 µs → `t-40`.

Default `invert` mengikuti firmware (kaki kiri, slot 9–17, terbalik). Verifikasi
tetap perlu karena orientasi pemasangan servo bisa berbeda.

---

# SET_HOME — menaruh servo ke home

**Bisa langsung dipakai untuk merakit body.** Pemetaan kaki sudah tertanam di
`servo_map.h`, jadi TES_SERVO tidak perlu dijalankan lebih dulu. Kalau ada
mapping tersimpan di EEPROM, itu yang dipakai (dianggap lebih baru).

Saat boot sketsa menghitung mundur 4 detik lalu **otomatis ke NETRAL 90°** —
kirim apa saja lewat Serial untuk membatalkan. Matikan lewat
`#define AUTO_HOME_ON_BOOT 0` bila robot sudah berdiri dan Anda tidak mau ia
bergerak sendiri tiap kali di-reset.

Dua arti "home", keduanya disediakan:

| | |
|---|---|
| `n` | **NETRAL** — semua sendi 90° (1500 µs). Dipakai saat memasang horn/kaki/body. |
| `s` | **BERDIRI** — pose siap jalan, diturunkan dari IK firmware (lihat bawah). |
| `v<n>` | goyang kaki `n` — pastikan penomoran kaki benar |
| `V` | goyang keenam kaki berurutan |
| `k<n>` | hanya kaki `n` (0–5) ke pose berdiri |
| `a` | kedua lengan ke pose parkir |
| `g<slot> <deg>` | satu slot ke sudut tertentu, mis. `g4 120` |
| `p` / `L` | pose sekarang / tabel mapping |
| `r` | matikan semua PWM (servo bebas) |

## Home itu berapa derajat? (dihitung, bukan dikira)

Angka pose berdiri **tidak boleh ditebak** — harus sama dengan yang nanti
dikirim firmware saat robot diam, kalau tidak robot akan menyentak begitu
firmware dijalankan.

Dari `config.h`: `COXA=20`, `FEMUR=80`, `TIBIA=90`, `STAND_RADIUS=70`,
`STAND_HEIGHT=100`. Kaki netral berada di `(70, 0, −100)` pada frame kaki, jadi
`LegIK::solve(70, 0, −100)` menghasilkan sudut geometris:

```
coxa = 0.00°     femur = −10.57° (dari horizontal)     knee = 82.02°
```

`Hexapod::angleToPulse` memakai baseline 90° (dan tibia dipusatkan di 90):

| Sendi | Sudut servo | Pulse |
|---|---|---|
| coxa | **90.00°** | 1500 µs |
| femur | **79.43°** | 1383 µs |
| tibia | **82.02°** | 1411 µs |

Ketiganya dekat 90° — itu memang tujuan desainnya, supaya rentang gerak
tersedia simetris ke dua arah. Sudah dimasukkan sebagai knob di atas
`SET_HOME.ino`. **Kalau `STAND_HEIGHT`/`STAND_RADIUS` diubah, angka ini harus
dihitung ulang.**

Sebagai acuan bila tinggi badan diubah (bentang tetap 70 mm):

| `STAND_HEIGHT` | femur | tibia |
|---|---|---|
| 60 mm | 109.19° | 54.31° |
| 80 mm | 93.52° | 67.11° |
| **100 mm** | **79.43°** | **82.02°** |
| 120 mm | 65.67° | 99.59° |

## Aturan pemasangan mekanik pada NETRAL 90°

Ini konsekuensi langsung dari `angleToPulse` (baseline 90°) dan menentukan
bagaimana horn servo harus dipasang. Saat `n` (semua 1500 µs):

- **coxa** — kaki harus menunjuk lurus ke **arah hadap netralnya** (kolom "arah
  hadap" di tabel kaki: K0 = 60°, K1 = 0°, dst).
- **femur** — harus **horizontal** (sejajar bidang badan).
- **tibia** — sudut lutut **90°**, yaitu tibia tegak lurus terhadap femur.

Kalau pada 1500 µs posisinya meleset sedikit, jangan bengkokkan mekanik —
pasang horn di gigi terdekat lalu sisanya diselesaikan dengan `t<us>` (trim) di
TES_SERVO.

## `invert` — hasil uji arah per sendi

Ditentukan **per sendi** dengan `u<slot>`, bukan per kaki — ternyata memang tidak
seragam. Hasil ukur lengkap:

| | coxa | femur | tibia |
|---|---|---|---|
| K0 / K1 / K2 (kanan) | **1** | **1** | 0 |
| K3 / K4 / K5 (kiri) | **1** | 0 | **1** |

Tiga hal yang terbaca dari pola ini:

- **Coxa dibalik di keenam kaki.** Ini bukan kebetulan pemasangan per kaki, tapi
  konvensi arah coxa di firmware yang memang berlawanan dengan cara servo
  terpasang di badan. Dibiarkan sebagai `invert` per slot — firmware sudah
  mendukungnya dan tidak perlu ubah kode.
- **Femur dan tibia saling melengkapi antar sisi** (kanan femur, kiri tibia),
  persis yang diharapkan dari modul kaki kanan/kiri yang terpasang bercermin.
  Konsistensi ini jadi tanda datanya dapat dipercaya.
- **Tidak bisa disederhanakan jadi rumus** "satu sisi dibalik semua" — karena
  itu tabelnya ditulis eksplisit per slot di `servo_map.h`.

Yang penting: **invert mencerminkan sudut terhadap 90°**, jadi 90° tetap 90°.
Pose NETRAL tidak berubah sama sekali — horn yang sudah terpasang tidak perlu
dibongkar, berapa kali pun invert diubah.

### Uji arah — jangan menebak

`u<slot>` di SET_HOME **menyebutkan dulu apa yang seharusnya terjadi**, baru
menggerakkan sendi 90° → 120° → 90°. Penilaiannya jadi tidak tergantung tafsir
"naik/turun" saat pose berdiri, yang mudah salah baca (di pose berdiri femur
memang turun sementara badan justru naik).

| Sendi | Yang benar saat 90° → 120° |
|---|---|
| coxa | kaki berayun **berlawanan arah jarum jam** dilihat dari atas — kaki kanan maju, kaki kiri mundur |
| femur | ujung kaki **naik** |
| tibia | lutut **membuka**, kaki jadi lebih lurus |

Aturan coxa berlaku seragam untuk keenam kaki karena `coxaGeo` positif selalu
berarti berputar CCW terhadap badan. `U<kaki>` menguji ketiga sendi satu kaki
berurutan. Kaki harus **menggantung bebas** saat diuji.

### Mengubahnya

Langsung dari SET_HOME, tanpa pindah sketsa:

| | |
|---|---|
| `u<slot>` / `U<kaki>` | uji arah 1 sendi / 1 kaki |
| `i<slot>` | balik arah satu sendi (langsung diterapkan bila servo aktif) |
| `I<kaki>` | balik arah ketiga sendi satu kaki |
| `F` | muat mapping bawaan program, abaikan EEPROM |
| `W` | simpan mapping aktif ke EEPROM |

Kalau EEPROM pernah diisi mapping lama, tekan **`F`** lalu **`W`** agar bawaan
yang baru dipakai dan tersimpan.

## Keselamatan

- **Topang badan robot atau gantung kakinya** saat pertama kali menekan `n`.
  Posisi mekanik servo saat itu tidak diketahui, jadi bisa melompat jauh.
- `n` sengaja bergerak **satu servo per satu** (jeda 150 ms) supaya arus puncak
  tidak menumpuk — 24 servo bergerak serentak bisa menjatuhkan tegangan dan
  me-reset Teensy.
- Setelah posisi diketahui, `s` bergerak **halus (ramp 1,5 detik)**.
- `r` kapan saja untuk melepas semua servo.
- Angka `HOME_FEMUR_DEG` / `HOME_TIBIA_DEG` masih perkiraan. Setel setelah kaki
  terpasang, sebelum pose berdiri dipakai untuk kalibrasi IK.

## Setelah ini

Hasil `D` dari TES_SERVO ditempel ke `HEXAPOD_KRSRI_2026/config.h`
(`SERVO_PIN_MAP`, `TUNE_PIN_MAP`, `ARM_PIN_MAP_R/L`). Yang juga harus berubah di
firmware: `HexaServos` masih memakai satu `SERVO_I2C_BUS` untuk kedua driver,
padahal sekarang driver 0 di `Wire2` dan driver 1 di `Wire1`.
