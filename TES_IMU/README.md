# TES_IMU — kelayakan IMU Yahboom 10-axis untuk heading

Sketsa berdiri sendiri untuk Teensy 4.1. Board **Teensy 4.1**, Serial Monitor
**115200**, line ending **Newline**. Library: **Adafruit PWM Servo Driver**.

IMU di **`Serial2`** — **RX2 pin 7**, **TX2 pin 8**. Protokol **WIT**, frame
`0x55` 11 byte, **921600 baud**. TX IMU → pin 7 Teensy, dan **GND wajib
tersambung** (paling sering terlupa; tanpa itu frame korup, bukan kosong).

> `config.h` firmware masih menulis `IMU_SERIAL Serial1` — **salah**, harus
> diubah ke `Serial2` saat port firmware.
>
> `Serial2` tadinya dicadangkan untuk Raspberry Pi 5. Pi harus pindah, dan
> **tidak boleh** ke `Serial4` (pin 16/17 = `Wire1`) atau `Serial6`
> (pin 25/24 = `Wire2`) karena pinnya sudah dipakai I2C. Yang bebas:
> `Serial1` (0/1), `Serial3` (15/14), `Serial5` (21/20), `Serial7` (28/29),
> `Serial8` (34/35).

## Pertanyaan yang dijawab alat ini

Bukan "IMU-nya hidup atau tidak" — itu bagian gampang. Yang menentukan nasib
navigasi:

> **Yaw melenceng berapa derajat saat 18 servo bekerja?**

Karena kalau jawabannya besar, seluruh rencana heading berbasis kompas gugur
dan harus pindah ke sudut dinding dari sepasang lidar samping.

## Dua jenis gangguan, nasibnya berbeda jauh

| | Sumber | Bisa dikoreksi? |
|---|---|---|
| **Statis** | besi rangka, magnet servo dalam keadaan diam | **Ya** — konstan terhadap badan robot, itulah gunanya mencatat `HEAD_UTARA/TIMUR/SELATAN/BARAT` per arena |
| **Dinamis** | arus 18 servo, magnet bergerak | **Tidak** — berubah-ubah mengikuti gerakan |

Kalau yang dinamis besar, kompas tidak bisa dipakai heading-hold **sambil
berjalan** — persis saat ia paling dibutuhkan.

Gate `IMU_MAX_YAW_JUMP` 30° di firmware tidak menolong: ia menolak **lonjakan**,
sedangkan kegagalan sebenarnya berupa **pergeseran bertahap**. Yaw melenceng 40°
dalam 2 detik lolos mulus.

## ⚠ Keselamatan

Perintah `g` dan `s2` **menggerakkan servo**. Taruh robot di atas dudukan /
kardus sehingga **kaki menggantung bebas**. Robot tidak boleh digeser atau
diputar selama uji berlangsung — seluruh pengukuran adalah selisih terhadap
fase pertama.

## Uji inti: `g`

Tiga fase, masing-masing 8 detik (`p<detik>` untuk mengubah):

| Fase | Keadaan servo | Yang diukur |
|---|---|---|
| 1 | **mati** (tanpa pulsa) | garis dasar yaw & kuat medan |
| 2 | **bertenaga tapi diam** di pose berdiri | gangguan **statis** |
| 3 | **bergerak** (goyang ±10°, beda fase per kaki) | gangguan **dinamis** |

Keluarannya:

```
  fase                yaw     sebar   |mag|
  1 servo MATI        137.2   0.8     412
  2 servo DIAM        137.9   1.1     418   geser +0.7
  3 servo GERAK       152.4   11.3    533   geser +15.2
```

`|mag|` ikut dilaporkan karena itu **buktinya**: kalau kuat medan magnet
naik seiring servo bergerak, gangguannya memang magnetik — bukan hanyutan giro
atau robot yang bergeser sendiri.

Ambang vonis (dipakai nilai terburuk antara pergeseran dan sebaran fase 3):

| | Arti | Tindakan |
|---|---|---|
| < 5° | yaw layak jadi acuan utama | catat `HEAD_*`, pakai PD `holdHeading` |
| 5–15° | meragukan | silangkan dengan sudut dinding lidar, longgarkan `HEADING_TOL_DEG` |
| > 15° | tidak layak sambil berjalan | jauhkan/perisai IMU, atau pindah ke acuan lidar, atau pakai yaw hanya saat diam |

## Uji lain

**`d<detik>` — hanyutan saat diam.** Robot benar-benar diam, servo mati.
Melaporkan hanyut total dan derajat/menit. Hanyut < 2° berarti fusi magnetnya
bekerja (kompas terkunci ke medan bumi). Hanyut besar berarti ia sebenarnya
mengandalkan giro yang terintegrasi — itu akan terus melenceng sepanjang lomba.

**`f` — statistik frame WIT.** Byte/detik, frame/detik, **rasio checksum
gagal**, byte terbuang saat resinkronisasi, jumlah lonjakan yaw > 30°, dan
cacah per tipe paket (`0x51` accel, `0x52` gyro, `0x53` sudut, `0x54` magnet).

Checksum gagal > 1% berarti kabel terlalu panjang untuk 921600 baud atau GND
buruk — turunkan baud **di IMU-nya** lewat aplikasi WIT, lalu `b<n>` di sini
supaya cocok.

## Dua hal yang sengaja dibuat berbeda dari firmware

**1. Parser resinkronisasi sejati.** `Imu.cpp` membuang seluruh 11 byte saat
checksum gagal. Kalau satu byte hilang di kabel, cara itu terus salah bingkai
dan frame berikutnya ikut hancur — terbaca sebagai "IMU rusak" padahal cuma
salah sinkron. Di sini: checksum gagal → buang **satu** byte, coba lagi.

**2. Buffer RX Serial2 ditambah 2 KB.** Bawaan Teensy 64 byte = hanya **0,7 ms**
data pada 921600 baud. Tanpa tambahan itu, jeda loop sedikit saja sudah membuat byte
hilang, dan gejalanya persis seperti IMU berisik padahal masalahnya di sisi
Teensy.

Kalau uji di sini bersih tapi firmware tetap kacau, dua hal inilah tersangka
pertamanya.

## Perintah

| Perintah | Fungsi |
|---|---|
| **pengukuran inti** | |
| `g` | uji gangguan servo 3 fase — **yang menentukan** |
| `d<detik>` | uji hanyutan yaw saat diam (mis. `d60`) |
| `f` | statistik frame WIT |
| **tampilan** | |
| `a` | tampilkan data terus-menerus |
| `t` | tare roll/pitch jadi nol |
| `z` | jadikan yaw sekarang sebagai referensi (kolom `rel`) |
| `r` | reset statistik frame |
| `x` | berhenti (servo ikut dimatikan) |
| **servo** | |
| `s0` `s1` `s2` | servo mati / diam bertenaga / goyang |
| `w<der>` | amplitudo goyang, 2–30° |
| `p<detik>` | lama tiap fase uji `g`, 3–60 detik |
| **lain** | |
| `b<n>` | baud 0=115200 1=230400 2=460800 3=921600 |
| `h` | bantuan |

## Urutan pemakaian yang disarankan

```
f          -> pastikan jalurnya bersih dulu (frame/s wajar, checksum gagal ~0%)
a          -> lihat yaw bergerak saat robot diputar tangan; masuk akal?
d60        -> hanyutan saat diam
g          -> UJI PENENTU: taruh di dudukan, kaki menggantung
```

Catat angka fase 3 dari `g`. Itu yang menentukan apakah pemetaan kompas
(4 arah U/T/S/B) layak dibangun, atau harus pindah ke acuan lidar.

## Setelah ini

Kalau yaw lolos, langkah berikutnya adalah mencatat `HEAD_UTARA/TIMUR/SELATAN/BARAT`.
Cara paling presisi bukan dengan mata: **sikukan robot ke dinding arena memakai
lidar samping** sampai `LEFT_FRONT ≈ LEFT_REAR`, baru catat yaw-nya. Lalu
periksa apakah keempat nilainya berjarak ~90°:

- berjarak 88–92° → kompas linier, cukup catat Utara dan sisanya aritmetika;
- ada yang 70° atau 110° → ada distorsi soft-iron, keempatnya harus dicatat
  sendiri-sendiri dan sektor di antaranya tetap tidak bisa dipercaya penuh.
