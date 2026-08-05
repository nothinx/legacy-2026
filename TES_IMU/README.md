# TES_IMU — kelayakan IMU Yahboom 10-axis untuk heading

Sketsa berdiri sendiri untuk Teensy 4.1. Board **Teensy 4.1**, Serial Monitor
**115200**, line ending **Newline**. Library: **Adafruit PWM Servo Driver**.

IMU di **`Serial2`** — **RX2 pin 7**, **TX2 pin 8**. Protokol **WIT**, frame
`0x55` 11 byte. TX IMU → pin 7 Teensy, dan **GND wajib tersambung** (paling
sering terlupa; tanpa itu frame korup, bukan kosong).

**Baud terukur: 9600** — bukan 921600 seperti dugaan awal. Itu memang bawaan
pabrik perangkat WIT. Perintah `B` memindai 9600…921600 dan memakai yang jalan,
jadi tidak perlu menebak.

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

## Setelan di aplikasi Yahboom/WIT (lewat CP2102)

Selagi IMU tersambung ke PC, ada empat hal yang sebaiknya sekalian dibereskan.

**1. Naikkan baud ke 115200.** Di 9600 baud kapasitasnya hanya 960 byte/detik,
dan tiap frame 11 byte:

| Paket aktif | Byte per set | Laju maks di 9600 | di 115200 |
|---|---|---|---|
| sudut saja | 11 | ~87 Hz | ~1000 Hz |
| sudut + magnet | 22 | ~43 Hz | ~520 Hz |
| accel + gyro + sudut + magnet | 44 | **~21 Hz** | ~260 Hz |

**2. Return rate harus muat di baud.** Kalau rate diset lebih tinggi dari
kapasitas tabel di atas, frame terpotong dan muncul sebagai **checksum gagal** —
bukan sebagai "lambat". Ini jebakan paling umum di sensor WIT: orang menaikkan
rate, datanya jadi rusak, lalu menyalahkan kabel. Perintah `f` menghitung
kapasitas ini dan memperingatkan kalau sudah mentok.

**3. Matikan paket yang tidak dipakai.** Alat ini butuh **sudut (`0x53`)** dan
**magnet (`0x54`)**. Accel & gyro hanya untuk tampilan — mematikannya
melipatgandakan laju sudut yang bisa dicapai.

**4. Kalibrasi magnetometer — dan lakukan dengan IMU SUDAH TERPASANG di robot.**

Ini yang paling penting dan paling sering dilakukan terbalik. Kalibrasi
hard-iron di aplikasi WIT mengukur medan tetap di sekitar sensor lalu
mengurangkannya. Kalau dikalibrasi saat IMU masih di meja, ia hanya membuang
medan **meja** — begitu dipasang di robot, besi rangka dan magnet servo
memasukkan medan baru yang tidak terkoreksi.

Jadi: pasang IMU di posisi finalnya, servo **tidak bertenaga**, lalu putar
**seluruh robot** perlahan mengikuti prosedur kalibrasi aplikasi.

Ini menyambung langsung ke uji `g`: kalibrasi hard-iron menghilangkan komponen
**statis**, sehingga yang tersisa untuk diukur `g` benar-benar komponen
**dinamis** — satu-satunya yang tidak bisa diperbaiki. Karena itu kalibrasi
sebaiknya dituntaskan **sebelum** menjalankan `g`, kalau tidak angka fase 2
hanya mengukur sesuatu yang sebenarnya masih bisa dibereskan.

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
| `B` | **pindai baud** 9600…921600, pakai yang jalan |
| `b<baud>` | set baud langsung (mis. `b9600`, `b115200`) |
| `h` | bantuan |

## Urutan pemakaian yang disarankan

```
B          -> pindai baud, temukan yang jalan
f          -> jalur bersih? (checksum gagal ~0%, laju sudut cukup)
a          -> lihat yaw bergerak saat robot diputar tangan; masuk akal?
d60        -> hanyutan saat diam
g          -> UJI PENENTU: taruh di dudukan, kaki menggantung
```

Kalibrasi magnetometer (dengan IMU terpasang di robot) sebaiknya sudah selesai
sebelum `d60` dan `g` — lihat bagian setelan aplikasi di atas.

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
