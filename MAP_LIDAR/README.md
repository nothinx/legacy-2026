# MAP_LIDAR — memetakan channel mux ke posisi fisik lidar

Sketsa berdiri sendiri untuk Teensy 4.1. Buka `MAP_LIDAR.ino` di Arduino IDE →
Board **Teensy 4.1** → Serial Monitor **115200**, line ending **Newline**.
Library: **VL53L0X by Pololu**.

`TES_LIDAR` menjawab *"sensornya hidup atau tidak"*. Sketsa ini menjawab dua
pertanyaan berikutnya:

1. **channel mux nomor berapa itu lidar yang mana secara fisik**, dan
2. **pembacaannya meleset berapa milimeter** dari jarak sebenarnya.

Keluarannya blok kode siap tempel untuk `lidar_map.h` dan `config.h`.

## Tata letak yang diasumsikan

6 lidar: **depan**, **belakang**, **2 kiri**, **2 kanan**. Yang di samping
dipasang **di celah antar kaki** supaya kaki tidak masuk berkas ukur.

```
                    DEPAN
                  [ FRONT ]
            kaki kiri depan | kaki kanan depan
     [LEFT_FRONT]     B O D I      [RIGHT_FRONT]
            kaki kiri tengah | kaki kanan tengah
     [LEFT_REAR ]                  [RIGHT_REAR ]
            kaki kiri blkg   | kaki kanan blkg
                  [ BACK  ]
                   BELAKANG
```

Indeks posisi **searah jarum jam mulai dari depan** — dipilih begitu supaya
nanti bisa dipakai interpolasi arah / wall-following tanpa tabel tambahan:

| p | nama | letak |
|---|---|---|
| 0 | `FRONT` | hidung, menghadap depan |
| 1 | `RIGHT_FRONT` | kanan, celah kaki depan–tengah |
| 2 | `RIGHT_REAR` | kanan, celah kaki tengah–belakang |
| 3 | `BACK` | buritan, menghadap belakang |
| 4 | `LEFT_REAR` | kiri, celah kaki tengah–belakang |
| 5 | `LEFT_FRONT` | kiri, celah kaki depan–tengah |

**Nomor channel mux tidak diasumsikan sama dengan nomor posisi.** Kabel boleh
tercolok di channel mana pun; wizard yang menentukan pasangannya.

## Alur pemakaian

```
boot   -> muat pemetaan dari EEPROM (kalau ada), cari bus lidar, init
w      -> wizard: dekatkan tangan ke tiap lidar saat diminta
s      -> periksa tabel hasil
a      -> lihat denah, halangi satu sisi, pastikan angka yang berubah benar
f0 c50 -> kalibrasi FRONT pada jarak 50 cm  (ulangi di 10/20/100 cm)
T      -> tabel titik kalibrasi
e      -> simpan ke EEPROM
g      -> cetak kode untuk lidar_map.h + config.h
```

### Cara kerja wizard (`w`)

1. Robot didiamkan di tempat lapang; program mencatat **baseline** jarak diam
   tiap channel (median 5 bacaan).
2. Untuk tiap posisi, program menyebut letaknya dan menunggu Anda mendekatkan
   telapak tangan **10–20 cm** di depan lidar itu.
3. Sebuah channel dianggap "kena" bila jaraknya **< 40 cm** *dan* **turun ≥ 15 cm**
   dari baseline, bertahan **3 sweep** berturut-turut. Syarat turun-dari-baseline
   inilah yang membuatnya tetap benar walau sensor sudah menatap dinding dekat.
4. Kalau **dua** channel kena sekaligus (tangan di antara dua lidar), wizard
   menolak dan minta diulang — tidak menebak.
5. Setelah satu posisi dipetakan, wizard menunggu tangan ditarik dulu supaya
   satu lambaian tidak terhitung dua kali.
6. Channel yang sudah dipakai posisi lain tidak ikut dipertimbangkan lagi.

Enter (atau karakter apa pun) saat wizard berjalan = batal.

Kalau satu posisi saja yang salah, tidak perlu ulang semua: `n<p>` mengulang
satu posisi, atau `m<p><c>` untuk menetapkan manual (mis. `m05` = FRONT ke ch5).

### Kalibrasi jarak (`c<cm>`)

Pilih sasaran dengan `f<p>`, taruh target rata & tegak lurus pada jarak yang
diukur **dari permukaan lensa**, lalu ketik jaraknya, mis. `c50`.

Program mengambil 30 sampel **mentah** (offset diabaikan saat mengukur) lalu
melaporkan rata-rata, min/maks, simpangan baku, dan selisih terhadap jarak
sebenarnya. Offset posisi itu langsung diset ke `jarak_asli − rata_rata`, jadi
`jarak_terkoreksi_mm = jarak_mentah_mm + offset`.

Simpangan > 15 mm ditandai: biasanya target terlalu gelap/miring, atau I2C
tidak stabil (coba `k1` → 100 kHz).

Ulangi di **10 / 20 / 50 / 100 cm** lalu `T`:

- "meleset" hampir sama di semua jarak → **offset tetap**, sudah tertangani.
- "meleset" membesar seiring jarak → butuh **faktor skala**, bukan offset.
  Sketsa ini tidak menyimpan skala; catat angkanya dan tangani di firmware.

Offset yang tersimpan hanya dipakai untuk tampilan `a`/`l` dan untuk dicetak
oleh `g`. Kalibrasi selalu memakai nilai mentah, jadi mengukur ulang tidak
menumpuk koreksi.

## Kalau angkanya meloncat-loncat

Urutan memeriksanya, dari yang paling sering:

**1. Targetnya memang di luar jangkauan.** Ini penyebab paling umum dan bukan
kerusakan. Spek "~200 cm" itu kondisi lab (target putih 88% reflektif, ruang
gelap, tegak lurus). Terukur di robot ini: **stabil sampai ~100 cm**, di atas
itu mengarang.

Jahatnya, nilai karangan itu **bukan `8190` yang bersih** — jadi gate
`nilai >= 8000` meloloskannya dan ia tampak seperti jarak sungguhan. Karena itu
alat ini punya **batas percaya** (`t<cm>`, bawaan 100 cm): apa pun di atasnya
ditampilkan `>>>`, tidak diberi angka. Nilainya ikut tercetak sebagai
`LIDAR_MAX_CM` oleh perintah `g`.

Naikkan batasnya **hanya** kalau `q` pada jarak itu benar-benar menunjukkan
sebaran kecil — jangan karena ingin jangkauan lebih jauh.

**2. Ukur dulu, jangan menebak.** `f<p>` lalu `q`: 50 sampel mentah, dilaporkan
median, min/maks, **sebar**, dan simpangan baku. Patokan kasar simpangan baku:

| σ | Arti |
|---|---|
| < 5 mm | tenang |
| 5–15 mm | wajar untuk VL53L0X, median-3 di firmware cukup |
| 15–40 mm | berisik — naikkan profil, atau permukaan target buruk |
| > 40 mm | curigai crosstalk, daya 3V3, atau cahaya matahari |

**3. Profil pengukuran (`P0`/`P1`/`P2`).** Jangan mencampur preset ST:

| Profil | Budget | VCSEL | Rate limit | Sifat |
|---|---|---|---|---|
| `P0` CEPAT | 20 ms | 18/14 | 0,10 | jangkauan maksimum, **paling berisik** |
| `P1` SEIMBANG | 50 ms | 18/14 | 0,25 | bawaan |
| `P2` TENANG | 200 ms | 14/10 | 0,25 | paling stabil, jangkauan ~1,2 m |

Rate limit **0,10** menyuruh sensor menerima pantulan lemah — itu yang membuat
bacaan jauh jadi sampah, bukan jadi "tak ada target". Naikkan ke 0,25 dan
sensor lebih jujur mengaku tidak melihat apa-apa.

**4. Crosstalk antar lidar.** Mux hanya memutus **I2C**, tidak mematikan laser.
Kalau keenam sensor mode kontinu, keenam laser 940 nm menyala bersamaan dan
saling mengganggu. Bawaan alat ini **single-shot** — hanya satu lidar menembak
pada satu saat, jadi crosstalk mustahil. `o` untuk membandingkan dengan kontinu.

**5. Filter tampilan (`F`).** Denah `a` memakai median berjalan 5 sampel
(satu sampel baru per refresh, bukan 5 sekaligus — dalam single-shot itu akan
makan ~2 detik per denah). `F` mematikannya kalau ingin melihat nilai mentah.
Kalibrasi `c<cm>` dan uji `q` **selalu** memakai nilai mentah.

**6. Sisanya:** I2C 400 kHz di kabel dupont panjang (`k1` → 100 kHz), tegangan
3V3 drop di modul terjauh, cahaya matahari langsung, target gelap/miring.

## Perintah

| Perintah | Fungsi |
|---|---|
| **pemetaan** | |
| `w` | wizard: petakan 6 posisi lewat lambaian tangan |
| `n<p>` | ulang wizard untuk satu posisi (mis. `n3`) |
| `m<p><c>` | set manual posisi `p` ke channel `c` (mis. `m05`) |
| `u<p>` | lepas pemetaan posisi `p` |
| `s` | tabel pemetaan sekarang |
| **membaca** | |
| `a` | denah robot + jarak, terus-menerus |
| `l` | satu baris per sweep, terus-menerus |
| `r<n>` | baca satu **channel** mentah terus-menerus |
| `x` | berhenti membaca |
| **kestabilan** | |
| `q` | uji kestabilan 50 sampel di posisi fokus |
| `t<cm>` | batas percaya; di atasnya ditampilkan `>>>` (bawaan 100) |
| `P0` `P1` `P2` | profil CEPAT / SEIMBANG / TENANG (langsung init ulang) |
| `o` | ganti single-shot ↔ kontinu |
| `F` | filter median berjalan untuk tampilan on/off |
| **kalibrasi** | |
| `f<p>` | pilih posisi fokus |
| `c<cm>` | ukur pada jarak sebenarnya, set offset |
| `T` | tabel titik kalibrasi sesi ini |
| `z` | nolkan semua offset |
| **lain** | |
| `d` | cari bus lidar + init |
| `i` | init ulang lidar |
| `k1` / `k4` | clock I2C 100 / 400 kHz (lalu `i`) |
| `g` | cetak kode untuk `lidar_map.h` + `config.h` |
| `e` / `E` / `X` | simpan / muat / hapus EEPROM |
| `h` | bantuan |

## Tampilan `a`

```
            DEPAN            (cm)
           +-------+
           | F  45 |
 +-------+ +-------+ +-------+
 | LF  12|           |RF  30 |
 +-------+  H E X A  +-------+
 | LR  15|           |RR  28 |
 +-------+ +-------+ +-------+
           | B >>> |
           +-------+
           BELAKANG    >>> = lebih jauh dari batas percaya 100 cm
                       ... = tak ada target, --- = belum dipetakan
```

`>>>` bukan error — artinya ada pantulan tapi jaraknya di luar batas yang
terbukti bisa dipercaya, jadi angkanya sengaja tidak ditampilkan.

Pakai ini untuk **memverifikasi** hasil wizard: halangi satu sisi dengan
tangan, angka di sisi itulah yang harus berubah. Kalau yang berubah kotak
lain, pemetaannya tertukar — perbaiki dengan `m<p><c>`.

## Penyimpanan

Pemetaan + offset disimpan di **EEPROM** Teensy (alamat 0, ±20 byte, ada magic
+ checksum) sehingga tetap ada setelah reboot. Urutan sumber saat boot:

1. EEPROM bila magic & checksum sah,
2. kalau tidak, tabel `LIDAR_CH_DEFAULT` / `LIDAR_OFFSET_DEFAULT` di `lidar_map.h`.

EEPROM hanya untuk sketsa ini. Yang dipakai firmware adalah hasil `g` yang
ditempel ke `config.h` — jadi **jangan lupa `g` dan tempel**, jangan andalkan
EEPROM saja.

## Yang perlu diubah di firmware setelah ini

`config.h` sekarang masih memakai warisan lama yang **salah** untuk tata letak
ini:

```c
#define LIDAR_FRONT   0
#define LIDAR_FRONT_R 1     // tidak ada — tidak ada lidar diagonal depan
#define LIDAR_RIGHT   2     // sekarang jadi dua: RIGHT_FRONT & RIGHT_REAR
#define LIDAR_BACK    3
#define LIDAR_LEFT    4     // sekarang jadi dua: LEFT_FRONT & LEFT_REAR
#define LIDAR_FRONT_L 5     // tidak ada
```

Selain nama, ada dua konsekuensi nyata:

- **Tidak ada lidar diagonal depan.** Deteksi halangan serong harus disusun
  dari `FRONT` + sepasang sensor samping, bukan dari sensor 45°. Kode navigasi
  yang membaca `LIDAR_FRONT_R` / `LIDAR_FRONT_L` harus ditulis ulang.
- **Sepasang sensor per sisi justru menguntungkan**: selisih
  `LEFT_FRONT − LEFT_REAR` memberi **sudut robot terhadap dinding** secara
  langsung — wall-following bisa memakai itu tanpa bergantung pada yaw IMU
  (yang masih diragukan karena gangguan magnet dari servo).

Juga tetap berlaku: `LIDAR_MAX_CM` 400 → **200**, dan `LidarArray` harus pindah
ke Pololu VL53L0X dengan **satu objek per channel**.
