# TES_GERAK — uji gait, body kinematics, dan pivot

Alat bringup untuk bagian **gerak** yang selama ini belum punya alat uji.
`KALIBRASI/` sudah membuktikan robot bisa berjalan, tapi gait di sana versi
sederhana (ayun sinus, putar didekati) dan dibuat untuk menilai **kemulusan
servo**, bukan menilai gerakannya sendiri. Yang belum pernah diuji sama sekali:

- **body kinematics** — badan miring/geser dengan enam kaki tetap menapak
- **pivot terukur** — berputar 90°, bukan "berputar sampai kelihatan cukup"
- **gait yang sesungguhnya dipakai firmware** — ayun sikloid, slew, profil medan

`motion.h` di folder ini adalah port dari `HexaGait.cpp` + `Hexapod::solvePose`.
Yang terbukti di sini langsung sah untuk firmware, dan yang gagal di sini akan
gagal juga di firmware.

## Status

**Belum pernah lolos compiler** — tidak ada Teensyduino di sesi ini. Compile di
Arduino IDE sebelum dipercaya. Matematikanya sudah diverifikasi terpisah dengan
replika Python (hasilnya di bawah); yang belum diverifikasi adalah sintaks C++
dan perilaku di robot nyata.

Butuh: Teensyduino, Adafruit PWM Servo Driver. IMU opsional — semua uji kering
dan body kinematics jalan tanpa IMU; hanya pivot & kalibrasi yang butuh yaw.

## Tiga jenis uji

| Jenis | Perintah | Servo | Butuh |
|---|---|---|---|
| **KERING** | `S` `bl` `F` `N` | tidak bergerak sama sekali | — |
| **STATIS** | `br` `bp` `bw` `bx` `by` `bz` `bd` | badan bergerak, kaki menapak | lantai rata |
| **BERJALAN** | `g` `y` `O` `o` `C` `M` | robot benar-benar jalan | lantai luas |

Uji kering dulu, selalu. Perintah `S` menangkap "sendi harus bergerak 620°/detik"
dan `bl` menangkap "IK mentok di roll 12°" **sebelum** servo dipaksa mencobanya.

## Urutan yang disarankan

1. `S` — simulasi gait profil DATAR. Lihat laju sendi vs batas servo.
2. `F` lalu `N300` — cek bentang badan & profil SEMPIT untuk celah 30 cm R11.
3. `s` — berdiri (topang robot dulu kalau ini kali pertama setelah power-on).
4. `bl` — batas pose badan menurut IK.
5. `bd` — demo body 6 sumbu. **Periksa arah tiap sumbu** (tabel tanda di bawah).
   Kaki harus tetap di tempatnya; kalau kaki ikut bergeser, body transform salah.
6. `g` / `x` — jalan maju sebentar, pastikan tidak ada yang aneh.
7. `C4` — kalibrasi pivot. Ini menetapkan **tanda arah putar** dan derajat per
   siklus. Tanpa ini `O`/`o` bisa berputar menjauhi sasaran.
8. `O90`, `O-90` — pivot tertutup. Ukur hasilnya dengan busur/lantai bergaris.
9. `M5` lalu `m<cm>` — odometri maju: mm per siklus + persen slip.
10. `W` — simpan hasil kalibrasi ke EEPROM.

## Tanda sumbu body kinematics

Sudah diverifikasi lewat hitungan; **verifikasi ulang secara fisik** dengan `bd`
karena inilah satu-satunya cara membuktikan pemasangan servo sesuai konvensi:

| Perintah | Arti | Bukti di sudut servo |
|---|---|---|
| `br10` | badan miring **KANAN** | femur kanan +23,9° (sisi kanan turun), kiri −22,8° |
| `bp10` | badan **MENDONGAK** | femur depan −19,3°, femur belakang +20,9° |
| `bw10` | badan berputar ke **KIRI** | coxa berputar, femur/tibia hampir diam |
| `bx25` | geser **KANAN** | |
| `by25` | geser **DEPAN** | |
| `bz20` | badan **NAIK** | femur −13,8° di semua kaki (kaki memanjang) |

Femur naik = sisi itu **turun**, karena telapaknya menapak lantai.

Kalau ada yang terbalik di robot nyata, **itu temuan** — catat, dan betulkan di
tabel `invert[]` `servo_map.h`, bukan dengan membalik tanda perintah.

## Beda dari firmware — tiga hal yang harus diadopsi

Sama seperti `TES_IMU` sengaja beda dari `Imu.cpp`, tiga hal berikut sengaja
tidak menyalin firmware. Semuanya sudah diukur, bukan pendapat.

### 1. Invers body transform yang sesungguhnya

`Hexapod::solvePose` memanggil `rotatePoint(p, -roll, -pitch, -yaw)` — fungsi
**maju** dengan sudut dinegatifkan. Itu bukan invers dari `Rz*Ry*Rx`; inversnya
harus **urutan dibalik**. Galat yang ditimbulkan (posisi ujung kaki melenceng):

| sudut | 1 sumbu saja | roll+pitch | 3 sumbu |
|---|---|---|---|
| 5° | 0,00 mm | 1,2 mm | 2,0 mm |
| 10° | 0,00 mm | 4,6 mm | 8,2 mm |
| 15° | 0,00 mm | **9,9 mm** | 18,4 mm |
| 30° | 0,00 mm | 34 mm | 69 mm |

Selama hanya satu sumbu yang dipakai, cara firmware **kebetulan tepat**. Begitu
roll dan pitch aktif bersamaan — persis yang terjadi saat stabilisasi IMU di
medan miring, dengan `STAB_MAX_DEG` 15 — kaki melenceng sampai 1 cm dan robot
"mendorong" lantai. `moRotInv()` di `motion.h` membalik urutannya.

### 2. Fase gait diakumulasi, bukan `fmod(elapsed, cycleTime)`

Firmware: `phase = fmod(millis()-_cycleStart, cycleTime) / cycleTime`. Saat
`cycleTime` ikut di-ramp (ganti profil medan sambil berjalan), pembaginya berubah
di tengah jalan sementara `elapsed` sudah besar — dan fasenya **melompat**:

| lama berjalan terus-menerus | lompatan fase saat ganti profil |
|---|---|
| 2 detik | 0,03 (masih wajar) |
| 10 detik | 0,25 |
| 30 detik ke atas | **0,48 — nyaris setengah siklus** |

Langkah fase normal per tick cuma 0,017–0,022. Lompatan 0,5 berarti **kedua
tripod bertukar peran seketika**: tiga kaki yang sedang menapak langsung
diperintahkan mengayun, tiga yang di udara langsung disuruh menapak. Robot
menjatuhkan badannya. Ini terjadi tepat pada saat paling berbahaya — masuk
tangga setelah berjalan lama. `motion.h` menambah `dt/cycleTime` ke fase, jadi
kebal terhadap perubahan `cycleTime`.

### 3. `GaitProfile` perlu field `standR`

Di firmware `STAND_RADIUS` masih `#define`, jadi profil tidak bisa mengubah
bentang kaki. **Tanpa itu `profileNarrow()` untuk R11 mustahil** — satu-satunya
cara mengecilkan badan hexapod adalah menarik kakinya masuk. `motion.h`
menambahkan `standR` ke `GaitProfile` dan ikut me-ramp-nya.

## Uji tanpa hardware: `test_motion.py`

```bash
cd TES_GERAK
python test_motion.py        # berakhir "[PASS] ..." kalau semua lolos
```

Replika Python dari `motion.h` yang memeriksa sendiri: pose berdiri cocok dengan
angka yang sudah tercatat, invers rotasi benar-benar invers, tanda keenam sumbu
body kinematics, seluruh siklus gait di dalam jangkauan IK, lintasan tanpa
lompatan, fase kebal terhadap `cycleTime` yang di-ramp, dan profil SEMPIT muat
celah 30 cm. Semuanya lolos per 7 Agustus 2026.

Ini **tidak** menggantikan uji di robot — ia tidak tahu apa-apa tentang trim,
gesekan, atau lantai. Gunanya menangkap kesalahan yang tak akan terlihat dengan
memandangi robot. `g++` tidak terpasang di mesin ini, jadi tesnya Python; kalau
`motion.h` diubah, ubah replikanya lalu jalankan ulang.

## Hasil hitungan (replika Python, belum di robot)

Pose berdiri baku cocok persis dengan yang sudah tercatat — coxa 90,00°,
femur 79,43°, tibia 82,02°, bentang 32,0 cm — jadi port ini setia pada
kinematika yang sudah terbukti.

**Laju sendi maksimum, tulis PWM 50 Hz** (batas RDS3235: 400°/s tanpa beban,
realistis ~260°/s berbeban):

| profil | coxa | femur | tibia | bentang |
|---|---|---|---|---|
| 0 DATAR | 217 | 254 | 256 | 32,0 cm |
| 1 TANGGA | 217 | **388** | **312** | 32,0 cm |
| 2 MERUNDUK | 199 | **303** | 220 | 32,0 cm |
| 3 SEMPIT | 229 | 163 | 169 | **27,0 cm** |
| putar di tempat (DATAR) | **348** | 188 | 239 | 32,0 cm |

Tiga hal yang perlu diputuskan sebelum lomba:

- **Profil TANGGA meminta femur 388°/s** — praktis mustahil berbeban. Servo akan
  tertinggal dari perintah, kaki mendarat di tempat yang salah, robot ngesot
  bukan melangkah. `profileStairs()` firmware perlu siklus lebih lama
  (1200 → ~1600 ms) atau angkat lebih rendah. MERUNDUK (303°/s) juga lewat.
- **Putar di tempat meminta coxa 348°/s** — juga di atas batas berbeban. Kalau
  `C4` menunjukkan derajat/siklus jauh lebih kecil dari harapan, inilah
  sebabnya, bukan lantai licin.
- Profil DATAR pun sudah di 254–256°/s, artinya **gait bawaan berjalan tepat di
  batas kemampuan servo**. Tidak ada cadangan; jangan perbesar langkah lagi.

Hanya SEMPIT yang punya cadangan besar — masuk akal, langkahnya paling pendek.

Angka-angka ini keluar juga dari perintah `S` di robot, dengan `writeHz` yang
sedang berlaku. Ubah `w` lalu jalankan `S` lagi untuk melihat pengaruhnya.

## Daftar perintah

```
DASAR
  n            netral 90 der (TOPANG ROBOT)
  s            BERDIRI + mulai kendali gerak
  x            STOP (kaki kembali ke pose berdiri)   Enter kosong = sama
  r            lepas semua PWM        L  tabel pemetaan servo

BODY KINEMATICS (6 kaki tetap menapak)
  br<der>      roll  + = miring KANAN
  bp<der>      pitch + = MENDONGAK
  bw<der>      yaw badan + = ke KIRI (pivot halus tanpa melangkah)
  bx/by/bz<mm> geser badan kanan+ / depan+ / naik+
  b0           nolkan semua          bd  DEMO 6 sumbu berurutan
  bl           UJI BATAS tiap sumbu (kering)

GAIT
  g / G        MAJU / MUNDUR         [ / ]  geser KIRI / KANAN
  y / Y        putar KIRI / KANAN di tempat
  v<x> <y> <w> vektor bebas dalam persen, mis. v0 80 -40
  f<0..3>      profil DATAR / TANGGA / MERUNDUK / SEMPIT

UJI KERING (servo TIDAK bergerak)
  S            simulasi gait: laju sendi vs batas servo, jangkauan IK
  F            tabel profil + bentang badan
  N<mm>        cari bentang kaki untuk celah, mis. N300 (R11)

PIVOT & KALIBRASI (butuh IMU)
  C<siklus>    KALIBRASI pivot: der/siklus + TANDA arah (mis. C4)
  O<der>       pivot relatif tertutup, mis. O90 / O-90
  o<n>         pivot ke arah kompas (0=U 1=T 2=S 3=B)
  M<siklus>    jalan maju N siklus untuk diukur mistar
  m<cm>        masukkan hasil ukur -> mm per siklus + slip

KNOB
  h<mm> tinggi badan   R<mm> bentang kaki   k<mm> panjang langkah
  e<mm> tinggi angkat  p<ms> siklus         d<%> duty tumpu
  l<x10> slew (l30 = 3,0)                   w<hz> laju tulis PWM
  ?  knob   T  telemetri   W/E  simpan/muat EEPROM   H  bantuan
```

## Kalibrasi pivot (`C`) — kenapa penting

Menjawab dua hal yang tidak bisa ditebak:

1. **Tanda arah.** Yaw IMU biasanya membesar searah jarum jam; gait `+1` memutar
   berlawanan jarum jam. Kalau tandanya salah, pivot tertutup akan berputar
   **menjauhi** sasaran sampai batas waktu habis. `C` mengukurnya dan menyimpan
   `pivotSign`; `O`/`o` memakainya. **Jalankan `C` sebelum `O`/`o` yang pertama.**
2. **Derajat per siklus.** Angka ini yang membuat misi bisa merencanakan belokan
   ("90° butuh 6 siklus ≈ 5,4 detik") alih-alih memutar sambil menebak.

`C` juga melaporkan **asimetri** kiri/kanan. Beda >25% biasanya berarti trim coxa
belum rata atau satu kaki menyeret — betulkan di `KALIBRASI/` dulu.

Pivot tertutup memakai PD yang sama dengan firmware (`heading.kp` 0,020,
`heading.kd` 0,004), suku D dari **gyro Z** bukan turunan yaw: yaw berisik dan
mendiferensiasikannya hanya memperbesar noise, sedangkan gyro kebal magnet servo.
Tambahannya `PIVOT_MIN_CMD` 0,25 — di bawah itu langkahnya terlalu pendek untuk
mengalahkan gesekan dan robot cuma bergetar di tempat sementara error tak
pernah mengecil.

## Odometri (`M` + `m`) — kenapa perlu diukur tangan

Panjang langkah 60 mm **bukan** jarak yang ditempuh: kaki selip saat fase tumpu.
Selisihnya bisa 30–40%. Navigasi yang menghitung "sudah jalan berapa jauh" harus
memakai angka terukur, bukan `stepLength`. Jalankan `M5`, ukur dengan mistar,
ketik `m<cm>`.

## EEPROM

Alat ini memakai **alamat 2048**, dan hanya **membaca** blok kompas 1792 yang
ditulis `TES_IMU`.

| Alamat | Isi | Ditulis oleh |
|---|---|---|
| 0 .. ~280 | blok `Calib` firmware | `Calib.cpp` |
| 1024 .. ~1150 | `ServoMap` | `servo_map.h` |
| 1536 .. ~1560 | pemetaan & offset lidar | `MAP_LIDAR` |
| 1792 .. ~1812 | 4 arah kompas arena | `TES_IMU` |
| **2048 .. ~2068** | **der/siklus pivot, mm/siklus maju, tanda pivot** | **`TES_GERAK`** |

## Catatan

- `servo_map.h` dan `kinematics.h` adalah **salinan** dari `KALIBRASI/`. Kalau
  salah satunya diubah, salin ke semua folder sketsa (Arduino IDE tidak bisa
  berbagi file antar folder).
- Semua perhitungan gerak berbasis `dt` yang **diberikan pemanggil**, bukan
  `millis()` internal. Itu yang memungkinkan mesin yang sama dijalankan
  cepat-cepat di RAM untuk `S`/`N` tanpa menggerakkan satu servo pun.
- `bl` dan `N` hanya tahu batas **IK dan rentang servo 0..180**, tidak tahu
  tabrakan mekanis antar kaki atau kabel yang tertarik. Uji statis pelan-pelan
  sebelum mempercayai angkanya.
