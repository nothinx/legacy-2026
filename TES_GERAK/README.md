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
gagal juga di firmware. Arahnya sudah dua arah: empat kesalahan yang ketahuan di
sini sudah diperbaiki di firmware juga (lihat bagian "Empat perbaikan").

## Status

**Belum pernah lolos compiler** — tidak ada Teensyduino di sesi ini, dan itu
berlaku untuk alat ini MAUPUN untuk perubahan firmware yang menyertainya
(`types.h`, `HexaGait.*`, `Hexapod.*`, `config.h`). Compile di Arduino IDE
sebelum dipercaya. Matematikanya sudah diverifikasi terpisah dengan replika
Python (hasilnya di bawah); yang belum diverifikasi adalah sintaks C++ dan
perilaku di robot nyata.

Butuh: Teensyduino, Adafruit PWM Servo Driver. IMU opsional — semua uji kering
dan body kinematics jalan tanpa IMU; hanya pivot & kalibrasi yang butuh yaw.

## Tiga jenis uji

| Jenis | Perintah | Servo | Butuh |
|---|---|---|---|
| **KERING** | `S` `bl` `F` `N` | tidak bergerak sama sekali | — |
| **STATIS** | `br` `bp` `bw` `bx` `by` `bz` `bd` `J` `K` `A` `Q` `Z` | badan bergerak, kaki menapak | lantai keras & rata |
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
6. `J` — **ratakan telapak** (ada kaki tidak menapak saat berdiri), lalu `A` —
   **ratakan badan**. Urutannya wajib begitu. Lakukan sebelum uji berjalan:
   kaki yang menggantung atau badan yang miring membuat beban tiap kaki tidak
   rata, dan itu mencemari `C` dan `M`.
7. `g` / `x` — jalan maju sebentar, pastikan tidak ada yang aneh.
8. `C4` — kalibrasi pivot. Ini menetapkan **tanda arah putar** dan derajat per
   siklus. Tanpa ini `O`/`o` bisa berputar menjauhi sasaran.
9. `O90`, `O-90` — pivot tertutup. Ukur hasilnya dengan busur/lantai bergaris.
10. `M5` lalu `m<cm>` — odometri maju: mm per siklus + persen slip.
11. `W` — simpan hasil kalibrasi ke EEPROM.

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

## Empat perbaikan — sudah ikut diterapkan ke firmware

Alat ini menemukan empat hal yang salah di `HEXAPOD_KRSRI_2026/`. Keempatnya
sudah diperbaiki di sana juga, jadi `motion.h` dan firmware sekarang sepakat.
Semuanya diukur, bukan pendapat — angkanya keluar dari `test_motion.py`.

### 1. Invers body transform yang sesungguhnya

`Hexapod::solvePose` dulu memanggil `rotatePoint(p, -roll, -pitch, -yaw)` —
fungsi **maju** dengan sudut dinegatifkan. Itu bukan invers dari `Rz*Ry*Rx`;
inversnya harus **urutan dibalik**. Galat yang ditimbulkan (ujung kaki
melenceng):

| sudut | 1 sumbu saja | roll+pitch | 3 sumbu |
|---|---|---|---|
| 5° | 0,00 mm | 1,2 mm | 2,0 mm |
| 10° | 0,00 mm | 4,6 mm | 8,2 mm |
| 15° | 0,00 mm | **9,9 mm** | 18,4 mm |
| 30° | 0,00 mm | 34 mm | 69 mm |

Selama hanya satu sumbu yang dipakai, cara lama **kebetulan tepat**. Begitu
roll dan pitch aktif bersamaan — persis yang terjadi saat stabilisasi IMU di
medan miring, dengan `STAB_MAX_DEG` 15 — kaki melenceng sampai 1 cm dan robot
"mendorong" lantai alih-alih berdiri di atasnya. `moRotInv()` di `motion.h` dan
`rotatePointInv()` di `types.h` membalik urutannya.

Ikutannya: nama sumbu di `types.h` dulu `roll(X), pitch(Y)`, padahal di frame
ini (+X kanan, +Y depan) rotasi terhadap X itu **pitch** dan terhadap Y itu
**roll** — terbalik. Parameternya sekarang bernama netral `rotX/rotY/rotZ`, dan
`Hexapod::setStabilization` yang memetakan roll→`_rotY`, pitch→`_rotX`. Sisa
pertanyaannya ada di pemasangan IMU, dan itu tidak bisa dijawab dari kode:
lihat `STAB_SWAP_ROLL_PITCH` di `config.h` beserta cara mengujinya dalam 2 menit.

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

### 3. `GaitProfile` perlu field `standRadius`

`STAND_RADIUS` dulu `#define`, jadi profil tidak bisa mengubah bentang kaki.
**Tanpa itu `profileNarrow()` untuk R11 mustahil** — satu-satunya cara
mengecilkan badan hexapod adalah menarik kakinya masuk. Sekarang `standRadius`
jadi field profil di kedua tempat, ikut di-ramp seperti field lain, dan
`Hexapod::profileNarrow()` sudah ada.

### 4. Vektor langkah harus dinormalkan

Yang paling parah, dan tidak kelihatan sampai keempat arah diuji sekaligus.
Perintah maju dan putar **saling menambah**:

| perintah | langkah kaki terpanjang | laju coxa |
|---|---|---|
| maju 1,0 | 60 mm | 256°/s |
| putar 1,0 | 96 mm | 348°/s |
| maju 0,8 + putar 0,7 | **115 mm** | **417°/s** |
| maju 0,8 + putar 1,0 | 136 mm | **520°/s** |

`stepLength` 60 mm ternyata bukan batas apa pun. Dan baris ketiga bukan kasus
buatan — **itulah yang dikeluarkan `followWall()` setiap saat**: maju sambil
mengoreksi heading. Jadi kondisi paling berat justru kondisi normal, dan servo
diminta 2× lipat kemampuannya sepanjang lomba.

Perbaikannya: hitung vektor langkah keenam kaki dulu, lalu bagi **semuanya**
dengan satu faktor skala sehingga yang terpanjang pas `stepLength`. Satu faktor
untuk semua, bukan per kaki — kalau tiap kaki diskalakan sendiri, arah vektornya
berubah dan putaran murni tidak lagi murni. Hasilnya 520 → **219°/s**, tanpa
mengubah bentuk gerakan sama sekali.

Efek sampingnya perlu diketahui: keenam kaki berada di radius ~160 mm dari pusat
sedangkan komponen rotasi dinormalkan terhadap 100 mm, jadi **perintah putar di
atas 0,63 tidak menambah kecepatan** — sudah mentok normalisasi. PD pivot
karenanya jenuh pada error ≳31°. Itu wajar dan tidak perlu "diperbaiki" dengan
menaikkan `heading.kp`.

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

**Laju sendi maksimum sesudah normalisasi, tulis PWM 50 Hz** (batas RDS3235:
400°/s tanpa beban, realistis ~260°/s berbeban):

| profil | maju | putar | maju+putar | geser | bentang saat maju |
|---|---|---|---|---|---|
| 0 DATAR | 256 | 217 | 222 | **277** | 32,0 cm |
| 1 TANGGA | 252 | 200 | 222 | **274** | 32,0 cm |
| 2 MERUNDUK | 248 | 187 | 212 | **270** | 32,0 cm |
| 3 SEMPIT | 229 | 229 | 229 | 214 | **27,0 cm** |

Siklus TANGGA dan MERUNDUK **sudah diubah** supaya masuk batas — 1200→1800 ms
(femur 388→252) dan 900→1100 ms (303→248), di `motion.h` maupun di
`Hexapod::profileStairs/profileCrouch`. Konsekuensinya menaiki tangga jadi
3,9 cm/detik, dan itu memang harga yang wajar.

Yang tersisa dan **sengaja tidak diubah**:

- **DATAR tetap 900 ms** (256°/s, tepat di batas). Ia gait sehari-hari dan
  memperlambatnya berarti kehilangan 10% kecepatan lomba, sementara ~260°/s
  sendiri cuma perkiraan (65% dari spesifikasi tanpa beban). Kalau di lantai
  servo terlihat tertinggal — kaki mendarat tidak di tempatnya, robot ngesot —
  naikkan `gait.cycle_time` ke 1000.
- **Menggeser menyamping (strafe) 270–277°/s**, satu-satunya arah yang masih
  lewat. Dibiarkan karena `Navigation` tidak pernah mengisi `NavCmd.strafe`:
  misi hanya memakai maju + putar. Kalau strafe nanti dipakai, batasi
  perintahnya ke ~0,9 atau perlambat siklusnya dulu.

Angka yang sama keluar dari perintah `S` di robot (dengan `writeHz` yang sedang
berlaku), lengkap dengan ringkasan keempat arah.

**Bentang badan per arah** — yang melebarkan badan ternyata bukan berbelok:

| arah | DATAR | SEMPIT |
|---|---|---|
| maju | 32,0 cm | 27,0 cm |
| putar di tempat | 32,0 cm | 27,0 cm |
| maju + putar | 32,0 cm | 27,0 cm |
| **geser menyamping** | 37,9 cm | **31,5 cm** |

Berbelok tidak melebarkan badan karena kaki tengah — yang paling lebar —
bergerak **searah badan** saat berputar. Yang melebarkan cuma strafe. Untuk R11
artinya: robot **boleh** mengoreksi heading di dalam celah 30 cm, dan itu
kebetulan persis yang dilakukan navigasi. Yang tidak boleh cuma strafe.

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
  S            simulasi gait + ringkasan 4 arah (maju/putar/gabungan/geser)
  F            tabel profil + bentang badan
  N<mm>        cari bentang kaki untuk celah, mis. N300 (R11)

PIVOT & KALIBRASI (butuh IMU)
  C<siklus>    KALIBRASI pivot: der/siklus + TANDA arah (mis. C4)
  O<der>       pivot relatif tertutup, mis. O90 / O-90
  o<n>         pivot ke arah kompas (0=U 1=T 2=S 3=B)
  M<siklus>    jalan maju N siklus untuk diukur mistar
  m<cm>        masukkan hasil ukur -> mm per siklus + slip

TELAPAK PER KAKI (ada kaki tidak menapak saat berdiri) — LAKUKAN DULU
  J            OTOMATIS: IMU jadi sensor sentuh, robot berdiri di 3 kaki
  K            MANUAL: uji kertas (0..5 pilih, +/- turun-naik, s langkah, x keluar)
  j            tabel offset telapak      j0  nolkan

RATA BADAN (butuh IMU, keenam kaki menapak, robot tidak dipegang) — SESUDAH 'J'
  A[der]       RATAKAN otomatis; angka opsional = sudut uji sumbu (bawaan 5)
  Q            uji BADAN vs LANTAI: ratakan, putar robot 180 der, ratakan lagi
  a            status trim + ekuivalen mm per kaki
  a0           nolkan trim            aj  lupakan sumbu IMU (ukur ulang)
  Z            rekam acuan "rata" (badan harus rata menurut waterpass)
  Z0           acuan kembali ke nol

KNOB
  h<mm> tinggi badan   R<mm> bentang kaki   k<mm> panjang langkah
  e<mm> tinggi angkat  p<ms> siklus         d<%> duty tumpu
  l<x10> slew (l30 = 3,0)                   w<hz> laju tulis PWM
  ?  knob   T  telemetri   W/E  simpan/muat EEPROM   H  bantuan
```

## Telapak per kaki (`J`/`K`) — ada kaki yang tidak menapak

**Gejala:** pada 90° semua sendi terlihat benar, tapi pada pose **berdiri** ada
kaki yang menggantung.

### Kenapa 90° bisa benar tapi berdiri tidak

`trim[]` di `servo_map.h` satuannya **mikrodetik** dan ditambahkan *setelah*
konversi derajat→pulse:

```c
float us = PULSE_MIN + (deg / 180.0f) * (PULSE_MAX - PULSE_MIN);
us += m.trim[s];
```

Itu **offset murni** — kalau 90° benar, semua sudut ikut benar, **selama
gain-nya benar**. Gain itulah yang tidak dijamin: 500–2500 µs dianggap tepat
180° (11,11 µs/°), padahal tiap servo berbeda beberapa persen. Galatnya **nol
tepat di 90°** dan tumbuh sebanding jaraknya dari 90 — dan pose berdiri memang
jauh dari 90 (femur 79,43° = −10,6°, tibia 82,02° = −8,0°).

Angka dari `test_motion.py` (bagian 9):

| sumber | akibat di telapak |
|---|---|
| 1 mm di telapak | 0,72° femur = 8,0 µs |
| galat gain 2% | 0,3 mm |
| galat gain 5% | 0,7 mm |
| galat gain 10% | 1,5 mm |
| **1 gerigi spline 25T = 14,4°** | **20 mm** |

Gerigi horn yang meleset **bukan** penyebabnya di sini: 20 mm akan langsung
terlihat pada 90°. Yang tersisa justru yang tak terlihat pada 90°: galat gain,
panjang femur/tibia yang tidak persis 80/90 mm, dan **sag servo saat menahan
beban** — yang memang tidak muncul saat 90° tanpa beban.

Karena itu koreksinya harus **diukur pada pose berdiri dan dalam keadaan
berbeban**, bukan dengan menyetel ulang sudut netral. Bentuknya per **kaki**,
bukan per servo, karena yang penting posisi telapak: `mo.zOff[6]` dalam mm,
negatif = kaki dipanjangkan.

### `J` — otomatis, IMU jadi sensor sentuh

Robot ditumpu **satu tripod** {K0 K2 K4}, tiga kaki lain diangkat 25 mm. Satu
kaki uji diturunkan bertahap; begitu ia menyentuh lantai dan terus memanjang, ia
**mengungkit badan**, dan itu terlihat jelas di roll/pitch IMU. Tinggi saat
ungkitan mulai terdeteksi = tinggi lantai di bawah kaki itu relatif badan.

Kenapa harus tripod dan bukan 5 kaki menumpu: dengan **3** titik tumpu badan
tertentu sepenuhnya, jadi kaki keempat yang menyentuh **pasti** memiringkan
badan. Dengan 5 titik tumpu sistemnya berlebih — kaki keenam bisa menekan tanpa
badan bergerak berarti, dan sentuhannya tak terdeteksi.

**Hanya SATU tripod yang jadi acuan, dan itu syarat kebenaran, bukan penghematan
waktu.** Tiga titik selalu *tepat* membentuk bidang, jadi galat ketiga kaki
penumpu seluruhnya berupa kemiringan badan — dan itu memang tugas `A`. Ketiga
kaki yang diangkat diukur **terhadap bidang itu**, sehingga hasilnya langsung
membuat keenam telapak sebidang: **eksak, sekali ukur, tanpa iterasi**
(`test_motion.py` 11: sisa dari bidang 9e-16 mm).

Versi pertama mengukur **kedua** tripod lalu memperbarui keenam offset serentak.
Itu **tidak konvergen** — tiap tripod diukur memakai offset tripod lawannya,
jadi memperbarui keduanya sekaligus membuat hasilnya berayun. Terukur di uji:
`9,80 → 0 → 9,80 → 0`. Sekarang justru dijadikan uji regresi.

Karena acuannya tidak berubah, **pengukuran kedua memberi angka yang sama
persis** — itulah yang membuatnya sah sebagai uji konsistensi. Bedanya dilaporkan
sebagai ketidakpastian nyata alat ini di robot ini.

Rinciannya:

- pindai **kasar 1,5 mm** untuk mengurung, lalu **halus 0,3 mm**, lalu
  interpolasi linear di perpotongan ambang (sudut ungkit tumbuh mulus dengan
  kedalaman tekan, jadi titik potongnya berarti)
- ambang = `max(0,6°, 5 × sebar derau)` — derau diukur dulu, tidak ditebak
- berhenti berdasarkan **jumlah sampel**, bukan lamanya waktu. Laju frame sudut
  yang benar-benar sampai bisa jauh di bawah 200 Hz yang disetel di aplikasi WIT
  (di robot ini terukur ~50 Hz), dan jendela tetap dalam milidetik membuat
  pengukuran gagal karena kekurangan sampel padahal IMU-nya sehat. `T` sekarang
  menampilkan laju terukur; di bawah 20 Hz `J` memperingatkan.
- **komponen bidang dibuang di akhir** (kuadrat terkecil pada basis `{1, y, x}`).
  Menambahkan bidang apa pun ke keenam offset **tidak** mengubah kesebidangan
  telapak — bidang dikurangi bidang tetap bidang — yang berubah hanya sikap
  badan, dan itu tugas `A`. Jadi komponen bidang boleh dipindahkan ke `A` secara
  cuma-cuma. Untungnya nyata: tanpa itu seluruh koreksi menumpuk di tiga kaki
  yang diukur; dengan itu offset terbesar **turun setengahnya** (10,0 → 5,0 mm),
  sehingga jangkauan IK tidak dimakan percuma.
- **jendela pindai ±18 mm sengaja lebih lebar dari batas offset akhir ±12 mm.**
  Pengukuran mentah bisa ~2× hasil akhir karena komponen bidang belum dibuang:
  kaki meleset 14 mm terukur −14 mm tapi berakhir di 7 mm. Jendela ±12 akan
  menolaknya padahal hasilnya sehat.
- **idempoten**: menjalankan `J` lagi pada robot yang sudah terkalibrasi tidak
  menggeser apa pun (diuji, pergeseran 9e-16 mm).

Butuh **lantai keras dan datar**. Di karpet atau busa telapak tenggelam dan
sentuhan tidak pernah mengungkit badan; `J` akan melapor "tidak menyentuh".

Empat hasil per kaki dibedakan, dan bedanya penting: **sentuh** (normal),
**kepanjangan >8 mm**, **kependekan >8 mm**, dan **pengukuran gagal**. Yang
terakhir wajib terpisah — melaporkan IMU yang macet sebagai "kaki kependekan"
mengirim orang membongkar mekanik yang sebenarnya tidak apa-apa.

### `K` — manual, uji kertas

Tidak butuh IMU dan tidak peduli lantai lunak. Selipkan kertas di bawah tiap
telapak lalu tarik:

- **lolos tanpa gesekan** = kaki menggantung → turunkan (`+`)
- **tersangkut kuat** = kaki keberatan → naikkan (`-`)

Sasarannya keenam telapak terasa sama. Tombol di dalam mode: `0`–`5` pilih kaki,
`+`/`-` turun/naik, `s` ganti langkah (0,2 / 0,5 / 1,0 mm), `n` nolkan kaki ini,
`N` nolkan semua, `t` tabel, `x` keluar. Boleh diketik beruntun (`+++` Enter).

`K` juga jaring pengaman kalau `J` gagal, dan cara mendekatkan kaki yang
melesetnya di luar ±8 mm supaya `J` bisa bekerja.

### Urutannya wajib: `J` dulu, baru `A`

```
s     berdiri di lantai keras & datar
J     ratakan TELAPAK  (semua kaki menapak)   <- ~1 menit, 2 pass
A     ratakan BADAN    (badan sejajar gravitasi)
W     simpan
```

Keduanya melengkapi, tidak menggantikan, dan **tidak** tumpang tindih:

| | yang diperbaiki | yang tidak bisa |
|---|---|---|
| `J`/`K` → `zOff[6]` | keenam telapak jadi **sebidang** | tidak tahu bidang itu miring atau tidak |
| `A` → `trimRoll/Pitch` | badan **rata** terhadap gravitasi | tidak melihat kaki yang menggantung |

Terbalik urutannya, `A` akan meratakan bidang yang salah: IMU hanya melihat
bidang lewat kaki yang **menapak**. `A` sekarang memperingatkan kalau `zOff`
masih nol semua.

Offset ikut terbawa **selama berjalan**, bukan cuma saat berdiri (diuji di
`test_motion.py` 9b) — kalau tidak, kaki yang tadinya menggantung akan
menggantung lagi begitu robot melangkah dan kalibrasinya sia-sia. Rentang ±12 mm
berseling tetap muat di IK pada keempat profil (9c).

## Rata badan otomatis (`A`) — badan miring walau trim sudah disetel

Masalahnya nyata dan penyebabnya menumpuk dari banyak sumber kecil: galat gain
servo, panjang kaki yang tidak persis sama, rangka yang tidak simetris, dan sag
servo yang berbeda-beda karena beban tiap kaki berbeda. Menyetelnya dengan mata
sulit bukan karena kurang teliti, tapi karena **mata tidak bisa membedakan
"badan miring" dari "lantai miring"**. IMU bisa: ia mengukur arah **gravitasi**,
bukan arah lantai.

**`A` mengoreksi kemiringan badan, bukan kaki yang menggantung.** Kalau ada
telapak yang tidak menyentuh lantai, jalankan [`J`/`K`](#telapak-per-kaki-jk--ada-kaki-yang-tidak-menapak)
dulu — `A` hanya melihat bidang lewat kaki yang *menapak*, jadi ia akan
meratakan bidang yang salah.

`A` menutup lingkarannya — miringkan badan sampai IMU membaca rata, simpan sudutnya.

### Kenapa tidak sekadar "kurangi sudut IMU dari pose badan"

Tiga hal yang membuatnya tidak sesederhana itu, dan masing-masing sudah ditangani:

**1. Sumbu IMU belum tentu sumbu badan.** Bergantung orientasi modul dipasang,
roll IMU bisa jadi pitch badan, dan tandanya bisa terbalik. Karena itu `A`
**mengukur** hubungan keduanya dulu: badan dimiringkan ±5° ke tiap sumbu dan
dilihat apa yang berubah di IMU. Hasilnya matriks Jacobian 2×2

```
jac[i][j] = d(sudut IMU ke-i) / d(perintah badan ke-j)
```

lalu koreksinya lewat Newton, `dPerintah = J⁻¹ · galat`. Efek sampingnya berguna:
ini sekaligus **menjawab `STAB_SWAP_ROLL_PITCH` di `config.h` firmware**, yang
sampai sekarang masih 0 dan belum diverifikasi fisik. `A` mencetak nilai yang
benar untuknya.

Kalau determinan Jacobian di bawah 0,25 — badan dimiringkan tapi IMU nyaris tak
bereaksi — `A` **berhenti** alih-alih membagi dengan angka kecil. Itu bukan
kegagalan matematika, itu gejala: kaki selip, robot masih dipegang, ada kaki yang
menggantung, atau IMU membeku.

**2. IMU belum tentu terpasang sejajar pelat badan.** Kalau miring 2°, `A` akan
meratakan sampai IMU nol — yaitu badan **miring 2°**. Diserap `Z`: buat badan
benar-benar rata menurut alat di luar IMU (waterpass di pelat atas; boleh dibantu
`br`/`bp`), lalu `Z` merekam pembacaan IMU saat itu sebagai acuan "rata".
Selama IMU tidak dilepas, cukup sekali.

Untuk pemasangan yang **jauh** dari tegak, acuannya disetel sendiri. IMU di
robot ini terpasang terbalik: roll diam di **−179,98°**. Modul selalu dipasang
pada kelipatan 90° (tegak / terbalik / miring ke samping) dan pecahan derajatnya
yang jadi urusan `Z`, jadi kalau simpangan dari acuan melebihi 45°, `A`
membulatkan acuan ke kelipatan 90° terdekat dan melapor. Tanpa itu `A` mengira
badan miring 180° dan mendorong trim sampai mentok 15°.

**Semua sudut dihitung tahan lipat ±180.** Di dekat batas itu +179,9 dan −179,9
**bertetangga** (beda 0,2°), bukan berjauhan (359,8°). Merata-ratakan mentah
memberi **0°** — badan terbalik dilaporkan tegak — dan mengurangkan mentah
memberi 359,8°, yang melewati ambang apa pun. Karena itu rerata dihitung sebagai
selisih terhadap sampel pertama, dan setiap pengurangan sudut lewat `wrap180()`.
Diuji di `test_motion.py` bagian 10, termasuk konvergensi `A` dengan IMU
terbalik.

**3. Lantai belum tentu rata.** Trim hasil `A` = miring badan + miring lantai,
dan dari satu posisi keduanya tak terbedakan. `Q` memisahkannya: ratakan, angkat
robot dan putar **180° di titik yang sama**, ratakan lagi. Komponen lantai
berbalik tanda di frame badan, komponen badan tidak:

```
t1 = -(badan + lantai)        t2 = -(badan - lantai)
badan = -(t1+t2)/2            lantai = -(t1-t2)/2
```

Rata-rata kedua trim itulah yang benar. Tanpa `Q`, kemiringan lantai tempat uji
ikut tersimpan sebagai trim, dan robot jadi miring di arena.

### Yang TIDAK bisa dilakukan IMU

Satu IMU mengukur **2 derajat kebebasan** (arah gravitasi), sedangkan galat
tinggi 6 kaki punya **6**. Yang bisa disimpulkan hanyalah **bidang** paling cocok
melalui keenam telapak — dan memiringkan badan terhadap bidang itu persis
koreksinya. Galat sisa antar kaki (satu kaki pendek sendiri sehingga
**menggantung**, tidak menahan beban) tidak terlihat oleh IMU dan tidak bisa
diperbaiki dengan memiringkan badan.

Karena itu `A` dan `a` mencetak **ekuivalen trim dalam mm per telapak**. Kalau
angka satu kaki menonjol sendiri, yang salah kaki itu — betulkan trim-nya di
`KALIBRASI/`, jangan ditumpuk terus ke trim badan. Gejala kaki menggantung:
sisa miring tetap besar setelah 8 iterasi.

### Ongkosnya

Trim berlaku **selalu**, termasuk selama berjalan, dan sengaja terpisah dari
`bodyRoll`/`bodyPitch` supaya `b0` tidak menghapusnya. Konsekuensinya ia memakan
jangkauan IK dan menambah laju sendi. Diukur di `test_motion.py` (profil DATAR,
maju):

| trim roll | laju sendi maks | di luar jangkauan IK |
|---|---|---|
| 0° | 256°/s | 0 |
| 3° | 260°/s (+4) | 0 |
| 5° | 262°/s (+6) | 0 |
| 8° | 266°/s (+10) | 0 |

Murah. Tapi perhatikan DATAR/maju memang sudah 256°/s tanpa trim — trim mendorong
sedikit lewat ~260. Penyebabnya bukan trim; kalau servo terlihat tertinggal,
naikkan siklus DATAR 900 → 1000 ms.

Trim di atas **8°** bukan lagi soal setelan: itu kaki bengkok, horn meleset satu
gerigi, atau rangka tidak simetris. `A` memperingatkan dan tetap membatasi di 15°.

### Urutan pakai

```
s        berdiri di lantai datar, keenam kaki menapak, JANGAN dipegang
A        ratakan (kali pertama ia mengukur sumbu IMU dulu, ~10 detik)
Q        (sekali di tempat uji) pastikan itu badan, bukan lantai
W        simpan ke EEPROM — 's' berikutnya langsung berdiri rata
```

`Z` hanya kalau ada waterpass dan IMU dicurigai tidak sejajar pelat badan.

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

Batas atasnya juga ada: **perintah putar > 0,63 tidak menambah cepat** karena
sudah mentok normalisasi vektor langkah. `PIVOT_KP` 0,020 mencapai 0,63 pada
error 31°, jadi di atas 31° pivot berjalan pada kecepatan tetap. Itu perilaku
yang benar — jangan menaikkan `heading.kp` untuk "mempercepat", yang didapat
cuma overshoot di dekat sasaran.

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
| **2048 .. ~2128** | **der/siklus pivot, mm/siklus maju, tanda pivot, trim rata + acuan + sumbu IMU, offset telapak 6 kaki** | **`TES_GERAK`** |

Blok itu **versi 2** sejak kalibrasi rata ditambahkan. EEPROM yang masih berisi
versi 1 tetap terbaca (angka pivot & odometri tidak hilang); trim rata-nya nol
dan tinggal dijalankan `A` lalu `W`.

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
