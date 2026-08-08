# Panduan Kalibrasi Kaki & Badan

Panduan pakai untuk `TES_GERAK/`. Ditulis 9 Agustus 2026, sesudah sesi pertama
di robot nyata. Latar belakang teknis dan alasan tiap keputusan ada di
[`TES_GERAK/README.md`](TES_GERAK/README.md); yang di sini urutan kerjanya saja.

---

## Masalah yang dijawab

Dua gejala berbeda yang sering tertukar, dan **obatnya berbeda**:

| Gejala | Alat | Yang diperbaiki |
|---|---|---|
| Ada kaki **tidak menapak** saat berdiri | `J` atau `K` | tinggi telapak per kaki (`zOff[6]`, mm) |
| Keenam kaki menapak tapi **badan miring** | `A` | sikap badan (`trimRoll/trimPitch`, derajat) |

Keduanya melengkapi, bukan menggantikan, dan **urutannya wajib `J` dulu**.
IMU hanya melihat bidang lewat kaki yang *menapak* — selama masih ada kaki
menggantung, `A` meratakan bidang yang salah. `A` sekarang memperingatkan
sendiri kalau `zOff` masih nol semua.

**Kenapa 90° bisa terlihat benar tapi berdiri tidak.** `trim[]` di `servo_map.h`
satuannya mikrodetik dan ditambahkan setelah konversi derajat→pulse, jadi ia
offset murni: kalau 90° benar, semua sudut ikut benar **selama gain-nya benar**.
Gain itu yang tidak dijamin — 500–2500 µs dianggap tepat 180°, padahal tiap
servo beda beberapa persen. Galatnya **nol tepat di 90°** dan tumbuh sebanding
jaraknya dari 90, dan pose berdiri memang jauh dari 90 (femur −10,6°, tibia
−8,0°). Tambah beda panjang femur/tibia dan sag servo saat berbeban — yang
memang tidak muncul saat 90° tanpa beban. Karena itu koreksinya harus diukur
**pada pose berdiri, dalam keadaan berbeban**.

---

## Persiapan

- **Lantai keras dan datar.** Bukan karpet atau busa: telapak tenggelam,
  sentuhan tidak pernah mengungkit badan, dan `J` akan melapor "tidak menyentuh".
- Ruang bebas di sekeliling robot.
- Tangan **dekat** robot tapi **jangan menyentuh** selama pengukuran. Robot
  bertumpu di tiga kaki bergantian; menyentuhnya merusak pengukuran.
- Baterai terisi. Sag servo bergantung tegangan, jadi kalibrasi dengan baterai
  lemah tidak berlaku saat baterai penuh.

---

## Urutan lengkap dari nol

```
T          cek IMU: "laju frame sudut TERUKUR" harus > 20 Hz
s          BERDIRI
j0         nolkan offset telapak lama          <- lihat catatan di bawah
J          kalibrasi telapak   (~1 menit)
A          ratakan badan       (~20 detik)
W          simpan ke EEPROM
```

Sesudah `W`, perintah `s` berikutnya langsung berdiri dengan kaki rata dan badan
rata — termasuk setelah Teensy dimatikan, karena nilainya dibaca dari EEPROM
saat boot.

> **`j0` dulu kalau EEPROM masih berisi hasil sebelum 9 Agustus 2026.** Angka
> yang tersimpan dari algoritma lama diukur dengan cara yang terbukti berayun
> (lihat "Riwayat perbaikan"), jadi tidak sah. Kalau `j` menunjukkan semua nol,
> `j0` boleh dilewati.

### Kalau `J` gagal

Pakai `K` (manual, uji kertas) untuk mendekatkan dulu, lalu `J` lagi.

```
K          masuk mode manual
  0..5     pilih kaki
  + / -    turunkan / naikkan telapak kaki terpilih
  s        ganti langkah (0,2 / 0,5 / 1,0 mm)
  n / N    nolkan kaki ini / semua kaki
  t        tabel offset
  x        keluar
```

Uji kertas: selipkan kertas di bawah telapak lalu tarik.

- **lolos tanpa gesekan** = kaki menggantung → `+` (turunkan)
- **tersangkut kuat** = kaki keberatan → `-` (naikkan)

Sasarannya keenam telapak terasa sama. Boleh diketik beruntun: `+++` lalu Enter.

`K` tidak butuh IMU dan tidak peduli lantai lunak, jadi ia jaring pengaman kalau
`J` gagal karena alasan apa pun.

### Opsional

| Perintah | Kapan dipakai |
|---|---|
| `Q` | sekali di tempat uji baru: memisahkan miring **badan** dari miring **lantai** lewat putar 180°. Tanpa ini, kemiringan lantai ikut tersimpan sebagai trim. |
| `Z` | kalau punya waterpass dan ingin acuan lebih halus daripada kelipatan 90°. Jarang perlu. |
| `a` / `j` | lihat status trim badan / offset telapak kapan saja. |

---

## Membaca hasilnya

### Angka yang sehat

- **beda antar dua pengukuran `J` < 1 mm** — alatnya konsisten.
- **sisa miring `A` < 0,3°** — badan rata.
- **rentang offset telapak < 8 mm** — masih bisa dijelaskan galat gain servo
  dan beda panjang link.

### Kapan curiga

| Yang terlihat | Artinya |
|---|---|
| `rentang > 8 mm` | Terlalu besar untuk galat gain (10% cuma ~1,5 mm) atau beda panjang link (~1 mm). Kalibrasi menutupinya, tapi ada yang mekanis. **Periksa kaki dengan offset paling ekstrem.** |
| `K<n> KEPANJANGAN/KEPENDEKAN di luar ±18 mm` | Bukan setelan. Horn servo meleset segerigi (spline 25T = 14,4° = ~20 mm di telapak), femur/tibia terbalik atau beda panjang, atau lantai lunak. |
| `pengukuran GAGAL (IMU/dibatalkan)` | **Bukan soal kaki itu.** Cek `T`. |
| `laju frame sudut < 20 Hz` | Naikkan output rate di aplikasi WIT. |
| sisa `A` mandek besar setelah 8 iterasi | Masih ada kaki menggantung — ulangi `J`. |
| sisa `A` **membesar** tiap iterasi | Sumbu IMU tersimpan tidak cocok lagi (IMU dilepas/dipasang ulang?). Ketik `aj` lalu `A`. |

### Yang perlu diperiksa fisik di robot ini

Sesi 9 Agustus 2026 menemukan **rentang 11,9 mm**, jauh di atas 8 mm. Yang paling
menonjol **K1** — sudah diberi −6 mm manual, `J` masih menemukan titik sentuh di
−10 mm. Kalibrasi menutupinya, tapi periksa fisik K1 dibanding kaki lain:

- gerigi horn servo femur & tibia,
- kekencangan sekrup horn,
- panjang link femur/tibia diukur dengan penggaris.

---

## Yang IMU **tidak bisa** lakukan

Satu IMU mengukur 2 derajat kebebasan (arah gravitasi); galat tinggi 6 kaki punya
6. Karena itu:

- `J` memakai **satu tripod sebagai acuan**. Tiga titik selalu tepat membentuk
  bidang, jadi galat ketiga kaki penumpu seluruhnya berupa kemiringan badan —
  dan itu tugas `A`. Tiga kaki yang diangkat diukur terhadap bidang itu.
- Kaki yang **menggantung** (tidak menahan beban) tidak terlihat IMU sama sekali.
  Itu sebabnya `A` dan `a` mencetak ekuivalen trim dalam **mm per telapak**:
  kalau satu kaki angkanya menonjol sendiri, yang salah kaki itu.

---

## Riwayat perbaikan (kenapa hasil sebelum 9 Agustus 2026 tidak sah)

Lima kesalahan ditemukan lewat robot nyata dan model Python, semuanya sudah
dikunci sebagai uji regresi di `TES_GERAK/test_motion.py` (bagian 8–11):

1. **Sudut IMU tidak tahan lipat ±180.** IMU di robot ini terpasang terbalik,
   roll diam di −179,98°. Di sana +179,9 dan −179,9 **bertetangga** (0,2°),
   bukan berjauhan (359,8°). Rerata mentah memberi **0°** — badan terbalik
   dilaporkan tegak.
2. **Jendela pengukuran dipatok dalam milidetik.** Laju frame sudut yang
   benar-benar sampai terukur ~50 Hz, bukan 200 Hz yang disetel di aplikasi WIT.
   Sekarang berhenti berdasarkan **jumlah sampel**.
3. **Kegagalan pengukuran dilaporkan sebagai temuan mekanis** ("kaki kependekan")
   — mengirim orang membongkar mekanik yang tidak apa-apa.
4. **Hasil dua pass dijumlahkan** padahal pengukuran mengembalikan offset
   absolut. Melipatgandakan koreksi.
5. **Algoritma dua-tripod tidak konvergen.** Tiap tripod diukur memakai offset
   tripod lawannya, jadi memperbarui keenam offset serentak membuat hasilnya
   berayun: `9,80 → 0 → 9,80 → 0`. Diganti satu tripod acuan — eksak, sekali
   ukur, tanpa iterasi.

Ikutan yang bagus dari perbaikan ke-5: komponen **bidang** dibuang di akhir.
Menambahkan bidang ke keenam offset tidak menggeser satu pun telapak terhadap
lantai (yang berubah hanya sikap badan, tugas `A`), jadi ia dipindahkan cuma-cuma
dan offset terbesar turun setengahnya — 10,0 → 5,0 mm. Jangkauan IK tidak dimakan
percuma.

---

## Peringatan

**Kode ini belum pernah lolos compiler dalam sesi mana pun** — tidak ada
Teensyduino di lingkungan tempat ia ditulis. Yang sudah dijamin: kurung seimbang
(pemindai berbasis state) dan 67 uji matematika di `test_motion.py`. Compile di
Arduino IDE sebelum mempercayainya, dan kalau ada error, teks errornya yang
paling berguna untuk dibagikan.

Jalankan ulang `python test_motion.py` di folder `TES_GERAK/` setiap kali
`motion.h` diubah.
