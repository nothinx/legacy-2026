# KALIBRASI — trim, uji IK per kaki, uji jalan

Sketsa Teensy 4.1. Library: **Adafruit PWM Servo Driver Library**.
Serial Monitor **115200**, line ending **Newline**.

Tiga tahap dalam satu sketsa, dikerjakan berurutan:

1. **Trim** — ratakan netral tiap servo
2. **IK** — buktikan gerak tiap kaki mulus sebelum menyusunnya jadi gait
3. **Jalan** — gait tripod dengan kecepatan yang bisa disetel

`kinematics.h` adalah **salinan persis** IK firmware (`LegIK::solve` +
`Hexapod::solvePose`), jadi apa yang mulus di sini akan mulus juga di firmware.
Mapping servo dibaca dari EEPROM, atau dari bawaan `servo_map.h` bila EEPROM kosong.

## Kenapa `n`/`s` di SET_HOME terasa lambat

Itu setelan, bukan batas servo. SET_HOME memakai jeda 150 ms × 24 servo (3,6 detik)
dan ramp 1500 ms — sengaja, karena saat perakitan posisi mekanik servo tidak
diketahui dan lompatan mendadak bisa merusak gearbox.

Sebagai pembanding: perpindahan netral→berdiri hanya **10,57°** di femur dan
**7,98°** di tibia. RDS3235 pada 0,15 s/60° menempuhnya dalam **~26 ms**.

Di sketsa ini transisi pose default **400 ms** dan semuanya jadi knob:

| Knob | Arti | Default |
|---|---|---|
| `v<ms>` | durasi transisi pose | 400 |
| `q<ms>` | jeda antar servo saat netral pertama | 60 |
| `f<hz>` | refresh PWM PCA9685 | 50 |
| `p<ms>` | 1 siklus gait penuh | 900 |
| `k<mm>` | panjang langkah | 60 |
| `e<mm>` | tinggi angkat kaki | 40 |
| `h<mm>` | tinggi badan | 100 |
| `R<mm>` | bentang kaki | 70 |

`?` menampilkan semuanya sekaligus dengan perkiraan laju jalan.

## Di mana batas kecepatan sebenarnya

Empat batas, urut dari yang paling mengikat:

**1. Kecepatan mekanik servo.** RDS3235 ±0,15 s/60° tanpa beban (~400°/s), realistis
~250°/s dengan beban. Ini plafon keras — tidak bisa dinaikkan lewat kode, hanya
lewat tegangan (7,4 V lebih cepat dari 6 V, dalam batas spesifikasi servo).

**2. Refresh PWM.** Pada 50 Hz servo hanya menerima perintah baru tiap **20 ms**.
Untuk siklus gait 900 ms itu 45 titik pembaruan — cukup halus. Tapi kalau siklus
diturunkan ke 400 ms, tinggal 20 titik dan gerakan mulai terlihat patah-patah.
Naikkan `f` ke 100–200 Hz. RDS3235 digital umumnya sanggup; **uji dulu** — kalau
servo berdengung, panas, atau bergetar di posisi diam, turunkan lagi.

**3. Jangkauan kaki.** Langkah panjang + badan tinggi bisa membuat target di luar
jangkauan; sketsa memberi peringatan `di luar jangkauan` dan meng-clamp. Kalau
muncul, kecilkan `k` atau ubah `h`.

**4. Bus I2C — bukan masalah.** 18 servo @400 kHz ≈ 2,9 ms per refresh, ~15% dari
jendela 20 ms. Bahkan di 200 Hz masih muat.

Laju jalan = `2 × panjang langkah / siklus`. Default 60 mm dan 900 ms →
**13,3 cm/detik**. Untuk lomba, mulai dari sini lalu turunkan `p` bertahap sambil
memantau apakah kaki mulai selip atau badan bergoyang.

## Perintah

### Trim & netral

| | |
|---|---|
| `n` | NETRAL 90° semua |
| `L` | tabel mapping + trim |
| `t<slot> <us>` | set trim, mis. `t4 -40` |
| `j<slot> <deg>` | jog satu sendi, mis. `j4 110` |
| `i<slot>` | balik arah sendi |
| `W` / `F` | simpan ke EEPROM / muat bawaan program |

Cara menyetel trim: `n`, lalu lihat sendi mana yang tidak lurus/simetris. Pakai
`j<slot> <deg>` untuk mencari posisi lurusnya, hitung selisih pulse dari 1500 µs,
masukkan lewat `t<slot> <us>`. Contoh: lurus di 1460 µs → `t7 -40`. Selesai semua,
`W`.

### Uji IK per kaki

| | |
|---|---|
| `s` | BERDIRI (pose home IK) |
| `c<kaki>` | kaki menggambar lingkaran di bidang vertikal — **uji femur + tibia** |
| `o<kaki>` | kaki ayun mendatar — **uji coxa** |
| `b` | badan naik-turun, keenam kaki di tanah — uji beban & kemulusan bersama |
| `x` | stop, kembali ke pose berdiri |

Yang dicari saat uji: gerakan **kontinu tanpa sentakan**, tidak ada titik di mana
kaki "melompat". Sentakan biasanya berarti trim belum rata, invert salah di salah
satu sendi, atau target menyentuh batas jangkauan.

`b` layak dijalankan lama — ini yang paling mirip beban sebenarnya dan paling
cepat memperlihatkan servo yang lemah atau catu daya yang drop.

### Jalan

| | |
|---|---|
| `g` / `G` | gait tripod maju / mundur |
| `y` | putar di tempat |
| `x` | stop, kembali berdiri |
| `r` | lepas semua servo |

Gait tripod: kaki 0/2/4 dan 1/3/5 bergantian, beda fase setengah siklus. Ayun
memakai profil sinus untuk angkatnya.

## Keselamatan

- Uji `g`/`b` pertama kali dengan **badan diganjal** — kalau ada sendi yang arahnya
  masih salah, robot bisa menjatuhkan diri.
- Catu daya servo harus kuat. 18 servo bergerak serentak menarik arus besar;
  tegangan drop bisa me-reset Teensy di tengah gait.
- `r` kapan saja untuk melepas semua servo.

## Setelah kalibrasi beres

Nilai yang didapat di sini masuk ke firmware:

| Dari sini | Ke firmware |
|---|---|
| trim & invert per servo | `Calib::applyDefaults` di `Calib.cpp`, atau via GUI tuner |
| `p` siklus gait | `gait.cycle_time` |
| `k` panjang langkah | `gait.step_length` |
| `e` tinggi angkat | `gait.step_height` |
| `h` tinggi badan | `STAND_HEIGHT` di `config.h` |
| `R` bentang kaki | `STAND_RADIUS` di `config.h` |
| `f` refresh PWM | `SERVO_PWM_FREQ` + `SERVO_COMMIT_MS` di `config.h` |
