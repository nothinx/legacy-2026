/* =====================================================================
   lidar_map.h — pemetaan posisi fisik lidar hexapod KRSRI 2026

   6 unit VL53L0X (TOF200C) di belakang TCA9548A 0x70 @ Wire (SDA18/SCL19).

   TATA LETAK FISIK (dilihat dari atas, hidung robot ke atas kertas):

                        DEPAN
                      [ FRONT ]
              kaki kiri depan  | kaki kanan depan
       [LEFT_FRONT]      B O D I      [RIGHT_FRONT]
              kaki kiri tengah | kaki kanan tengah
       [LEFT_REAR ]                   [RIGHT_REAR ]
              kaki kiri blkg   | kaki kanan blkg
                      [ BACK  ]
                       BELAKANG

   Empat lidar samping dipasang DI CELAH ANTAR KAKI supaya kaki tidak
   masuk berkas ukur: kiri/kanan masing-masing satu di celah kaki
   depan-tengah, satu di celah kaki tengah-belakang.

   Urutan indeks = SEARAH JARUM JAM mulai dari depan. Sengaja begitu supaya
   nanti bisa dipakai untuk interpolasi arah / wall-following tanpa tabel
   tambahan.

   NOMOR CHANNEL MUX TIDAK DIASUMSIKAN. Kabel boleh tercolok di channel mana
   pun; sketsa MAP_LIDAR yang menentukan channel mana untuk posisi mana
   (perintah 'w'), lalu hasilnya ditempel balik ke tabel di bawah.
   ===================================================================== */
#ifndef LIDAR_MAP_H
#define LIDAR_MAP_H

#include <stdint.h>

#define NUM_LIDAR     6      // lidar terpasang
#define NUM_MUX_CH    8      // channel TCA9548A

// ---------------------------------------------------------------- posisi
enum LidarPos : uint8_t {
    LID_FRONT       = 0,     // hidung robot, menghadap depan
    LID_RIGHT_FRONT = 1,     // sisi KANAN, celah kaki depan-tengah
    LID_RIGHT_REAR  = 2,     // sisi KANAN, celah kaki tengah-belakang
    LID_BACK        = 3,     // buritan, menghadap belakang
    LID_LEFT_REAR   = 4,     // sisi KIRI,  celah kaki tengah-belakang
    LID_LEFT_FRONT  = 5      // sisi KIRI,  celah kaki depan-tengah
};

static const char* const LIDAR_NAME[NUM_LIDAR] = {
    "FRONT", "RIGHT_FRONT", "RIGHT_REAR", "BACK", "LEFT_REAR", "LEFT_FRONT"
};

// dipakai untuk tampilan denah — harus 2 karakter
static const char* const LIDAR_SHORT[NUM_LIDAR] = { "F ", "RF", "RR", "B ", "LR", "LF" };

// instruksi yang dibacakan ke operator saat wizard pemetaan
static const char* const LIDAR_WHERE[NUM_LIDAR] = {
    "DEPAN  - hidung robot, menghadap ke depan",
    "KANAN  - celah antara kaki kanan DEPAN dan TENGAH",
    "KANAN  - celah antara kaki kanan TENGAH dan BELAKANG",
    "BELAKANG - buritan, menghadap ke belakang",
    "KIRI   - celah antara kaki kiri TENGAH dan BELAKANG",
    "KIRI   - celah antara kaki kiri DEPAN dan TENGAH"
};

// ------------------------------------------------------- hasil pemetaan
// posisi -> channel mux.  -1 = belum dipetakan.
// Tempel ulang blok ini dari keluaran perintah 'g' setelah wizard selesai.
//
// Terpasang terbalik rapi: channel = 5 - posisi. Kabel jelas dicolok berurutan
// mengelilingi robot berlawanan arah jarum jam, sementara indeks posisi di sini
// searah jarum jam. Jangan disederhanakan jadi rumus — kalau satu kabel pindah
// colokan, tabel eksplisit ini yang gampang dibetulkan.
static const int8_t LIDAR_CH_DEFAULT[NUM_LIDAR] = {
     5,   // LID_FRONT
     4,   // LID_RIGHT_FRONT
     3,   // LID_RIGHT_REAR
     2,   // LID_BACK
     1,   // LID_LEFT_REAR
     0    // LID_LEFT_FRONT
};

// koreksi sistematis per lidar, dalam MILIMETER.
// jarak_terkoreksi_mm = jarak_mentah_mm + offset.  Isi lewat perintah 'c<cm>'.
static const int16_t LIDAR_OFFSET_DEFAULT[NUM_LIDAR] = { 0, 0, 0, 0, 0, 0 };

#endif  // LIDAR_MAP_H
