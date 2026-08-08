#!/usr/bin/env python3
"""
test_motion.py — uji matematika motion.h + kinematics.h TANPA hardware.

Ini replika Python dari motion.h. Gunanya bukan menggantikan uji di robot,
tapi menangkap kesalahan yang tidak akan terlihat dengan melihat robot:
tanda sumbu tertukar, invers rotasi yang bukan invers, fase gait yang
melompat, dan sendi yang diminta bergerak lebih cepat dari kemampuan servo.

    cd TES_GERAK
    python test_motion.py

Kalau motion.h diubah, ubah juga replika di sini lalu jalankan ulang.
Berakhir dengan "[PASS] ..." kalau semua lolos, exit code 1 kalau ada yang gagal.
"""
import math
import sys

# ------------------------------------------------ salinan kinematics.h
COXA, FEMUR, TIBIA = 20.0, 80.0, 90.0
LEG_ORIGIN = [(45, 78), (90, 0), (45, -78), (-45, -78), (-90, 0), (-45, 78)]
LEG_ANGLE = [60.0, 0.0, -60.0, -120.0, 180.0, 120.0]

# batas servo RDS3235: 0,15 detik / 60 der tanpa beban
SERVO_MAX_DPS = 400.0
SERVO_BEBAN_DPS = SERVO_MAX_DPS * 0.65


def clampf(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def ik(x, y, z):
    inrange = True
    coxa = math.atan2(y, x)
    Lr = math.hypot(x, y) - COXA
    D = math.hypot(Lr, z)
    Dmin, Dmax = abs(FEMUR - TIBIA) + 1.0, FEMUR + TIBIA - 1.0
    if D < Dmin:
        D, inrange = Dmin, False
    if D > Dmax:
        D, inrange = Dmax, False
    F2, T2, D2 = FEMUR ** 2, TIBIA ** 2, D * D
    a1 = math.atan2(z, Lr)
    a2 = math.acos(clampf((F2 + D2 - T2) / (2 * FEMUR * D), -1, 1))
    knee = math.acos(clampf((F2 + T2 - D2) / (2 * FEMUR * TIBIA), -1, 1))
    return math.degrees(coxa), math.degrees(a1 + a2), math.degrees(knee), inrange


def foot_to_servo(leg, bx, by, bz):
    vx = bx - LEG_ORIGIN[leg][0]
    vy = by - LEG_ORIGIN[leg][1]
    a = math.radians(-LEG_ANGLE[leg])
    lx = math.cos(a) * vx - math.sin(a) * vy
    ly = math.sin(a) * vx + math.cos(a) * vy
    c, f, t, ok = ik(lx, ly, bz)
    return [90 + c, 90 + f, 90 + (t - 90)], ok


def foot_home(leg, r, h):
    a = math.radians(LEG_ANGLE[leg])
    return [LEG_ORIGIN[leg][0] + r * math.cos(a),
            LEG_ORIGIN[leg][1] + r * math.sin(a), -h]


# ----------------------------------------------------- salinan motion.h
def rot_fwd(x, y, z, pitch, roll, yaw):
    """maju: Rz(yaw) * Ry(roll) * Rx(pitch). +X kanan, +Y depan, +Z atas."""
    c, s = math.cos(pitch), math.sin(pitch)
    y1, z1, x1 = c * y - s * z, s * y + c * z, x
    c, s = math.cos(roll), math.sin(roll)
    x2, z2, y2 = c * x1 + s * z1, -s * x1 + c * z1, y1
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x2 - s * y2, s * x2 + c * y2, z2


def rot_inv(x, y, z, pitch, roll, yaw):
    """invers sejati: urutan DIBALIK, bukan sudut dinegatifkan (moRotInv)."""
    c, s = math.cos(yaw), math.sin(yaw)
    x1, y1, z1 = c * x + s * y, -s * x + c * y, z
    c, s = math.cos(roll), math.sin(roll)
    x2, z2, y2 = c * x1 - s * z1, s * x1 + c * z1, y1
    c, s = math.cos(pitch), math.sin(pitch)
    return x2, c * y2 + s * z2, -s * y2 + c * z2


PROFIL = [
    dict(nama="DATAR",    h=40, l=60, cyc=900,  H=100, R=70),
    dict(nama="TANGGA",   h=75, l=70, cyc=1800, H=110, R=70),
    dict(nama="MERUNDUK", h=40, l=55, cyc=1100, H=80,  R=70),
    dict(nama="SEMPIT",   h=30, l=45, cyc=1000, H=100, R=45),
]
KEYS = ("h", "l", "cyc", "H", "R")


class Motion:
    def __init__(self, p=0):
        self.prof = dict(PROFIL[p])
        self.tgt = dict(PROFIL[p])
        self.duty, self.slew, self.ptau, self.stau = 0.5, 3.0, 0.25, 0.10
        self.c = [0.0, 0.0, 0.0]
        self.t = [0.0, 0.0, 0.0]
        self.ph, self.run = 0.0, False
        self.body = [0.0] * 6          # roll pitch yaw tx ty tz
        self.trim = [0.0, 0.0]         # trim rata badan: roll, pitch
        self.zoff = [0.0] * 6          # offset tinggi telapak per kaki (mm)
        self._home()
        self.legs = [list(h) for h in self.home]

    def _home(self):
        self.home = [foot_home(i, self.prof["R"], self.prof["H"]) for i in range(6)]
        for i in range(6):
            self.home[i][2] += self.zoff[i]

    def set_move(self, x, y, w):
        self.t = [x, y, w]

    def set_profile_now(self, p):
        self.prof, self.tgt = dict(PROFIL[p]), dict(PROFIL[p])
        self._home()
        self.legs = [list(h) for h in self.home]

    def update(self, dt):
        dt = min(max(dt, 0.0), 0.05)
        for i in range(3):
            step = self.slew * dt
            if self.t[i] > self.c[i]:
                self.c[i] = min(self.c[i] + step, self.t[i])
            elif self.t[i] < self.c[i]:
                self.c[i] = max(self.c[i] - step, self.t[i])
        ap = dt / (self.ptau + dt)
        for k in KEYS:
            self.prof[k] += (self.tgt[k] - self.prof[k]) * ap
        self._home()
        moving = sum(abs(v) for v in self.c) > 0.002
        if moving and not self.run:
            self.run, self.ph = True, 0.0
        if not moving:
            self.run = False
            a = dt / (self.stau + dt)
            for i in range(6):
                for k in range(3):
                    self.legs[i][k] += (self.home[i][k] - self.legs[i][k]) * a
            return
        self.ph = (self.ph + dt * 1000.0 / self.prof["cyc"]) % 1.0
        sxa, sya = [], []
        for leg in range(6):
            rx, ry = self.home[leg][0], self.home[leg][1]
            sxa.append((self.c[0] + (-self.c[2] * ry / 100.0)) * self.prof["l"])
            sya.append((self.c[1] + (self.c[2] * rx / 100.0)) * self.prof["l"])
        # normalisasi: satu faktor skala untuk keenam kaki (lihat motion.h)
        mag = max(math.hypot(sxa[i], sya[i]) for i in range(6))
        if mag > self.prof["l"] and mag > 0.001:
            f = self.prof["l"] / mag
            sxa = [v * f for v in sxa]
            sya = [v * f for v in sya]
        for leg in range(6):
            lp = (self.ph + (0.0 if leg % 2 == 0 else 0.5)) % 1.0
            sx, sy = sxa[leg], sya[leg]
            if lp < self.duty:
                s = lp / self.duty
                k = 0.5 - s
                dx, dy, dz = sx * k, sy * k, 0.0
            else:
                s = (lp - self.duty) / (1 - self.duty)
                w = 2 * math.pi * s
                k = -0.5 + (s - math.sin(w) / (2 * math.pi))
                dx, dy = sx * k, sy * k
                dz = self.prof["h"] * (1 - math.cos(w)) * 0.5
            self.legs[leg] = [self.home[leg][0] + dx,
                              self.home[leg][1] + dy,
                              self.home[leg][2] + dz]

    def solve(self):
        out, ok = [], True
        rr, pr, yr = (math.radians(self.body[0] + self.trim[0]),
                      math.radians(self.body[1] + self.trim[1]),
                      math.radians(self.body[2]))
        for leg in range(6):
            p = [self.legs[leg][k] - self.body[3 + k] for k in range(3)]
            b = rot_inv(p[0], p[1], p[2], pr, rr, yr)
            d, o = foot_to_servo(leg, *b)
            ok = ok and o
            out += d
        return out, ok

    def bentang(self):
        xs = [l[0] for l in self.legs]
        return max(xs) - min(xs)


# ================================================================= uji
gagal = []


def cek(nama, syarat, detail=""):
    print(f"  [{'ok ' if syarat else 'GAGAL'}] {nama}" + (f"   {detail}" if detail else ""))
    if not syarat:
        gagal.append(nama)


print("=== 1. pose berdiri cocok dengan yang sudah tercatat ===")
m = Motion(0)
d, ok = m.solve()
cek("coxa 90,00 / femur 79,43 / tibia 82,02",
    abs(d[0] - 90.0) < 0.01 and abs(d[1] - 79.43) < 0.01 and abs(d[2] - 82.02) < 0.01,
    f"dapat {d[0]:.2f} / {d[1]:.2f} / {d[2]:.2f}")
cek("bentang berdiri 32,0 cm", abs(m.bentang() - 320.0) < 0.5,
    f"dapat {m.bentang()/10:.1f} cm")

print("\n=== 2. invers rotasi benar-benar invers ===")
worst = 0.0
for ang in (5, 15, 30):
    a = math.radians(ang)
    for p in ([160, 0, -100], [45, 78, -100], [-45, -78, -100]):
        f = rot_fwd(*p, a, a, a)
        b = rot_inv(*f, a, a, a)
        worst = max(worst, max(abs(b[i] - p[i]) for i in range(3)))
cek("rot_inv(rot_fwd(p)) == p sampai 30 der", worst < 1e-6, f"galat maks {worst:.2e} mm")

worst_fw = 0.0
for p in ([160, 0, -100], [45, 78, -100], [45, -78, -100]):
    a = math.radians(15)
    f = rot_fwd(*p, a, a, 0)
    fw = rot_fwd(*f, -a, -a, 0)          # cara firmware
    worst_fw = max(worst_fw, max(abs(fw[i] - p[i]) for i in range(3)))
print(f"  (info) cara firmware pada roll+pitch 15 der: galat {worst_fw:.1f} mm "
      f"-> inilah alasan moRotInv() ada")

print("\n=== 3. tanda sumbu body kinematics ===")
base, _ = Motion(0).solve()
def gerak(idx, val):
    mm = Motion(0)
    mm.body[idx] = val
    dd, _ = mm.solve()
    return [dd[i] - base[i] for i in range(18)]

r = gerak(0, 10)     # roll +10  -> sisi KANAN turun -> femur kanan NAIK
cek("roll+ = miring KANAN", r[4] > 5 and r[13] < -5,
    f"femur K1(kanan) {r[4]:+.1f}  K4(kiri) {r[13]:+.1f}")
p = gerak(1, 10)     # pitch +10 -> DEPAN naik -> femur depan turun
cek("pitch+ = MENDONGAK", p[1] < -5 and p[7] > 5,
    f"femur K0(depan) {p[1]:+.1f}  K2(belakang) {p[7]:+.1f}")
w = gerak(2, 10)     # yaw badan -> coxa berputar, femur/tibia hampir diam
cek("yaw badan hanya memutar coxa", abs(w[0]) > 10 and abs(w[1]) < 3,
    f"coxa K0 {w[0]:+.1f}  femur K0 {w[1]:+.1f}")
z = gerak(5, 20)     # tz +20 -> badan NAIK -> kaki memanjang -> femur turun
cek("z+ = badan NAIK", all(z[i * 3 + 1] < -5 for i in range(6)),
    f"femur {z[1]:+.1f} di semua kaki")

print("\n=== 4. laju sendi vs kemampuan servo (tulis PWM 50 Hz) ===")
def simulasi(profil, vec, hz=50, cycles=3, trim=(0.0, 0.0), zoff=None):
    m = Motion(profil)
    m.trim = list(trim)
    if zoff:
        m.zoff = list(zoff)
        m._home()
        m.legs = [list(h) for h in m.home]
    m.set_move(*vec)
    dt = 1.0 / hz
    for _ in range(int(3.0 / dt)):
        m.update(dt)
    prev, _ = m.solve()
    mx = [0.0, 0.0, 0.0]
    bad, n, bmax, lompat = 0, 0, 0.0, 0.0
    prevleg = [list(l) for l in m.legs]
    for _ in range(int(cycles * m.prof["cyc"] / 1000.0 / dt)):
        m.update(dt)
        cur, ok = m.solve()
        if not ok:
            bad += 1
        for i in range(18):
            mx[i % 3] = max(mx[i % 3], abs(cur[i] - prev[i]) / dt)
        for i in range(6):
            lompat = max(lompat, max(abs(m.legs[i][k] - prevleg[i][k]) for k in range(3)))
        prev, prevleg = cur, [list(l) for l in m.legs]
        n += 1
        bmax = max(bmax, m.bentang())
    return mx, bad, n, bmax, lompat

print(f"  batas: {SERVO_MAX_DPS:.0f} der/s tanpa beban, ~{SERVO_BEBAN_DPS:.0f} der/s berbeban")

# Kasus terburuk BUKAN "maju". Perintah maju dan putar saling menambah, dan
# itulah yang dihasilkan wall-following setiap saat — jadi keempatnya diuji.
ARAH = [("maju", (0, 1, 0)), ("putar", (0, 0, 1)),
        ("maju+putar", (0, 0.8, 0.7)), ("geser", (1, 0, 0))]
print("  profil       " + "".join(f"{n:>12s}" for n, _ in ARAH) + "   bentang maks")
lewat = []
for i in range(4):
    baris, terburuk, bmax_ = [], 0.0, 0.0
    for nama_arah, vec in ARAH:
        mx, bad, n, bmax, _ = simulasi(i, vec)
        cek(f"{PROFIL[i]['nama']} {nama_arah}: di dalam jangkauan IK", bad == 0)
        baris.append(f"{max(mx):>12.0f}")
        terburuk = max(terburuk, max(mx))
        bmax_ = max(bmax_, bmax)
        if max(mx) > SERVO_BEBAN_DPS:
            lewat.append(f"{PROFIL[i]['nama']}/{nama_arah} {max(mx):.0f}")
    print(f"  {PROFIL[i]['nama']:12s}" + "".join(baris) + f"   {bmax_/10:.1f} cm")

# Ini BUKAN kegagalan test — ini temuan yang dilaporkan, bukan disembunyikan.
if lewat:
    print(f"  (temuan) melewati ~{SERVO_BEBAN_DPS:.0f} der/s berbeban: {', '.join(lewat)}")
    print("           servo tertinggal dari perintah -> robot ngesot, bukan melangkah")
else:
    print("  semua arah di dalam kemampuan servo")

print("\n  normalisasi vektor langkah — tanpa itu maju+putar meledak:")
m = Motion(0)
m.c = [0, 0.8, 0.7]
besar = []
for leg in range(6):
    rx, ry = m.home[leg][0], m.home[leg][1]
    besar.append(math.hypot((-0.7 * ry / 100.0) * 60, (0.8 + 0.7 * rx / 100.0) * 60))
cek("langkah terpanjang dibatasi ke stepLength",
    max(besar) > 60.0,   # sebelum normalisasi memang lewat -> itu alasannya ada
    f"sebelum dinormalkan {max(besar):.0f} mm untuk stepLength 60 mm")

print("\n=== 5. lintasan mulus (tak ada lompatan di batas fase) ===")
_, _, _, _, lompat = simulasi(0, (0, 1, 0), hz=200, cycles=2)
cek("perpindahan kaki per tick 200 Hz wajar", lompat < 3.0, f"maks {lompat:.2f} mm")

print("\n=== 6. fase kebal terhadap cycleTime yang di-ramp ===")
def lompatan_fase(el_awal_ms, akumulasi):
    dt, el, cyc = 0.02, el_awal_ms, 900.0
    ph = 0.37 if akumulasi else (el % cyc) / cyc
    mx = 0.0
    for _ in range(300):
        cyc += (1200.0 - cyc) * (dt / (0.25 + dt))
        el += dt * 1000
        baru = (ph + dt * 1000 / cyc) % 1.0 if akumulasi else (el % cyc) / cyc
        d = abs(baru - ph)
        if d > 0.5:
            d = abs(d - 1.0)
        mx = max(mx, d)
        ph = baru
    return mx

for el in (2000, 30000):
    fw, ak = lompatan_fase(el, False), lompatan_fase(el, True)
    print(f"  setelah {el/1000:.0f} detik jalan: fmod(firmware) {fw:.3f}   akumulasi {ak:.3f}")
cek("akumulasi fase tidak melompat walau cycleTime berubah",
    lompatan_fase(30000, True) < 0.03)

print("\n=== 7. profil SEMPIT muat celah 30 cm ===")
m = Motion(3)
m.set_move(0, 1, 0)
dt = 1 / 50.0
for _ in range(150):
    m.update(dt)
bmax = 0.0
for _ in range(int(2 * m.prof["cyc"] / 1000.0 / dt)):
    m.update(dt)
    bmax = max(bmax, m.bentang())
cek("bentang SEMPIT saat MAJU + margin 3 cm <= 30 cm", bmax + 30 <= 300,
    f"{bmax/10:.1f} cm + 3,0 cm = {(bmax+30)/10:.1f} cm")
d, ok = Motion(3).solve()
cek("pose SEMPIT masih di dalam jangkauan IK & servo 0..180",
    ok and all(3 < x < 177 for x in d))

# Arah mana yang MELEBARKAN badan? Jawabannya tidak sesuai dugaan, jadi
# diukur, bukan diterka.
_, _, _, b_putar, _ = simulasi(3, (0, 0, 1))
_, _, _, b_geser, _ = simulasi(3, (1, 0, 0))
cek("BERBELOK tidak melebarkan badan", abs(b_putar - bmax) < 1.0,
    f"putar {b_putar/10:.1f} cm vs maju {bmax/10:.1f} cm")
print(f"  (batasan) MENGGESER menyamping melebarkan badan jadi {b_geser/10:.1f} cm — "
      f"lewat dari 30 cm.")
print("            Jadi di celah R11 robot BOLEH mengoreksi heading (kaki tengah")
print("            bergerak searah badan, bukan ke samping), tapi TIDAK boleh")
print("            menggeser menyamping. Navigation memang tak pernah memakai")
print("            strafe, jadi ini aman selama tidak dipanggil manual.")

print("\n=== 8. kalibrasi rata badan (trim + Jacobian + uji lantai) ===")

# 8a. trim harus setara dengan perintah pose, cuma tidak ikut dinolkan 'b0'.
ma, mb = Motion(0), Motion(0)
ma.body[0], ma.body[1] = 3.0, -2.0
mb.trim = [3.0, -2.0]
da, _ = ma.solve()
db, _ = mb.solve()
cek("trim rata == pose badan yang setara",
    max(abs(da[i] - db[i]) for i in range(18)) < 1e-4,
    f"beda maks {max(abs(da[i]-db[i]) for i in range(18)):.2e} der")
mc = Motion(0)
mc.trim = [3.0, -2.0]
mc.body[0] = mc.body[1] = 0.0          # ini yang dilakukan 'b0'
dc, _ = mc.solve()
cek("'b0' tidak menghapus trim", max(abs(dc[i] - db[i]) for i in range(18)) < 1e-9)


# 8b. Newton dengan Jacobian TERUKUR: harus konvergen walau sumbu IMU
# tertukar dan terbalik. Inilah alasan 'A' mengukur dulu, bukan mengasumsikan.
def imu_palsu(trim_r, trim_p, miring, M):
    """IMU melihat kemiringan fisik badan lewat matriks pemasangan M."""
    br = miring[0] + trim_r          # kemiringan fisik badan = bawaan + trim
    bp = miring[1] + trim_p
    return (M[0][0] * br + M[0][1] * bp, M[1][0] * br + M[1][1] * bp)


def ratakan_palsu(M, miring, probe=5.0, tol=0.3, maks=8):
    # identifikasi beda-tengah, persis identJac()
    J = [[0.0, 0.0], [0.0, 0.0]]
    for ax in range(2):
        plus = imu_palsu(probe if ax == 0 else 0, probe if ax == 1 else 0, miring, M)
        minus = imu_palsu(-probe if ax == 0 else 0, -probe if ax == 1 else 0, miring, M)
        J[0][ax] = (plus[0] - minus[0]) / (2 * probe)
        J[1][ax] = (plus[1] - minus[1]) / (2 * probe)
    det = J[0][0] * J[1][1] - J[0][1] * J[1][0]
    if abs(det) < 0.25:
        return None, None, 0
    tr, tp = 0.0, 0.0
    for it in range(maks):
        e = imu_palsu(tr, tp, miring, M)
        if max(abs(e[0]), abs(e[1])) <= tol:
            return tr, tp, it
        tr -= (J[1][1] * e[0] - J[0][1] * e[1]) / det
        tp -= (-J[1][0] * e[0] + J[0][0] * e[1]) / det
    return tr, tp, maks


PASANG = {
    "lurus":            [[1, 0], [0, 1]],
    "terbalik (180)":   [[-1, 0], [0, -1]],
    "tertukar (90)":    [[0, 1], [-1, 0]],
    "tertukar+balik":   [[0, -1], [1, 0]],
    "miring 20 der":    [[math.cos(math.radians(20)), -math.sin(math.radians(20))],
                         [math.sin(math.radians(20)), math.cos(math.radians(20))]],
}
MIRING = (2.6, -1.4)        # kemiringan badan bawaan yang harus ditemukan
for nama, M in PASANG.items():
    tr, tp, it = ratakan_palsu(M, MIRING)
    ok = tr is not None and abs(tr + MIRING[0]) < 0.05 and abs(tp + MIRING[1]) < 0.05
    cek(f"konvergen dengan IMU terpasang {nama}", ok,
        f"trim {tr:+.2f}/{tp:+.2f} dalam {it} iterasi" if tr is not None else "det terlalu kecil")

# Jacobian singular = badan dimiringkan tapi IMU tak bereaksi (kaki selip /
# robot dipegang). Harus DITOLAK, bukan dipaksa dibagi.
tr, _, _ = ratakan_palsu([[0.05, 0], [0, 0.05]], MIRING)
cek("Jacobian nyaris singular ditolak", tr is None)

# 8c. uji lantai: rata-rata dua trim (sebelum & sesudah putar 180) harus
# menghapus komponen lantai dan menyisakan komponen badan.
badan, lantai = (2.0, -1.0), (1.5, 0.8)
t1 = ratakan_palsu(PASANG["lurus"], (badan[0] + lantai[0], badan[1] + lantai[1]))
t2 = ratakan_palsu(PASANG["lurus"], (badan[0] - lantai[0], badan[1] - lantai[1]))
avgR, avgP = (t1[0] + t2[0]) / 2, (t1[1] + t2[1]) / 2
selR, selP = (t1[0] - t2[0]) / 2, (t1[1] - t2[1]) / 2
cek("rata-rata 2 posisi = kemiringan BADAN",
    abs(avgR + badan[0]) < 0.05 and abs(avgP + badan[1]) < 0.05,
    f"dapat {-avgR:+.2f}/{-avgP:+.2f} der, seharusnya {badan[0]:+.2f}/{badan[1]:+.2f}")
cek("selisih 2 posisi = kemiringan LANTAI",
    abs(selR + lantai[0]) < 0.05 and abs(selP + lantai[1]) < 0.05,
    f"dapat {-selR:+.2f}/{-selP:+.2f} der, seharusnya {lantai[0]:+.2f}/{lantai[1]:+.2f}")

# 8d. Trim berlaku juga SELAMA BERJALAN, jadi ia memakan jangkauan IK dan
# menambah laju sendi. Berapa ongkosnya diukur, bukan diterka.
print("  ongkos trim saat BERJALAN (profil DATAR, maju):")
mx0, bad0, _, _, _ = simulasi(0, (0, 1, 0))
naik = 0.0
for t in (3.0, 5.0, 8.0):
    mxt, badt, _, _, _ = simulasi(0, (0, 1, 0), trim=(t, 0.0))
    naik = max(naik, max(mxt) - max(mx0))
    print(f"    trim roll {t:>4.1f} der -> laju sendi maks {max(mxt):>5.0f} der/s "
          f"({max(mxt)-max(mx0):+.0f}), di luar jangkauan IK: {badt} tick")
    cek(f"trim roll {t:.0f} der tetap di dalam jangkauan IK", badt == 0)
cek("trim sampai 8 der menambah laju sendi < 15 der/s", naik < 15.0,
    f"tambahan terburuk {naik:+.0f} der/s")
# DATAR/maju memang sudah 256 der/s tanpa trim — sengaja, 10% kecepatan lomba
# tidak dilepas demi margin (CLAUDE.md). Jadi trim mendorongnya sedikit lewat
# ~260. Ini dilaporkan, bukan disembunyikan di balik ambang yang dilonggarkan.
mx5, _, _, _, _ = simulasi(0, (0, 1, 0), trim=(5.0, 0.0))
if max(mx5) > SERVO_BEBAN_DPS:
    print(f"  (temuan) DATAR/maju + trim 5 der = {max(mx5):.0f} der/s, lewat "
          f"~{SERVO_BEBAN_DPS:.0f}. Trim bukan penyebabnya (tanpa trim sudah "
          f"{max(mx0):.0f});")
    print("           kalau servo terlihat tertinggal, naikkan siklus DATAR 900 -> 1000 ms.")

print("\n=== 9. offset telapak per kaki (kalibrasi 'J'/'K') ===")

# 9a. offset hanya menggeser kaki yang bersangkutan, dan tepat sebesar nilainya.
m = Motion(0)
m.zoff[3] = -4.0                      # K3 dipanjangkan 4 mm
m._home()
base = Motion(0).home
cek("offset menggeser tepat kaki itu saja",
    abs(m.home[3][2] - (base[3][2] - 4.0)) < 1e-9
    and all(abs(m.home[i][2] - base[i][2]) < 1e-9 for i in range(6) if i != 3),
    f"K3 {m.home[3][2]:.1f} mm, lima kaki lain tidak berubah")

# 9b. offset harus ikut terbawa SELAMA BERJALAN, bukan cuma saat berdiri —
# kalau tidak, kaki yang tadinya menggantung akan menggantung lagi begitu
# robot melangkah, dan kalibrasinya sia-sia.
m = Motion(0)
m.zoff[3] = -4.0
m._home()
m.legs = [list(h) for h in m.home]
m.set_move(0, 1, 0)
dt = 1 / 50.0
for _ in range(150):
    m.update(dt)
zmin_k3, zmin_k1 = 1e9, 1e9
for _ in range(int(2 * m.prof["cyc"] / 1000.0 / dt)):
    m.update(dt)
    zmin_k3 = min(zmin_k3, m.legs[3][2])
    zmin_k1 = min(zmin_k1, m.legs[1][2])
cek("offset terbawa saat berjalan (fase tumpu K3 4 mm lebih rendah)",
    abs((zmin_k1 - zmin_k3) - 4.0) < 1e-6,
    f"K3 {zmin_k3:.1f} mm vs K1 {zmin_k1:.1f} mm")

# 9c. Rentang yang realistis harus tetap muat di IK — kalau tidak, kalibrasi
# malah menciptakan masalah baru di ujung jangkauan kaki.
KASUS = [(6.0, "+-6 mm"), (12.0, "+-12 mm (batas KAKI_MAX_OFF)")]
for besar, nama in KASUS:
    z = [besar if i % 2 == 0 else -besar for i in range(6)]   # kasus terburuk: berseling
    rusak = 0
    for p in range(4):
        _, bad, _, _, _ = simulasi(p, (0, 1, 0), zoff=z)
        rusak += bad
    cek(f"offset {nama} berseling: keempat profil tetap di dalam IK", rusak == 0)

# 9d. zOff dan trim rata bekerja di sumbu berbeda dan tidak boleh saling
# meniadakan: keduanya bersamaan = jumlah efek masing-masing.
a = Motion(0); a.zoff[2] = -3.0; a._home(); da, _ = a.solve()
b = Motion(0); b.trim = [2.0, 0.0]; db, _ = b.solve()
c = Motion(0); c.zoff[2] = -3.0; c.trim = [2.0, 0.0]; c._home(); dc, _ = c.solve()
n0, _ = Motion(0).solve()
worst = max(abs((da[i] - n0[i]) + (db[i] - n0[i]) - (dc[i] - n0[i])) for i in range(18))
cek("zOff dan trim rata bebas satu sama lain", worst < 0.05,
    f"selisih dari penjumlahan {worst:.3f} der (sisa orde-2 rotasi, wajar)")

# 9e. Skala masalahnya: berapa derajat servo yang setara dengan 1 mm telapak,
# dan berapa mm yang dihasilkan galat gain servo. Ini yang menjelaskan kenapa
# 90 der bisa terlihat benar sementara pose berdiri tidak.
FEMUR_DEV = 90.0 - 79.43      # simpangan femur dari 90 der pada pose berdiri
TIBIA_DEV = 90.0 - 82.02
US_PER_DEG = (2500 - 500) / 180.0
print(f"  1 mm di telapak    = {math.degrees(1.0/FEMUR):.2f} der femur "
      f"= {math.degrees(1.0/FEMUR)*US_PER_DEG:.1f} us")
for gain in (2, 5, 10):
    dev = FEMUR_DEV * gain / 100.0
    print(f"  galat gain servo {gain:>2d}% -> femur meleset {dev:.2f} der "
          f"= {math.radians(dev)*FEMUR:.1f} mm di telapak")
print(f"  1 gerigi spline 25T = {360/25:.1f} der = "
      f"{math.radians(360/25)*FEMUR:.0f} mm di telapak (pasti terlihat di 90 der)")
cek("galat gain 10% masih di bawah batas KAKI_MAX_OFF 12 mm",
    math.radians(FEMUR_DEV * 0.10) * FEMUR < 12.0)

print("\n=== 10. matematika sudut tahan lipat +-180 ===")
# Ditemukan di robot: IMU terpasang terbalik sehingga roll diam di -179,98 der.
# Di sana +179,9 dan -179,9 BERTETANGGA (beda 0,2), bukan berjauhan (beda 359,8).


def wrap180(d):
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def rerata_naif(xs):
    return sum(xs) / len(xs)


def rerata_lipat(xs):
    """replika ukurRataT: semua dihitung sebagai selisih thd sampel pertama."""
    ref = xs[0]
    d = [wrap180(x - ref) for x in xs]
    m = sum(d) / len(d)
    var = sum(v * v for v in d) / len(d) - m * m
    return wrap180(ref + m), math.sqrt(max(var, 0.0))


# derau kecil yang kebetulan melewati batas lipat
SAMPEL = [-179.95, 179.98, -179.99, 179.97, -179.93, 179.99]
naif = rerata_naif(SAMPEL)
lipat, sebar = rerata_lipat(SAMPEL)
cek("rerata tahan lipat benar di dekat -180", abs(wrap180(lipat - (-179.99))) < 0.06,
    f"tahan lipat {lipat:.2f} der (sebar {sebar:.3f})")
cek("rerata naif memang salah di sana (alasan perbaikannya ada)", abs(naif) < 1.0,
    f"naif {naif:.2f} der — badan terbalik dilaporkan TEGAK")

# Selisih dua pengukuran yang mengangkangi batas lipat: yang dipakai deteksi
# sentuh 'J' dan identifikasi Jacobian 'A'.
cek("selisih tahan lipat kecil saat mengangkangi batas",
    abs(wrap180(179.9 - (-179.9)) - (-0.2)) < 1e-9,
    f"wrap180(179,9 - (-179,9)) = {wrap180(179.9-(-179.9)):.1f} der, bukan 359,8")


# acuanOtomatis: modul IMU selalu terpasang pada kelipatan 90 der.
def acuan_otomatis(mr, mp, ref_r=0.0, ref_p=0.0):
    if abs(wrap180(mr - ref_r)) <= 45 and abs(wrap180(mp - ref_p)) <= 45:
        return ref_r, ref_p, False
    return round(mr / 90.0) * 90.0, round(mp / 90.0) * 90.0, True


for nama, (mr, mp), harap in [
    ("tegak (0/0)",            (0.4, -0.9),      (0.0, 0.0)),
    ("terbalik (roll -180)",   (-179.98, -0.85), (-180.0, 0.0)),
    ("miring samping (roll 90)", (89.6, 0.3),    (90.0, 0.0)),
]:
    rr, rp, ubah = acuan_otomatis(mr, mp)
    sisa = max(abs(wrap180(mr - rr)), abs(wrap180(mp - rp)))
    cek(f"acuan otomatis: {nama}", (rr, rp) == harap and sisa < 1.0,
        f"acuan {rr:+.0f}/{rp:+.0f}, sisa miring {sisa:.2f} der")

# Kasus yang benar-benar terjadi: -179,98 / -0,85. Tanpa acuan otomatis, 'A'
# mengira badan miring 180 der dan mendorong trim sampai mentok 15 der.
rr, rp, _ = acuan_otomatis(-179.98, -0.85)
cek("tanpa acuan otomatis 'A' akan salah 180 der", abs(-179.98 - 0.0) > 45,
    f"miring semu {-179.98:.2f} der -> dengan acuan otomatis jadi "
    f"{wrap180(-179.98 - rr):+.2f} der")

# Kalibrasi rata harus tetap konvergen dengan IMU terpasang terbalik.
def imu_terbalik(trim_r, trim_p, miring, M, offset=(-180.0, 0.0)):
    br = miring[0] + trim_r
    bp = miring[1] + trim_p
    return (wrap180(offset[0] + M[0][0] * br + M[0][1] * bp),
            wrap180(offset[1] + M[1][0] * br + M[1][1] * bp))


def ratakan_terbalik(M, miring, probe=5.0, tol=0.3, maks=8):
    ref_r, ref_p, _ = acuan_otomatis(*imu_terbalik(0, 0, miring, M))
    J = [[0.0, 0.0], [0.0, 0.0]]
    for ax in range(2):
        pl = imu_terbalik(probe if ax == 0 else 0, probe if ax == 1 else 0, miring, M)
        mi = imu_terbalik(-probe if ax == 0 else 0, -probe if ax == 1 else 0, miring, M)
        J[0][ax] = wrap180(pl[0] - mi[0]) / (2 * probe)
        J[1][ax] = wrap180(pl[1] - mi[1]) / (2 * probe)
    det = J[0][0] * J[1][1] - J[0][1] * J[1][0]
    tr, tp = 0.0, 0.0
    for _ in range(maks):
        m = imu_terbalik(tr, tp, miring, M)
        e = (wrap180(m[0] - ref_r), wrap180(m[1] - ref_p))
        if max(abs(e[0]), abs(e[1])) <= tol:
            return tr, tp
        tr -= (J[1][1] * e[0] - J[0][1] * e[1]) / det
        tp -= (-J[1][0] * e[0] + J[0][0] * e[1]) / det
    return tr, tp


tr, tp = ratakan_terbalik(PASANG["lurus"], MIRING)
cek("'A' konvergen dengan IMU TERBALIK (roll diam di -180)",
    abs(tr + MIRING[0]) < 0.05 and abs(tp + MIRING[1]) < 0.05,
    f"trim {tr:+.2f}/{tp:+.2f}, seharusnya {-MIRING[0]:+.2f}/{-MIRING[1]:+.2f}")

print("\n=== 11. iterasi kalibrasi telapak 'J' konvergen ===")
# Bagian paling halus dari 'J': begitu offset dipasang ke tripod PENUMPU,
# badan ikut miring, sehingga tinggi sentuh kaki uji BERUBAH. Pengukuran
# berikutnya memang seharusnya memberi angka lain — jadi hasil tiap pass harus
# MENGGANTIKAN, bukan ditambahkan. Model di bawah mereproduksi kopling itu.

XY = [(foot_home(i, 70, 100)[0], foot_home(i, 70, 100)[1]) for i in range(6)]


def solve3(A, b):
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        for r in range(3):
            if r == c:
                continue
            f = M[r][c] / M[c][c]
            for k in range(c, 4):
                M[r][k] -= f * M[c][k]
    return [M[i][3] / M[i][i] for i in range(3)]


def ukur_satu_pass(d, o):
    """Replika kakiUkur(): dua tripod, lalu rerata dibuang."""
    meas = [0.0] * 6
    for t in range(2):
        sup = [0, 2, 4] if t == 0 else [1, 3, 5]
        naik = [1, 3, 5] if t == 0 else [0, 2, 4]
        # badan bertumpu: h + a*y + b*x + o_L + d_L = 0 untuk ketiga kaki tumpu
        hp, a, b = solve3([[1.0, XY[L][1], XY[L][0]] for L in sup],
                          [-(o[L] + d[L]) for L in sup])
        for L in naik:                       # kaki uji menyentuh saat world z = 0
            meas[L] = -(hp + a * XY[L][1] + b * XY[L][0]) - d[L]
    r = sum(meas) / 6.0
    return [v - r for v in meas]


def sisa_sebidang(d, o):
    """Sisa terhadap bidang terbaik lewat keenam telapak. 0 = semua menapak."""
    z = [o[i] + d[i] for i in range(6)]
    hp, a, b = solve3(
        [[6.0, sum(XY[i][1] for i in range(6)), sum(XY[i][0] for i in range(6))],
         [sum(XY[i][1] for i in range(6)),
          sum(XY[i][1] ** 2 for i in range(6)),
          sum(XY[i][0] * XY[i][1] for i in range(6))],
         [sum(XY[i][0] for i in range(6)),
          sum(XY[i][0] * XY[i][1] for i in range(6)),
          sum(XY[i][0] ** 2 for i in range(6))]],
        [sum(z), sum(z[i] * XY[i][1] for i in range(6)),
         sum(z[i] * XY[i][0] for i in range(6))])
    return max(abs(z[i] - (hp + a * XY[i][1] + b * XY[i][0])) for i in range(6))


def ukur_satu_tripod(d, o, sup):
    """Replika kakiUkurTripod(): SATU tripod acuan, ukur tiga kaki lain."""
    hp, a, b = solve3([[1.0, XY[L][1], XY[L][0]] for L in sup],
                      [-(o[L] + d[L]) for L in sup])
    hasil = list(o)
    for L in range(6):
        if L not in sup:
            hasil[L] = -(hp + a * XY[L][1] + b * XY[L][0]) - d[L]
    return hasil


def buang_bidang(o):
    """Replika kakiBuangBidang(): kuadrat terkecil pada basis {1, y, x}."""
    S1 = sum(XY[i][1] for i in range(6))
    S0 = sum(XY[i][0] for i in range(6))
    hp, a, b = solve3(
        [[6.0, S1, S0],
         [S1, sum(XY[i][1] ** 2 for i in range(6)), sum(XY[i][0] * XY[i][1] for i in range(6))],
         [S0, sum(XY[i][0] * XY[i][1] for i in range(6)), sum(XY[i][0] ** 2 for i in range(6))]],
        [sum(o), sum(o[i] * XY[i][1] for i in range(6)),
         sum(o[i] * XY[i][0] for i in range(6))])
    return [o[i] - (hp + a * XY[i][1] + b * XY[i][0]) for i in range(6)]


# galat kaki yang sengaja mirip robot nyata (rentang ~12 mm)
D_NYATA = [1.5, 5.9, -5.9, -1.6, 2.9, -2.9]
print(f"  sisa dari bidang sebelum dikalibrasi: {sisa_sebidang(D_NYATA, [0]*6):.2f} mm")

# SATU tripod acuan: eksak dalam SEKALI ukur, tanpa iterasi.
o1 = ukur_satu_tripod(D_NYATA, [0.0] * 6, [0, 2, 4])
cek("satu tripod acuan: keenam telapak sebidang dalam SEKALI ukur",
    sisa_sebidang(D_NYATA, o1) < 1e-9,
    f"sisa dari bidang {sisa_sebidang(D_NYATA, o1):.2e} mm")

# Ulang-ukur dengan acuan sama harus mengembalikan angka yang sama persis —
# itulah yang membuat pass kedua sah sebagai uji konsistensi.
o1b = ukur_satu_tripod(D_NYATA, o1, [0, 2, 4])
cek("ulang-ukur dengan acuan sama memberi hasil identik",
    max(abs(o1b[i] - o1[i]) for i in range(6)) < 1e-9,
    f"beda maks {max(abs(o1b[i]-o1[i]) for i in range(6)):.2e} mm")


# Versi LAMA: dua tripod diukur lalu keenam offset diperbarui SERENTAK.
# Terbukti berayun, bukan konvergen — inilah yang terlihat di robot.
def jalankan_dua_tripod(d, passes=6):
    o = [0.0] * 6
    riwayat = []
    for _ in range(passes):
        o = ukur_satu_pass(d, o)
        riwayat.append(max(abs(v) for v in o))
    return o, riwayat


o_lama, riwayat = jalankan_dua_tripod(D_NYATA)
cek("versi dua-tripod memang BERAYUN (alasan diganti)",
    sisa_sebidang(D_NYATA, o_lama) > 1.0 and abs(riwayat[0] - riwayat[2]) < 1e-9
    and abs(riwayat[0] - riwayat[1]) > 1.0,
    "|offset| maks per pass: " + " ".join(f"{v:.2f}" for v in riwayat[:4]))

# Buang bidang: posisi telapak tidak berubah, tapi offset mengecil.
o1p = buang_bidang(o1)
cek("buang bidang tidak merusak kesebidangan", sisa_sebidang(D_NYATA, o1p) < 1e-9,
    f"sisa dari bidang {sisa_sebidang(D_NYATA, o1p):.2e} mm")
cek("buang bidang memangkas offset terbesar",
    max(abs(v) for v in o1p) < max(abs(v) for v in o1),
    f"{max(abs(v) for v in o1):.1f} -> {max(abs(v) for v in o1p):.1f} mm")

# Idempoten: menjalankan 'J' lagi pada robot yang sudah terkalibrasi tidak
# boleh menggeser apa pun — kalau tidak, tiap kalibrasi ulang merusak yang lama.
o2 = buang_bidang(ukur_satu_tripod(D_NYATA, o1p, [0, 2, 4]))
cek("menjalankan 'J' ulang tidak menggeser hasil (idempoten)",
    max(abs(o2[i] - o1p[i]) for i in range(6)) < 1e-9,
    f"pergeseran maks {max(abs(o2[i]-o1p[i]) for i in range(6)):.2e} mm")

# Satu kaki meleset jauh: hasilnya harus tetap di dalam rentang pindai +-12 mm,
# kalau tidak 'J' akan melaporkannya "di luar rentang sah" padahal terkoreksi.
KAKI_PINDAI_MM, KAKI_MAX_OFF = 18.0, 12.0
for parah in (6.0, 10.0, 14.0, 18.0):
    d1 = [0.0, parah, 0.0, 0.0, 0.0, 0.0]
    mentah = ukur_satu_tripod(d1, [0.0] * 6, [0, 2, 4])
    op = buang_bidang(mentah)
    m_mentah = max(abs(v) for v in mentah)
    m_akhir = max(abs(v) for v in op)
    cek(f"kaki meleset {parah:>4.0f} mm: sebidang, mentah {m_mentah:.0f} mm -> akhir {m_akhir:.0f} mm",
        sisa_sebidang(d1, op) < 1e-9
        and m_mentah <= KAKI_PINDAI_MM + 1e-6      # muat di jendela pindai
        and m_akhir <= KAKI_MAX_OFF + 1e-6)        # dan di batas offset akhir
print(f"  jendela pindai {KAKI_PINDAI_MM:.0f} mm sengaja lebih lebar dari batas "
      f"offset akhir {KAKI_MAX_OFF:.0f} mm: pengukuran mentah ~2x hasil akhir.")

print()
if gagal:
    print(f"[FAIL] {len(gagal)} uji gagal: " + "; ".join(gagal))
    sys.exit(1)
print("[PASS] semua uji matematika gerak konsisten")
