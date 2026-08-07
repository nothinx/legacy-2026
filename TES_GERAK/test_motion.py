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
    dict(nama="TANGGA",   h=75, l=80, cyc=1200, H=110, R=70),
    dict(nama="MERUNDUK", h=40, l=55, cyc=900,  H=80,  R=70),
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
        self._home()
        self.legs = [list(h) for h in self.home]

    def _home(self):
        self.home = [foot_home(i, self.prof["R"], self.prof["H"]) for i in range(6)]

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
        for leg in range(6):
            lp = (self.ph + (0.0 if leg % 2 == 0 else 0.5)) % 1.0
            rx, ry = self.home[leg][0], self.home[leg][1]
            sx = (self.c[0] + (-self.c[2] * ry / 100.0)) * self.prof["l"]
            sy = (self.c[1] + (self.c[2] * rx / 100.0)) * self.prof["l"]
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
        rr, pr, yr = (math.radians(self.body[0]), math.radians(self.body[1]),
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
def simulasi(profil, vec, hz=50, cycles=3):
    m = Motion(profil)
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
print("  profil        coxa  femur  tibia   luar-IK   bentang")
for i in range(4):
    mx, bad, n, bmax, _ = simulasi(i, (0, 1, 0))
    print(f"  {PROFIL[i]['nama']:12s} {mx[0]:5.0f} {mx[1]:6.0f} {mx[2]:6.0f}    {bad}/{n:<4d}  {bmax/10:.1f} cm")
    cek(f"{PROFIL[i]['nama']}: seluruh siklus di dalam jangkauan IK", bad == 0)
mxp, badp, np_, bmaxp, _ = simulasi(0, (0, 0, 1))
print(f"  {'putar':12s} {mxp[0]:5.0f} {mxp[1]:6.0f} {mxp[2]:6.0f}    {badp}/{np_:<4d}  {bmaxp/10:.1f} cm")
cek("putar di tempat di dalam jangkauan IK", badp == 0)

# Ini BUKAN kegagalan test — ini temuan yang harus dilaporkan, bukan disembunyikan.
lambat = [PROFIL[i]["nama"] for i in range(4)
          if max(simulasi(i, (0, 1, 0))[0]) > SERVO_BEBAN_DPS]
if max(mxp) > SERVO_BEBAN_DPS:
    lambat.append("putar")
if lambat:
    print(f"  (temuan) melewati ~{SERVO_BEBAN_DPS:.0f} der/s berbeban: {', '.join(lambat)}")
    print("           servo akan tertinggal dari perintah -> robot ngesot, bukan melangkah")

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
cek("bentang SEMPIT + margin 3 cm <= 30 cm", bmax + 30 <= 300,
    f"{bmax/10:.1f} cm + 3,0 cm = {(bmax+30)/10:.1f} cm")
d, ok = Motion(3).solve()
cek("pose SEMPIT masih di dalam jangkauan IK & servo 0..180",
    ok and all(3 < x < 177 for x in d))

print()
if gagal:
    print(f"[FAIL] {len(gagal)} uji gagal: " + "; ".join(gagal))
    sys.exit(1)
print("[PASS] semua uji matematika gerak konsisten")
