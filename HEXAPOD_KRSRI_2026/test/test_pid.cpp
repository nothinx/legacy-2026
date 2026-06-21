// Uji mandiri PD di PC (tanpa hardware).
//   g++ -I.. test_pid.cpp -o tp && ./tp
// Cek: P murni saat dt=0/awal, suku D bertanda benar, reset() membersihkan state.
#include "../types.h"
#include <cstdio>
#include <cassert>

int main() {
    int fails = 0;

    // 1) Sampel pertama (has=false): D harus 0, output = kp*err.
    Pid p{2.0f, 10.0f};
    float u0 = p.step(5.0f, 0.1f);
    if (fabsf(u0 - 10.0f) > 1e-5f) { printf("FAIL sampel awal: %.4f\n", u0); fails++; }

    // 2) Error turun 5->3 dalam dt=0.1s: D=(3-5)/0.1=-20 -> u=2*3+10*(-20)=-194.
    float u1 = p.step(3.0f, 0.1f);
    if (fabsf(u1 - (-194.0f)) > 1e-3f) { printf("FAIL suku D: %.4f\n", u1); fails++; }

    // 3) dt<=0 -> D diabaikan (cegah div-by-zero), output = kp*err.
    Pid q{2.0f, 10.0f};
    q.step(5.0f, 0.1f);
    float u2 = q.step(9.0f, 0.0f);
    if (fabsf(u2 - 18.0f) > 1e-5f) { printf("FAIL dt=0: %.4f\n", u2); fails++; }

    // 4) reset() -> sampel berikutnya kembali P murni.
    p.reset();
    float u3 = p.step(4.0f, 0.1f);
    if (fabsf(u3 - 8.0f) > 1e-5f) { printf("FAIL reset: %.4f\n", u3); fails++; }

    printf(fails ? "\n[FAIL] %d kasus PD gagal\n" : "\n[PASS] semua kasus PD konsisten\n", fails);
    return fails ? 1 : 0;
}
