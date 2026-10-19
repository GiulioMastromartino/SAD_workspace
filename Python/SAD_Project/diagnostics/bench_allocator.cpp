// Native microbenchmark: legacy per-step JacobiSVD vs cached pseudo-inverse.
//
// Build (from Python/SAD_Project):
//   c++ -O3 -DNDEBUG -std=gnu++17 -arch arm64 \
//       -Icpp -Icpp/core -Icpp/actuators \
//       -I../env/lib/python3.11/site-packages/bsk_sdk/include/eigen3 \
//       diagnostics/bench_allocator.cpp \
//       build/cpp/actuators/libsad_actuators.a build/libsad_core.a -o /tmp/bench_alloc
//
// The timing registry is left disabled, so these are production numbers.
#include "sad_actuator_thruster.h"

#include <chrono>
#include <cmath>
#include <cstdio>

using namespace sad;

static ThrusterParams nominal_params() {
    static const double pos[12][3] = {
        { 0.05,  0.10, 0.0}, { 0.05, -0.10, 0.0},
        {-0.05,  0.10, 0.0}, {-0.05, -0.10, 0.0},
        { 0.05, 0.0,  0.15}, {-0.05, 0.0,  0.15},
        { 0.05, 0.0, -0.15}, {-0.05, 0.0, -0.15},
        {0.0,  0.10,  0.15}, {0.0,  0.10, -0.15},
        {0.0, -0.10,  0.15}, {0.0, -0.10, -0.15}};
    static const double dir[12][3] = {
        {-1,0,0},{-1,0,0},{1,0,0},{1,0,0},
        {0,0,-1},{0,0,-1},{0,0,1},{0,0,1},
        {0,-1,0},{0,-1,0},{0,1,0},{0,1,0}};
    ThrusterParams p;
    p.n = 12;
    p.pos.resize(12); p.dir.resize(12); p.tmax.assign(12, 0.5);
    for (int i=0;i<12;++i) {
        p.pos[i] = Vec3(pos[i][0], pos[i][1], pos[i][2]);
        p.dir[i] = Vec3(dir[i][0], dir[i][1], dir[i][2]);
    }
    p.isp = 220.0; p.g0 = 9.80665;
    return p;
}

template <typename Fn>
static double time_ns(Fn fn, int n) {
    for (int i=0;i<10000;++i) fn(i);          // warm up
    auto t0 = std::chrono::steady_clock::now();
    double sink = 0.0;
    for (int i=0;i<n;++i) sink += fn(i);
    auto t1 = std::chrono::steady_clock::now();
    double ns = std::chrono::duration<double, std::nano>(t1-t0).count() / n;
    std::printf("  (sink %.3e)\n", sink);
    return ns;
}

int main() {
    ThrusterParams p = nominal_params();
    ThrusterAllocator a;
    thruster_allocator_init(a, p);

    double out[12];
    const int N = 5000000;

    auto wrench = [](int i) {
        // sweep the command so nothing is loop-invariant
        double s = ((i % 41) - 20) / 20.0;
        return Vec3(0.0, 0.0, s * 0.2);
    };

    double cached = time_ns([&](int i) {
        Vec3 T = wrench(i);
        thruster_allocate_cached_into(a, p, Vec3::Zero(), T, out);
        return out[0];
    }, N);

    double legacy = time_ns([&](int i) {
        Vec3 T = wrench(i);
        std::vector<double> u = thruster_allocate(p, Vec3::Zero(), T);
        return u[0];
    }, N / 20);  // SVD is ~100x slower; fewer samples

    std::printf("\n  allocator microbenchmark (12 thrusters, double, timing off)\n");
    std::printf("  legacy  (build A + JacobiSVD + solve): %9.1f ns/call\n", legacy);
    std::printf("  cached  (precomputed pinv mat-vec)   : %9.1f ns/call\n", cached);
    std::printf("  speedup                              : %9.2fx\n", legacy / cached);
    std::printf("  per evaluate(N=50, 461,587 calls)    : legacy %.1f ms -> cached %.1f ms\n",
                legacy * 461587 / 1e6, cached * 461587 / 1e6);
    return 0;
}
