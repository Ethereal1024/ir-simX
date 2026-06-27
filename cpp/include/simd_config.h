#pragma once
#include <cstdint>

// ── SIMD width detection ────────────────────────────────────────
#if defined(__AVX512F__)
    constexpr int SIMD_WIDTH = 16;
#elif defined(__AVX2__) || defined(USE_AVX2)
    constexpr int SIMD_WIDTH = 8;
#else
    constexpr int SIMD_WIDTH = 1;
#endif

// ── Runtime configuration for batch simulation ──────────────────
struct BatchConfig {
    int batch_size = 1;
    bool share_obstacles = true;  // mode A: single obstacle set; mode B: per-environment
};
