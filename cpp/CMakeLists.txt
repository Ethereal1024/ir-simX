cmake_minimum_required(VERSION 3.15)
project(cpp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ── Detect SIMD support ─────────────────────────────────────────
include(CheckCXXSourceCompiles)

# AVX512 test
set(CMAKE_REQUIRED_FLAGS "-mavx512f -mavx512cd -mavx512dq -mavx512bw -mfma")
check_cxx_source_compiles("
    #include <immintrin.h>
    int main() {
        __m512 a = _mm512_set1_ps(1.0f);
        __m512 b = _mm512_fmadd_ps(a, a, a);
        return 0;
    }" HAS_AVX512)
set(CMAKE_REQUIRED_FLAGS "")

# AVX2 test
set(CMAKE_REQUIRED_FLAGS "-mavx2 -mfma")
check_cxx_source_compiles("
    #include <immintrin.h>
    int main() {
        __m256 a = _mm256_set1_ps(1.0f);
        __m256 b = _mm256_fmadd_ps(a, a, a);
        return 0;
    }" HAS_AVX2)
set(CMAKE_REQUIRED_FLAGS "")

message(STATUS "AVX2  support: ${HAS_AVX2}")
message(STATUS "AVX512 support: ${HAS_AVX512}")

if(HAS_AVX512)
    # AVX512 requires GCC 12+ or Clang 14+ for avx512bw/dq flags
    # Disable for now on GCC 11; AVX2 is sufficient
    set(SIMD_FLAGS -mavx2 -mfma)
    add_compile_definitions(USE_AVX2)
elseif(HAS_AVX2)
    set(SIMD_FLAGS -mavx2 -mfma)
    add_compile_definitions(USE_AVX2)
else()
    set(SIMD_FLAGS "")
endif()

# ── Sources ─────────────────────────────────────────────────────
set(CORE_SOURCES
    src/geometry.cpp
    src/lidar.cpp
    src/collision.cpp
    src/kinematics.cpp
    src/astar.cpp
    src/world.cpp
)

set(CORE_HEADERS
    include/geometry.h
    include/lidar.h
    include/collision.h
    include/kinematics.h
    include/astar.h
    include/world.h
)

# ── pybind11 module ─────────────────────────────────────────────
find_package(pybind11 REQUIRED)

pybind11_add_module(cpp
    ${CORE_SOURCES}
    ${CORE_HEADERS}
    bindings/pybind_module.cpp
)

target_include_directories(cpp PRIVATE include)
target_compile_options(cpp PRIVATE -O3 -march=native)
if(SIMD_FLAGS)
    foreach(flag ${SIMD_FLAGS})
        target_compile_options(cpp PRIVATE ${flag})
    endforeach()
endif()
target_link_libraries(cpp PRIVATE pybind11::headers)

# ── Tests (standalone, no Python) ───────────────────────────────
option(BUILD_TESTS "Build standalone tests" ON)
if(BUILD_TESTS)
    enable_testing()

    add_executable(test_lidar tests/test_lidar.cpp ${CORE_SOURCES} ${CORE_HEADERS})
    target_include_directories(test_lidar PRIVATE include)
    target_compile_options(test_lidar PRIVATE -O3 -march=native)
    if(SIMD_FLAGS)
        foreach(flag ${SIMD_FLAGS})
            target_compile_options(test_lidar PRIVATE ${flag})
        endforeach()
    endif()

    add_executable(test_collision tests/test_collision.cpp ${CORE_SOURCES} ${CORE_HEADERS})
    target_include_directories(test_collision PRIVATE include)
    target_compile_options(test_collision PRIVATE -O3 -march=native)
    if(SIMD_FLAGS)
        foreach(flag ${SIMD_FLAGS})
            target_compile_options(test_collision PRIVATE ${flag})
        endforeach()
    endif()

    add_executable(benchmark tests/benchmark.cpp ${CORE_SOURCES} ${CORE_HEADERS})
    target_include_directories(benchmark PRIVATE include)
    target_compile_options(benchmark PRIVATE -O3 -march=native)
    if(SIMD_FLAGS)
        foreach(flag ${SIMD_FLAGS})
            target_compile_options(benchmark PRIVATE ${flag})
        endforeach()
    endif()

    add_test(NAME test_lidar COMMAND test_lidar)
    add_test(NAME test_collision COMMAND test_collision)
endif()
