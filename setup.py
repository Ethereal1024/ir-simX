from setuptools import setup, Extension
import pybind11
import os, subprocess, tempfile, glob, site as _site

# ── Clean stale C extensions from site-packages ─────────────────
# Editable install (pip install -e .) does NOT remove old .so files
# left in site-packages by previous non-editable installs.  Python's
# PathFinder (sys.meta_path[2]) runs before the editable finder
# (sys.meta_path[4]), so the stale .so is loaded instead of the
# newly built one, silently discarding all C++ changes.
for sp in _site.getsitepackages():
    for f in glob.glob(os.path.join(sp, 'irsim_core*.so*')):
        try:
            os.remove(f)
            print(f'[setup] removed stale: {f}')
        except OSError:
            pass

# ── Detect SIMD ────────────────────────────────────────────────
cpp_args = ["-O3", "-march=native", "-std=c++17"]
has_avx2 = False
test_code = '''
#include <immintrin.h>
int main() {
    __m256 a = _mm256_set1_ps(1.0f);
    __m256 b = _mm256_fmadd_ps(a, a, a);
    return 0;
}'''
try:
    with tempfile.NamedTemporaryFile(suffix='.cpp', mode='w', delete=False) as f:
        f.write(test_code); f.flush()
        result = subprocess.run(
            [os.environ.get('CXX', 'c++'), '-mavx2', '-mfma', f.name, '-o', '/dev/null'],
            capture_output=True)
        has_avx2 = result.returncode == 0
        os.unlink(f.name)
except: pass
if has_avx2:
    cpp_args.extend(["-mavx2", "-mfma", "-DUSE_AVX2"])
    print("irsim_core: AVX2 enabled")

# ── C++ extension ──────────────────────────────────────────────
ext_modules = [
    Extension(
        "irsim_core._core",
        sources=[
            "irsim_core/src/geometry.cpp",
            "irsim_core/src/lidar.cpp",
            "irsim_core/src/collision.cpp",
            "irsim_core/src/kinematics.cpp",
            "irsim_core/src/astar.cpp",
            "irsim_core/src/world.cpp",
            "irsim_core/bindings/pybind_module.cpp",
        ],
        include_dirs=[
            pybind11.get_include(),
            "irsim_core/include",
        ],
        extra_compile_args=cpp_args,
        language="c++",
    ),
]

setup(
    ext_modules=ext_modules,
)
