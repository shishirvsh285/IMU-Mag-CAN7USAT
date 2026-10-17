# cpp_check

Checks that the Arduino C++ filter (`../AdaptiveMahony/AdaptiveMahony.h`, float) gives the same
attitude as the Python one (`../adaptive_mahony.py`, double) on identical simulated data.

```bash
g++ -O2 -std=c++11 -o cpp_check/run_filter cpp_check/run_filter.cpp
python tune_sweep.py 3           # makes sim_cache.pkl, the simulated flights used here
python cpp_check/compare_cpp.py
```

`run_filter` reads lines of `dt gx gy gz ax ay az mx my mz awx awy awz` on stdin and prints the
quaternion after each update.

Last result: largest difference 0.35° (during the deployment tumble, from float rounding), and the same
tilt error to two decimals in every scenario.
