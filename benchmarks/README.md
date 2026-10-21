# RA-PSF benchmark setup

The Monte Carlo runner uses MuJoCo for the robot plant and the native RVO2
Python binding for dynamic obstacles.  RVO2 is required.  The runner raises if
the binding cannot be loaded.

## Python environment

Use Python 3.12 for the current Windows build.  Install the project
requirements, then build the Rust extension:

```powershell
uv pip install -r requirements.txt
maturin develop --release --manifest-path crates/ra_psf_core/Cargo.toml
```

## Native RVO2 binding

Build `sybrenstuvel/Python-RVO2` with CMake, a C++ compiler, and Cython.  The
following commands are an example for MSYS2 UCRT64 on Windows:

```powershell
git clone https://github.com/sybrenstuvel/Python-RVO2.git $env:TEMP\Python-RVO2
uv pip install Cython
python setup.py build_ext --inplace --compiler=mingw32
```

Set the binding and native runtime paths before running a benchmark:

```powershell
$env:RVO2_PYTHON_PATH = "$env:TEMP\Python-RVO2"
$env:RVO2_DLL_DIR = "C:\msys64\ucrt64\bin"
```

On Windows, the extension should be linked with the MinGW C++ runtime so it
does not conflict with CasADi's bundled runtime.  Add
`-static-libstdc++ -static-libgcc` to the extension link arguments when the
binding is built in a mixed numerical environment.

## Checks

Run the deterministic tests and the 500-solve latency gate:

```powershell
python -m pytest -q tests
python benchmarks/benchmark_safety_filter.py
python benchmarks/benchmark_safety_filter.py --backend rust
python benchmarks/benchmark_safety_filter.py --backend python
```

Run a short five-controller smoke benchmark before a full Monte Carlo run:

```powershell
python benchmarks/run_mujoco_monte_carlo.py --trials 1 --duration 0.5
```

The runner writes JSON and CSV records plus per-step traces for the selected
trace seed.  The full benchmark defaults to 200 trials and writes records to
`Output/Plots/Paper/ablation_results`.

The horizon experiments accept `--horizon`, `--risk-horizon`, and
`--max-obstacles`.  The generated scene contains at most five static and
dynamic obstacles combined, so the runner rejects a slot bound that would
silently omit an obstacle.  `--slack-fallback` enables the experimental
actuator-aware backup policy, and `--slack-fallback-threshold` controls the
minimum positive QP slack that activates it.  Both fallback behavior and its
fallback behavior is disabled by default; the threshold defaults to `0.05`.

Generate the publication diagnostics from a benchmark summary:

```powershell
python benchmarks/generate_publication_plots.py --summary Output/Plots/Paper/ablation_results/mujoco_rvo2_ablation.json --output Output/Plots/Paper
```

The speed-latency and friction sweeps are separate so that their trial counts
can be controlled independently:

```powershell
python benchmarks/run_phase_sweep.py --output Output/Plots/Paper/ablation_results/phase_sweep.json
python benchmarks/run_slip_ablation.py --output Output/Plots/Paper/ablation_results/slip_ablation.json
```

The Rust backend uses a persistent full-horizon OSQP workspace and warm-starts
both primal and dual variables between solves.  The Python backend is the
reference implementation used for numerical parity checks.
