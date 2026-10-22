# Benchmark and validation tools

The Monte Carlo runner uses MuJoCo for the robot plant and the native RVO2
Python binding for dynamic obstacles.  RVO2 is required.  The runner raises if
the binding cannot be loaded.

## Python environment

Use Python 3.12 for the local build. Install the project
requirements, then build the Rust extension:

```powershell
uv pip install -r requirements.txt
# Ensure a current Rust toolchain is selected before running maturin.
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

Generate reference-path previews or run the older blended baseline sweep with:

```powershell
python benchmarks/generate_trajectory_plots.py
python benchmarks/run_legacy_hybrid_scenarios.py
```

These are presentation and baseline tools. They are separate from the
publication-oriented RA-PSF Monte Carlo runner.

The runner writes JSON and CSV records plus per-step traces for the selected
trace seed.  The full benchmark defaults to 200 trials and writes records to
`Output/Plots/Paper/ablation_results`.

The horizon experiments accept `--horizon`, `--risk-horizon`, and
`--max-obstacles`.  The generated scene contains at most five static and
dynamic obstacles combined, so the runner rejects a slot bound that would
silently omit an obstacle.  `--slack-fallback` enables the experimental
actuator-aware backup policy, and `--slack-fallback-threshold` controls the
minimum positive QP slack that activates it. Fallback behavior is disabled by
default; the threshold defaults to `0.05`.

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

## Recoverability-gap validation

Run the deterministic Stage A validation before tuning or launching a large
Monte Carlo benchmark:

```bash
python -m benchmarks.validate_recoverability_gap \
  --output Output/Plots/Paper/recoverability_gap_validation
```

Add `--include-mujoco --mujoco-samples 8` to measure a small deterministic
prefix in the current MuJoCo plant. Those measurements are reported separately
because the plant includes force transmission and friction effects that are not
part of the analytic actuator-lag recurrence.

The output contains both JSON and CSV records for the continuous proposition,
the exact zero-order-hold discrete recurrence, and the predicted safe-boundary
classification. The discrete calculation follows the model convention used by
`AugmentedDifferentialDriveModel.step`: position advances with beginning-of-step
velocity, then the first-order actuator lag is updated. Any disagreement between
the continuous and discrete predictions is reported rather than hidden in a
tuned correction factor.

Trace files from `run_mujoco_monte_carlo.py` now include pre-action state,
previous applied command, requested command, applied command, obstacle snapshot,
pre-step and post-step clearance, actuator time constants, solver status, and
fallback state. The legacy `command_v` and `command_omega` fields remain for
plot compatibility.
