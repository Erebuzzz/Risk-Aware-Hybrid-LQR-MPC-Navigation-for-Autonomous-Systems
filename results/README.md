# Results (plots and evaluation outputs)

Generated artifacts are written under the repo root (folder **`Output/`** is gitignored so local runs do not clutter git).

## Plots from standalone simulation

After you run [`run_simulation.py`](../run_simulation.py), figures appear under:

```text
Output/Plots/<Mode>/<trajectory>/...
Output/Plots/<Mode>/<trajectory>/<scenario>/...
```

See **“Output / results layout”** in the main [`README.md`](../README.md).

## Trajectory preview figures

```bash
python generate_trajectory_plots.py
```

Output: `Output/Plots/Trajectories/<trajectory_name>/`

## Evaluation (Monte Carlo, etc.)

Default output directory for statistical scripts:

```text
Output/Plots/Evaluation/
```

(Regenerate with `python evaluation/statistical_runner.py --help` and related commands in README.)

## Report figures (submission)

The written report may use **`report/figures/`** for images referenced from [`report/report.tex`](../report/report.tex). Placeholders in the PDF still build if PNGs are absent.
