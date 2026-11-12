# Interval Type-2 Fuzzy Logic Control for Vision-Based UAV Target Tracking: Reproduction and Extension

> 🚧 Work in progress. This README will be completed as the project develops.

## Student Information
- **Student:** Mustafa Burak Loğoğlu — 1306230027
- **Department:** Computer Engineering, İstanbul University-Cerrahpaşa
- **Course:** Fuzzy Logic (Bulanık Mantık), 2026–2027 Fall
- **Instructor:** Tolgahan Öztürk

## Original Paper
C. Li, W. Zhao, L. Zhao, L. Ju, and H. Zhang, "Application of fuzzy logic control theory combined with target tracking algorithm in unmanned aerial vehicle target tracking," *Sci. Rep.*, vol. 14, Art. no. 18506, 2024, doi: [10.1038/s41598-024-58140-5](https://doi.org/10.1038/s41598-024-58140-5).

## Installation
```bash
git clone https://github.com/Lucryle/bulanikmantik.git
cd bulanikmantik
python -m venv .venv
# Windows: .venv\Scripts\activate    Linux/macOS: source .venv/bin/activate
pip install -r requirements.txt
```

## Usage
```bash
# unit tests
python -m pytest
# membership functions and controller input-output curves -> results/figures/
python experiments/baseline/plot_control_curves.py
# one closed-loop tracking trial (scenario A/B/C, controller, MF setting, seed)
python experiments/run_single_trial.py --scenario C --controller it2_paper_tuned --mf benchmark --seed 0
python experiments/run_single_trial.py --scenario C --controller it2_pd_tuned --seed 0

# every table and figure (tuning + all experiments)
python experiments/run_all.py                 # add --skip-tuning to reuse the tuned gains
```
Single steps of `run_all.py`:
```bash
python experiments/tune.py                            # fair tuning, same budget for every controller
python experiments/baseline/run_reproduction.py       # R1–R3
python experiments/proposed/run_extension.py          # E1–E2
python experiments/proposed/run_ablations.py          # E3–E6
```

## Experiments
All controllers are tuned with the same budget on separate tuning seeds
(`experiments/tune.py`, assumption A21) and then compared on the same 30 test
seeds (paired tests: t-test, Wilcoxon, Cohen's d_z).

| ID | Experiment | Tables (`results/tables/`) | Figures (`results/figures/`) |
|---|---|---|---|
| R1 | Scenarios A/B/C: paper IT2-FLC vs PID | `repro_R1_*` | `repro_R1_scenarios.png` |
| R2 | Wind 0 / 4.9 / 9.6 / 12.1 m/s | `repro_R2_*` | `repro_R2_wind.png` |
| R3 | MF settings Benchmark / Setting 1 / Setting 2 | `repro_R3_mf_sensitivity` | `repro_R3_mf_sensitivity.png` |
| R4 | MFs and control curves | – | `mf_settings.png`, `control_curves.png` |
| – | Paper vs our relative improvement | `repro_vs_paper` | – |
| E1 | **Proposed IT2 fuzzy-PD** vs paper IT2-FLC vs PID (scenarios + wind) | `ext_E1_*`, `ext_baseline_vs_proposed` | `ext_E1_*.png`, `ext_pd_control_surface.png`, `ext_example_trajectories.png` |
| E2 | Type-1 vs Type-2 | `ext_E2_*` | `ext_E2_t1_vs_t2.png` |
| E3 | FOU width sweep | `abl_E3_fou` | `abl_E3_fou.png` |
| E4 | Pixel noise and missed detections | `abl_E4_*` | `abl_E4_*.png` |
| E5 | Particle filter on/off, latency | `abl_E5_*` | `abl_E5_*.png` |
| E6 | Karnik–Mendel vs Nie–Tan vs Type-1 (accuracy and computation time) | `abl_E6_*` | `abl_E6_compute_time.png` |

**Proposed extension (IT2-FPD).** The paper's controller has one input and 5
rules, i.e. it is a nonlinear P controller. We add the error derivative ė as a
second input (25-rule Mac Vicar-Whelan table, `src/fuzzy/it2_flc.py`,
assumption A22) so the controller can tell a growing error from a shrinking one.

## Repository Layout
| Path | Contents |
|---|---|
| `configs/` | Simulation, scenario, controller and membership-function parameters |
| `src/fuzzy/` | Type-1 / Interval Type-2 membership functions, Karnik–Mendel type reduction, controllers |
| `src/control/` | PID baseline and common controller interface |
| `src/sim/` | UAV, camera, target, wind, simulated detector, trial loop |
| `src/estimation/` | Particle filter |
| `src/metrics.py`, `src/stats.py` | Capture time, tracking error, robustness; paired statistical tests |
| `src/experiment.py` | Parallel trial runner and table helpers |
| `experiments/` | Tuning, reproduction (`baseline/`) and extension (`proposed/`) experiments, `run_all.py` |
| `demo/` | Live demo |
| `results/` | Generated tables and figures |
| `tests/` | Unit tests |
| `docs/` | Assumptions and reference-paper verification notes |
| `references/` | Bibliography |

## Documentation
- [Modelling assumptions](docs/ASSUMPTIONS.md)
- [Reference paper verification](docs/PAPER_VERIFICATION.md)
- [Course project guideline (Turkish)](docs/Bulanik_Mantik_Donem_Projesi_Yonergesi.pdf), kept as a reference.
