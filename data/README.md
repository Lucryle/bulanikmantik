# Data

This project uses **no external dataset**. All experiments are simulations
(UAV kinematics, pinhole camera, moving ground target, wind, simulated
detector) whose parameters are in `configs/`.

Randomness is controlled by seeds (`configs/default.yaml → seeds`):
- **Tuning seeds** (1000–1009): used only to tune PID gains and FLC scaling factors.
- **Test seeds** (0–29): used only for the reported results. Every method runs on the same seeds (paired comparison).

Per-trial raw outputs are written to `results/raw/` (not tracked in git).
Summary tables and figures are in `results/tables/` and `results/figures/`.
