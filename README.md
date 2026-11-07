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

## Repository Layout
| Path | Contents |
|---|---|
| `configs/` | Simulation, scenario, controller and membership-function parameters |
| `src/fuzzy/` | Type-1 / Interval Type-2 membership functions, Karnik–Mendel type reduction, controllers |
| `src/control/` | PID baseline and common controller interface |
| `src/sim/` | UAV, camera, target, wind, simulated detector, trial loop |
| `src/estimation/` | Particle filter |
| `experiments/` | Reproduction (`baseline/`) and extension (`proposed/`) experiments |
| `demo/` | Live demo |
| `results/` | Generated tables and figures |
| `tests/` | Unit tests |
| `docs/` | Assumptions and reference-paper verification notes |
| `references/` | Bibliography |

## Documentation
- [Modelling assumptions](docs/ASSUMPTIONS.md)
- [Reference paper verification](docs/PAPER_VERIFICATION.md)
- [Course project guideline (Turkish)](docs/Bulanik_Mantik_Donem_Projesi_Yonergesi.pdf), kept as a reference.
