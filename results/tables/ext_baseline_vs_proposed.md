# Baseline vs proposed (scenarios A+B+C pooled, paired seeds)

| metric | baseline (paper IT2-FLC) | proposed (IT2-FPD) | change (%) |
|---|---|---|---|
| Capture time (s) | 15.27 | 17.38 | 13.9 |
| Capture rate (%) | 84.44 | 76.67 | -9.2 |
| Tracking error after capture (m) | 0.804 | 0.868 | 8 |
| Robustness index, std (m) | 0.452 | 0.405 | -10.4 |
| Mean error, whole trial (m) | 1.982 | 1.94 | -2.1 |
| Target out of view (%) | 5.204 | 4.783 | -8.1 |
| Control effort | 114.5 | 142.5 | 24.5 |
| Execution time (ms / step) | 0.672 | 0.975 | 45 |

Error metrics use seeds where both controllers captured the target.
