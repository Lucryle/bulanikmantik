# E1 – Paired tests of the proposed controller

| scenario | metric | A | B | n pairs | mean A | mean B | change A vs B (%) | t | p (t-test) | p (Wilcoxon) | Cohen d_z |
|---|---|---|---|---|---|---|---|---|---|---|---|
| A | capture_time | IT2-FPD (proposed) | IT2-FLC (paper) | 30 | 29.76 | 24.35 | 22.21 | 4.535 | 9.229e-05 | 0.0002142 | 0.8279 |
| A | tracking_error | IT2-FPD (proposed) | IT2-FLC (paper) | 14 | 0.538 | 0.4657 | 15.52 | 3.092 | 0.008584 | 0.006714 | 0.8263 |
| A | mean_error_all | IT2-FPD (proposed) | IT2-FLC (paper) | 30 | 2.672 | 3.086 | -13.43 | -1.237 | 0.2261 | 0.8872 | -0.2258 |
| B | capture_time | IT2-FPD (proposed) | IT2-FLC (paper) | 30 | 13.48 | 12.2 | 10.56 | 0.6081 | 0.5479 | 0.1373 | 0.111 |
| B | tracking_error | IT2-FPD (proposed) | IT2-FLC (paper) | 24 | 0.8463 | 0.7048 | 20.08 | 2.55 | 0.01792 | 0.02486 | 0.5204 |
| B | mean_error_all | IT2-FPD (proposed) | IT2-FLC (paper) | 30 | 1.825 | 1.509 | 20.97 | 1.293 | 0.2061 | 0.06057 | 0.2361 |
| C | capture_time | IT2-FPD (proposed) | IT2-FLC (paper) | 30 | 8.908 | 9.255 | -3.746 | -0.3724 | 0.7123 | 0.0735 | -0.06799 |
| C | tracking_error | IT2-FPD (proposed) | IT2-FLC (paper) | 28 | 1.052 | 1.058 | -0.6257 | -0.266 | 0.7922 | 0.5519 | -0.05027 |
| C | mean_error_all | IT2-FPD (proposed) | IT2-FLC (paper) | 30 | 1.323 | 1.35 | -2.005 | -1.568 | 0.1276 | 0.1706 | -0.2864 |
| A | capture_time | IT2-FPD (proposed) | PID | 30 | 29.76 | 25.07 | 18.7 | 4.05 | 0.0003491 | 0.0003385 | 0.7394 |
| A | tracking_error | IT2-FPD (proposed) | PID | 14 | 0.538 | 0.4555 | 18.11 | 18.72 | 8.709e-11 | 0.0001221 | 5.004 |
| A | mean_error_all | IT2-FPD (proposed) | PID | 30 | 2.672 | 2.973 | -10.13 | -0.9402 | 0.3549 | 0.03272 | -0.1717 |
| B | capture_time | IT2-FPD (proposed) | PID | 30 | 13.48 | 15.67 | -13.94 | -0.8832 | 0.3844 | 0.03955 | -0.1613 |
| B | tracking_error | IT2-FPD (proposed) | PID | 22 | 0.8356 | 0.9859 | -15.24 | -9.741 | 3.062e-09 | 4.768e-07 | -2.077 |
| B | mean_error_all | IT2-FPD (proposed) | PID | 30 | 1.825 | 2.04 | -10.51 | -2.118 | 0.04285 | 9.984e-07 | -0.3867 |
| C | capture_time | IT2-FPD (proposed) | PID | 30 | 8.908 | 10.3 | -13.5 | -1.475 | 0.1509 | 0.8036 | -0.2693 |
| C | tracking_error | IT2-FPD (proposed) | PID | 29 | 1.065 | 1.115 | -4.463 | -1.541 | 0.1346 | 0.00128 | -0.2861 |
| C | mean_error_all | IT2-FPD (proposed) | PID | 30 | 1.323 | 1.393 | -5.01 | -5.814 | 2.653e-06 | 1.598e-05 | -1.062 |

change = (A − B) / B; negative = A is lower (better for every metric here).
