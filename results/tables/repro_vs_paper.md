# Paper vs our reproduction (relative change of the FLC versus PID)

| paper source | metric | paper PID | paper FLC | paper change (%) | our PID | our IT2-FLC | our change (%) |
|---|---|---|---|---|---|---|---|
| abstract | capture time (s) | 5.2 | 4.2 | -19.2 | 17.01 | 15.27 | -10.3 |
| abstract | tracking error (m) | 0.8 | 0.68 | -15 | 0.886 | 0.774 | -12.6 |
| Table 3 (benchmark) | capture time (s) | 4.8 | 4.2 | -12.5 | 17.01 | 15.27 | -10.3 |
| Table 3 (benchmark) | tracking error (m) | 0.75 | 0.68 | -9.3 | 0.886 | 0.774 | -12.6 |

Ours: scenarios A+B+C pooled, 30 seeds each. Tracking error uses seeds where both methods captured the target. Absolute values are not comparable (simulation vs real UAV).
