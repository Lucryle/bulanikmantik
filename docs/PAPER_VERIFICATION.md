# Reference Paper — Verified Facts and Inconsistencies

This file was checked against the published PDF of:

> C. Li, W. Zhao, L. Zhao, L. Ju, and H. Zhang, "Application of fuzzy logic
> control theory combined with target tracking algorithm in unmanned aerial
> vehicle target tracking," *Scientific Reports*, vol. 14, Art. no. 18506,
> 2024, doi: 10.1038/s41598-024-58140-5.
> Received 13 Nov 2023 · Accepted 26 Mar 2024 · Published 9 Aug 2024.
> License: CC BY-NC-ND 4.0. PMCID: PMC11316052.

Page numbers refer to the PDF (17 pages).

## 1. What the paper states

### 1.1 Controller (pp. 3–5, Figs. 1–2, Eqs. 1–15)
- There are two IT2 fuzzy controllers: the upper one controls yaw (input Δx) and the lower one controls speed (input Δy).
- Normalisation: k_x = Δx/x_c (1), k_y = Δy/y_c (2), k_yaw = yaw/(π/3) (3), k_v = v/2 (4).
- Linguistic terms:
  - Inputs: E = {NB, NS, MM, PS, PB}.
  - Yaw: F = {LB, LS, ZO, RS, RB}.
  - Speed: G = {GB, GS, ZO, BS, BB}.
- Rules:
  - Yaw, Eqs. 5–9: NB→LB, NS→LS, MM→ZO, PS→RS, PB→RB.
  - Speed, Eqs. 10–14: NB→GB, NS→GS, MM→ZO, PS→BS, PB→BB.
- Inference is written as E_Δx × E_Δy → F × G (15). Output uses "central set defuzzification".
- Not stated: inference operator, type-reduction algorithm, FOU parameters, output MFs, Δx MFs.

### 1.2 Membership functions for Δy (p. 10, given in the running text, **not in a table**)

| Term | Benchmark | Setting 1 | Setting 2 |
|---|---|---|---|
| NB | (0, 5, 10) | (0, 6, 12) | (0, 4, 10) |
| NS | (3, 8, 13) | (4, 9, 14) | (3, 7, 12) |
| MM | (8, 13, 18) | (9, 14, 19) | (7, 12, 17) |
| PS | (13, 18, 23) | (14, 19, 24) | (12, 17, 22) |
| PB | (18, 23, 28) | (19, 24, 29) | (17, 22, 28) |

### 1.3 Experimental setup (p. 9, Table 1)
- **Table 1** (Parameter settings):
  - UAV: DJI Phantom 4 Pro.
  - Control algorithm: fuzzy logic + target tracking.
  - Contrast (comparison) algorithm: traditional PID.
  - Image resolution: 1920×1080.
  - **Number of edge devices: 3.**
- Scenes:
  - A: fast target movement.
  - B: complex environment change (obstacles, lighting).
  - C: motion-mode change.
- 10 experiments per scene.

### 1.4 Results

**Scene-wise values for the proposed method (p. 11, text, read from Figs. 7–9):**

| Scene | Capture time (s) | Tracking error (m) | Robustness index |
|---|---|---|---|
| A | ≈4.2 | ≈0.68 | ≈0.84 |
| B | ≈4.0 | ≈0.65 | ≈0.87 |
| C | ≈4.3 | ≈0.70 | ≈0.83 |

**Table 2 — wind speeds (p. 12, proposed method only, no PID values):**

| No. | Max wind | Capture time (s) | Tracking error (m) |
|---|---|---|---|
| 1 | 12.1 m/s | 3.8 | 0.72 |
| 2 | 9.6 m/s | 4.1 | 0.67 |
| 3 | 4.9 m/s | 4.5 | 0.69 |

**Table 3 — sensitivity analysis (p. 14):**

| Algorithm | Setting | Capture (s) | Error (m) | Robustness |
|---|---|---|---|---|
| Proposed | Benchmark | 4.2 | 0.68 | 0.84 |
| Proposed | Setting 1 | 4.1 | 0.70 | 0.82 |
| Proposed | Setting 2 | 4.3 | 0.65 | 0.86 |
| PID | Default | 4.8 | 0.75 | 0.78 |
| PID | Setting 1 | 5.0 | 0.72 | 0.76 |
| PID | Setting 2 | 4.9 | 0.78 | 0.80 |
| CNN | Default | 5.2 | 0.80 | 0.75 |
| CNN | Setting 1 | 5.5 | 0.78 | 0.73 |
| CNN | Setting 2 | 5.3 | 0.82 | 0.77 |
| DQN | Default | 5.0 | 0.78 | 0.77 |
| DQN | Setting 1 | 5.2 | 0.76 | 0.75 |
| DQN | Setting 2 | 5.1 | 0.79 | 0.78 |
| MPC | Default | 5.5 | 0.85 | 0.72 |
| MPC | Setting 1 | 5.7 | 0.82 | 0.70 |
| MPC | Setting 2 | 5.6 | 0.87 | 0.73 |

**Table 4 — t-tests of "our algorithm in different scenes" (p. 14).** The comparator and the sign convention are not stated.

| Scene | Index | t | p | Result |
|---|---|---|---|---|
| A | Capture time | −3.21 | 0.002 | significant |
| A | Tracking error | 2.15 | 0.035 | significant |
| A | Robustness | −1.98 | 0.049 | significant |
| B | Capture time | −2.45 | 0.015 | significant |
| B | Tracking error | 1.75 | 0.078 | not significant |
| B | Robustness | −2.15 | 0.028 | significant |
| C | Capture time | −2.75 | 0.009 | significant |
| C | Tracking error | 1.89 | 0.062 | not significant |
| C | Robustness | −2.35 | 0.020 | significant |

**Table 5 — comparison with recent methods (p. 14):**

| Method | Capture (s) | Error (m) | Robustness |
|---|---|---|---|
| Proposed | 4.17 | 0.68 | 0.85 |
| Zatout et al. [32] | 4.8 | 0.75 | 0.79 |
| de Koning & Jamshidnejad [33] | 5.2 | 0.82 | 0.75 |
| Tsitses et al. [34] | 4.5 | 0.72 | 0.81 |

**Abstract (p. 1):**
- Capture time 5.2 s (PID) → 4.2 s, a 20% reduction.
- Tracking error 0.8 m (PID) → 0.68 m, a 15% reduction.
- "Fluctuation range of tracking error is only half of that of traditional PID".

**Text (p. 12):**
- Capture time is 12–20% shorter than PID, CNN, DQN and MPC.
- Tracking error is 13–18% lower.

## 2. Numeric checks we computed

| Claim | Check | Verdict |
|---|---|---|
| Abstract: PID 5.2 s / 0.80 m | Table 3 lists PID (default) as **4.8 s / 0.75 m**. The pair 5.2 / 0.80 matches the **CNN** default row instead. | ❌ Inconsistent |
| −20% capture time | 5.2 → 4.2 = −19.2%. Against Table 3 PID: 4.8 → 4.2 = **−12.5%** | ⚠️ Depends on which PID value is used |
| −15% tracking error | 0.80 → 0.68 = −15.0%. Against Table 3 PID: 0.75 → 0.68 = **−9.3%** | ⚠️ Same issue |
| "12–20% shorter capture time" vs PID/CNN/DQN/MPC | From Table 3 defaults: −12.5% / −19.2% / −16.0% / −23.6% | ⚠️ MPC falls outside the range |
| "13–18% lower tracking error" | From Table 3 defaults: −9.3% / −15.0% / −12.8% / −20.0% | ❌ Does not match |
| Robustness index = standard deviation, "lower than PID" (p. 13) | Table 3: proposed 0.84 > PID 0.78 (higher in every row) | ❌ Contradicts the definition |
| "Fluctuation range is half of PID" (abstract) | No table supports this | ❌ Unsupported |
| Setting 2 is "slightly better in capture time and error" (p. 13) | Setting 2 has the **worst** capture time (4.3 s) and the best error (0.65 m) | ❌ Half true |
| PID setting 1 "performs slightly better in time and error" (p. 13) | PID S1 capture time 5.0 s > default 4.8 s, so it is worse; error 0.72 < 0.75 | ❌ Half true |
| Table 5: "0.63 s and 1.03 s shorter than Zatout / de Koning" | 4.8 − 4.17 = 0.63, 5.2 − 4.17 = 1.03 | ✅ |
| Table 5: "slightly **longer** by 0.33 s than Tsitses" | 4.17 < 4.5, so it is **shorter** by 0.33 s | ❌ Wrong direction |
| Table 5: error "0.07 / 0.14 lower than Zatout / de Koning; 0.04 lower than Tsitses" | 0.07, 0.14, 0.04 | ✅ |
| Table 5: robustness "0.06 / 0.04 higher than Zatout / Tsitses" | 0.06, 0.04 | ✅ |
| Table 5: robustness "lower by 0.1 than de Koning" | 0.85 > 0.75, so it is **higher** by 0.10 | ❌ Wrong direction |
| Table 2: wind | Capture time is *shortest* at the *highest* wind (3.8 s at 12.1 m/s). No explanation given, and no PID values. | ⚠️ Counter-intuitive |
| Table 4 signs | Tracking-error t-values are positive while the other indices are negative. The comparator is not stated. | ⚠️ Ambiguous |

## 3. Other presentation issues
- **Figs. 10–12** have y-axis tick labels that are not monotonic (e.g. "0.08, 0.11, 0.14, 0.1, 0.12"), so numbers cannot be read from them.
- **Fig. 3** has the same caption as Fig. 2 ("Frame diagram of UAV interval type II fuzzy controller") although it shows the edge-computing framework.
- **Fig. 4** labels target detection as "CNN Algorithm", while the text says a Vision Transformer is used.
- On p. 9 the text calls the state-transition and observation equations "Eq. (17)" and "(18)", but they are numbered (18) and (19).
- The introduction (p. 2) describes the particle-filter initialisation as "particle swarm optimization".
- Setting 1 PB = (19, 24, 29) goes past the 0–28 scale. The benchmark MM peak (13) is not at the midpoint of the scale (14).

## 4. Details missing for reproduction
- Δx MFs, FOU parameters, output MFs, t-norm and type-reduction algorithm.
- PID gains and the CNN/DQN/MPC configurations.
- Particle count, motion and noise models, resampling scheme.
- ViT architecture and training data; edge latency.
- UAV dynamic model, simulator or flight environment, target speeds.
- Definition of capture time and how the robustness index is computed.
- No code is released. The supplementary material was not inspected.

## 5. Consequence for this project
- For the "comparison with the reference" table we use the **relative improvement over PID**. We report both PID baselines from the paper: the abstract value (5.2 s / 0.80 m) and Table 3 default (4.8 s / 0.75 m / 0.78).
- The wind experiment (Table 2) has no PID values in the paper. Our wind experiment therefore compares against our own PID only.
