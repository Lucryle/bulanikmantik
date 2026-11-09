# Modelling Assumptions

The reference paper (Li et al., *Sci. Rep.* 14, 18506, 2024) leaves many
implementation details unspecified (see `docs/PAPER_VERIFICATION.md`, §4).
Every parameter below that is **not** taken from the paper is an assumption
made for this reproduction. Each assumption has an ID that is referenced in
`configs/*.yaml`, and all of them can be changed from the configs.

Legend: **Paper** = value stated in the paper · **Assumed** = our choice.

## Simulation

| ID | Item | Value | Source | Justification |
|---|---|---|---|---|
| A1 | Time step / trial length | dt = 0.05 s (20 Hz), 40 s | Assumed | 20 Hz is a typical vision-based control rate; 40 s is about 10× the paper's capture times (≈4 s), so the post-capture tracking phase is long enough to measure. |
| A2 | UAV model | Planar kinematics at constant altitude h = 10 m, with first-order lags τ_v = 0.3 s and τ_r = 0.2 s | Assumed | The paper gives no dynamic model. The controllers command only yaw and forward speed, so a planar unicycle model with actuator lag is the simplest model consistent with those outputs. |
| A3 | Meaning of the "yaw" output | Interpreted as **yaw rate** r ∈ [−π/3, π/3] rad/s | Paper gives the range (Eq. 3: k_yaw = yaw / (π/3)); the interpretation is assumed | A yaw-angle command from a pixel error alone would need an absolute heading reference. Commanding yaw rate makes the pixel-error loop a standard visual-servo loop. |
| A3b | Speed output | v ∈ [−2, 2] m/s | Paper (Eq. 4: k_v = v / 2) | — |

## Sensing

| ID | Item | Value | Source | Justification |
|---|---|---|---|---|
| A4 | Camera | Pinhole camera, 1920×1080, horizontal FOV 84°, pointing straight down | Resolution from the paper (Table 1); FOV assumed | 84° is the DJI Phantom 4 Pro camera's FOV (the paper's UAV, Table 1). A straight-down camera makes the pixel ↔ ground mapping linear and easy to explain. |
| A5 | Target detector | The Vision Transformer is replaced by a simulated detector: true pixel position + N(0, σ_px²), σ_px = 5 px, miss probability p_miss, false-positive probability p_fp | Assumed | The paper gives no ViT architecture, training data or accuracy. The fuzzy-logic contribution sits in the control layer, and the course focuses on fuzzy logic. |
| A6 | Edge-computing latency | Fixed perception delay of 0.1 s | Assumed | The paper reports 3 edge devices (Table 1) but no latency figures. |
| A7 | Particle filter | State [x, y, vx, vy]; constant-velocity motion model (acceleration noise 1 m/s²); Gaussian likelihood on the ground plane with σ = 0.2 m plus a 5 % uniform outlier term; N = 500; systematic resampling when ESS < N/2 followed by a small jitter (regularised PF); 5 % of the particles re-drawn around each new measurement; uniform initialisation over a 40 m × 40 m area; delayed detections compared with each particle's position at capture time (x − v·latency) | The paper gives the steps (Eqs. 18–20): uniform init, motion update, weighting, normalisation, resampling, weighted mean. Everything else is assumed | Standard choices (Arulampalam *et al.*, 2002). σ = 0.2 m is larger than the pure pixel noise (5 px ≈ 0.05 m) because it also covers latency and pose errors. Without the jitter and the re-drawn particles the filter often locked onto a wrong velocity and never recovered (checked in development). |

## Environment

| ID | Item | Value | Source | Justification |
|---|---|---|---|---|
| A8 | Wind | Disturbance d(t) = α·W·ŵ + OU gust; α = 0.04, gust σ = 0.02·W, τ = 2 s; W ∈ {4.9, 9.6, 12.1} m/s | W values from the paper (Table 2); the model is assumed | With v_max = 2 m/s, applying 12.1 m/s wind directly would blow the UAV away. A real UAV's inner attitude controller rejects most of the wind, so only a residual fraction α drifts the vehicle. **This is a key limitation.** |
| A12 | Scenario definitions | See `configs/scenarios/`. The target starts at a random point inside the first camera image, at least 2 m from the UAV | The paper only names the scenarios (A fast target, B obstacles and lighting changes, C motion-mode change); the numbers are assumed | Scenario A keeps the target speed ≤ v_max, otherwise tracking is physically impossible. |

## Fuzzy controller

| ID | Item | Value | Source | Justification |
|---|---|---|---|---|
| A11 | MF scale | The paper's Δy MFs use a 0–28 scale; they are normalised with k = (u − 14) / 14 to [−1, 1] | Assumed | Eq. 2 (k_y = Δy / y_c) implies a normalised input in [−1, 1]. The paper does not say how the 0–28 scale relates to it. 14 is the midpoint of the scale (note that the benchmark MM peak is at 13, not 14). |
| A13 | Δx membership functions | Same as the Δy MFs | Assumed | The paper gives MFs only for Δy. |
| A14 | FOU (type-2 uncertainty) | UMF = (a−δ, b, c+δ), LMF = (a+δ, b, c−δ) with height h_L = 0.8; δ is swept in the sensitivity study | Assumed | The paper gives no FOU parameters. |
| A15 | Inference / type reduction | min t-norm (equivalent to the firing level for a single input); Karnik–Mendel centre-of-sets type reduction; y = (y_l + y_r)/2 | The paper says only "central set defuzzification" (p. 5); the rest is assumed | KM centre-of-sets is the standard IT2 type reducer (Karnik & Mendel, 2001). |
| A16 | Output term centres | LB [−1, −0.9], LS [−0.55, −0.45], ZO [−0.05, 0.05], RS [0.45, 0.55], RB [0.9, 1] (normalised) | Assumed | The paper gives no output MFs. |
| A19 | Outer terms | NB and PB are shoulder MFs: membership stays at its maximum for every input beyond the peak | Assumed | With plain triangles, NB (0, 5, 10) maps to (−1, −0.64, −0.29), so an error of exactly −1 would fire no rule at all. Shoulders are the usual fix. |
| A20 | Offset at zero error (finding, not a choice) | Benchmark: y(0) ≈ +0.10, Setting 2: y(0) ≈ +0.20, Setting 1: y(0) = 0 | Follows from the paper's MFs + A11 | The MM peak is at 13 (benchmark) and 12 (setting 2) instead of the middle of the scale (14), so the controller is not symmetric. In closed loop this leaves a steady pixel offset. Setting 1 (MM peak at 14) is the only symmetric set. See `results/figures/control_curves.png`. |
| A17 | Sign convention | Δx > 0 → target to the right → yaw right. Δy < 0 → move forward (paper p. 5: "when Δy is negative, the UAV will move forward") | Paper (rule semantics) | Defined in one place in the code and checked by a unit test. |

## Evaluation

| ID | Item | Value | Source | Justification |
|---|---|---|---|---|
| A9 | Capture time | First time the ground-plane error stays below 0.5 m for at least 1 s | Assumed | The paper reports "average target capture time" without defining it. |
| A9b | Robustness index | Standard deviation of the post-capture tracking error (lower is better) | The paper's wording ("robustness index (standard deviation)", p. 13) | The paper's own numbers contradict this definition (see PAPER_VERIFICATION §3). |
| A10 | Seeds / statistics | 10 tuning seeds (1000–1009) kept separate from 30 test seeds (0–29); paired tests | Assumed | The paper uses 10 trials per scene. 30 paired seeds give tighter confidence intervals, and the separate tuning set keeps the comparison fair. |
| A18 | PID baseline | Same inputs and outputs, saturation and anti-windup; gains tuned with the same search budget as the FLC | Assumed | The paper gives no PID gains. |
