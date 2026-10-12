# STAR — Stochastic Traffic-Adaptive Replanning

**Adaptive path planning and collision avoidance for autonomous vehicles on unstructured roads.**

STAR is a path-planning framework for driving on lane-less, mixed-traffic roads — the kind where
auto-rickshaws cut in, pushcarts drift across the carriageway, stray animals wander onto it, and
lane geometry may not exist at all. Instead of assuming structured lanes and cooperative actors,
it treats surrounding traffic as a *stochastic, occasionally chaotic* system: it measures how
unpredictable the traffic currently is, predicts erratic lateral motion with heavy-tailed risk
fields, and keeps a set of pre-computed escape manoeuvres ready so it can react in milliseconds
when something unexpected happens.

---

## The core idea

Conventional planners are lane-centric and lean on rigid Time-to-Collision (TTC) thresholds. On
unstructured roads those assumptions break down. STAR replaces fixed assumptions with three
mechanisms:

| Mechanism | What it does |
| --- | --- |
| **TEI** — Traffic Entropy Index | Quantifies how *chaotic* the current traffic is, in real time |
| **HTRF** — Heavy-Tailed Risk Field | Predicts erratic swerves with a bivariate Cauchy distribution instead of a Gaussian |
| **Contingency Trajectory Portfolio** | Pre-computed escape manoeuvres that can be hot-swapped in **< 20 ms** |

These feed a behaviour supervisor that scales the planning horizon and switches driving state
*before* a situation becomes critical.

---

## How it works

### 1. Perception — Occupancy Grid Map (OGM)

A local grid is built around the ego vehicle from obstacle detections (`build_ogm`). Cells are
marked occupied by obstacle footprints and by road edges, then **inflated by the ego radius** so
that a single point check accounts for the ego's footprint. The OGM is then *augmented* with the
HTRF risk overlay below — so it maps not only static unpaved boundaries but also assigns
probabilistic risk to dynamic obstacles.

### 2. TEI — Traffic Entropy Index

TEI converts "how messy is the traffic right now?" into a single scalar, using all obstacles
within a 50 m radius of the ego:

```
Step 1 — Heading entropy (directional chaos)
         H = -Σ p_i · log(p_i)          over 8 heading bins spanning [-π, π]

Step 2 — Speed variance (sudden acceleration events)
         σ² = var(speed of nearby obstacles)

Step 3 — TEI = 0.7 · H + 0.3 · σ²
```

A high TEI means nearby actors are pointing in many different directions and moving at
inconsistent speeds — the signature of a crowded market or a chaotic junction. A low TEI means
traffic is structured and predictable, like a highway.

### 3. HTRF — Heavy-Tailed Risk Field

A Gaussian (bivariate normal) risk model *underestimates* sudden lateral movements, because its
tails decay too quickly. STAR instead places a **bivariate Cauchy lobe** on every obstacle:

```
f(p) = 1 / (2π·√|Σ|) · (1 + (p − μ)ᵀ Σ⁻¹ (p − μ))^(−3/2)
```

where `p = [x, y]` is a spatial coordinate, `μ` is the obstacle's predicted position, and `Σ` is
the uncertainty covariance matrix. `Σ` is **aligned with the obstacle's heading**:

- spread *along* the heading grows with speed → `σ_along = σ₀ + k·speed`
- spread *across* the heading is scaled per obstacle class, because a cow or a pushcart is far
  less predictable than a car:

| Actor class | Cross-heading multiplier |
| --- | --- |
| `cow` (erratic) | 2.5× |
| `pedestrian` | 1.5× |
| vehicles / static | 1.0× |

The heavy tails mean a sudden swerve is never treated as "impossible", only as "unlikely" — which
produces realistically large safety buffers exactly where unstructured traffic needs them.

### 4. Adaptive planning horizon

The TEI drives the planning horizon, so the planner thinks far ahead when the road is calm and
shortens its horizon when it must react quickly:

```
f = clamp( (TEI − 0.2) / (1.0 − 0.2), 0, 1 )
H = 5.0 − f · (5.0 − 2.0)          % 5 s in structured traffic → 2 s in chaos
```

### 5. Behaviour supervisor

A state machine with **30% hysteresis** on its thresholds (to stop state chattering) selects the
driving state:

```
EMERGENCY  ← TTC < 1.5 s            (hard brake / evasive manoeuvre)
YIELD      ← TTC < 4.0 s            (slow, give way)
CAUTIOUS   ← TEI > 0.7              (pre-emptive caution, before anything is critical)
NOMINAL    ← otherwise              (proceed at nominal speed)
```

The key property: because **TEI triggers CAUTIOUS early**, the system slows down *before* the
situation becomes dangerous, rather than waiting for a TTC threshold to be crossed.

### 6. Planning and the hot-swap portfolio

In `NOMINAL` / `CAUTIOUS` / `YIELD`, a Frenet-style lateral search evaluates candidate offsets
over the TEI-driven horizon and picks the lowest-cost path:

```
cost = w_risk · HTRF_risk(path)
     + 1000 · collision(path)          % hard constraint
     + 0.5 · |d|                       % stay near the road centre
     + 1.0 · |d − d_ego|               % penalise abrupt lateral motion
```

In `EMERGENCY`, the planner does **no search at all**. A background task continuously prepares a
**contingency portfolio** of escape manoeuvres — *brake in lane*, *swerve left*, *swerve right* —
and the emergency path simply picks the first collision-free one. That trades optimality for
latency, giving an instantaneous hot-swap instead of a full replan.

### 7. Asynchronous decoupling

The expensive work (TEI + HTRF + portfolio) is **decoupled from the reactive loop**: the
portfolio is rebuilt only every 10 ticks, so heavy computation never blocks the planner. This is
what keeps the main planning loop responsive even during high-frequency chaotic events such as
several auto-rickshaws swerving at once.

---

## Repository contents

| File | Description |
| --- | --- |
| `star_basic.m` | Self-contained reference implementation of the STAR core loop |

`star_basic.m` is a **single-file, base-MATLAB** prototype (no toolboxes required) that runs the
complete closed loop: occupancy grid mapping, TEI, HTRF, the adaptive horizon, the behaviour
state machine, the contingency portfolio and hot-swap, plus live visualisation and latency
metrics. The road is modelled as a straight corridor along `+x`, so Frenet longitudinal distance
`s = x` and lateral offset `d = y`.

> This file is the runnable algorithmic demonstration of the architecture. A production system
> would swap the base-MATLAB modules for a Stateflow supervisor, an MPC-based Frenet generator and
> a full vehicle-dynamics plant — the interfaces stay the same.

---

## Running it

Requires **MATLAB R2019b or newer** — base MATLAB only, no toolboxes or Simulink needed.

```matlab
>> star_basic
```

Two figures are produced:

1. **Live animation** — the HTRF risk heat map, occupied-grid contour, obstacle footprints, the
   planned path and the ego vehicle, with the current state, speed, TEI and horizon in the title.
2. **Time-series plots** — ego speed, and TEI vs. adaptive horizon over time.

The console reports the run summary, including a safety check and planner latency:

```
Finished: x = 90.0 m after ... s
Minimum clearance to any obstacle: ... m (no collision)
Normal planner latency : ... ms mean (... calls)
Hot-swap latency       : ... ms mean (... calls)
```

### Scenario

Four non-standard actors are simulated on the same road — a **cow** drifting laterally, a
**pedestrian** crossing, an oncoming **rickshaw**, and a stationary **pushcart** — chosen to
exercise the heavy-tailed risk model and the emergency hot-swap path.

### Tunable parameters

All parameters live in the `P` struct at the top of `star_basic.m`: road geometry and grid
resolution, speed set-points per state, TTC thresholds, the TEI caution threshold and horizon
limits, candidate lateral offsets, HTRF spread and class multipliers, and the portfolio rebuild
period.

---

## Design rationale

- **Chaos-aware, not lane-centric.** Unstructured roads are not a degraded version of structured
  ones; STAR models their unpredictability directly instead of assuming it away.
- **Probabilistic, not thresholded.** Heavy-tailed risk replaces brittle TTC-only logic, so sudden
  swerves degrade gracefully instead of being ignored until too late.
- **Pre-emptive, not reactive.** TEI raises caution while there is still time and space to act.
- **Latency-bounded by construction.** Pre-computed manoeuvres and an asynchronously rebuilt
  portfolio keep the worst-case reaction inside a hard time budget.

---

## Roadmap

- [x] Reference implementation of TEI, HTRF, adaptive horizon and hot-swap portfolio
- [ ] Port the supervisor to a Stateflow chart and the planner to an MPC formulation
- [ ] Replace the kinematic ego model with a full bicycle model
- [ ] Validate against real unstructured-driving data (e.g. the Indian Driving Dataset)
- [ ] Add perception noise and false-positive detections to stress-test the OGM

---

## References

- [Indian Driving Dataset (IDD)](https://idd.insaan.iiit.ac.in/)
- [Mendeley — unstructured road datasets](http://data.mendeley.com/datasets/jr3yv3wfhx/1)
- [Adaptive path planning literature (IJAI / IAES)](https://ijai.iaescore.com/index.php/IJAI/article/viewFile/20730/13215)
- [SSRN — collision avoidance for mixed traffic](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=6171186)
- [IEEE — path planning under uncertainty](https://ieeexplore.ieee.org/document/5451248)
