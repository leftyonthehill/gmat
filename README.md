# PLEO Station Keeping via GMAT API

This repo studies **station keeping** for satellites in proliferated Low Earth
Orbit (PLEO) constellations. Dynamics come from NASA's **General Mission
Analysis Tool (GMAT)** API.

Live path: `station_keeping_maneuver_subs.py` (main loop) +
`leo_station_keeping_controller.py` (RIC state machine). Scenario knobs
(duration, step sizes, RIC bounds ±10 / ±40 / ±15 km, duty times, plot flags)
are in `simulationParameters.py`. Maneuver priority is **I > C > R**.

R- and C-axis corrections are **deadband** responses (wait for a geometric
window, burn while the window is open, recover). The **I-axis** path is a
**1D finite-burn duration shooter**: fire I+, coast, score on min I, then
reverse-step RK89 to retry a longer/shorter burn. That reverse-step loop is a
**sim reset / targeting aid**, not a flyable closed-loop ops sequence.

`testThrusting.py` is **sunset / legacy** (pre-controller monolith with older
1-rev I-score and 2× C-window assumptions). Do not run it as the live driver.

## Architecture
- `simulationParameters.py` — duration, step sizes, RIC bounds, plot/print flags
- `createSatellite.py`, `createForceModel.py`, `createPropagator.py`, `createStationKeepingObjects.py` — GMAT object wrappers
- `station_keeping_maneuver_subs.py` — main loop: propagate, collect telemetry, apply controller actions
- `leo_station_keeping_controller.py` — detect bound violations, wait for a burn window, thrust, recover (priority I > C > R)
- `data_outputs.py` — Matplotlib plots (not `plotting.py`)
- `support_functions.py` — time grid, ECI→RIC, epoch helpers, maneuver printouts
- `load_gmat.py` — GMAT API bootstrap; `GmatInstall` is hardcoded to `C:/gmat-win-R2026a` (no desktop/laptop path split)

## Control notes (docs only)
- **I-axis**: duration targeting with reverse-RK89 retries (coast ≥ 4 periods and mean Δa < 0, then score on min I). Free variable = burn duration. Not flyable closed-loop.
- **R-axis**: deadband; live window is TA within `MANEUVER_ARC_HALF_ANGLE` of 90°/270° with instantaneous `|Δω| ≤ 3°`. Radial near 90/270 is e-control (Gauss cos f ≈ 0), not ω / line-of-apsides torque.
- **C-axis**: deadband; live half-width is `4 * MANEUVER_ARC_HALF_ANGLE` (= 160° when half-angle = 20). Critical angle uses `arctan(ΔΩ / (Δi×10) * sin i)`, not `atan2`.
- Default spacecraft power is **20 kW nuclear** (`Satellite.setPowerSystem`) so GMAT does not skip burns during eclipse (no battery model).

## ECI → RIC
`xyz2ric` returns relative **position** in the RIC frame built from the
reference state. The “velocity” components are the **inertial Δv rotated**
into that frame (`C @ (v_truth − v_ref)`); they omit the transport term
`−ω × δr` and are **not** body-frame RIC rates.

## Residual gotchas for readers
1. Live entry path is `station_keeping_maneuver_subs.py`, not `testThrusting.py`.
2. I-axis is a duration shooter with reverse-step sim resets — not ops closed-loop.
3. Plant asymmetry: reference is drag-free 4×4 geopotential only; truth has drag, SRP, and 3rd-body.
4. C-window is ±(4× half-angle) (160° at default 20°) plus the arctan×10 heuristic.

## Requirements
- **Python** 3.10+
- **NASA GMAT R2026a**
- numpy, matplotlib

## Setup
1. **Install GMAT** from NASA's SourceForge page:
   https://sourceforge.net/projects/gmat/

2. **Create the API connection.** Navigate to `.../GMAT Install/application/api` and run:
   ```bash
   python BuildApiStartupFile.py
   ```

3. **Clone this repo:**
   ```bash
   git clone https://github.com/leftyonthehill/gmat.git
   ```

4. **Point the API at your GMAT install.** In `load_gmat.py`, set `GmatInstall`
   to your GMAT directory (committed default: `C:/gmat-win-R2026a`).

5. **Install Python libraries:**
   ```bash
   pip install numpy matplotlib
   ```

6. **Run the live driver:**
   ```bash
   python station_keeping_maneuver_subs.py
   ```

## Contributing
This is a personal research project. Issues and pull requests are welcome if
they improve the station-keeping logic or modularity.
