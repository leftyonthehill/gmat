# Closed-Loop Station Keeping State Machine

This repo studies **closed-loop station keeping** for satellites in proliferated
Low Earth Orbit (PLEO) constellations. Dynamics come from NASA's
**General Mission Analysis Tool (GMAT)** API.

The simulation uses tight operational bounds typical of PLEO. The live driver
is `station_keeping_maneuver_subs.py`. Control decisions (the RIC state machine)
live in `leo_station_keeping_controller.py`. Scenario knobs (duration, step
sizes, RIC bounds, duty times, plot flags) are in `simulationParameters.py`.

`testThrusting.py` is a **legacy** monolith from before the controller was
extracted. It still carries older control assumptions (1-rev I-score, 2× C-window,
incomplete renames). Do not run it as the live driver.

## Architecture
- `simulationParameters.py` — duration, step sizes, RIC bounds, plot/print flags
- `createSatellite.py`, `createForceModel.py`, `createPropagator.py`, `createStationKeepingObjects.py` — GMAT object wrappers
- `station_keeping_maneuver_subs.py` — main loop: propagate, collect telemetry, apply controller actions
- `leo_station_keeping_controller.py` — detect bound violations, wait for a burn window, thrust, recover (priority I > C > R)
- `data_outputs.py` — Matplotlib plots (not `plotting.py`)
- `support_functions.py` — time grid, ECI→RIC, epoch helpers, maneuver printouts
- `load_gmat.py` — GMAT API bootstrap; `GmatInstall` is hardcoded to `C:/gmat-win-R2026a` (no desktop/laptop path split)

## Control notes (docs only)
- **I-axis** maneuvers use a reverse-RK89 targeting shooter (coast ≥ 4 periods with mean Δa < 0, then score on min I). This is a simulation targeting aid, not a flyable ops sequence.
- Default spacecraft power is **20 kW nuclear** (`Satellite.setPowerSystem`) so GMAT does not skip burns during eclipse (no battery model).
- Live **C-window** half-width is `4 * MANEUVER_ARC_HALF_ANGLE` (legacy `testThrusting.py` still uses 2×).
- Live **R-window** waits for true anomaly approaching 90°/270° within the half-angle, with `|Δω| ≤ 3°`.

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
