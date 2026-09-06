""" Station keeping scenario starting point (live driver).

This script drives a two-satellite (reference and truth) GMAT scenario
and applies actions from `StationKeepingController` to keep the truth
spacecraft within user-defined operational bounds of the reference in
the Radial/In-Track/Cross-Track (RIC) frame. The reference spacecraft
is only perturbed by Earth's geopotential (4x4 model), while the truth
spacecraft carries electric thrusters in the +/-R, +/-I, +/-C
directions to counter the same Earth geopotential model, atmospheric
drag, solar radiation pressure, and third body effects (Sun and Moon).
Over time, the truth spacecraft drifts away from its reference and it
must be corrected.

Main loop
---------
Each iteration:
1. Step forward both spacecraft's RK89 integrators by `dt` (`DT_COAST`
   while coasting, `DT_THRUST` while thrusting).
2. Compute the truth spacecraft's Cartesian offset from its reference
   in the RIC frame (using xyz2ric).
3. Compute the differences between each Keplerian element between both
   spacecraft (truth_element - reference_element)
4. At each `DT_COAST`-aligned time step, perform the following updates:
   - `RIC_History` / `diffCOEs`: instantaneous values
   - `RIC_Amp_History`: RIC position/velocity oscillation amplitudes
     (via a rolling `RIC_Amp_Buffer` with maxlen 1.5 orbits).
   - `diffCOEs_avg`: averaged diff_coe difference (via a rolling
     `diffCOEs_buffer` over `REVOLUTIONS_TO_AVG` orbits).
5. Feed telemetry into `StationKeepingController.update()` and apply
   the returned action (start/stop burn, back-prop, recover, etc.).

State machine
-------------
Control decisions live in `StationKeepingController`
(`leo_station_keeping_controller.py`), not in this script. Local
`state` / `interrupted_state` / `thruster_axis` mirror the controller
for integrator switching. Maneuver axis priority is I > C > R.

Notes
-----
- `elapsed_time` is a float (seconds). The coast time grid `t` is built
  from multiples of `DT_COAST`. Time is converted to days only for
  maneuver printouts / plots.
- Backwards propagation (`integrator.Step(-time)`) is a **sim reset**
  for the I-axis duration shooter; RK89 is only reversible within
  numerical tolerance. Small discontinuities at these seams are
  expected and acceptable.
- Thruster, force model, and propagator setups are delegated to
  `StationKeepingObjects`. This script owns the main loop and
  telemetry collection; control law lives in the controller.

Outputs
-------
Calls `output_plots()` in `data_outputs.py` to render RIC
position/velocity, oscillation-amplitude, and COE-difference plots
(see `simulationParameters.py` for which plots are enabled). There is
no `output_terminal` helper today; maneuver lines go through
`support_functions` print helpers when `PRINT_MANEUVER_MESSAGE` is set.
"""
