""" Support script to plot and visualize station keeping data. """

import datetime
import matplotlib.pyplot as plt
import numpy as np

from createSatellite import Satellite

from simulationParameters import (
    PLOT_3D_RIC,
    PLOT_RIC_POS,
    PLOT_RIC_POS_AMP,
    PLOT_RIC_VELO,
    PLOT_RIC_VELO_AMP,
    PLOT_COE_DIFFS,
    PLOT_PHASE_DIFF,
    PLOT_MANEUVER_MARKERS,
)

from supportFunctions import *

def output_terminal(
        elapsed_time : float,
        sat_t0 : float,
        ref_sat : Satellite,
        truth_sat : Satellite,
):
    print(f"Current time: T+{(elapsed_time / 86400)} days")

    terminal_time = sat_t0 + datetime.timedelta(seconds=elapsed_time)
    print(getEpoch_As_ModITC(terminal_time))
    print("\nReference COEs:")
    output_state = ""
    for i in ref_sat.getKeplerianState():
        output_state += " " * 4 + str(i) + ",\n"
    print(output_state)
    print(" " * 4 + f'"{getEpoch_As_Str(terminal_time)}"')
    print("\nTruth COEs:")
    output_state = ""
    for i in truth_sat.getKeplerianState():
        output_state += " " * 4 + str(i) + ",\n"
    print(output_state)
    print(" " * 4 + f'"{getEpoch_As_Str(terminal_time)}"')

def output_plots(
        timings: list[list | float],
        coes: list[dict],
        ric: list[dict]):
    """ Generate plots to visualize the scenario.

    Measuring the differences as (Truth - Reference), there are 9
    graphs available to plot. Each plot can include a point for when
    each thruster fired, if desired (PLOT_MANEUVER_MARKERS).
    - 3D trajectory in RIC frame (PLOT_3D_RIC)
    - Position in each RIC axis over time (plot_RIC_v_Time)
    - Amplitude of the oscillation in RIC position over time
      (PLOT_RIC_POS_AMP)
    - Velocity in each RIC axis over time (PLOT_RIC_VELO)
    - Amplitude of the oscillation in RIC frame velocity over time 
      (PLOT_RIC_VELO_AMP)
    - Differences in the instantaneous and averaged values for each
      orbital element over time:
        - Semi-major Axis (del_a)
        - Eccentricity (del_e)
        - Inclination (del_i)
        - Right Ascension of the Ascending Node (del_raan)
        - Argument of Periapsis (del_aop)
        - True Anomaly (del_f)
    - Differences in the instantaneous and averaged values for the True
      Latitude (del_theta)
    

    Additionally, the scenario can print to the terminal what maneuver
    was completed, how long it took to complete, and at what time did
    the maneuver conclude (terminal_Completed_Firings).

    Parameters
    ----------
    timings : list[list | float]
        A compound list containing the following time-related scenario
        information:
        - List containing the maneuver shut off times.
        - List containing the maneuver start times and the maneuver.
          type identifier (R-axis = "m", I-axis = "r", and
          C-axis = "c").
        - List of all time-stamps in the scenario.
        - Float describing the number of orbital revolutions were
          averaged to study trends.
        - Simulation step size while coasting.
    coes : list[dict]
        A compound list containing information about the differences in
        the truth and reference orbital elements:
        - Dict containing the instantaneous differences in the orbital
          elements.
        - Dict containing the averaged differences in the orbital
          elements.
    ric : list[dict]
        A compound list containing information about the differences in
        the truth and reference Cartesian states:
        - Dict containing the instananeous differences in the Cartesian
          states.
        - Dict continaind the averaged differences in the Cartesian
          State.
    """

    # Separting out the components of the inputs
    burnEnds, burnStarts, t, revs_to_avg, dtCoast, STEPS_PER_AVG_ORBIT = timings
    diffCOEs_dict, diffCOEs_avg = coes
    RIC_History, RIC_Amp_History = ric

    # Convert the time scale from seconds to days
    t = np.array(t) / 86400
    for i in diffCOEs_dict.keys():
        diffCOEs_dict[i] = {
            k / 86400: v for k, v in sorted(diffCOEs_dict[i].items())}
        diffCOEs_avg[i] = {
            k / 86400: v for k, v in sorted(diffCOEs_avg[i].items())}

    for i in RIC_History.keys():
        if i in {"R_dot", "I_dot", "C_dot"}:
            factor = 1000
        else:
            factor = 1

        RIC_History[i] = {
            k / 86400: v * factor
            for k, v in sorted(RIC_History[i].items())
        }
        RIC_Amp_History[i] = {
            k / 86400: v * factor
            for k, v in sorted(RIC_Amp_History[i].items())
        }

    # ------------------------- RIC Frame Plots -------------------------------

    # If enabled plot the 3D visualization of the trajectory.
    if PLOT_3D_RIC:
        ax_ric_traj = plt.figure().add_subplot(projection='3d')

        if len(burnStarts) > 0:
            # If there any maneuvers in the simulation, separate the maneuver
            # windows by the associated with each maneuver type.
            burnEnds.append(0)
            for i, burn_time in enumerate(burnStarts):
                # If the stored time 'k' falls between maneuvers in burnEnds
                # and the start of one in burnStarts, plot the segment in blue
                # to visualize the coasting period.
                R = {k:v for k, v in RIC_History["R"].items()
                     if (burnEnds[i-1] - dtCoast) / 86400 <= k <= burn_time[0] / 86400}
                I = {k:v for k, v in RIC_History["I"].items()
                     if (burnEnds[i-1] - dtCoast) / 86400 <= k <= burn_time[0] / 86400}
                C = {k:v for k, v in RIC_History["C"].items()
                     if (burnEnds[i-1] - dtCoast) / 86400 <= k <= burn_time[0] / 86400}

                ax_ric_traj.plot(
                    [*R.values()],
                    [*I.values()],
                    [*C.values()],
                    'b')

                # If the stored time 'k' falls during a maneuver, plot the
                # segment in the color corresponding to that maneuver type.
                R = {k:v for k, v in RIC_History["R"].items()
                     if burn_time[0] / 86400 <= k <= burnEnds[i] / 86400}
                I = {k:v for k, v in RIC_History["I"].items()
                     if burn_time[0] / 86400 <= k <= burnEnds[i] / 86400}
                C = {k:v for k, v in RIC_History["C"].items()
                     if burn_time[0] / 86400 <= k <= burnEnds[i] / 86400}
                ax_ric_traj.plot(
                    [*R.values()],
                    [*I.values()],
                    [*C.values()],
                    burn_time[1])

                # if burnStarts and burnEnds are of the same length, then the
                # scenario ended during a maneuver. This sets the final time
                # step of the maneuver to be the end time of the last maneuver.
                if i == 0 and len(burnStarts) == len(burnEnds):
                    burnEnds[-1] = t[-1]

            # If all maneuvers have a defined start and end time, as in not
            # interrupted by the end of the simulation, then plot the last
            # section of the trajectory.
            if len(burnStarts) != len(burnEnds):
                R = {k:v for k, v in RIC_History["R"].items()
                     if burnEnds[-2] <= k}
                I = {k:v for k, v in RIC_History["I"].items()
                     if burnEnds[-2] <= k}
                C = {k:v for k, v in RIC_History["C"].items()
                     if burnEnds[-2] <= k}
                ax_ric_traj.plot(
                    [*R.values()],
                    [*I.values()],
                    [*C.values()],
                    'b')
        else:
            # If there are no maneuvers in the duration of the scenario, simply
            # plot the trajectory.
            R = RIC_History["R"].items()
            I = RIC_History["I"].items()
            C = RIC_History["C"].items()
            ax_ric_traj.plot([*R.values()], [*I.values()], [*C.values()], 'b')

        # Plot title and labels
        ax_ric_traj.set_xlabel('R (km)')
        ax_ric_traj.set_ylabel('I (km)')
        ax_ric_traj.set_zlabel('C (km)')
        ax_ric_traj.set_title('3D Trajectory of Earth Orbiter')
        ax_ric_traj.axis('equal')
        ax_ric_traj.set_title("3D RIC Positions Over Time")

    # Separate out the maneuvers by type
    r_burns = []
    i_burns = []
    c_burns = []
    for i in burnStarts:
        # The telemetry is stored at intervals of 'dtCoast' but the maneuver
        # ignition time could occur outside of the reported time steps. If this
        # is the case, then grab the next available time after the maneuver has
        # started.
        burn_time = round_to_time_step(i[0]) / 86400

        # Maneuver codes:
        # "m" = R-axis burns
        # "r" = I-axis burns
        # "c" = C-axis burns
        if i[1] == "m":
            r_burns.append(burn_time)
        elif i[1] == "r":
            i_burns.append(burn_time)
        elif i[1] == "c":
            c_burns.append(burn_time)

    def plot_maneuver_markers(
            ax: plt.Axes,
            data_to_plot_on: list,
            data_to_screen: dict) -> None:
        """
        Highlight where each maneuver begins on each of the enabled
        plots.

        Parameters
        ----------
        ax : plt.Axes
            The plot to put the maneuver marker on.
        dataToPlotOn : list
            A list of keys from 'dataToScreen' to define what data the
            maneuver markers will be plotted over.
        dataToScreen : dict
            A dict containing the information to be plotted.
        """

        # If there are at least 1 R-axis maneuver, place a marker when the
        # maneuver began.
        if len(r_burns) > 0:
            ax.plot(
                r_burns,
                [float(data_to_screen[data_to_plot_on[0]][i]) for i in r_burns],
                "*",
                c="m",
                label="R-axis maneuver")

        # If there are at least 1 I-axis maneuver, place a marker when the
        # maneuver began.
        if PLOT_MANEUVER_MARKERS and len(i_burns) > 0:
            ax.plot(
                i_burns,
                [float(data_to_screen[data_to_plot_on[1]][i]) for i in i_burns],
                "*",
                c="r",
                label="I-axis maneuver")

        # If there are at least 1 R-axis maneuver, place a marker when the
        # maneuver began.
        if PLOT_MANEUVER_MARKERS and len(c_burns) > 0:
            ax.plot(
                c_burns,
                [float(data_to_screen[data_to_plot_on[2]][i]) for i in c_burns],
                "*",
                c="c",
                label="C-axis maneuver")

    # If enabled, plot the RIC positions over time.
    if PLOT_RIC_POS:
        ax = plt.figure().add_subplot()

        # Plot the position in each axis as its own line.
        ax.plot(
            RIC_History["R"].keys(),
            RIC_History["R"].values(),
            label="R")
        ax.plot(
            RIC_History["I"].keys(),
            RIC_History["I"].values(),
            label="I")
        ax.plot(
            RIC_History["C"].keys(),
            RIC_History["C"].values(),
            label="C")

        # If enabled, plot the maneuver markers
        if PLOT_MANEUVER_MARKERS:
            plot_maneuver_markers(
                ax,
                ["R", "I", "C"],
                RIC_History)

        # Plot title, labels, and legend
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (km)')
        ax.set_title("True Position in Reference RIC Frame vs Time")
        ax.legend()

    # If enabled, plot the RIC position oscillation amplitudes over time
    if PLOT_RIC_POS_AMP:
        ax = plt.figure().add_subplot()

        # Plot the amplitudes in each axis as its own line.
        ax.plot(
            RIC_Amp_History["R"].keys(),
            RIC_Amp_History["R"].values(),
            label="R")
        ax.plot(
            RIC_Amp_History["I"].keys(),
            RIC_Amp_History["I"].values(),
            label="I")
        ax.plot(
            RIC_Amp_History["C"].keys(),
            RIC_Amp_History["C"].values(),
            label="C")

        # If enabled, plot the maneuver markers
        if PLOT_MANEUVER_MARKERS:
            plot_maneuver_markers(
                ax,
                ["R", "I", "C"],
                RIC_Amp_History)

        # Plot title, labels, and legend
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (km)')
        ax.set_title("Oscillation Amplitude of Position in RIC Frame vs Time")
        ax.legend()

    # If enabled, plot the RIC frame velocity over time
    if PLOT_RIC_VELO:
        ax = plt.figure().add_subplot()

        # Plot the velocity in each axis as its own line.
        ax.plot(
            RIC_History["R_dot"].keys(),
            RIC_History["R_dot"].values(),
            label="R_dot")
        ax.plot(
            RIC_History["I_dot"].keys(),
            RIC_History["I_dot"].values(),
            label="I_dot")
        ax.plot(
            RIC_History["C_dot"].keys(),
            RIC_History["C_dot"].values(),
            label="C_dot")

        # If enabled, plot the maneuver markers
        if PLOT_MANEUVER_MARKERS:
            plot_maneuver_markers(
                ax,
                ["R_dot", "I_dot", "C_dot"],
                RIC_History)

        # Plot title, labels, and legend
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (m/sec)')
        ax.set_title("True Velocity in Reference RIC Frame vs Time")
        ax.legend()

    # If enabled, plot the RIC frame velocity oscillation amplitudes over time
    if PLOT_RIC_VELO_AMP:
        ax = plt.figure().add_subplot()

        # Plot the amplitudes in each axis as its own line.
        ax.plot(
            RIC_Amp_History["R_dot"].keys(),
            RIC_Amp_History["R_dot"].values(),
            label="R_dot")
        ax.plot(
            RIC_Amp_History["I_dot"].keys(),
            RIC_Amp_History["I_dot"].values(),
            label="I_dot")
        ax.plot(
            RIC_Amp_History["C_dot"].keys(),
            RIC_Amp_History["C_dot"].values(),
            label="C_dot")

        # If enabled, plot the maneuver markers
        if PLOT_MANEUVER_MARKERS:
            plot_maneuver_markers(
                ax,
                ["R_dot", "I_dot", "C_dot"],
                RIC_Amp_History)

        # Plot title, labels, and legend
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (m/sec)')
        ax.set_title("Oscillation Amplitude of Velocity in Reference RIC " \
        "Frame vs Time")
        ax.legend()

    # -------------------- Differences in COE Plots ---------------------------
    #
    # If enabled, plot the difference in semi-major axis between the truth and
    # reference states compared to the average value over 'revs_to_avg' orbits.
    if PLOT_COE_DIFFS["del_a"]:
        ax = plt.figure().add_subplot()
        ax.plot(
            [*diffCOEs_dict["del_a"].keys()],
            [*diffCOEs_dict["del_a"].values()],
            label="del_a")
        ax.plot(
            [*diffCOEs_avg["del_a"].keys()],
            [*diffCOEs_avg["del_a"].values()],
            "--",
            label=f"{revs_to_avg} orbit average")

        # If enabled, plot the maneuver markers
        if PLOT_MANEUVER_MARKERS:
            plot_maneuver_markers(
                ax,
                ["del_a", "del_a", "del_a"],
                diffCOEs_avg)

        # Plot title, labels, and legend
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (km)')
        ax.set_title("Truth-Reference Differences in SMA vs Time")
        ax.legend()

    # If enabled, plot the difference in eccentricity between the truth and
    # reference states compared to the average value over 'revs_to_avg' orbits.
    if PLOT_COE_DIFFS["del_e"]:
        ax = plt.figure().add_subplot()
        ax.plot(
            [i for i in diffCOEs_dict["del_e"].keys()],
            [i for i in diffCOEs_dict["del_e"].values()],
            label="del_e")
        ax.plot(
            [i for i in diffCOEs_avg["del_e"].keys()],
            [i for i in diffCOEs_avg["del_e"].values()],
            "--",
            label=f"{revs_to_avg} orbit average")

        # If enabled, plot the maneuver markers
        if PLOT_MANEUVER_MARKERS:
            plot_maneuver_markers(
                ax,
                ["del_e", "del_e", "del_e"],
                diffCOEs_avg)

        # Plot title, labels, and legend
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset')
        ax.set_title("Truth-Reference Differences in Eccentricity vs Time")
        ax.legend()

    # If enabled, plot the difference in inclination between the truth and
    # reference states compared to the average value over 'revs_to_avg' orbits.
    if PLOT_COE_DIFFS["del_i"]:
        ax = plt.figure().add_subplot()
        ax.plot(
            [i for i in diffCOEs_dict["del_i"].keys()],
            [i for i in diffCOEs_dict["del_i"].values()],
            label="del_i")
        ax.plot(
            [i for i in diffCOEs_avg["del_i"].keys()],
            [i for i in diffCOEs_avg["del_i"].values()],
            "--",
            label=f"{revs_to_avg} orbit average")

        # If enabled, plot the maneuver markers
        if PLOT_MANEUVER_MARKERS:
            plot_maneuver_markers(
                ax,
                ["del_i", "del_i", "del_i"],
                diffCOEs_avg)

        # Plot title, labels, and legend
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (deg)')
        ax.set_title("Truth-Reference Differences in Inclination vs Time")
        ax.legend()

    # If enabled, plot the difference in right ascension of the ascending node
    # between the truth and reference states compared to the average value over
    # 'revs_to_avg' orbits.
    if PLOT_COE_DIFFS["del_raan"]:
        ax = plt.figure().add_subplot()
        ax.plot(
            [i for i in diffCOEs_dict["del_raan"].keys()],
            [i for i in diffCOEs_dict["del_raan"].values()],
            label="del_raan")
        ax.plot(
            [i for i in diffCOEs_avg["del_raan"].keys()],
            [i for i in diffCOEs_avg["del_raan"].values()],
            "--",
            label=f"{revs_to_avg} orbit average")

        # If enabled, plot the maneuver markers
        if PLOT_MANEUVER_MARKERS:
            plot_maneuver_markers(
                ax,
                ["del_raan", "del_raan", "del_raan"],
                diffCOEs_avg)

        # Plot title, labels, and legend
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (deg)')
        ax.set_title("Truth-Reference Differences in Right Ascension vs Time")
        ax.legend()

    # If enabled, plot the difference in argument of periapsis between the
    # truth and reference states compared to the average value over
    # 'revs_to_avg' orbits.
    if PLOT_COE_DIFFS["del_aop"]:
        ax = plt.figure().add_subplot()
        ax.plot(
            [i for i in diffCOEs_dict["del_aop"].keys()],
            [i for i in diffCOEs_dict["del_aop"].values()],
            label="del_aop")
        ax.plot(
            [i for i in diffCOEs_avg["del_aop"].keys()],
            [i for i in diffCOEs_avg["del_aop"].values()],
            "--",
            label=f"{revs_to_avg} orbit average")

        # If enabled, plot the maneuver markers
        if PLOT_MANEUVER_MARKERS:
            plot_maneuver_markers(
                ax,
                ["del_aop", "del_aop", "del_aop"],
                diffCOEs_avg)

        # Plot title, labels, and legend
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (deg)')
        ax.set_title("Truth-Reference Differences in Argument of Perigee vs" \
        " Time")
        ax.legend()

    # If enabled, plot the difference in true anomaly between the truth and
    # reference states compared to the average value over 'revs_to_avg' orbits.
    if PLOT_COE_DIFFS["del_f"]:
        ax = plt.figure().add_subplot()
        ax.plot(
            [i for i in diffCOEs_dict["del_f"].keys()],
            [i for i in diffCOEs_dict["del_f"].values()],
            label="del_f")
        ax.plot([
            i for i in diffCOEs_avg["del_f"].keys()],
            [i for i in diffCOEs_avg["del_f"].values()],
            "--",
            label=f"{revs_to_avg} orbit average")

        # If enabled, plot the maneuver markers
        if PLOT_MANEUVER_MARKERS:
            plot_maneuver_markers(
                ax,
                ["del_f", "del_f", "del_f"],
                diffCOEs_avg)

        # Plot title, labels, and legend
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (deg)')
        ax.set_title("Truth-Reference Differences in True Anomaly vs Time")
        ax.legend()

    # If enabled, plot the difference in true latitude between the truth and
    # reference states compared to the average value over 'revs_to_avg' orbits.
    if PLOT_PHASE_DIFF:
        del_f = np.array([*diffCOEs_dict["del_f"].values()])
        del_f_avg = np.array([*diffCOEs_avg["del_f"].values()])
        del_aop = np.array([*diffCOEs_dict["del_aop"].values()])
        del_aop_avg = np.array([*diffCOEs_avg["del_aop"].values()])

        del_theta = del_f + del_aop
        del_theta_avg = del_f_avg + del_aop_avg
        for i, tau in enumerate(del_theta):
            if -180 > tau > 180:
                del_theta[i] = (360 - tau if tau > 180 else 360 + tau)
                tau_to_avg = (
                                    del_theta[i - STEPS_PER_AVG_ORBIT:i]
                                    if i >= STEPS_PER_AVG_ORBIT else del_theta[:i]
                                )
                del_theta_avg[i] = np.mean(tau_to_avg)

        ax = plt.figure().add_subplot()
        ax.plot(t, del_theta, label="del_theta")
        ax.plot(t, del_theta_avg, "--", label=f"{revs_to_avg} orbit average")

        # If enabled, plot the maneuver markers
        if PLOT_MANEUVER_MARKERS:
            del_theta_avg_dict = {t[i]: del_theta_avg[i] 
                                  for i in range(len(del_theta_avg))}
            diffCOEs_avg["del_theta"] = del_theta_avg_dict
            plot_maneuver_markers(
                ax,
                ["del_theta", "del_theta", "del_theta"],
                diffCOEs_avg)

        # Plot title, labels, and legend
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (deg)')
        ax.set_title("Truth-Reference Differences in True Latitude vs Time")
        ax.legend()

    # If any plots are enabled, show the plots.
    if any([
        PLOT_3D_RIC,
        PLOT_RIC_POS,
        PLOT_RIC_VELO,
        PLOT_COE_DIFFS.items()
        ]):

        plt.show()
