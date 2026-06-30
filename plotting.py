import matplotlib.pyplot as plt
import numpy as np

"""
Measuring the differences as (Truth - Reference), there are 9 graphs available to plot:
    - 3D trajectory in RIC frame (plot_3D_RIC)
    - Position in each RIC axis over time (plot_RIC_v_Time)
    - Differences in the instantaneous and averaged values for each orbital element over time:
        - Semi-major Axis (del_a)
        - Eccentricity (del_e)
        - Inclination (del_i)
        - Right Ascension of the Ascending Node (del_raan)
        - Argument of Periapsis (del_aop)
        - True Anomaly (del_f)
    - Differences in the instantaneous and averaged values for the True Latitude (del_theta)
Each plot can include a point for when each thruster fired, if desired (plot_Show_Firings)

Additionally, the scenario can print to the terminal what maneuver was completed, how long it took
to complete, and at what time did the maneuver conclude (terminal_Completed_Firings)
"""

plot_3D_RIC = False

plot_rRIC_v_Time = True

plot_rRIC_Amp_v_Time = True

plot_vRIC_v_Time = False

plot_vRIC_Amp_v_Time = False

plot_COE_diffs = {
    "del_a": False,
    "del_e": True,
    "del_i": False,
    "del_raan": False,
    "del_aop": False,
    "del_f": False
}

plot_True_Lat_diff = False

plot_Show_Firings = True

terminal_Completed_Firings = True

def outputPlots(timings: list, coes: list, ric: list):
    burnEnds, burnStarts, t, revs_to_avg, dtCoast = timings
    COEs_keys, diffCOEs_dict, diffCOEs_avg = coes
    RIC_keys, RIC_History, RIC_Amp_History = ric
    t = np.array(t) / 86400
    for i in COEs_keys:
        diffCOEs_dict[i] = {k / 86400: v for k, v in sorted(diffCOEs_dict[i].items())}
        diffCOEs_avg[i] = {k / 86400: v for k, v in sorted(diffCOEs_avg[i].items())}

    for i in RIC_keys:
        RIC_History[i] = {k / 86400: v for k, v in sorted(RIC_History[i].items())}
        RIC_Amp_History[i] = {k / 86400: v for k, v in sorted(RIC_Amp_History[i].items())}


    if plot_3D_RIC:
        ax_ric_traj = plt.figure().add_subplot(projection='3d')
        if len(burnStarts) > 0:
            burnEnds.append(0)
            for i in range(len(burnStarts)):
                R = {k:v for k, v in RIC_History["R"].items() if t[burnEnds[i-1]] <= k < t[burnStarts[i][0]]}
                I = {k:v for k, v in RIC_History["I"].items() if t[burnEnds[i-1]] <= k < t[burnStarts[i][0]]}
                C = {k:v for k, v in RIC_History["C"].items() if t[burnEnds[i-1]] <= k < t[burnStarts[i][0]]}
                ax_ric_traj.plot([*R.values()], [*I.values()], [*C.values()], 'b')

                R = {k:v for k, v in RIC_History["R"].items() if t[burnStarts[i][0]] <= k < t[burnEnds[i]]}
                I = {k:v for k, v in RIC_History["I"].items() if t[burnStarts[i][0]] <= k < t[burnEnds[i]]}
                C = {k:v for k, v in RIC_History["C"].items() if t[burnStarts[i][0]] <= k < t[burnEnds[i]]}
                ax_ric_traj.plot([*R.values()], [*I.values()], [*C.values()], burnStarts[i][1])
                
                if i == 0 and len(burnStarts) == len(burnEnds):
                    burnEnds[-1] = t[-1]

            if len(burnStarts) != len(burnEnds):
                R = {k:v for k, v in RIC_History["R"].items() if t[burnEnds[-2]] <= k}
                I = {k:v for k, v in RIC_History["I"].items() if t[burnEnds[-2]] <= k}
                C = {k:v for k, v in RIC_History["C"].items() if t[burnEnds[-2]] <= k}
                ax_ric_traj.plot([*R.values()], [*I.values()], [*C.values()], 'b')
        else:
            R = {k:v for k, v in RIC_History["R"].items()}
            I = {k:v for k, v in RIC_History["I"].items()}
            C = {k:v for k, v in RIC_History["C"].items()}
            ax_ric_traj.plot([*R.values()], [*I.values()], [*C.values()], 'b')

        ax_ric_traj.set_xlabel('R (km)')
        ax_ric_traj.set_ylabel('I (km)')
        ax_ric_traj.set_zlabel('C (km)')
        ax_ric_traj.set_title('3D Trajectory of Earth Orbiter')
        ax_ric_traj.axis('equal')
        ax_ric_traj.set_title("3D RIC Positions Over Time")

    # separate out the maneuvers by type
    R_burns = []
    I_burns = []
    C_burns = []
    for i in burnStarts:
        burnTime = (i[0] + i[0] % dtCoast) / 86400
        if i[1] == "m":
            R_burns.append(burnTime)
        elif i[1] == "r":
            I_burns.append(burnTime)
        elif i[1] == "c":
            C_burns.append(burnTime)

    if plot_rRIC_v_Time:
        ax = plt.figure().add_subplot()
        ax.plot([i for i in RIC_History["R"].keys()], [i for i in RIC_History["R"].values()], label="R")
        ax.plot([i for i in RIC_History["I"].keys()], [i for i in RIC_History["I"].values()], label="I")
        ax.plot([i for i in RIC_History["C"].keys()], [i for i in RIC_History["C"].values()], label="C")

        if plot_Show_Firings and len(R_burns) > 0:
            ax.plot(R_burns, [float(RIC_History["R"][i]) for i in R_burns], "*", c="m", label="R-axis maneuver")
            
        if plot_Show_Firings and len(I_burns) > 0:
            ax.plot(I_burns, [float(RIC_History["I"][i]) for i in I_burns], "*", c="r", label="I-axis maneuver")
            
        if plot_Show_Firings and len(C_burns) > 0:
            ax.plot(C_burns, [float(RIC_History["C"][i]) for i in C_burns], "*", c="c", label="C-axis maneuver")

        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (km)')
        ax.set_title("True Position in Reference RIC Frame vs Time")
        ax.legend()

    if plot_rRIC_Amp_v_Time:
        ax = plt.figure().add_subplot()
        ax.plot([i for i in RIC_Amp_History["R"].keys()], [i for i in RIC_Amp_History["R"].values()], label="R")
        ax.plot([i for i in RIC_Amp_History["I"].keys()], [i for i in RIC_Amp_History["I"].values()], label="I")
        ax.plot([i for i in RIC_Amp_History["C"].keys()], [i for i in RIC_Amp_History["C"].values()], label="C")

        if plot_Show_Firings and len(R_burns) > 0:
            ax.plot(R_burns, [float(RIC_Amp_History["R"][i]) for i in R_burns], "*", c="m", label="R-axis maneuver")
            
        if plot_Show_Firings and len(I_burns) > 0:
            ax.plot(I_burns, [float(RIC_Amp_History["I"][i]) for i in I_burns], "*", c="r", label="I-axis maneuver")
            
        if plot_Show_Firings and len(C_burns) > 0:
            ax.plot(C_burns, [float(RIC_Amp_History["C"][i]) for i in C_burns], "*", c="c", label="C-axis maneuver")

        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (km)')
        ax.set_title("Oscillation Amplitude of Position in RIC Frame vs Time")
        ax.legend()

    if plot_vRIC_v_Time:
        ax = plt.figure().add_subplot()
        ax.plot([i for i in RIC_History["R_dot"].keys()], [i for i in RIC_History["R_dot"].values()], label="R_dot")
        ax.plot([i for i in RIC_History["I_dot"].keys()], [i for i in RIC_History["I_dot"].values()], label="I_dot")
        ax.plot([i for i in RIC_History["C_dot"].keys()], [i for i in RIC_History["C_dot"].values()], label="C_dot")

        if plot_Show_Firings and len(burnStarts) > 0:
            R = {k:v for k, v in RIC_History["R_dot"].items() if k in burnStarts[:][0]}
            I = {k:v for k, v in RIC_History["I_dot"].items() if k in burnStarts[:][0]}
            C = {k:v for k, v in RIC_History["C_dot"].items() if k in burnStarts[:][0]}
            ax.plot([i for i in R.keys()], [i for i in R.values()], "*", c="m", label="R-axis maneuver")
            ax.plot([i for i in I.keys()], [i for i in I.values()], "*", c="r", label="I-axis maneuver")
            ax.plot([i for i in C.keys()], [i for i in C.values()], "*", c="c", label="C-axis maneuver")

        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (km/sec)')
        ax.set_title("True Velocity in Reference RIC Frame vs Time")
        ax.legend()

    if plot_vRIC_Amp_v_Time:
        ax = plt.figure().add_subplot()
        ax.plot([i for i in RIC_Amp_History["R_dot"].keys()], [i for i in RIC_Amp_History["R_dot"].values()], label="R_dot")
        ax.plot([i for i in RIC_Amp_History["I_dot"].keys()], [i for i in RIC_Amp_History["I_dot"].values()], label="I_dot")
        ax.plot([i for i in RIC_Amp_History["C_dot"].keys()], [i for i in RIC_Amp_History["C_dot"].values()], label="C_dot")

        if plot_Show_Firings and len(burnStarts) > 0:
            R = {k:v for k, v in RIC_Amp_History["R_dot"].items() if k in burnStarts[:][0]}
            I = {k:v for k, v in RIC_Amp_History["I_dot"].items() if k in burnStarts[:][0]}
            C = {k:v for k, v in RIC_Amp_History["C_dot"].items() if k in burnStarts[:][0]}
            ax.plot([i for i in R.keys()], [i for i in R.values()], "*", c="m", label="R-axis maneuver")
            ax.plot([i for i in I.keys()], [i for i in I.values()], "*", c="r", label="I-axis maneuver")
            ax.plot([i for i in C.keys()], [i for i in C.values()], "*", c="c", label="C-axis maneuver")

        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (km/sec)')
        ax.set_title("True Velocity in Reference RIC Frame vs Time")
        ax.legend()

    if plot_COE_diffs["del_a"]:
        ax = plt.figure().add_subplot()
        ax.plot([*diffCOEs_dict["del_a"].keys()], [*diffCOEs_dict["del_a"].values()], label="del_a")
        ax.plot([*diffCOEs_avg["del_a"].keys()], [*diffCOEs_avg["del_a"].values()], "--", label=f"{revs_to_avg} orbit average")

        if plot_Show_Firings and len(R_burns) > 0:
            ax.plot(R_burns, [diffCOEs_avg["del_a"][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
            
        if plot_Show_Firings and len(I_burns) > 0:
            ax.plot(I_burns, [diffCOEs_avg["del_a"][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
            
        if plot_Show_Firings and len(C_burns) > 0:
            ax.plot(C_burns, [diffCOEs_avg["del_a"][i] for i in C_burns], "*", c="c", label="C-axis maneuver")

        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (km)')
        ax.set_title("Truth-Reference Differences in SMA vs Time")
        ax.legend()

    if plot_COE_diffs["del_e"]:
        ax = plt.figure().add_subplot()
        ax.plot([i for i in diffCOEs_dict["del_e"].keys()], [i for i in diffCOEs_dict["del_e"].values()], label="del_e")
        ax.plot([i for i in diffCOEs_avg["del_e"].keys()], [i for i in diffCOEs_avg["del_e"].values()], "--", label=f"{revs_to_avg} orbit average")

        if plot_Show_Firings and len(R_burns) > 0:
            ax.plot(R_burns, [diffCOEs_avg["del_e"][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
            
        if plot_Show_Firings and len(I_burns) > 0:
            ax.plot(I_burns, [diffCOEs_avg["del_e"][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
            
        if plot_Show_Firings and len(C_burns) > 0:
            ax.plot(C_burns, [diffCOEs_avg["del_e"][i] for i in C_burns], "*", c="c", label="C-axis maneuver")

        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset')
        ax.set_title("Truth-Reference Differences in Eccentricity vs Time")
        ax.legend()

    if plot_COE_diffs["del_i"]:
        ax = plt.figure().add_subplot()
        ax.plot([i for i in diffCOEs_dict["del_i"].keys()], [i for i in diffCOEs_dict["del_i"].values()], label="del_i")
        ax.plot([i for i in diffCOEs_avg["del_i"].keys()], [i for i in diffCOEs_avg["del_i"].values()], "--", label=f"{revs_to_avg} orbit average")

        if plot_Show_Firings and len(R_burns) > 0:
            ax.plot(R_burns, [diffCOEs_avg["del_i"][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
            
        if plot_Show_Firings and len(I_burns) > 0:
            ax.plot(I_burns, [diffCOEs_avg["del_i"][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
            
        if plot_Show_Firings and len(C_burns) > 0:
            ax.plot(C_burns, [diffCOEs_avg["del_i"][i] for i in C_burns], "*", c="c", label="C-axis maneuver")
        
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (deg)')
        ax.set_title("Truth-Reference Differences in Inclination vs Time")
        ax.legend()

    if plot_COE_diffs["del_raan"]:
        ax = plt.figure().add_subplot()
        ax.plot([i for i in diffCOEs_dict["del_raan"].keys()], [i for i in diffCOEs_dict["del_raan"].values()], label="del_raan")
        ax.plot([i for i in diffCOEs_avg["del_raan"].keys()], [i for i in diffCOEs_avg["del_raan"].values()], "--", label=f"{revs_to_avg} orbit average")

        if plot_Show_Firings and len(R_burns) > 0:
            ax.plot(R_burns, [diffCOEs_avg["del_raan"][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
            
        if plot_Show_Firings and len(I_burns) > 0:
            ax.plot(I_burns, [diffCOEs_avg["del_raan"][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
            
        if plot_Show_Firings and len(C_burns) > 0:
            ax.plot(C_burns, [diffCOEs_avg["del_raan"][i] for i in C_burns], "*", c="c", label="C-axis maneuver")

        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (deg)')
        ax.set_title("Truth-Reference Differences in Right Ascension vs Time")
        ax.legend()

    if plot_COE_diffs["del_aop"]:
        ax = plt.figure().add_subplot()
        ax.plot([i for i in diffCOEs_dict["del_aop"].keys()], [i for i in diffCOEs_dict["del_aop"].values()], label="del_aop")
        ax.plot([i for i in diffCOEs_avg["del_aop"].keys()], [i for i in diffCOEs_avg["del_aop"].values()], "--", label=f"{revs_to_avg} orbit average")

        if plot_Show_Firings and len(R_burns) > 0:
            ax.plot(R_burns, [diffCOEs_avg["del_aop"][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
            
        if plot_Show_Firings and len(I_burns) > 0:
            ax.plot(I_burns, [diffCOEs_avg["del_aop"][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
            
        if plot_Show_Firings and len(C_burns) > 0:
            ax.plot(C_burns, [diffCOEs_avg["del_aop"][i] for i in C_burns], "*", c="c", label="C-axis maneuver")

        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (deg)')
        ax.set_title("Truth-Reference Differences in Argument of Perigee vs Time")
        ax.legend()

    if plot_COE_diffs["del_f"]:
        ax = plt.figure().add_subplot()
        ax.plot([i for i in diffCOEs_dict["del_f"].keys()], [i for i in diffCOEs_dict["del_f"].values()], label="del_f")
        ax.plot([i for i in diffCOEs_avg["del_f"].keys()], [i for i in diffCOEs_avg["del_f"].values()], "--", label=f"{revs_to_avg} orbit average")

        if plot_Show_Firings and len(R_burns) > 0:
            ax.plot(R_burns, [diffCOEs_avg["del_f"][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
            
        if plot_Show_Firings and len(I_burns) > 0:
            ax.plot(I_burns, [diffCOEs_avg["del_f"][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
            
        if plot_Show_Firings and len(C_burns) > 0:
            ax.plot(C_burns, [diffCOEs_avg["del_f"][i] for i in C_burns], "*", c="c", label="C-axis maneuver")
        
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (deg)')
        ax.set_title("Truth-Reference Differences in True Anomaly vs Time")
        ax.legend()

    if plot_True_Lat_diff:
        del_f = np.array([*diffCOEs_dict["del_f"].values()])
        del_f_avg = np.array([*diffCOEs_avg["del_f"].values()])
        del_aop = np.array([*diffCOEs_dict["del_aop"].values()])
        del_aop_avg = np.array([*diffCOEs_avg["del_aop"].values()])

        del_theta = del_f + del_aop
        del_theta_avg = del_f_avg + del_aop_avg

        ax = plt.figure().add_subplot()
        ax.plot(t, del_theta, label="del_theta")
        ax.plot(t, del_theta_avg, "--", label=f"{revs_to_avg} orbit average")

        if plot_Show_Firings and len(R_burns) > 0:
            ax.plot(R_burns, [del_theta_avg[i] for i in R_burns], "*", c="m", label="R-axis maneuver")
            
        if plot_Show_Firings and len(I_burns) > 0:
            ax.plot(I_burns, [del_theta_avg[i] for i in I_burns], "*", c="r", label="I-axis maneuver")
            
        if plot_Show_Firings and len(C_burns) > 0:
            ax.plot(C_burns, [del_theta_avg[i] for i in C_burns], "*", c="c", label="C-axis maneuver")
            
        ax.set_xlabel('Time (Days)')
        ax.set_ylabel('Offset (deg)')
        ax.set_title("Truth-Reference Differences in True Latitude vs Time")
        ax.legend()

    if any([
        plot_3D_RIC,
        plot_rRIC_v_Time,
        plot_vRIC_v_Time,
        plot_COE_diffs.items()
        ]):
        
        plt.show()