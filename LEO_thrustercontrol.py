def waitForR():
    return None
def waitForI():
    return None

def waitForC():
    return None

def thrustR():
    return None

def thrustI():
    if thrusterAxis != "":
        burn_duration += dt

        if (burn_duration >= minDutyTime and maneuverAttempts == 0) or maneuverAttempts > 0:
            burnEnds.append(elapsed)
            del_a_energy_maneuver = del_a_energy

            gator_truth = truthObjs.satEnginesOff(thrusterAxis)
            thrusterAxis = ""

            dt = dtCoast - round(elapsed % dtCoast)
            maneuverAttempts += 1
            minIPosition = rvRIC[1]
    else:
        minIPosition = rvRIC[1] if rvRIC[1] < minIPosition else minIPosition
        coast_duration += dt
        dt = dtCoast
        
        if coast_duration > period_sec:
            # Termination conditions:
            # - achieves deadband target by the time SMA changes sign (no change)
            # - undershoots deadband target when SMA changes sign (more thrusting)
            # - overshoots deadband target (less thrusting)
            
            # achieves deadband target by the time SMA changes sign (no change)
            if diffCOEs_avg["del_a"][elapsed] < 0 and -I_deadband_min * I_bounds > minIPosition >= -I_bounds:
                if terminal_Completed_Firings:
                    terminalStr = f"t = {(burnStarts[-1][0] / 86400):2.2f} days | " if (burnStarts[-1][0] / 86400) >= 10 else f"t = {(burnStarts[-1][0] / 86400):1.3f} days | "
                    terminalStr += f"I+ burn duration (min) = "
                    terminalStr += f"{(burn_duration / 60):2.2f} | " if (burn_duration / 60) >= 10 else f"{(burn_duration / 60):1.3f} | "
                    terminalStr += f"Recovered del_a = {(del_a_energy_maneuver):0.5f} / {(del_a_target):0.5f} km | "
                    
                    accel = 0.2 / truth_Sat_wrapper.mass # m/s
                    deltaV = accel * burn_duration
                    totalDeltaV += deltaV
                    terminalStr += f"deltaV = {deltaV:1.3f} m/s | " 
                    terminalStr += f"total deltaV = {totalDeltaV:1.3f} m/s"
                    print(terminalStr)
                
                state = interpruptedState
                interpruptedState = "nominal"
                
                """for j in COEs_keys:
                    COEs_restore_history = {k:v for k, v in diffCOEs_dict[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]]}
                    diffCOEs_buffer[j] = deque([*COEs_restore_history.values()], maxlen=int(steps_to_avg))

                for j in RIC_keys:
                    RIC_restore_history = {k:v for k, v in RIC_History[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]] and k % dtCoast == 0}
                    RIC_Amp_Buffer[j] = deque([*RIC_restore_history.values()], maxlen=int(1.5 * steps_per_rev))

                t = t[:burnEnds[-1] + 1]"""
                
                restoreFromTime = burnEnds[-1] - burnEnds[-1] % dtCoast 
                tStep = t.index(restoreFromTime)
                tHistoryCOEs = t[(tStep - steps_to_avg):tStep]
                for j in COEs_keys:
                    [diffCOEs_buffer[j].append(diffCOEs_dict[j][k]) for k in tHistoryCOEs]

                tHistoryRIC = t[int(tStep - 1.5 * steps_per_rev):tStep]
                for j in RIC_keys:
                    [RIC_Amp_Buffer[j].append(RIC_Amp_History[j][k]) for k in tHistoryRIC]

                gator_ref.Step(-coast_duration)
                gator_truth.Step(-coast_duration)
                elapsed = elapsed - coast_duration
                
                burn_duration = 0
                coast_duration = 0
                maneuverLog = []
                
                # Update numerical integrator references
                gator_ref.UpdateSpaceObject()
                gator_truth.UpdateSpaceObject()

            # undershoots deadband target when SMA changes sign (more thrusting)    
            elif diffCOEs_avg["del_a"][elapsed] < 0 and (diffCOEs_avg["del_a"][elapsed] - diffCOEs_avg["del_a"][elapsed - 10 * steps_per_rev * dtCoast]) < 0 and -I_deadband_min * I_bounds <= minIPosition:
                # print(f"Maneuver #{maneuverAttempts}: I-position = {minIPosition:1.4}km | burn time = {burn_duration} sec")
                """
                for j in COEs_keys:
                    COEs_restore_history = {k:v for k, v in diffCOEs_dict[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]]}
                    diffCOEs_buffer[j] = deque([*COEs_restore_history.values()], maxlen=int(steps_to_avg))

                for j in RIC_keys:
                    RIC_restore_history = {k:v for k, v in RIC_History[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]] and k % dtCoast == 0}
                    RIC_Amp_Buffer[j] = deque([*RIC_restore_history.values()], maxlen=int(1.5 * steps_per_rev))
                """

                restoreFromTime = burnEnds[-1] - burnEnds[-1] % dtCoast 
                tStep = t.index(restoreFromTime)
                tHistoryCOEs = t[(tStep - steps_to_avg):tStep]
                for j in COEs_keys:
                    [diffCOEs_buffer[j].append(diffCOEs_dict[j][k]) for k in tHistoryCOEs]

                tHistoryRIC = t[int(tStep - 1.5 * steps_per_rev):tStep]
                for j in RIC_keys:
                    [RIC_Amp_Buffer[j].append(RIC_Amp_History[j][k]) for k in tHistoryRIC]

                # Propagate spacecraft
                gator_ref.Step(-coast_duration)
                gator_truth.Step(-coast_duration)
                elapsed = elapsed - coast_duration
                coast_duration = 0

                # Update numerical integrator references
                gator_ref.UpdateSpaceObject()
                gator_truth.UpdateSpaceObject()
                
                burnEnds.pop()
                # Update the the elpased time

                thrusterAxis = "I+"
                estimateSteps = np.ceil((minIPosition + I_deadband_min * I_bounds) / 0.8) if (0 >= minIPosition) else 1
                dt = dtThrust * estimateSteps
                if burn_duration + dt in maneuverLog: 
                    estimateSteps -=1
                    dt = dtThrust * estimateSteps
                maneuverLog.append(burn_duration)
                gator_truth = truthObjs.satEnginesOn(thrusterAxis)
                
                if burn_duration + dt < 0:
                    print(f"Negative thrust time!")
                    burnStarts.pop()
                    break
                if maneuverAttempts == 100:
                    print(f"Max burns!  current burn duration = {burn_duration} sec")
                    burnStarts.pop()
                    break
                
            # overshoots deadband target (less thrusting)
            elif diffCOEs_avg["del_a"][elapsed] < 0 and (diffCOEs_avg["del_a"][elapsed] - diffCOEs_avg["del_a"][elapsed - 10 * steps_per_rev * dtCoast]) < 0 and minIPosition < -I_bounds:
                # print(f"Maneuver #{maneuverAttempts}: I-position = {minIPosition:1.4}km | burn time = {burn_duration} sec")
                stepsToBackTrack = 5
                if burn_duration - dtThrust * stepsToBackTrack in maneuverLog: 
                    stepsToBackTrack -=1
                maneuverLog.append(burn_duration)
                
                """deleteToIndex = burnEnds[-1] - stepsToBackTrack - 1
                for k_super, v_dict in diffCOEs_dict.items():
                    for k_sub, v in v_dict.items():
                        if t[deleteToIndex] <= k_sub < t[burnEnds[-1]] and k_sub % dtCoast != 0:
                            del diffCOEs_dict[k_super][k_sub]
                            del diffCOEs_avg[k_super][k_sub]
                
                for k_super, v_dict in RIC_History.items():
                    for k_sub, v in v_dict.items():
                        if t[deleteToIndex] <= k_sub < t[burnEnds[-1]] and k_sub % dtCoast != 0:
                            del RIC_History[k_super][k_sub]
                            del RIC_Amp_History[k_super][k_sub]
                
                for j in COEs_keys:
                    COEs_restore_history = {k:v for k, v in diffCOEs_dict[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]]}
                    diffCOEs_buffer[j] = deque([*COEs_restore_history.values()], maxlen=int(steps_to_avg))

                for j in RIC_keys:
                    RIC_restore_history = {k:v for k, v in RIC_History[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]] and k % dtCoast == 0}
                    RIC_Amp_Buffer[j] = deque([*RIC_restore_history.values()], maxlen=int(1.5 * steps_per_rev))
                """

                restoreFromTime = burnEnds[-1] - burnEnds[-1] % dtCoast - (stepsToBackTrack * dtThrust // dtCoast) * dtCoast
                tStep = t.index(restoreFromTime)
                tHistoryCOEs = t[(tStep - steps_to_avg):tStep]
                for j in COEs_keys:
                    [diffCOEs_buffer[j].append(diffCOEs_dict[j][k]) for k in tHistoryCOEs]

                tHistoryRIC = t[int(tStep - 1.5 * steps_per_rev):tStep]
                for j in RIC_keys:
                    [RIC_Amp_Buffer[j].append(RIC_Amp_History[j][k]) for k in tHistoryRIC]
                burnEnds.pop()
                
                # Propagate spacecraft
                gator_ref.Step(-coast_duration)
                gator_truth.Step(-coast_duration)
                elapsed = elapsed - coast_duration
                coast_duration = 0

                # Update numerical integrator references
                gator_ref.UpdateSpaceObject()
                gator_truth.UpdateSpaceObject()
                
                # Update the the elpased time

                thrusterAxis = "I+"
                dt = dtThrust
                gator_truth = truthObjs.satEnginesOn(thrusterAxis)
                backPropTime = -stepsToBackTrack * dtThrust
                burn_duration -= stepsToBackTrack * dtThrust
                gator_ref.Step(backPropTime)
                gator_truth.Step(backPropTime)
                
                # Update numerical integrator references
                gator_ref.UpdateSpaceObject()
                gator_truth.UpdateSpaceObject()
                if burn_duration + dt <= 0:
                    print(f"Negative thrust time! Min I = {minIPosition}")
                    burnStarts.pop()
                    break
                if maneuverAttempts == 100:
                    print(f"Max burns! current burn duration = {burn_duration} sec | Min I = {minIPosition}")
                    burnStarts.pop()
                    break
        else: 
            RuntimeError("Invalid logic condition during I-axis deadband control")
            exit

def thrustC():
    return None

def checkR():
    return None

def checkI():
    return None

def checkC():
    return None