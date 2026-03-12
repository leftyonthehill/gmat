def xyz2ric(refState, offsetState):
    """Converts state vector from inertial frame to RIC frame based on reference state."""
    import numpy as np

    r_ref = np.array(refState[:3])
    v_ref = np.array(refState[3:6])
    r_offset = np.array(offsetState[:3])
    v_offset = np.array(offsetState[3:6])

    # Compute unit vectors for RIC frame
    R = r_ref / np.linalg.norm(r_ref)
    h_vec = np.cross(r_ref, v_ref)
    C = h_vec / np.linalg.norm(h_vec)
    I = np.cross(C, R)

    # Rotation matrix from inertial to RIC
    rotMatrix = np.vstack((R, I, C))
    
    # Position in RIC frame
    delta_r = r_offset - r_ref
    delta_v = v_offset - v_ref

    r_RIC = rotMatrix @ delta_r
    v_RIC = rotMatrix @ delta_v

    return r_RIC, v_RIC, rotMatrix