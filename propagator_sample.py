# Orbit and attitude propagator for a Mars mission simulating a nano-satellite
# switching between three attitude references:
# 
# 1. Power Mode (mode = 1): solar panels point toward the sun (+Y in inertial frame)
# 2. Science Mode (mode = 0): sensors point toward Mars
# 3. Communication Mode (mode = 2): antenna points toward a mothercraft in 
#    Geosynchronous Mars Orbit, whenever the mothercraft is visible and the satellite
#    is not on the sunlit side of Mars
#
# Attitude is represented using Modified Rodrigues Parameters (MRP)

# Brief explanation of the parameters included in the propagation:
# K, P: control gains
# u: control action
# I: satellite inertia matrix
# L: torque perturbation
# r_LMO, r_GMO: orbital radius for nano-satellite and mothercraft
# EU_LMO: actual state variables of the nano-satellite
# EU_GMO: actual state variables of the mothercraft
# HN: DCM from inertial to nano-satellite body frame

# Main simulation loop: propagate orbit and attitude, compute control,
# integrate MRP and angular rates, and handle attitude mode switching

# t: actual time
# tvec: vector that contains the time intervals of the whole duration of the mission

for t in time_vector:
    
    #Time Step
    dt = t - prev_t
    
    #Previous time
    prev_t = t

    #Orbit propagation of the nano-satellite    
    EU_LMO = EU_LMO + dotEU_LMO * dt
    #Orbit propagation of the mothercraft
    EU_GMO = EU_GMO + dotEU_GMO * dt
    
    # Obtain DCM of the Body frame of the nano-satellite
    HN = R313(EU_LMO[0], EU_LMO[1], EU_LMO[2])
    
    #Position vector of the nano-satellite
    R_LMO = pos_vector(r_LMO, EU_LMO[0], EU_LMO[1], EU_LMO[2])
    
    #Velocity vector of the nano-satellite
    V_LMO = vel_vector(r_LMO, EU_LMO[0], EU_LMO[1], EU_LMO[2])
    
    #Position vector of the mothercraft
    R_GMO = pos_vector(r_GMO, EU_GMO[0], EU_GMO[1], EU_GMO[2])
    
    #Velocity vector of the mothercraft
    V_GMO = vel_vector(r_GMO, EU_GMO[0], EU_GMO[1], EU_GMO[2])

    #Power mode: nano-satellite is in the region facing the sun    
    if (R_LMO[1,0] > 0):
        mode = 1
        
    # Communication mode: The nano-satellite is within range of the mothercraft
    # Compare if the angle between the position vectors of the nano-satellite and the mothercraft
    # is lower or equal than 35 degrees
    elif (angle_vectors(R_LMO, R_GMO) <= np.radians(35)): 
        mode = 2
    #Science mode: any other point along the orbit that are not within the previous conditions
    else:
        mode = 0
    
    # DCM of the Body Frame of the mothercraft
    DCM_Rgmo = GMO_ref_DCM(R_LMO,R_GMO)

    #Reference Calculation
    
    # Get DCM of the attitude reference of the actual pointing mode
    MRP_DCM_ref = reference(HN, DCM_R_N_ant, R_LMO, R_GMO, dt, mode)[0]
    
    #Obtain the attitude reference in MRP
    MRP_ref = dcm_to_mrp(MRP_DCM_ref)
    
    #Obtain the angular rate reference of the actual pointing mode
    w_ref = reference(HN, DCM_R_N_ant, R_LMO, R_GMO, dt, mode)[1]
 
    # Attitude and angular rate error calculation
    
    # Attitude error calculation
    MRP_err = track_error (MRP, reference(HN, DCM_R_N_ant, R_LMO, R_GMO, dt, mode)[0], W, 
                           reference(HN, DCM_R_N_ant, R_LMO, R_GMO, dt, mode)[1])[0]
    
    #Angular rate error calculation
    w_err = track_error (MRP, reference(HN, DCM_R_N_ant, R_LMO, R_GMO, dt, mode)[0], W, 
                           reference(HN, DCM_R_N_ant, R_LMO, R_GMO, dt, mode)[1])[1]
    if mode == 2:
        DCM_R_N_ant = MRP_DCM_ref
    else:
        DCM_R_N_ant = DCM_Rgmo
    
# Runge Kutta 4-th order integrator

    # Update the control action every second
    if abs((t % 1.0)) < 1e-6:
        # Calculate the control action according to the errors calculated previously
        # and the control gains K and P
        u = control(K, P, MRP_err, w_err)
    
    # Attitude integration with MRP

    k1 = dt*mrp_dot(MRP,W)
    k2 = dt*mrp_dot(MRP + 0.5*k1,W)
    k3 = dt*mrp_dot(MRP + 0.5*k2,W)
    k4 = dt*mrp_dot(MRP + k3,W)
    MRP = MRP + (1/6) * (k1 + 2*k2 + 2*k3 + k4)

    # Angular rates integration
    k1 = dt*wdot(W, I, u, L)
    k2 = dt*wdot(W + 0.5*k1, I, u, L)
    k3 = dt*wdot(W + 0.5*k2, I, u, L)
    k4 = dt*wdot(W + k3, I, u, L)

    W = W + (1/6) * (k1 + 2*k2 + 2*k3 + k4)
    
    #Change to MRP shadow set to avoid singularities
    if np.linalg.norm(MRP) > 1:
        MRP = mrp_shadow(MRP)
    DCM_BN = mrp_to_dcm(MRP)
    

   
