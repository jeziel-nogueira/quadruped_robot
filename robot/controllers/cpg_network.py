import numpy as np

class CPGNetwork:
    def __init__(self, config):
        self.load_config(config)
        self.reset_states()

    def load_config(self, config):
        """Load parameters from the configuration dictionary."""
        c = config["cpg"]
        fb = config["feedback"]
        
        # ODE Constants
        self.tau_m = c["tau_m"]
        self.K_motor = c["K_motor"]
        self.K_a = c["K_a"]
        
        # Master coefficients
        self.a_M = c["a_M"]
        self.b_M = c["b_M"]
        self.c_M = c["c_M"]
        self.d_M = c["d_M"]
        
        # Slave coefficients
        self.a_S = c["a_S"]
        self.b_S = c["b_S"]
        self.c_S = c["c_S"]
        self.d_S = c["d_S"]
        
        # Couplings
        self.k_12 = c["k_12"]
        self.k_21 = c["k_21"]
        self.k_M = np.array(c["k_M"])
        self.k_S = np.array(c["k_S"])
        
        # Feedback gains
        self.K_y = fb["K_y"]
        self.K_p = fb["K_p"]
        self.K_u = fb["K_u"]
        self.yaw_target = fb["yaw_target"]
        self.pitch_target = fb["pitch_target"]
        
        # Joint scaling and offsets
        self.scale_master = config["joints"]["scale_master"]
        self.scale_slave = config["joints"]["scale_slave"]
        self.joint_offsets = np.array(config["joints"]["offsets"])
        
        # Ultrasonic state defaults
        self.d_p_default = c["d_p_default"]
        
        # Gait control modifiers (multipliers)
        self.steering_amp_left = 1.0
        self.steering_amp_right = 1.0
        self.gait_enabled = True

    def reset_states(self):
        """Initialize all neuron states (positions u and velocities v) to small values to kickstart oscillation."""
        # Masters: 1 (controls FL & HR) and 2 (controls FR & HL)
        self.u_Me = np.array([0.1, -0.1])
        self.v_Me = np.array([0.0, 0.0])
        self.u_Mi = np.array([0.1, -0.1])
        self.v_Mi = np.array([0.0, 0.0])
        
        # Slaves: 0=FL, 1=FR, 2=HL, 3=HR
        self.u_Se = np.array([0.05, -0.05, -0.05, 0.05])
        self.v_Se = np.zeros(4)
        self.u_Si = np.array([0.05, -0.05, -0.05, 0.05])
        self.v_Si = np.zeros(4)
        
        # Parent mapping from Leg Index to Master Index (0 or 1)
        self.parent_map = [0, 1, 1, 0] # FL/HR -> Master 0, FR/HL -> Master 1

    def set_steering(self, direction):
        """
        Adjust leg amplitudes for steering.
        direction: float from -1.0 (full left) to 1.0 (full right).
        """
        if direction > 0.0: # Steer Right
            self.steering_amp_left = 1.0
            self.steering_amp_right = 1.0 - 0.5 * direction
        elif direction < 0.0: # Steer Left
            self.steering_amp_left = 1.0 + 0.5 * direction
            self.steering_amp_right = 1.0
        else:
            self.steering_amp_left = 1.0
            self.steering_amp_right = 1.0

    def set_gait_enabled(self, enabled):
        self.gait_enabled = enabled

    def step(self, dt, imu_yaw, imu_pitch, obstacle_dist):
        """
        Perform one Euler integration step.
        imu_yaw: current body yaw angle (degrees or rad).
        imu_pitch: current body pitch angle (degrees or rad).
        obstacle_dist: distance from ultrasonic sensor (cm).
        """
        # 1. Compute Feedback variables
        # Obstacle avoidance dp parameter
        if obstacle_dist < 15.0 and obstacle_dist > 0:
            # Slower/Stop/Turn gait parameter dp becomes larger (reversing behavior in paper)
            d_p = 10.0
        else:
            d_p = self.d_p_default
            
        # Gyroscope yaw/pitch offsets
        Moffset_g = self.K_y * (imu_yaw - self.yaw_target)
        Soffset_g = self.K_p * (imu_pitch - self.pitch_target)
        
        # Combine constants for ODE
        K_g_m_a = self.K_motor * self.K_a
        K_g_m = self.K_motor
        
        # Stop gait by dampening CPG network if disabled
        if not self.gait_enabled:
            # Quickly decay state towards zero
            self.v_Me -= 5.0 * self.u_Me * dt
            self.u_Me += self.v_Me * dt
            self.v_Mi -= 5.0 * self.u_Mi * dt
            self.u_Mi += self.v_Mi * dt
            self.v_Se -= 5.0 * self.u_Se * dt
            self.u_Se += self.v_Se * dt
            self.v_Si -= 5.0 * self.u_Si * dt
            self.u_Si += self.v_Si * dt
            return self.get_joint_angles()

        # 2. Master Units ODE Integration
        # Master 0 derivatives
        d2u_Me0 = (-self.v_Me[0] 
                   - Moffset_g 
                   - K_g_m_a * self.a_M * np.arctan(self.u_Me[0]) 
                   + self.d_M * np.arctan(self.u_Se[0])  # feedback from Slave 0 (FL)
                   - K_g_m_a * self.c_M * np.arctan(self.u_Mi[0]) 
                   + K_g_m * d_p 
                   + self.k_12 * np.arctan(self.u_Me[1])) / self.tau_m
                   
        d2u_Mi0 = (-self.v_Mi[0] 
                   - Moffset_g 
                   + K_g_m_a * self.b_M * np.arctan(self.u_Me[0]) 
                   + K_g_m * d_p 
                   + self.k_M[3] * np.arctan(self.u_Se[3])) / self.tau_m  # feedback from Slave 3 (HR)

        # Master 1 derivatives
        d2u_Me1 = (-self.v_Me[1] 
                   + Moffset_g  # opposite side correction
                   - K_g_m_a * self.a_M * np.arctan(self.u_Me[1]) 
                   + self.d_M * np.arctan(self.u_Se[1])  # feedback from Slave 1 (FR)
                   - K_g_m_a * self.c_M * np.arctan(self.u_Mi[1]) 
                   + K_g_m * d_p 
                   + self.k_21 * np.arctan(self.u_Me[0])) / self.tau_m
                   
        d2u_Mi1 = (-self.v_Mi[1] 
                   + Moffset_g 
                   + K_g_m_a * self.b_M * np.arctan(self.u_Me[1]) 
                   + K_g_m * d_p 
                   + self.k_M[2] * np.arctan(self.u_Se[2])) / self.tau_m  # feedback from Slave 2 (HL)

        # Update Master states
        self.v_Me[0] += d2u_Me0 * dt
        self.u_Me[0] += self.v_Me[0] * dt
        self.v_Mi[0] += d2u_Mi0 * dt
        self.u_Mi[0] += self.v_Mi[0] * dt
        
        self.v_Me[1] += d2u_Me1 * dt
        self.u_Me[1] += self.v_Me[1] * dt
        self.v_Mi[1] += d2u_Mi1 * dt
        self.u_Mi[1] += self.v_Mi[1] * dt

        # 3. Slave Units ODE Integration
        for j in range(4):
            parent = self.parent_map[j]
            # Excitatory Slave j
            d2u_Se = (-self.v_Se[j] 
                      - Soffset_g 
                      - K_g_m_a * self.a_S * np.arctan(self.u_Se[j]) 
                      + self.d_S * np.arctan(self.u_Me[parent]) 
                      - K_g_m_a * self.c_S * np.arctan(self.u_Si[j]) 
                      + K_g_m) / self.tau_m
            
            # Inhibitory Slave j          
            d2u_Si = (-self.v_Si[j] 
                      - Soffset_g 
                      + K_g_m_a * self.b_S * np.arctan(self.u_Se[j]) 
                      + K_g_m) / self.tau_m
            
            self.v_Se[j] += d2u_Se * dt
            self.u_Se[j] += self.v_Se[j] * dt
            self.v_Si[j] += d2u_Si * dt
            self.u_Si[j] += self.v_Si[j] * dt

        return self.get_joint_angles()

    def get_joint_angles(self):
        """
        Map CPG neuron positions u_Me, u_Se, u_Si to 12 degrees of freedom.
        Each leg has: Hip Yaw/Roll, Hip Pitch, Knee Pitch.
        Returns a list of 12 angles in radians.
        """
        angles = np.zeros(12)
        # Leg index offsets in the 12-DoF array: FL=0-2, FR=3-5, HL=6-8, HR=9-11
        for j, leg_name in enumerate(["FL", "FR", "HL", "HR"]):
            parent = self.parent_map[j]
            base_idx = j * 3
            
            # 1st joint (hip yaw/roll): driven by parent master excitatory state
            u_master = self.u_Me[parent]
            angles[base_idx] = u_master * self.scale_master
            
            # 2nd joint (hip pitch): driven by slave excitatory state
            u_slave_hip = self.u_Se[j]
            angles[base_idx + 1] = u_slave_hip * self.scale_slave
            
            # 3rd joint (knee pitch): driven by slave inhibitory state
            u_slave_knee = self.u_Si[j]
            angles[base_idx + 2] = u_slave_knee * self.scale_slave
            
            # Apply steering adjustments to left/right limbs
            is_left = (j == 0 or j == 2)
            amp = self.steering_amp_left if is_left else self.steering_amp_right
            angles[base_idx] *= amp
            angles[base_idx + 1] *= amp
            angles[base_idx + 2] *= amp
            
        # Add offset trims
        angles += self.joint_offsets
        return angles.tolist()
        
    def get_states(self):
        """Helper to get current state data for telemetry."""
        return {
            "u_Me": self.u_Me.tolist(),
            "u_Mi": self.u_Mi.tolist(),
            "u_Se": self.u_Se.tolist(),
            "u_Si": self.u_Si.tolist(),
        }
