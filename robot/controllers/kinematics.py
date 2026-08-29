import numpy as np

class LegKinematics:
    def __init__(self, l1=0.04, l2=0.10, l3=0.10):
        """
        Initialize leg link lengths in meters.
        l1: Hip offset (distance from body to thigh attachment)
        l2: Thigh length (upper leg)
        l3: Calf length (lower leg)
        """
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

    def inverse_kinematics(self, x, y, z, leg_index):
        """
        Calculates joint angles (theta_roll, theta_pitch, theta_knee) for a leg.
        x, y, z: Foot position relative to shoulder mount joint.
        leg_index: 0=FL, 1=FR, 2=HL, 3=HR (used to handle left/right symmetries).
        """
        # Determine if left or right side
        # FL=0, FR=1, HL=2, HR=3 -> Left legs: 0, 2; Right legs: 1, 3
        is_left = (leg_index == 0 or leg_index == 2)
        side_sign = 1.0 if is_left else -1.0
        
        # 1. Roll angle (Hip roll/yaw)
        # In y-z plane: L1 is the shoulder offset
        d_squared = y**2 + z**2
        if d_squared < self.l1**2:
            raise ValueError("Target coordinate is out of workspace (too close to shoulder)")
            
        r = np.sqrt(d_squared - self.l1**2)
        theta_roll = np.arctan2(y, z) - np.arctan2(side_sign * self.l1, r)
        
        # 2. Planar Pitch & Knee angles
        # Map target to leg coordinate plane (after rotating by roll angle)
        # x is unchanged, z_planar is the distance in planar leg frame
        z_planar = -r # negative down
        
        # Cosine rule for knee angle (theta_knee)
        target_dist_sq = x**2 + z_planar**2
        cos_knee = (target_dist_sq - self.l2**2 - self.l3**2) / (2.0 * self.l2 * self.l3)
        cos_knee = np.clip(cos_knee, -1.0, 1.0)
        
        # Knee angle (0 is fully extended)
        theta_knee = np.arccos(cos_knee)
        
        # Hip pitch angle (theta_pitch)
        alpha = np.arctan2(z_planar, x)
        beta = np.arctan2(self.l3 * np.sin(theta_knee), self.l2 + self.l3 * np.cos(theta_knee))
        theta_pitch = alpha - beta
        
        return float(theta_roll), float(theta_pitch), float(theta_knee)

    def forward_kinematics(self, theta_roll, theta_pitch, theta_knee, leg_index):
        """
        Calculates foot position (x, y, z) relative to shoulder mount joint.
        """
        is_left = (leg_index == 0 or leg_index == 2)
        side_sign = 1.0 if is_left else -1.0
        
        # Position in the leg's 2D sagittal plane (pitch and knee joints)
        x_planar = self.l2 * np.cos(theta_pitch) + self.l3 * np.cos(theta_pitch + theta_knee)
        z_planar = self.l2 * np.sin(theta_pitch) + self.l3 * np.sin(theta_pitch + theta_knee)
        
        # Map 2D coordinate to 3D space based on hip roll rotation
        x = x_planar
        y = side_sign * self.l1 * np.cos(theta_roll) - z_planar * np.sin(theta_roll)
        z = side_sign * self.l1 * np.sin(theta_roll) + z_planar * np.cos(theta_roll)
        
        return float(x), float(y), float(z)
