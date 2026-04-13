import numpy as np

class CNTRL_TorqueVectoring:
    def __init__(self):
        # Constants (K_ prefix)
        self.K_DT = 0.01
        self.K_MAX_TRQ = 450.0  # Max Nm per motor
        self.K_TV_GAIN = 0.5    # Intensity of the vectoring
        self.K_V_FLOOR = 0.5    # REQ-TV-004 protection

    def run_step(self, In_Steer_rad, In_Vx_ms, In_DriverTrq_Nm):
        """
        Main Logic for Torque Vectoring
        """
        # 1. Division-by-Zero Protection
        v_ref = max(In_Vx_ms, self.K_V_FLOOR)

        # 2. Calculate Target Yaw Bias (Simplified logic)
        # Higher steer at higher speed = more bias needed to rotate
        trq_bias = In_Steer_rad * v_ref * self.K_TV_GAIN * 100

        # 3. Arbitrate Torque (Left vs Right)
        # Divide driver demand in half then add/subtract bias
        trq_l_raw = (In_DriverTrq_Nm / 2.0) - trq_bias
        trq_r_raw = (In_DriverTrq_Nm / 2.0) + trq_bias

        # 4. Actuator Protection & Saturation (Defensive Rule)
        # Ensure individual motors don't exceed limits
        Out_Trq_L = np.clip(trq_l_raw, 0, self.K_MAX_TRQ)
        Out_Trq_R = np.clip(trq_r_raw, 0, self.K_MAX_TRQ)

        # 5. Safety Constraint (REQ-TV-003)
        # Ensure sum of vectoring doesn't exceed total driver request
        total_out = Out_Trq_L + Out_Trq_R
        if total_out > In_DriverTrq_Nm:
            scale = In_DriverTrq_Nm / total_out
            Out_Trq_L *= scale
            Out_Trq_R *= scale

        return Out_Trq_L, Out_Trq_R