import numpy as np

class TorqueArbitrator:
    def __init__(self):
        # Calibration Parameters (from REQ Section 3)
        self.K_DT = 0.01          # 100Hz
        self.K_MAX_TRQ = 450.0    # Nm
        self.K_SLEW_POS = 500.0   # Nm/s
        self.K_SLEW_NEG = 800.0   # Nm/s
        self.K_TAU_PEDAL = 0.05   # s (Low Pass Filter)

        # Internal States
        self.prev_trq_out = 0.0
        self.filtered_pedal = 0.0
        self.fault_active = False

    def step(self, in_pedal_pct, in_brake_active, in_motor_temp):
        """
        Executes one control loop at 10ms.
        """
        # --- REQ-VCU-06: Pedal Plausibility Check ---
        if in_pedal_pct < -5.0 or in_pedal_pct > 105.0:
            self.fault_active = True
        
        # --- REQ-VCU-03: Signal Conditioning (LPF) ---
        # Alpha = dt / (tau + dt)
        alpha = self.K_DT / (self.K_TAU_PEDAL + self.K_DT)
        self.filtered_pedal = (alpha * in_pedal_pct) + ((1 - alpha) * self.filtered_pedal)

        # --- REQ-VCU-01: Driver Interpretation (Mapping) ---
        # Simple linear map: 0-100% -> 0-K_MAX_TRQ
        trq_driver = (np.clip(self.filtered_pedal, 0, 100) / 100.0) * self.K_MAX_TRQ

        # --- REQ-VCU-07: Thermal Derating ---
        # Lambda logic from Requirement 2.2
        thermal_lambda = np.clip(1.0 - (in_motor_temp - 100.0) / (120.0 - 100.0), 0.0, 1.0)
        trq_target = trq_driver * thermal_lambda

        # --- REQ-VCU-02: Brake Priority ---
        if in_brake_active or self.fault_active:
            trq_target = 0.0

        # --- REQ-VCU-04/05: Rate Limiter (Slew Filter) ---
        # Logic from Requirement 2.1
        delta_max_pos = self.K_SLEW_POS * self.K_DT
        delta_max_neg = self.K_SLEW_NEG * self.K_DT
        
        delta_raw = trq_target - self.prev_trq_out
        delta_clamped = np.clip(delta_raw, -delta_max_neg, delta_max_pos)
        
        trq_out_raw = self.prev_trq_out + delta_clamped

        # --- REQ-VCU-08: Output Saturation ---
        final_trq = np.clip(trq_out_raw, 0.0, self.K_MAX_TRQ)
        
        # Update State
        self.prev_trq_out = final_trq
        
        return final_trq