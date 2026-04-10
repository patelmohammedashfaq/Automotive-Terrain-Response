import numpy as np

class CTRL_RegenBrake_Logic:
    def __init__(self):
        # 1. Calibration Parameters (MAAB Standard)
        self.K_DT = 0.01  # 100Hz Loop Time
        self.K_MAX_SYSTEM_BRAKE_TRQ = -5000.0  # Nm (Max capability of vehicle)
        self.K_MAX_REGEN_TRQ = -1500.0  # Nm (Approx 0.2g deceleration limit)
        self.K_HYSTERESIS = 3  # For future fault debounce logic
        
        # 2. 1D Lookup Table (MAP) for Battery SoC Derating
        # As the battery gets full (>80%), regen capability drops to 0.
        self.MAP_SOC_BP = np.array([0.0, 50.0, 80.0, 95.0, 100.0]) # Breakpoints (%)
        self.MAP_SOC_DERATE = np.array([1.0, 1.0, 0.8, 0.2, 0.0])  # Multiplier

    def _observer_logic(self, in_soc_pct):
        """Private method to separate sensing from action."""
        # Interpolate the derating factor based on current SoC
        return np.interp(in_soc_pct, self.MAP_SOC_BP, self.MAP_SOC_DERATE)

    def step(self, in_brake_pedal_pct, in_soc_pct, in_v_veh):
        """Main execution loop called every 10ms (K_DT)."""
        
        # Prevent division-by-zero or low-speed judder
        if in_v_veh < max(in_v_veh, 0.5):
            in_v_veh = 0.5
            
        # 1. Calculate Total Demand
        # Map 0-100% pedal to 0 to -5000 Nm
        total_demand_trq = (in_brake_pedal_pct / 100.0) * self.K_MAX_SYSTEM_BRAKE_TRQ
        
        # 2. Calculate Available Regen (Constrained by SoC)
        soc_derate_factor = self._observer_logic(in_soc_pct)
        available_regen_trq = self.K_MAX_REGEN_TRQ * soc_derate_factor
        
        # 3. Blending Arbitration
        # Regen takes as much as it can (up to its limit). 
        # Note: using max() because torques are negative values.
        out_trq_regen = max(total_demand_trq, available_regen_trq)
        
        # Friction brakes make up the remainder
        out_trq_friction = total_demand_trq - out_trq_regen
        
        # 4. Actuator Protection (Clipping to hardware limits)
        out_trq_regen = np.clip(out_trq_regen, self.K_MAX_REGEN_TRQ, 0.0)
        out_trq_friction = np.clip(out_trq_friction, self.K_MAX_SYSTEM_BRAKE_TRQ, 0.0)
        
        return out_trq_regen, out_trq_friction