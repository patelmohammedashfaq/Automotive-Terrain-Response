import numpy as np

class MotorMonitorController:
    """
    ASIL-C Safety Monitor for Motor Torque Plausibility.
    Compares VCU Request vs. Inverter Actual Torque to detect safety violations.
    """
    def __init__(self):
        # ---------------------------------------------------------
        # 1. CALIBRATION (ECU DNA): Constants defined at the top [cite: 21]
        # ---------------------------------------------------------
        self.K_DT = 0.01                 # Sample Time (10ms) [cite: 7]
        self.K_TRQ_ERR_LIMIT = 50.0      # Fault Threshold in Nm [cite: 9]
        self.K_HYSTERESIS = 3            # Debounce counts to prevent nuisance trips [cite: 9]
        self.K_MAX_TRQ = 450.0           # Physical hardware limit of the motor [cite: 8]
        self.K_V_REF_FLOOR = 0.5         # Velocity floor to prevent division by zero [cite: 10]
        
        # ---------------------------------------------------------
        # 2. STATE VARIABLES: Initialized for persistence between steps
        # ---------------------------------------------------------
        self.fault_counter = 0           # Tracks consecutive fault counts
        self.safe_state_active = False   # Latch for the ISO 26262 Safe State

    def step(self, In_TrqReq_Nm, In_TrqAct_Nm):
        """
        Executes one loop of the safety logic at 100Hz (K_DT = 0.01).
        """
        
        # ---------------------------------------------------------
        # 3. SIGNAL MONITORING: Error calculation [cite: 23]
        # ---------------------------------------------------------
        # Calculate the absolute deviation between requested and actual torque
        torque_error = abs(In_TrqReq_Nm - In_TrqAct_Nm)

        # ---------------------------------------------------------
        # 4. DEBOUNCE LOGIC (Hysteresis): Managing transient spikes [cite: 24]
        # ---------------------------------------------------------
        # Increment counter if error > threshold; decrement otherwise (leaky bucket)
        if torque_error > self.K_TRQ_ERR_LIMIT:
            self.fault_counter += 1
        else:
            self.fault_counter = max(0, self.fault_counter - 1)

        # ---------------------------------------------------------
        # 5. SAFE STATE ARBITRATION: ISO 26262 Fault Management [cite: 23]
        # ---------------------------------------------------------
        # If fault persists beyond hysteresis threshold, enter Safe State
        if self.fault_counter >= self.K_HYSTERESIS:
            self.safe_state_active = True

        # Action: If Safe State is latched, force Torque to 0Nm (Safe Mode)
        if self.safe_state_active:
            # Safe State: Zero torque to eliminate unintended acceleration
            Out_TrqCmd_Nm = 0.0  
        else:
            # Normal Operation: Follow request but clip to hardware limits 
            Out_TrqCmd_Nm = np.clip(In_TrqReq_Nm, 0, self.K_MAX_TRQ)

        return Out_TrqCmd_Nm, self.safe_state_active