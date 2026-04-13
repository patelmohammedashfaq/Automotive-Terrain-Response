import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from fmpy import simulate_fmu

# 1. PATH MANAGEMENT (Industry Standard)
# Add the controller directory to the system path so we can import the logic
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '03_Controller')))

# Import the controller class from CNTRL_TorqueVectoring.py
from CNTRL_TorqueVectoring import CNTRL_TorqueVectoring

# Define the relative path to the FMU
FMU_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '02_Plant', 'PLNT_TorqueVectoring.fmu'))

def run_mil_simulation():
    """
    Model-in-the-Loop (MIL) Test Bench for Torque Vectoring
    Scenario: High-speed cornering (20m/s) with a step steering input.
    """
    # 2. SCENARIO CONFIGURATION
    t_stop = 10.0
    dt = 0.01  # K_DT = 10ms
    time = np.arange(0, t_stop, dt)
    
    # Inputs (Excitation Layer)
    in_drv_trq = 300.0  # Constant 300Nm demand
    in_steer = np.where(time > 1.0, 0.15, 0.0) # 0.15 rad steer at t=1s
    
    # 3. INITIALIZATION
    controller = CNTRL_TorqueVectoring()
    vx_feedback = 20.0  # Initial velocity
    results = []

    print(f"Starting MIL Simulation using: {FMU_PATH}")

    # 4. SIMULATION LOOP (MIL Orchestration)
    for i in range(len(time)):
        # Execute Controller Logic (The "Brain")
        trq_l, trq_r = controller.run_step(
            In_Steer_rad=in_steer[i], 
            In_Vx_ms=vx_feedback, 
            In_DriverTrq_Nm=in_drv_trq
        )
        
        # Step the Physics (The "Plant")
        # In a full FMPY implementation, you would use: 
        # model_output = fmu.doStep(currentCommunicationPoint=time[i], communicationStepSize=dt)
        # For this roadmap example, we simulate the feedback response:
        vx_feedback += (trq_l + trq_r) * 0.0001 # Simplified velocity gain
        
        results.append([time[i], in_steer[i], trq_l, trq_r])

    # 5. DATA POST-PROCESSING & VERIFICATION
    data = np.array(results)
    save_results(data)

def save_results(data):
    """Generates the ASPICE Layer 5 Verification Artifact"""
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(data[:, 0], data[:, 1], 'r', label='Steer Angle (rad)')
    plt.ylabel('Input')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(data[:, 0], data[:, 2], label='Trq_L (Inner Wheel)')
    plt.plot(data[:, 0], data[:, 3], label='Trq_R (Outer Wheel)')
    plt.axhline(y=150, color='black', linestyle='--', label='Baseline (50/50 Split)')
    plt.ylabel('Torque (Nm)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)
    
    # Ensure the verification folder exists
    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '05_Verification'))
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    plt.savefig(os.path.join(output_dir, 'Result_StepSteer_TV.png'))
    print(f"Verification Plot Saved to: 05_Verification/Result_StepSteer_TV.png")
    plt.show()

if __name__ == "__main__":
    run_mil_simulation()