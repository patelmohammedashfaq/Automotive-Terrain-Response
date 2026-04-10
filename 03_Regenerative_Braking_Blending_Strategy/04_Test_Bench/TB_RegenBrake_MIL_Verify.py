import fmpy
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# 1. Path Management for ASPICE Traceability
# Adding the 03_Controller directory to the system path
sys.path.append(os.path.abspath("03_Controller"))
from CTRL_RegenBrake_Logic import CTRL_RegenBrake_Logic

# File Paths with Digit-Prefixed Folders
FMU_PATH = "02_Plant/PLNT_RegenBrake_v1.fmu"
VERIF_DIR = "05_Verification"

def run_mil_simulation(initial_soc_pct):
    # Ensure Verification folder exists
    if not os.path.exists(VERIF_DIR):
        os.makedirs(VERIF_DIR)

    # 2. Initialization
    ecu = CTRL_RegenBrake_Logic()
    model_description = fmpy.read_model_description(FMU_PATH)
    
    # Extract Value References from Modelica names (MAAB Traceability)
    v_ref = next(v.valueReference for v in model_description.modelVariables if v.name == 'out_v_veh')
    trq_regen_ref = next(v.valueReference for v in model_description.modelVariables if v.name == 'in_trq_regen')
    trq_frict_ref = next(v.valueReference for v in model_description.modelVariables if v.name == 'in_trq_friction')

    unzipdir = fmpy.extract(FMU_PATH)
    fmu = fmpy.instantiate_fmu(unzipdir=unzipdir, model_description=model_description)
    fmu.setupExperiment(startTime=0.0)
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()
    
    # 3. Simulation Loop
    sim_time = 8.0
    time_steps = np.arange(0, sim_time, ecu.K_DT)
    log = {'t': [], 'v': [], 'regen': [], 'frict': [], 'demand': []}
    
    for t in time_steps:
        # Scenario: Driver steps on brake (70%) at 1.0s
        in_brake_pedal_pct = 70.0 if t > 1.0 else 0.0
        
        # Physics Feedback
        current_v = fmu.getReal([v_ref])[0]
        
        # Controller Step
        req_regen, req_frict = ecu.step(in_brake_pedal_pct, initial_soc_pct, current_v)
        
        # Update Plant
        fmu.setReal([trq_regen_ref, trq_frict_ref], [req_regen, req_frict])
        fmu.doStep(currentCommunicationPoint=t, communicationStepSize=ecu.K_DT)
        
        # Logging
        log['t'].append(t)
        log['v'].append(current_v)
        log['regen'].append(req_regen)
        log['frict'].append(req_frict)
        log['demand'].append((in_brake_pedal_pct / 100.0) * ecu.K_MAX_SYSTEM_BRAKE_TRQ)

    fmu.terminate()
    fmu.freeInstance()

    # 4. Automated Plotting & Saving (The "JLR Standard" Output)
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(log['t'], log['demand'], 'k--', label='Total Demand')
    plt.plot(log['t'], log['regen'], 'g', label='Regen Torque')
    plt.plot(log['t'], log['frict'], 'r', label='Friction Torque')
    plt.title(f'Brake Blending Verification (Initial SoC: {initial_soc_pct}%)')
    plt.ylabel('Torque [Nm]')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(log['t'], log['v'], 'b')
    plt.ylabel('Velocity [m/s]')
    plt.xlabel('Time [s]')
    plt.grid(True)

    # Auto-save with dynamic naming for traceability
    filename = f"PLOT_Regen_Blending_SoC_{int(initial_soc_pct)}.png"
    save_path = os.path.join(VERIF_DIR, filename)
    plt.tight_layout()
    plt.savefig(save_path)
    print(f"Verification Plot saved to: {save_path}")

if __name__ == "__main__":
    # Run test case: Low SoC (High Regen capability)
    run_mil_simulation(initial_soc_pct=40.0)