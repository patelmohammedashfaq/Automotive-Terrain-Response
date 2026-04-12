import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import sys

# Ensure we can find the controller in /03_Controller/
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '03_Controller')))
from CNTRL_TorqueArbitrator import TorqueArbitrator

os.makedirs('../05_Verification', exist_ok=True)

# ... [Your simulation loop logic here] ...

# --- 1. Scenario Configuration ---
pedal_grid = np.linspace(-10, 110, 25) # Includes fault zones
temp_grid = np.linspace(80, 130, 25)   # Includes derate/shutdown zones
brake_states = [False, True]

# --- 2. Batch Execution ---
batch_data = []

for brake in brake_states:
    for temp in temp_grid:
        for pedal in pedal_grid:
            # Re-init controller for steady-state check
            vcu = TorqueArbitrator()
            
            # Run simulation for 2s to allow Slew Rate to settle
            out_trq = 0.0
            for _ in range(200): 
                out_trq = vcu.step(pedal, brake, temp)
            
            # Automated Oracle Logic
            verdict = "PASS"
            if brake and out_trq > 0.01: verdict = "FAIL_BRAKE"
            if (pedal < 0 or pedal > 100) and out_trq > 0.01: verdict = "FAIL_FAULT"
            if temp >= 120 and out_trq > 0.01: verdict = "FAIL_THERMAL"
            
            batch_data.append({
                'Pedal': pedal, 'Temp': temp, 'Brake': brake, 
                'Torque': out_trq, 'Verdict': verdict
            })

df = pd.DataFrame(batch_data)

# --- 3. High-Fidelity Visualization ---
fig = plt.figure(figsize=(16, 10))
plt.suptitle("VCU Torque Arbitrator: Multi-Dimensional Verification Report", fontsize=18, fontweight='bold')

# Plot A: The Heatmap (Normal Operation)
ax1 = plt.subplot(2, 2, 1)
normal_ops = df[df['Brake'] == False]
pivot = normal_ops.pivot(index="Temp", columns="Pedal", values="Torque")
sns.heatmap(pivot, cmap="YlOrRd", ax=ax1, cbar_kws={'label': 'Steady State Torque (Nm)'})
ax1.set_title("Operating Envelope (Brake=OFF)\nREQ-01 (Map) & REQ-07 (Thermal)")

# Plot B: The Interlock Check (Brake Active)
ax2 = plt.subplot(2, 2, 2)
brake_ops = df[df['Brake'] == True]
ax2.scatter(brake_ops['Pedal'], brake_ops['Torque'], color='red', alpha=0.5)
ax2.set_title("Brake Priority Verification (Brake=ON)\nREQ-02: Output must be 0Nm")
ax2.set_ylabel("Torque (Nm)")
ax2.grid(True)

# Plot C: Fault Injection Zone
ax3 = plt.subplot(2, 2, 3)
sns.lineplot(data=normal_ops[normal_ops['Temp'] == 90], x='Pedal', y='Torque', ax=ax3, color='blue')
ax3.axvspan(-10, 0, color='gray', alpha=0.3, label='Fault Zone')
ax3.axvspan(100, 110, color='gray', alpha=0.3)
ax3.set_title("Plausibility Safety Check\nREQ-06: 0Nm in Gray Zones")
ax3.legend()

# Plot D: Verdict Summary
ax4 = plt.subplot(2, 2, 4)
df['Verdict'].value_counts().plot(kind='pie', autopct='%1.1f%%', colors=['green', 'red', 'orange'], ax=ax4)
ax4.set_title("Statistical Requirement Coverage")

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
# 3. Path Fix: Save results to the Verification folder
save_path = '../05_Verification/Full_Verification_Report.png'
plt.savefig(save_path)
print(f"Verification Artifacts Exported to: {save_path}")