1. Top-Level System Architecture (The "Integration Layer")
This diagram represents the high-level integration between your Python Controller (ECU) and the Modelica Plant (Physical Vehicle).

```mermaid
graph LR
    subgraph ECU_Software [03_Controller: CTRL_RegenBrake_Logic]
        direction LR
        Ctrl_Logic[Regen Blending Logic]
    end

    subgraph Physical_Plant [02_Plant: PLNT_RegenBrake_v1]
        direction LR
        Veh_Dynamics[Longitudinal Dynamics Model]
    end

    %% Inputs to ECU
    Brake_Pedal(Brake Pedal %) -->|in_brake_pedal_pct| Ctrl_Logic
    SoC(Battery SoC %) -->|in_soc_pct| Ctrl_Logic

    %% ECU to Plant
    Ctrl_Logic -->|out_trq_regen| Veh_Dynamics
    Ctrl_Logic -->|out_trq_friction| Veh_Dynamics

    %% Feedback Loop
    Veh_Dynamics -->|out_v_veh| Ctrl_Logic

    style ECU_Software fill:#e1f5fe,stroke:#01579b,stroke-width:2px
    style Physical_Plant fill:#fff3e0,stroke:#e65100,stroke-width:2px
```

2. Subsystem A: Driver Demand Interpretation
This subsystem converts the driver's physical intent (pedal position) into a target torque. In Simulink, this would be a 1-D Lookup Table or a Gain Block.

```mermaid
graph LR
    subgraph Subsystem_Interpretation [Brake Interpretation]
    direction LR
    In1[in_brake_pedal_pct] --> Div[Divide by 100]
    Div --> Prod[Product Block]
    K_MAX[K_MAX_SYSTEM_BRAKE_TRQ] --> Prod
    Prod --> Out1[sig_total_demand_trq]
    end

    style Subsystem_Interpretation fill:#f5f5f5,stroke:#333,stroke-width:2px
```

3. Subsystem B: Energy Management (SoC Observer)
This represents the _observer_logic method. It calculates the "Regen Ceiling" based on the current Battery State of Charge.

```mermaid
graph LR
    subgraph Subsystem_Energy_Mgmt [Battery Observer]
    direction LR
    In_SoC[in_soc_pct] --> MAP_SOC[MAP_SOC_DERATE]
    K_REGEN[K_MAX_REGEN_TRQ] --> Mult[Product Block]
    MAP_SOC --> Mult
    Mult --> Out_Limit[sig_available_regen_trq]
    end

    style Subsystem_Energy_Mgmt fill:#f5f5f5,stroke:#333,stroke-width:2px
```

4. Subsystem C: Blending & Arbitration logic
This is the core of your project. It coordinates the two braking systems and applies Actuator Protection (SOP Clipping).

```mermaid
graph LR
    subgraph Subsystem_Arbitration [Torque Blending & Clipping]
    direction LR
    
    Demand[sig_total_demand_trq]
    Limit[sig_available_regen_trq]
    
    %% Blending Math
    Demand --> Max_Block{Max Selector}
    Limit --> Max_Block
    
    Max_Block --> Regen_Pre_Clip[sig_regen_raw]
    Demand --> Sub[Subtract]
    Max_Block --> Sub
    
    %% Actuator Protection (SOP Compliance)
    Regen_Pre_Clip --> Clip_R[Saturation: K_MAX_REGEN to 0]
    Sub --> Clip_F[Saturation: K_MAX_SYSTEM to 0]
    
    Clip_R --> Out_R[out_trq_regen]
    Clip_F --> Out_F[out_trq_friction]
    end

    style Subsystem_Arbitration fill:#f5f5f5,stroke:#333,stroke-width:2px
```

5. Physics Plant Subsystem (Modelica Logic)
To show the recruiter you understand the "Plant" side of the V-Cycle, this diagram visualizes the equations inside your .mo file.

```mermaid
graph LR
    subgraph Subsystem_Plant [Vehicle Physics DNA]
    direction LR
    
    T_Regen[in_trq_regen] --> Sum_T((+))
    T_Frict[in_trq_friction] --> Sum_T
    
    Sum_T --> Div_R[Divide by r_w]
    Div_R --> F_Total[Total Force]
    
    F_Total --> Accel[1/mass]
    Accel --> Integ[Integrator: 1/s]
    Integ --> Out_V[out_v_veh]
    
    %% Aero Feedback
    Out_V --> Aero[Aero Drag: 0.5*rho*Cd*Af*v^2]
    Aero -->|Subtract| F_Total
    end

    style Subsystem_Plant fill:#f5f5f5,stroke:#333,stroke-width:2px
```

# MIL Verification Report: Regenerative Braking Blending
**Project ID:** PROJ_03  
**Status:** Partial (3 PASSED, 1 Unconclusive)  
**Date:** 2026-04-09  

## 1. Objective
To verify the smooth arbitration between electrical regenerative braking and hydraulic friction braking based on driver demand and battery constraints.

## 2. Requirement Traceability Matrix
| Req ID | Requirement Description | Result | Evidence |
| :--- | :--- | :--- | :--- |
| **REQ-BRK-01** | Map 0-100% pedal to total torque demand. | **PASS** | Demand traces black dashed line. |
| **REQ-BRK-02** | Prioritize Regen for initial deceleration. | **Unconclusive** | Not tested as brake is applied suddenly scenario. |
| **REQ-BRK-04** | Blend friction brakes when regen is limited. | **PASS** | Red line (Friction) fills the delta at t=1.0s. |
| **REQ-SAFE-05**| Saturate all outputs to physical limits. | **PASS** | No torque exceeds K_MAX_SYSTEM_BRAKE_TRQ. |

## 3. Analysis of Results
The simulation results shown in the figure below demonstrate the controller's performance during a 70% brake pedal application with a 40% Battery SoC.

![Regen Blending Verification Plot](./05_Verification/PLOT_Regen_Blending_SoC_40.png)

### Performance Observations:
1. The **Regen Torque** successfully capped at the calibrated **-1500 Nm**.
2. The **Friction Torque** seamlessly supplemented the remaining **-2000 Nm** to meet the total **-3500 Nm** demand.
3. No torque oscillations were observed, indicating a stable control loop at **100Hz (K_DT = 0.01s)**.
