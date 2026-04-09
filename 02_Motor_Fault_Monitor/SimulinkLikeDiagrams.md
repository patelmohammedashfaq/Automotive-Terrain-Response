# ASIL-C Motor Torque Plausibility Monitor
**Automotive Powertrain Control Systems | ISO 26262 Functional Safety**

## 📌 Executive Summary
This repository contains the Model-in-the-Loop (MIL) implementation of an ASIL-C compliant Motor Torque Plausibility Monitor. Designed for a high-performance EV/HEV powertrain (e.g., SUV/Defender class), the system monitors the torque path between the Vehicle Control Unit (VCU) and the Motor Inverter. It utilizes a debounce algorithm to detect hardware faults and arbitrates a Safe State to prevent unintended acceleration, fulfilling strict Fault Tolerant Time Interval (FTTI) requirements.

The project structure adheres to **ASPICE** guidelines, strictly separating Plant physics, Controller logic, and Verification test benches.

---

## 🏗️ 1. Top-Level System Architecture
The system operates in a closed-loop Model-in-the-Loop (MIL) environment. The Test Bench orchestrates the scenario, feeding driver intent to the ECU and injecting hardware faults to validate the safety mechanisms.

```mermaid
graph TD
    subgraph TB [04_Test_Bench: Scenario & Verification]
        Driver[Driver Intent: 200Nm] --> FI[Fault Injector]
    end

    subgraph CTRL [03_Controller: ASIL-C Monitor]
        ECU[MotorMonitorController]
    end

    subgraph PLNT [02_Plant: Vehicle Dynamics]
        Physics[Longitudinal Plant Model]
    end

    Driver -->|In_TrqReq_Nm| ECU
    FI -.->|Simulate Sensor Failure| Physics
    Physics -->|In_TrqAct_Nm| ECU
    ECU -->|Out_TrqCmd_Nm| Physics

    classDef tb fill:#e1f5fe,stroke:#01579b,stroke-width:2px;
    classDef ctrl fill:#e8f5e9,stroke:#1b5e20,stroke-width:2px;
    classDef plnt fill:#fff3e0,stroke:#e65100,stroke-width:2px;
    
    class TB tb;
    class CTRL ctrl;
    class PLNT plnt;


```mermaid
flowchart TD
    Start([10ms ECU Interrupt]) --> Read[Read: In_TrqReq & In_TrqAct]
    Read --> CalcErr[Torque Error = |Req - Act|]
    
    subgraph Fault_Detection [Monitoring Layer]
        CalcErr --> CheckThresh{Error > 50 Nm?}
        CheckThresh -- Yes --> Inc[Counter += 1]
        CheckThresh -- No --> Dec[Counter = Max 0, Counter - 1]
    end

    subgraph Safe_State_Arbitration [ISO 26262 Action Layer]
        Inc --> CheckCount{Counter >= 3?}
        Dec --> CheckCount
        CheckCount -- Yes --> Latch[SAFE_STATE_ACTIVE = TRUE]
        CheckCount -- No --> CheckLatch{Is Safe State ALREADY True?}
    end

    Latch --> CmdZero[Out_TrqCmd = 0.0 Nm]
    CheckLatch -- Yes --> CmdZero
    CheckLatch -- No --> CmdNorm[Out_TrqCmd = Clip Request to 450Nm]

    CmdZero --> Output([Return Command & Status])
    CmdNorm --> Output

    style Fault_Detection fill:#f9f9f9,stroke:#333,stroke-dasharray: 5 5
    style Safe_State_Arbitration fill:#ffebee,stroke:#b71c1c,stroke-width:2px

```mermaid
graph LR
    Input[In_Motor_Torque] --> Gain1[1 / r_wheel]
    Gain1 --> F_drive[F_drive N]

    V[v_veh] --> Square[v^2]
    Square --> Gain2[0.5 * rho * Cd * Af]
    Gain2 --> F_drag[F_drag N]

    V --> Gain3[b_damping / r_wheel]
    Gain3 --> F_damp[F_damping N]

    F_drive --> Sum((+))
    F_drag -->|-| Sum
    F_damp -->|-| Sum

    Sum --> Gain4[1 / Mass]
    Gain4 --> Accel[dv/dt]
    Accel --> Int[∫ dt]
    Int --> V
    V --> Output[Out_v_veh]

    style Sum fill:#fff,stroke:#333,stroke-width:2px
    style Int fill:#e0f7fa,stroke:#006064

