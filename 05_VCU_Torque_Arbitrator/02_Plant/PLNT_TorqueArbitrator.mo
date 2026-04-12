model PLNT_TorqueArbitrator "High-Fidelity Vehicle Dynamics Plant"
  // Physical Parameters (Portfolio DNA)
  parameter Real mass = 2200.0 "kg";
  parameter Real r_w = 0.35 "m";
  parameter Real J_wheel = 15.0 "kg.m2";
  parameter Real b_damping = 10.0 "N.m.s/rad";
  parameter Real Cd = 0.38 "Aerodynamic Drag";
  parameter Real Af = 2.8 "Frontal Area (m2)";
  parameter Real rho = 1.225 "Air Density (kg/m3)";
  parameter Real g = 9.81 "m/s2";

  // Inputs/Outputs for FMI
  input Real in_motor_torque "Nm";
  output Real out_v_veh "m/s";
  output Real out_a_veh "m/s2";

  // Internal States
  Real v_veh(start=0.0);
  Real F_prop;
  Real F_drag;
  Real F_rolling;

equation
  // Propulsion Force calculation
  F_prop = (in_motor_torque / r_w);
  
  // Resistance Forces
  F_drag = 0.5 * rho * Cd * Af * v_veh^2;
  F_rolling = mass * g * 0.015; // Assuming 1.5% rolling resistance

  // Longitudinal Dynamics (Newton's Second Law)
  // m*a = F_prop - F_resistances - Damping
  mass * der(v_veh) = F_prop - F_drag - F_rolling - (b_damping * v_veh / r_w);

  // Output assignments
  out_v_veh = v_veh;
  out_a_veh = der(v_veh);

end PLNT_TorqueArbitrator;
