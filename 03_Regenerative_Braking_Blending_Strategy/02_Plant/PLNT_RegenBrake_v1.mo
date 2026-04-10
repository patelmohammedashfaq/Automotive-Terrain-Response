model PLNT_RegenBrake_v1
  // Core Vehicle DNA (Immutable Baseline)
  parameter Real mass = 2200.0 "kg";
  parameter Real r_w = 0.35 "m";
  parameter Real J_wheel = 15.0 "kg.m2";
  parameter Real b_damping = 10.0 "N.m.s/rad";
  parameter Real Cd = 0.38 "1";
  parameter Real Af = 2.8 "m2";
  parameter Real rho = 1.225 "kg/m3";
  parameter Real g = 9.81 "m/s2";

  // Signal Interfaces (MAAB Naming)
  input Real in_trq_regen "Nm";
  input Real in_trq_friction "Nm";
  output Real out_v_veh "m/s";

  // State Variables
  Real v_veh(start=28.0); // Starting at approx 100 kph
  Real F_aero;
  Real F_brake_total;

equation
  // Aerodynamic drag
  F_aero = 0.5 * rho * Cd * Af * v_veh^2;
  
  // Total braking force at the contact patch (Torques are negative)
  F_brake_total = (in_trq_regen + in_trq_friction) / r_w;
  
  // Longitudinal Equation of Motion
  mass * der(v_veh) = F_brake_total - F_aero - (b_damping * v_veh / r_w^2);
  
  // Assign to output
  out_v_veh = v_veh;
end PLNT_RegenBrake_v1;
