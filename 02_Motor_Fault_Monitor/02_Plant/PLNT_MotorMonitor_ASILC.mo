model PLNT_MotorMonitor_ASILC
  /* * PHYSICAL PARAMETERS: The "Chassis DNA" 
   * These define the vehicle's physical characteristics for consistent simulation.
   */
  parameter Real mass = 2200.0 "kg";           // Vehicle Mass (SUV/Defender class) [cite: 2]
  parameter Real r_w = 0.35 "m";              // Wheel Radius [cite: 2]
  parameter Real J_wheel = 15.0 "kg.m2";      // Stabilized Wheel Inertia to minimize signal jitter [cite: 3]
  parameter Real b_damping = 10.0 "N.m.s/rad"; // Viscous damping for realistic torque decay [cite: 4]
  parameter Real Cd = 0.38 "1";               // Aerodynamic Drag Coefficient [cite: 5]
  parameter Real Af = 2.8 "m2";               // Frontal Area [cite: 5]
  parameter Real rho = 1.225 "kg/m3";         // Standard Air Density [cite: 11]
  parameter Real g = 9.81 "m/s2";             // Gravitational Acceleration [cite: 11]

  /* * INTERFACE: Signal definition following Left-to-Right flow 
   */
  input Real in_motor_torque "N.m";            // Torque input from the Motor Controller
  output Real out_v_veh "m/s";                // Vehicle Velocity output for the ECU

  /* * INTERNAL DYNAMICS 
   */
  Real v_veh(start=0) "m/s";                  // Longitudinal Velocity state
  Real F_drive "N";                           // Tractive Force at the contact patch
  Real F_drag "N";                            // Aerodynamic Resistance

equation
  // Convert Motor Torque to Tractive Force: F = T / r
  F_drive = in_motor_torque / r_w;
  
  // Aerodynamic Drag Equation: 0.5 * rho * Cd * Af * v^2
  F_drag = 0.5 * rho * Cd * Af * v_veh^2;
  
  // Longitudinal Dynamics (F = ma) including drag and internal damping
  mass * der(v_veh) = F_drive - F_drag - (b_damping * v_veh / r_w);
  
  // Assign internal state to the output interface
  out_v_veh = v_veh;
  
end PLNT_MotorMonitor_ASILC;
