REQ-SAFE-01: The system shall monitor the deviation between Requested_Torque and Actual_Torque.
REQ-SAFE-02: A fault shall be latched if the torque deviation exceeds 50 Nm for a duration longer than the Fault Tolerant Time Interval (FTTI).
REQ-SAFE-03: Upon fault detection, the system shall transition to a Safe State, commanding a torque request of 0 Nm to the motor.
REQ-SAFE-04: The system shall use Hysteresis logic (3 counts) to prevent nuisance trips caused by signal noise or transient spikes.