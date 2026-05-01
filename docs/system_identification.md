# System Identification

This file documents my attempts at system identification for the self balancing cube test apparatus.

## Basic Parameters

* m (mass of test apparatus) = 318g (this includes the support arm and bearings).
* d (distance to pivot point) =  83.169 mm
* I_w (reaction wheel moment of inertia) = 0.000736 kg m^2
  This includes the measured moment of inertia of the wheel with the 3/8 in nuts, plus the mass of the motor
  bell.

## Moment of Inertia Measurement for Whole Assembly

We computed the moment of inertia by measuring several falls from standing,
computing their slope. That slope combined with 

* I_p = m * g * d / Slope

gives us the moment of inertia for the whole assembly.

* I_p = 0.00118 kg m^2

## Torque Constant of Motor

We measured the acceleration of the wheel at various torques. We got
about 55 rad/s^2 per 800mA of added current.

* K_t = dTorque / dCurrent = 0.04048 Nm / 0.8 A = 0.0506 Nm/A
