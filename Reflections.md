# PID Controller

A PID controller was implemented that initializes with zero error and given co-efficients for each of the following components
- Propotional co-efficient (Kp)
- Integral co-oefficient (Ki)
- Differential co-efficient (Kd)

After the controller is initialized, the controller provides the steering input to use through the `TotalError` method taking into account the cross-track error(CTE).

The total error at any given instant is computed as 
```Kp * CTE + Kd * dCTE/dt + Ki * Total_CTE```

The co-efficients for the controller were selected using a two-phase method.

**Phase 1: Manual tuning**  
The propotional co-efficient was chosen first so that the car oscillates about a steady path. 
Then, a differential component was added to dampen the oscillations.

**Phase 2: Twiddle**  
The twiddle methodology was used to fine tune the co-efficients to give the minimum RMSE cross-track error over a set number of timesteps.

**Effect of individual co-efficients on the control input**
- As expected, the the integral co-efficient was very low since it only accounts for system bias.
- The differential component dampens reduces the control input as the error rate decreases there by decreasing oscillations in control input.  
As expected, it is large so that the car quickly "straight lines" after a steering operation given the short track width. 
- The propotional component controls the magnitude of the control input given the cross-track error.  
The propotional component was large enough to handle the track curvature.