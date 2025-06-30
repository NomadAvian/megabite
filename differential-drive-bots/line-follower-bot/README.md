# LFR

1. description
2. image
3. parts list
4. high level control structure

## Proportional-Integral-Differential (**PID**)

**PID** is a feedback control technique that generates correcting actions based on error calculation.

### Equation

$$
    u(t) = K_p e(t) + K_i \int e(t) dt + K_d {de(t) \over dt}
$$

1. **Proportional Action**: dominant response of the system.
2. **Integral Action**: optimizing steady-state behavior.
3. **Derivative Action**: shapes the damping behavior.

|CL Response|Rise Time|Overshoot|Settling Time|Steady-State Error|
|---|---|---|---|---|
|KP|decrease|increase|small change|decrease|
|KI|decrease|increase|increase|eliminate|
|KP|small change|decrease|decrease|no change|

### Error Calculations

We calculate the position error by taking the weighted average of sensor readings.

$$
    e(t) = {\sum {r_i \times w_i} \over W}  
$$

where, 
$r_i =$ reading of $i$th sensor
$w_i =$ weight of $i$th sensor
$W = \sum w_i =$ total weight of all sensors 

#### TO DO

 - [ ] nested PID control structure using rotary encoders
 - [ ] odometry