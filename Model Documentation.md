# Model Documentation
This documentation describes the overall algorithm of this project.

The whole procedure consists of two big steps:

1. Decision Making
2. Path Generation

## Decision Making

In the decision making step, we decide the ego vehicle's longitudinal speed control target, cruise control behavior, and the lane change behavior.

### Longitudinal Speed Control

Once the speed control target is decided, this part of code automatically controls the vehicle's speed to reach the target speed. 
(Namely, generates input for the trajectory which corresponds to the target speed.)
Since the maximum acceleration is restricted under 10m/s^2, this code limits the maximum rate of longitudinal speed change to 10m/s^2.

### Cruise Control

When a car is expected to be in front of the ego vehicle with certain distance and it is on the ego vehicle's lane, this part of the code changes the target velocity to be same with the front car.
This state of the ego vehicle is called cruise control mode.
The frontal area for the cruise control mode is set using Frenet coordinate as `[car_s car_s+(50 meters)]` when `car_s` is the current s coordinate of the ego vehicle.
The ego vehicle gets into cruise control mode if the front car's future s coordinate is inside the frontal area,
and the point where this criteria effects is when the remaining path data of the ego vehicle ends.

When the vehicle is in the cruise control mode, the `cruise_flag` is set and waits for the lane change manuever.

### Lane Change

First, we look for which lane is 