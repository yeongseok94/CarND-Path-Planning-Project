# Model Documentation
This documentation describes the overall algorithm of this project.

The whole procedure consists of two big steps:

1. Decision Making
2. Path Generation

## Decision Making

In the decision making step, we decide the ego vehicle's longitudinal speed control target, cruise control behavior, and the lane change behavior.

### Longitudinal Speed Control

Once the speed control target is decided, this part of code automatically controls the vehicle's speed (namely, generates the slow trajectory).
Since the maximum acceleration is restricted under 10m/s^2, 