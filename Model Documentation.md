# Model Documentation
This documentation describes the overall algorithm of this project.

The whole procedure consists of two big steps:

1. Decision Making
2. Path Generation

Although the decision making part is the first part in the code, the path generation part will be described first since it is the baseline code.

[//]: # (Image References)

[video1]: ./fullvideo.mp4 "fullvideo"
[video2]: ./highlight.mp4 "highlight"

## Path Generation

Path is generated for 50 time steps, using spline interpolation.

### Waypoint Settings

In the path generation step, we use spline interpolation with setting proper waypoints. The selected waypoints are:

* Last position and second last position of the previous remaining path data (which has length 45~49 time steps). This is to provide tangency with the endpoint of the previous trajectory.
* 3 points far away from the vehicle: In Frenet coordinate, `(car_s+40, 2+4*lane)`, `(car_s+80, 2+4*lane)`, `(car_s+120, 2+4*lane)` where `car_s` is ego vehicle's current s coordinate and `lane` is target lane.

### Spline Interpolation

Before we execute spline interpolation to the waypoints, we execute coordinate transform with respect to endpoint of the previous trajectory.
This is since spline library provided from the project is only available when the points given in ascending order in x-direction.

Then, we execute spline interpolation by creating spline object.

### Final Path Update (concatenation)

Finally, starting from the endpoint of the previous trajectory, we concatenate the created spline trajectory considering the vehicle's reference speed.


## Decision Making

In the decision making step, we decide the ego vehicle's longitudinal speed control target, cruise control behavior, and the lane change behavior.

### Longitudinal Speed Control

Once the speed control target is decided, this part of code automatically controls the vehicle's speed to reach the target speed. 
(Namely, generates input for the trajectory which corresponds to the target speed.)
Since the maximum acceleration is restricted under 10m/s^2, this code limits the maximum rate of longitudinal speed change to 10m/s^2.

If there is no other restriction on the target speed, the default target speed is always 45mph which is slightly lower than the speed limit 50mph.

### Cruise Control

When a car is expected to be in front of the ego vehicle with certain distance and it is on the ego vehicle's lane, this part of the code changes the target velocity to be same with the front car.
This state of the ego vehicle is called cruise control mode.
The frontal area for the cruise control mode is set using Frenet coordinate as `[car_s car_s+(50 meters)]` when `car_s` is the current s coordinate of the ego vehicle.
The ego vehicle gets into cruise control mode if the front car's future s coordinate is inside the frontal area,
and the point where this criteria effects is when the remaining path data of the ego vehicle ends.

When the vehicle is in the cruise control mode, the `cruise_flag` is set and waits for the lane change manuever.

### Lane Change

When the `cruise_flag` is set, the code gets into the lane change decision part.

First, we look for which lane has enough space to get in.
With the same longitudinal area of the previous step, `[car_s car_s+(50 meters)]` in Frenet coordinate, we search for any future car within this range.
If the future s coordinate of a vehicle is in this range, we check the vehicle's lane and mark that the corresponding lane is not empty.

Next, we decide whether or not to change lane.
The lane change in this code is only available to the adjacent lane.
If the adjacent lane is marked as empty, then this part of the code changes the target lane.
Also, to redesign the path appropriate to the lane change so that lane change occur right away,
we use only first 10 time steps of the previous trajectory so that the path generation part can automatically generate trajectory with corrected targets.

## Result Video

Full simulation video: [Link](./fullvideo.mp4)

Highlight: [Link](./highlight.mp4)