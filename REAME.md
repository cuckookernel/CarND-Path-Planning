

## Overview 

After trying several ideas, including quintic polynomials for trajectory generation, and not getting quite the 
desired behaviour, I ended up following the approach shown in the Project Q&A video. 

In short, this approach can be summarized as: 
1. Plan trajectories that go from the current car's location to the center of some lane and some distance ahead. A few points are generated in Frenet coordinates and converted to XY global coordinates.
 These are then converted to XY coordinates in the car's local coordinate system. Spline interpolation is applied to 
 generate points that are close together. These are converted back to XY global coordinates and emitted. 
2. At every generation step most points from the previous path are used and only a few new ones are generated. 
3. The target lane for the generated trajectory is chosen carefully so as to avoid collisions. 
4. The speed is adjusted gradually at every generation step towards a reference speed. The speed increment is such that the acceleration constraint is never violated. 

## Rubric points coverage

### Code compiles

Sure thing! Just try it! I would never submit code that doesn't compile... I am not a monster.

### Car is able to drive at least 4.32 miles without incident

I did a couple of tries and it worked! This is a result of the following points. 

### The car drives according to the speed limit 

This is guaranteed by defining `MAX_SPEED` in `constants.h` to be the equivalent of 45 mph and by only 
increasing the speed if it is below `MAX_SPEED` (See lines 34 - 38 of `main.cpp`)

### Max acceleration and jerk and not exceeded 

This is guaranteed by: 

  - Smoothly extending the previous path by using the last few points in it as seeds for the trajectory generated at every cycle. 
  ( see lines 203 - 218 in `traj_gen.h` )  
  - Using a speed increment `AARONS_ACCEL` defined in `constants.h` that is small enough so as to avoid acceleration and jerk violation. We had to set this to about 50% of what was shown in the video.
  - When planning for a lane change, actually use a reference distance  `LANE_CHANGE_DISTANCE` (defined in `constants.h`) of 60 m, twice what was used in the Q&A video.
  
### Car does not have collisions 

Function `evaluate_lane_change_v1` (`helpers.h`, line 252) called from `decide_on_action` (`main.cpp`, line 21) evaluates a cost function on each possible target lane that we can switch to -- only lanes that are next to the current lane are considered. The cost of any lane (computed by function  `lane_cost` in `helpers.h`, line 221) is set to infinity if a collision is deemed likely, see conditions checked  lines 234 and 235. 

If we decide not to change lane, then function `maybe_break_because_car_in_front` (implemented in `helpers.h`, line 175) which is also called from function  `decide_on_action` (`main.cpp`, line 21) returns true if the car in front is going to slow. This causes us to brake. 

### The car stays in its lane, except for the time between changing lanes.

When not changing lanes we trace a path that is essentially a straight line with constant `d` in Frenet coordinates.
The value of `d` corresponds to the center of the current lane which is also the target lane. As we always add 
a few points in the far distance which withe the same `d` coordinate that guarantees that the trajectory towards the end 
will be parallel to the lane lines and pass through the center of the lanes. After converting to XY coordinates
the resulting trajectory will probably be curved (and smooth thanks to the spline), but it will still be contained within the lane as the transformation between Frenet and XY coordinates, is not too crazy. If this weren't the case (if there where very sharp curves or corners in the lanes), we could have a problem, but luckily this doesn't happen in the very cool first-world kind of highway that this project uses. 

### The car is able to change lanes 

Function `decide_on_action` (`main.cpp`, line 21) calls `gen_trajectory_aaron_style` (`traj_gen.h`, line 242)  to generate/extend the trajectory with any target lane. A lane change is specified by setting `state.lane` to a lane that is different from the current lane and that is also considered safe (cost < infinity)  and convenient: cost < cost of current lane. The cost of new lane is smaller than current lane if the next car's ahead speed is higher than in the current lane.


## Reflection on path generation: 

Path generation follows the steps outline at the beginning of this document. 
Here we give more specific details and references to the code:

The main function for trajectory generation is `gen_trajectory_aaron_style` (traj_gen.h, line 242).

This function can be logically divided into several sections: 

- Re-use all 'unconsumed' points in the previous path (lines 247 - 249)
- Call  subroutine `draft_trajectory_local_aaron` (lines 197-240) which produces 5 points in the car's local XY coordinate system. 
    - These points are set to the last two points of the previous path (lines 203-218) plus three points far ahead (lines 225 - 227), situated on the center of the target lane but at multiples of some distance reference distance (chosen according to whether we want to change lane or not) 
    - Convert the points to the local car's coordinate system. (line 232)
- Use spline to generate, from the 5 points just obtained, a denser set of points taken from the beginning of the path. The spacing between the points is set in such a way that the the reference speed is approximately realized along the path (lines 269-272). These are immediately converted back to globak XY coordinates (line 274).

 