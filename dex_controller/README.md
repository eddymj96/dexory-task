# dex_controller
This package contains dexory controller

## Requirements

## Components

## Build Instructions

### Docker
    build docker file:


### Other
Install conan:

create default conan profile

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source i
install/setup.bash

## Tests
- unit tests (dex controller class not well tested)
- How to build tests
- How to run tests
- manual verification of differernt controllers, not sufficient - need to test with reproducable initial and target poses
- aware that linter errors exist - this needs to be fixed but can be done so easily
- Did not play around with controller parameters much


## Known issues

 - linter errors in tests
 - yaw controller issue
 - still not great performance

## Configuration
                                                                                                                        |
 | ---------------------      | ------ | -------------------------------------------------------------------------------------------------------- |
 | yaw_kp                     | double | Default: DiffDrive. Type of model [DiffDrive, Omni, Ackermann].                                          |
 | critics                    | string | Default: None. Critics (plugins) names                                                                   |
 | iteration_count            | int    | Default 1. Iteration count in MPPI algorithm. Recommend to keep as 1 and prefer more batches.            |
 | batch_size                 | int    | Default 1000. Count of randomly sampled candidate trajectories                                            |
 | time_steps                 | int    | Default 56. Number of time steps (points) in each sampled trajectory                                     |
 | model_dt                   | double | Default: 0.05. Time interval (s) between two sampled points in trajectories.                              |
 | vx_std                     | double | Default 0.2. Sampling standard deviation for VX                                                          |
 | vy_std                     | double | Default 0.2. Sampling standard deviation for VY                                                          |
 | wz_std                     | double | Default 0.4. Sampling standard deviation for Wz                                                          |
 | vx_max                     | double | Default 0.5. Max VX (m/s)                                                                                |
 | vy_max                     | double | Default 0.5. Max VY in either direction, if holonomic. (m/s)                                             |
 | vx_min                     | double | Default -0.35. Min VX (m/s)                                                                              |
 | wz_max                     | double | Default 1.9. Max WZ (rad/s)                                                                              |
 | temperature                | double | Default: 0.3. Selectiveness of trajectories by their costs (The closer this value to 0, the "more" we take in consideration controls with less cost), 0 mean use control with best cost, huge value will lead to just taking mean of all trajectories without cost consideration                                                   |
 | gamma                      | double | Default: 0.015. A trade-off between smoothness (high) and low energy (low). This is a complex parameter that likely won't need to be changed from the default of `0.1` which works well for a broad range of cases. See Section 3D-2 in "Information Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving" for detailed information.       |
 | visualize                  | bool   | Default: false. Publish visualization of trajectories, which can slow down the controller significantly. Use only for debugging.                                                                                                                                       |
 | retry_attempt_limit        | int    | Default 1. Number of attempts to find feasible trajectory on failure for soft-resets before reporting failure.

 ## Discussion
- Very simple and trivial solution that doesn't show my skills
- Should have used the shim controller, but had linker issues ahd I spen tlonger - would have debugged
- Tried the DWB controller it also seemed to work well, but decided to use the regulated pursuit controller instead - would need to do more comprehenisve tests
- Wasn't familiar with the ROS2 nav2 stack
- To make it *good* would need likely fork one of the controllers (likely MPC)

