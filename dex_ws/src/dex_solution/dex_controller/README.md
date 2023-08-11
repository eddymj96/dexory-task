# dex_controller
## Features

**Path Following**: This is achieved by simply implementing delegating the logic `nav2 regulated pure pursuit` controller, experimentation with the DWB controller was also performed but without cpmprehensive testing and parameter tweaking it is difficult to choose one over the other.

**Rotate in-place start**: Naturally, implementing the shim-controller here would seem quite obvious; however, due to some linker issues and a small desire to write some more custom code myself, I have implemented a simple PID controller that orientates itself to the trajectory before passing behaviour off to the regulated pure pursuit controller. The desired orientation could simply be the heading of the vector from the first waypoint to the next. In an effort to make something *slightly* more interesting, the controller can be set to take `N` (where `N` <=`path.size()`) number of points from the start of the trajectory. Some pricniple component analysis is then done to estimate an `average` vector from these points and thus a direction from which to begin. 

**Rotate in-place end**: The regulated pure pursuit controller, which this controller wraps around, can be set to rotate at the end of its trajectory simply by setting the option in the `.yaml` file.

**Planner improvement**: The planned path always delivered poses with defualt constructed orientations (all yaws were set to 0) so a very simple planner improvement was made by simply setting the orientation goals of every pose to the be the heading to the subsequent waypoint. It is unclear whether this had much impact on the performance of the pure pursuit controller (I would have to spend more time to see what it does with the planner points under the hood).

**Obstacle Detection**
The pure pursuit controller comes an option to parse the costmap and deal with obstacles already. So this was simply set as a flag in the controller parameters.

**Goal Checker**
Similar as above the `nav2` stack comes with a goal and progress checker plugin, these were added via the controller parameters.


## Components

The Dex Controller the Nav2 reguated 

## Build Instructions

### Building the package

Ensure you have built the latest docker image in `ade` directory:


Go to the workspace directory `dex_ws`, to build:

```console
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

One may need to create a default conan profile:

```console
conan profile new default --detect
conan profile update settings.compiler.libcxx=libstdc++11 default
```

After it has been built, Eensure you source the package:
```
source install/setup.bash
```


### Conan

The conan has been used to for importing Eigen, conan can be called directly from within a CMakeLists.txt for seamless integration. Thus, as a user all you need is to install it (*Note:* Ensure that it is version 1.59.0 as Conan 2 has been released but still lacks support for many packages)


### Docker
The dockerfile has been updated with Conan 1.59.0 and Doxygen, so ensure it has been rebuilt/up-to-date

# Documentation

Doxygen comments have been added to the header files; however, it doesn't provide too much more insight as the code and parameters are (hopefully) rather self evident.

To build, go to the `dexory_controller` directory:

```console
doxygen Doxyfile
```

Given more time, the use of Sphinx would be a much nicer way of presenting this information and would be consistent with the current `nav2` documentation online

## Tests

### Run tests

In the workspace directory:

```console
colcon test --packages-select dex_controller
```

View output:

```console
colcon test-result --all
```

### Unit tests
There are two classes that are unit tested. `SimplePid` and `DexController`. The Dex Controller tests fail to complete due to an unsolved bug with the regulated pursuit controller component, it has been marked as an issuein the `Known Issues` section below.


### Manual tests
To ensure the functional requirements, manual tests were done of the controller around the map, setting different Nav2 goals and attempting to follow the appropriate paths. Manual verifiation in this manner is okay for simple, hueristic understanding of the behaviour but is not adequately sufficient for behaviour analysis. Needs to be tested with a variety of reproducable initial and target poses such that direct comparisons can be made.


## Known issues
All the issues here remain issues due to a lack of time and all seem tractable with more time
 - Linter errors and failed tests, this can be fixed easily
 - Error in the `dex_controller_test.cpp` test, needs more investigation
 - For the same reason as the `dex_controller_test.cpp` fails, so does the launch file
 - The principle component analysis and orientation controller does not robustly align with the start of the trajectory 
 - Still not great performance from the controller still gets "stuck" on obstacles at times
 - Global plan orientations don't update in RVIZ (all have 0 yaw)

## Configuration

The parameters shown are those unique to the `DexController` to alter parameters for it's components (regulated pure pursuit controller, simple goal checker etc) please find the relvent documentation on the `nav2` website.

| Parameter            | Type   | Description                                                                                             |
|----------------------|--------|---------------------------------------------------------------------------------------------------------|
| yaw_kp               | double | Default: 1.4. Proportional control gain                                                                 |
| yaw_ki               | double | Default: 0.001. Integral control gain                                                                   |
| yaw_kd               | double | Default: 0.01. Derivative control gain                                                                  |
| yaw_error_threshold  | double | Default: 0.05. Orientation error threshold for controller of initial alignment to path                   |
| pca_point_no         | int    | Default: 5. Max number of initial trajectory points for principle component analysis                    |

