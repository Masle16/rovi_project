# Robotics and Vision, RoVi, Project

When running RobWorks plugins please be in the default folder(this one).

## Robotics

### Reachability Analysis

- The reachability analysis is located in [reachability_analysis](reachability_analysis).
- The code is contained in [reachability_analysis/src/main.cpp](reachability_analysis/src/main.cpp).
- The analysis of the grasping by the side in [reachability_analysis/cylinder_side/collision_free_solutions_side.m](reachability_analysis/cylinder_side/collision_free_solutions_side.m).
- The grasping by the top [reachability_analysis/cylinder_top/collision_free_solutions_top.m](reachability_analysis/cylinder_top/collision_free_solutions_top.m).
- The figures for the report [reachability_analysis/figures](reachability_analysis/figures).
- Further description can be found in the [reachability_analysis/README.md](reachability_analysis/README.md).

### Robot Motion Planing

**Point to Point Interpolation & Point to Point Interpolation with Parabolic Blend**

- The point to point interpolation with and without parabolic blend is located in [p2p_interpolation](p2p_interpolation).
- The code is located in [p2p_interpolation/main.cpp](p2p_interpolation/main.cpp), [p2p_interpolation/p2p_interpolation.cpp](p2p_interpolation/p2p_interpolation.cpp) and [p2p_interpolation/p2p_interpolation.h](p2p_interpolation/p2p_interpolation.h).
- The workcell used is [p2p_interpolation/Project_WorkCell/Scene.wc.xml](p2p_interpolation/Project_WorkCell/Scene.wc.xml).
The dataprocessing is placed in [p2p_interpolation/interpolation_results.m](p2p_interpolation/interpolation_results.m).

**Rapidly Random Trees Connect (RRT Connect)**

- The RRT-Connect method is located in [rrt_connect](rrt_connect).
- The code is located in [rrt_connect/src/main.cpp](rrt_connect/src/main.cpp).
- The workcell for can be found in [rrt_connect/workcell](rrt_connect/workcell).
- The dataprocessing is in [rrt_connect/analysis.m](rrt_connect/analysis.m).
- The figures for the report are in [rrt_connect/figures](rrt_connect/figures).

## Vision

**Method 2: Simulated Depth Sensor**

- The simulated depth sensor method is located in [simulated_depth_sensor](simulated_depth_sensor).
- The code is contained in [simulated_depth_sensor/sample_plugin/src](simulated_depth_sensor/sample_plugin/src). The code for the simulated depth sensor method is in alignment.hpp and other relevant functions are kept in util.hpp.
- The workcell used can be found in [simulated_depth_sensor/workcell](simulated_depth_sensor/workcell).
- The dataprocessing is located in [simulated_depth_sensor/analysis_of_results.m](simulated_depth_sensor/analysis_of_results.m).
- The figures for the report are in [simulated_depth_sensor/figures](simulated_depth_sensor/figures).

To run the sample plugin in RobWorks:

- Open up RobWorks in this location.
- Load the scene --> [simulated_depth_sensor/workcell/Scene.wc.xml](simulated_depth_sensor/workcell/Scene.wc.xml).
- Load the plugin --> [simulated_depth_sensor/sample_plugin/libs/Release/libsimulated_depth_sensor_plugin.so](simulated_depth_sensor/sample_plugin/libs/Release/libsimulated_depth_sensor_plugin.so).

**Method 3: Sparse Stereo**

- The sparse stereo method is located in [sparse_stereo](sparse_stereo).
- The code for the sparse stereo method is placed in [sparse_stereo/sample_plugin/src](sparse_stereo/sample_plugin/src).
- The workcell used for sparse stereo is at [sparse_stereo/workcell/Scene.wc.xml](sparse_stereo/workcell/Scene.wc.xml).

## Combination

- The combination of the robotics and vision is located in [combination](combination).
- The code for the combination is located in [combination/sample_plugin/src](combination/sample_plugin/src). The vision.hpp contains code for the method 3: simulated depth sensor. The robotics.hpp contains the code for the point to point interpolation with parabolic blend. The util.hpp contains other relevant functions.
- The workcell for the combination is located in [combination/workcell](combination/workcell).
- The figures for the report are in [combination/figures](combination/figures).
- A video showing the combination is at [combination/combination_video.webm](combination/combination_video.webm).

To executed the generated sample plugin for RobWorks:

- Open up RobWorks in this location.
- Load the scene --> [combination/workcell/Scene.wc.xml](combination/workcell/Scene.wc.xml).
- Load the plugin --> [combination/sample_plugin/libs/Release/libcombination_plugin.so](combination/sample_plugin/libs/Release/libcombination_plugin.so)

When the sample plugin is loaded the combination be performed by first pressing the button: "Estimate object pose". Close the pcl_viewer window to continue. Then press the button: "Move robot to object". Afterwards, press the button: "Move object to goal". Then press the button: "Reset workcell" to reset. The button: "Move object random" generates a random pose for the object.
