# Point Clouds of Different Objects

This folder contains point clouds of the different objects, which is located in the RobWorkStudio scene. Furthermore, the .ply file, which are created when converting to .pcd, is stored also. 

The .stl files of the parts are located at [parts location](Project_WorkCell/parts/).

## Converting the .stl file into PCDs

To convert the .stl files into PCDs [Meshlab](http://www.meshlab.net/) is used. The .stl file is loaded into Meshlab and then the filter Stratified Triangle Sampling with a million sampling points is used. This filter randomly samples the mesh surface. Afterwards, the .stl file is saved as .ply in Meshlab.

The .ply file is converted to .pcd by using commands from pcl in the terminal. The process is as follows:

```
pcl_ply2pcd input.ply output.pcd
pcl_voxel_grid input.pcd output.pcd -leaf 0.001 0.001 0.001
```

The commands above firstly converts the .ply file to .pcd file. Secondly, a voxel grid of the size (1mm, 1mm, 1mm) is applied to the point cloud to get a uniform and fixed resolution of the points.
