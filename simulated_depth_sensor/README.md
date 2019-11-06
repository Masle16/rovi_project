# Simulated depth sensor

## Task description

A method that gets a point cloud from the simulated depth sensor and then computes the object pose based that.

The pipeline here would typically be (you decide on which steps to do or not to do and which concrete choices to make here):

- Get point cloud
- Filter/Segment point cloud
- Use global pose estimation to find the object pose
- Use local pose estimation to refine the object pose
