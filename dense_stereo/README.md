# Dense stereo

## Task description

A method that gets images from the left and the right camera, computes
disparity information and then computes the object pose based on that.
The pipeline here would typically be (you decide on which steps to do or not to do and
which concrete choices to make here):

- Get images
- Rectify and undistort images (if necessary)
- Compute the disparity map
- Compute point cloud
- Filter/Segment point cloud
- Use global pose estimation to fine the object pose
- Use local pose estimation to refine the object pose