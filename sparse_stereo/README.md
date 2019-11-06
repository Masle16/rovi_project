# Sparse stereo

## Task description

A method that gets images from the left and the right camera, computes
a relevant feature in the two images, triangulates these features and then computes the
object pose based on that.
The pipeline here would typically be (you decide on which steps to do or not to do and
which concrete choices to make here):
– Get images
– Rectify and undistort images (if necessary)
– Find the relevant features in the two images (e.g., find the centre of the red ball,
find the corners of the square, ...)
– Match up the features from the left and the right image
– Triangulate the features
– Use the 3D feature locations to find the object pose (e.g., 3D centre of the ball,
position and orientation of the square plane, ...)