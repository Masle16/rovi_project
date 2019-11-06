# Image based

## Task description

A method that gets an image from the camera and then computes the
object pose based on that.
The pipeline here would typically be (you decide on which steps to do or not to do and
which concrete choices to make here):

– Get image
– Undistort images (if necessary)
– Use the image based pose estimation approach to find the object pose