# ROS Node for Image Stitching
ROS Node for Realtime Image Stitching for imags obtained from two different unsynnchronized/synnchronized cameras.

## Usage
```
rosrun stitch_image stitch_image _left:=/camera_left/image_color _right:=/camera_right/image_color
```

## Methodology for Image Stitching
2. Find SIFT Descriptors for both the streams
3. Find matching keypoints using Flann Matcher
4. Computer Homography Trasformation
5. Use the computer homography to stitch the images

_The steps 1-3 are only performed for the first frames. Once the homography matrix has been calculated, we only use the calculated homography to stich the images together. For best result, rectified undistorted images should be used._


## Todo
- [ ] Calculate homography on demand
- [ ] Provide homography as input

## Cite
```
@book{binnani2019vision,
  title={Vision-based Autonomous Driving},
  author={Binnani, Sumit},
  year={2019},
  publisher={University of California, San Diego}
}
```
