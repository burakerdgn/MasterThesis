# MasterThesis
**Object Detection and Tracking Systems with Stereo Vision for Autonomous Driving**


## Abstract
- Obtaining disparity image in order to obtain depth information of the each pixel of the image
- Changing one of the image channels with disparity image
- Observing the effect of the change on object tracking algorithms( CSRT , MOSSE etc.)

## Software Tools
- Ubuntu 16
- [Virtual Box](https://www.virtualbox.org/)
- [OpenCV](https://opencv.org/)
- [CMake](https://cmake.org/)

## Programming Language
- C ++

Disparity Image

![disp2](https://user-images.githubusercontent.com/42723084/120527898-2a4ee600-c3db-11eb-9fc6-e276ee50df4c.png)

 Unfiltered Bounding Boxes
 
 ![unfiltered](https://user-images.githubusercontent.com/42723084/120527568-c3c9c800-c3da-11eb-8e6a-b3e0f2eb3f48.png)
 
 
 Filtered Bounding Boxes
 
![sizefiltered](https://user-images.githubusercontent.com/42723084/120527389-93822980-c3da-11eb-8d84-d05e8c40be1a.png)

The image having a disparity image as third channel

![withdepth](https://user-images.githubusercontent.com/42723084/120527659-e1972d00-c3da-11eb-9b48-cc2481428453.png)



 3D represantation of the objects at ROS Rviz environment
![Screenshot from 2019-07-29 21-29-18](https://user-images.githubusercontent.com/42723084/120527204-5ae25000-c3da-11eb-9c22-50901e6f5cb0.png)
