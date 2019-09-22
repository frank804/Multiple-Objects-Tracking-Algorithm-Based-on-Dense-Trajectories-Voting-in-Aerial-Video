## Dependencies
The code is compatible with vs2015 Enterprise. The following dependencies are needed to run the tracker:
opencv3.4.1(GPU vision)

CUDA 9.1 

CUDNN　　

Intel i5-8400 CPU

8 GB RAM

Nvidia GeForce GTX1080Ti GPU

## Test Data
The test video can be downloaded on BaiDu Cloud Disk [here](https://pan.baidu.com/s/1wZUUkpHGpBks6QYFKUVSFg?fid=1052882489402014), and the detection result of the video is stored in detect_result.txt of the folder video. The test video has a resolution of 1920*1080.In the txt file, each row stores all the detection bounding box information of each frame, and the storage format is: 　　　
< frame number >  <x>  <y>  <w>  <h>  <x>  <y>  <w>  <h>…　　　
#Where (x, y) is the coordinates of the upper left corner of the detection bounding box, w and h are the width and height of the detection bounding box.
## Note
The uploaded code defaults to the CPU version of the proposed tracking system. To use GPU acceleration, uncomment the USE_GPU macro in Dense-Trajectories-Generation.h.
