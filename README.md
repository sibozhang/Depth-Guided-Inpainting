# Depth-Guided-Inpainting
This is code for "DVI: Depth Guided Video Inpainting for Autonomous Driving". ECCV 2020. [Project Page](https://sites.google.com/view/sibozhang/dvi) 

### Video Inpainting:
![](./inpainting.gif)

## Introduction

To get clear street-view and photo-realistic simulation in autonomous driving, we present an automatic video inpainting algorithm that can remove traffic agents from videos and synthesize missing regions with the guidance of depth/point cloud. By building a dense 3D map from stitched point clouds, frames within a video are geometrically correlated via this common 3D map. In order to fill a target inpainting area in a frame, it is straightforward to transform pixels from other frames into the current one with correct occlusion. Furthermore, we are able to fuse multiple videos through 3D point cloud registration, making it possible to inpaint a target video with multiple source videos. 

## Data preparation
Inpainting dataset consists of synchronized Labeled image and LiDAR scanned point clouds. It captured by HESAI Pandora All-in-One Sensing Kit. It is collected under various lighting conditions and traffic densities in Beijing, China.

Please download full data at [Apolloscape](http://apolloscape.auto/inpainting.html) or using link below. The first video inpainting dataset with depth. The synced lidar and image data also can be used for 3D perception and other tasks.

Sample data: 
[sample_mask_and_image.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2Fsample_mask_and_image.zip)
[sample_data.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2Fsample_data.zip)
[sample_lidar_bg.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2Fsample_lidar_bg.zip)
[sample_mask_and_image.zip](https://www.dropbox.com/s/e6w9qzrxyxe17x1/1534313590-1534313597_mask.zip?dl=0)
[sample_data.zip](https://www.dropbox.com/s/41el9iy9tzd955i/1534313590-1534313597.zip?dl=0)
[sample_lidar_bg.zip](https://www.dropbox.com/s/7ijiw6fjmfjz3yk/lidar_bg.zip?dl=0)
-->

[mask_and_image_0.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2F1534313570-1534313579_mask.zip)
[data_0.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2F1534313570-1534313579.zip)
[lidar_bg_0.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2F1534313570-1534313579_lidar_bg.zip)

[mask_and_image_1.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2F1534313592-1534313595_mask.zip)
[data_1.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2F1534313592-1534313595.zip)
[lidar_bg_1.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2F1534313592-1534313595_lidar_bg.zip)

[mask_and_image_2.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2F1534313648-1534313656_mask.zip)
[data_2.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2F1534313648-1534313656.zip)
[lidar_bg_2.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2F1534313648-1534313656_lidar_bg.zip)

[mask_and_image_3.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2F1534313856-1534313869_mask.zip)
[data_3.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2F1534313856-1534313869.zip)
[lidar_bg_3.zip](https://ad-apolloscape.cdn.bcebos.com/inpainting%2F1534313856-1534313869_lidar_bg.zip)


## Data Structure
The folder structure of the inpainting is as follows:

1) xxx-yyy_mask.zip: xxx.aaa.jpg is original image. xxx.aaa.png is labelled mask of cars. 

2) xxx-yyy.zip: Data includes ds_map.ply, global_poses.txt, rel_poses.txt, xxx.aaa_optR.xml. ds_map.ply is dense map build from lidar frames. 

3) lidar_bg.zip: lidar background point cloud in ply format.

## Data Prepareation

    catkin_ws
    ├── build
    ├── devel
    ├── src   
    code
    ├── libDAI-0.3.0
    ├── opengm
    data
    ├── pandora_liang
        ├── set2
              ├── 1534313590-1534313597
              ├── 1534313590-1534313597_mask
              ├── 1534313590-1534313597_results
              ├── lidar_bg
              ├── ...

    
## Set up
1. Install ROS Kinetic at http://wiki.ros.org/ROS/Installation

2. Install opengm
    
    download OpenGM 2.3.5 at http://hciweb2.iwr.uni-heidelberg.de/opengm/index.php?l0=library 

    or

    https://github.com/opengm/opengm for version 2.0.2

3. Build opengm with MRF:
    
    ```
    cd code/opengm
    mkdir build
    cd build 
    cmake -DWITH_MRF=ON ..
    make
    sudo make install
    ```
    
4. Make catkin:
    
    ```
    cd catkin_ws
    source devel/setup.bash
    catkin_make
    ```

## Evaluation

    cd catkin_ws 
    rosrun loam_velodyne videoInpaintingTexSynthFusion 1534313590 1534313597 1534313594 ../data/pandora_liang/set2

## How to label your own data and build 3D map 

1. Label Mask image of inpainting area

    ```
    python label_mask.py
    ```
    
    F: forward 
    S: undo
    D: backward

2. Build 3D map from lider frames
    
    ```
    LoamMapper.cpp
    ```
    
    Get global_pose.txt rel_pose.txt. Use meshlab to visualize ds_map.ply
    
## Citation
Please cite our paper in your publications if our dataset is used in your research.

DVI: Depth Guided Video Inpainting for Autonomous Driving.

Miao Liao, Feixiang Lu, Dingfu Zhou, Sibo Zhang, Wei Li, Ruigang Yang.  ECCV 2020. [PDF](https://arxiv.org/pdf/2007.08854.pdf), [Result Video](https://www.youtube.com/watch?v=iOIxdQIzjQs), [Presentation Video](https://www.youtube.com/watch?v=_pcqH1illCU&feature=youtu.be)

```
@article{liao2020dvi,
  title={DVI: Depth Guided Video Inpainting for Autonomous Driving},
  author={Liao, Miao and Lu, Feixiang and Zhou, Dingfu and Zhang, Sibo and Li, Wei and Yang, Ruigang},
  journal={arXiv preprint arXiv:2007.08854},
  year={2020}
}
```

ECCV 2020 Presentation Video

[![Depth Guided Video Inpainting for Autonomous Driving](https://res.cloudinary.com/marcomontalbano/image/upload/v1598335918/video_to_markdown/images/youtube--_pcqH1illCU-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=_pcqH1illCU&feature=youtu.be "Depth Guided Video Inpainting for Autonomous Driving")

Result Video

[![Depth Guided Video Inpainting for Autonomous Driving](https://res.cloudinary.com/marcomontalbano/image/upload/v1595308220/video_to_markdown/images/youtube--iOIxdQIzjQs-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=iOIxdQIzjQs "Depth Guided Video Inpainting for Autonomous Driving")

## Q & A
Get MRF-LIB working within opengm2:

    ~/code/opengm/build$ cmake -DWITH_MRF=ON ..  #turn on MRF option within opengm cmake
    ~/code/opengm/src/external/patches/MRF$ ./patchMRF-v2.1.sh
    
    Change to:
    TRWS_URL=https://download.microsoft.com/download/6/E/D/6ED0E6CF-C06E-4D4E-9F70-C5932795CC12/
    Within patchMRF-v2.1.sh


