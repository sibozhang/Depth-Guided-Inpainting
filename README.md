# Depth-Guided-Inpainting
This is code for "DVI: Depth Guided Video Inpainting for Autonomous Driving".

Miao Liao, Feixiang Lu, Dingfu Zhou, Sibo Zhang, Wei Li, Ruigang Yang.  ECCV 2020.

## Introduction

To get clear street-view and photo-realistic simulation in autonomous driving, we present an automatic video inpainting algorithm that can remove traffic agents from videos and synthesize missing regions with the guidance of depth/point cloud. By building a dense 3D map from stitched point clouds, frames within a video are geometrically correlated via this common 3D map. In order to fill a target inpainting area in a frame, it is straightforward to transform pixels from other frames into the current one with correct occlusion. Furthermore, we are able to fuse multiple videos through 3D point cloud registration, making it possible to inpaint a target video with multiple source videos. 

## Data preparation
Please download data at [Apolloscape](http://apolloscape.auto/inpainting.html). The first video inpainting dataset with depth. 

Sample data: 

[sample_mask_and_image.zip](https://www.dropbox.com/s/6arza9ee72slr69/1534313590-1534313597_mask.zip?dl=0)

[sample_data.zip](https://www.dropbox.com/s/lm1rxhdfqor68z6/1534313590-1534313597.zip?dl=0)

[sample_lidar_bg.zip](https://www.dropbox.com/s/e8km9r0ginxuuak/1534313590-1534313597_lidar_bg.zip?dl=0)


## Data Structure
The folder structure of the inpainting is as follows:

1) xxx-yyy_mask.zip: xxx.aaa.jpg is original image. xxx.aaa.png is labelled mask of cars. 

2) xxx-yyy.zip: Data includes ds_map.ply, global_poses.txt, rel_poses.txt, xxx.aaa_optR.xml. ds_map.ply is dense map build from lidar frames. 

3) xxx-yyy_lidar_bg.zip: lidar background point cloud in ply format.

## Set up

## Evaluation

## Citation
Please cite our paper in your publications if our dataset is used in your research.

DVI: Depth Guided Video Inpainting for Autonomous Driving.

Miao Liao, Feixiang Lu, Dingfu Zhou, Sibo Zhang, Wei Li, Ruigang Yang.  ECCV 2020. [PDF](https://arxiv.org/pdf/2007.08854.pdf), [Video](https://www.youtube.com/watch?v=iOIxdQIzjQs)

```
@article{liao2020dvi,
  title={DVI: Depth Guided Video Inpainting for Autonomous Driving},
  author={Liao, Miao and Lu, Feixiang and Zhou, Dingfu and Zhang, Sibo and Li, Wei and Yang, Ruigang},
  journal={arXiv preprint arXiv:2007.08854},
  year={2020}
}
```

[![Depth Guided Video Inpainting for Autonomous Driving](https://res.cloudinary.com/marcomontalbano/image/upload/v1595308220/video_to_markdown/images/youtube--iOIxdQIzjQs-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=iOIxdQIzjQs "Depth Guided Video Inpainting for Autonomous Driving")
