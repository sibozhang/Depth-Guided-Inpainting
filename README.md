# Depth-Guided-Inpainting
This is Code for "DVI: Depth Guided Video Inpainting for Autonomous Driving"
Miao Liao, Feixiang Lu, Dingfu Zhou, Sibo Zhang, Wei Li, Ruigang Yang.  ECCV 2020.

## Introduction

To get clear street-view and photo-realistic simulation in autonomous driving, we present an automatic video inpainting algorithm that can remove traffic agents from videos and synthesize missing regions with the guidance of depth/point cloud. By building a dense 3D map from stitched point clouds, frames within a video are geometrically correlated via this common 3D map. In order to fill a target inpainting area in a frame, it is straightforward to transform pixels from other frames into the current one with correct occlusion. Furthermore, we are able to fuse multiple videos through 3D point cloud registration, making it possible to inpaint a target video with multiple source videos. 

## Data preparation
Please download data at http://apolloscape.auto/

## Set up

## Evaluation

## Citation
Please cite our paper in your publications if our dataset is used in your research.

DVI: Depth Guided Video Inpainting for Autonomous Driving.

Miao Liao, Feixiang Lu, Dingfu Zhou, Sibo Zhang, Wei Li, Ruigang Yang.  ECCV 2020. [PDF](https://arxiv.org/pdf/2007.08854.pdf), [Video](https://www.youtube.com/watch?v=iOIxdQIzjQs)

```
@misc{liao2020dvi,
    title={DVI: Depth Guided Video Inpainting for Autonomous Driving},
    author={Miao Liao and Feixiang Lu and Dingfu Zhou and Sibo Zhang and Wei Li and Ruigang Yang},
    year={2020},
    eprint={2007.08854},
    archivePrefix={arXiv},
    primaryClass={cs.CV}
}
```

[![Depth Guided Video Inpainting for Autonomous Driving](https://res.cloudinary.com/marcomontalbano/image/upload/v1595308220/video_to_markdown/images/youtube--iOIxdQIzjQs-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=iOIxdQIzjQs "Depth Guided Video Inpainting for Autonomous Driving")
