# dvmm
The code will be released upon the acceptance of our paper 
> **DVMM: A Dual-View Combination Descriptor for Multi-Modal LiDARs Online Place Recognition**

![Figure_10](figure/DVMM_in_DCLSLAM.jpg)

## Introduction
DVMM is an online place recognition method for multi-modal LiDARs. This method introduces a dual-view combination descriptor, termed DVMM, by separately encoding azimuthal and vertical scene information. The place recognition process consists of two stages: loop closure detection and verification. In the detection stage, point clouds are projected onto an adaptive grid and a 1D azimuthal descriptor is generated via Gaussian weighted column summation. The azimuthal descriptor is utilized to retrieve potential loop candidates through vector matching. In the verification stage, point clouds within a fixed height range are encoded as a binary occupancy image, which serves as the cross-section descriptor. Accurate loop closures are determined by performing image matching on the cross-section descriptors.

## How to use
The code was tested on Ubuntu 18.04
### 1. Pre-requisite
   * [CMake](https://cmake.org/) version 3.10 or above
   * [PCL](https://github.com/PointCloudLibrary/pcl) version 1.8.0 (other versions may also work)
   * [OpenCV](https://opencv.org/) version 3.2.0 (other versions may also work)
   * [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) version 3.3.4 (other versions may also work)
   * [Boost](https://github.com/boostorg/boost) version 1.65 or above
### 2. Compile
Clone this repository and build

```
git clone https://github.com/duanxz0127/dvmm.git
cd dvmm
mkdir build
cd build
cmake ..
make
```

### 3. Run
```
cd ..
sh script/run.sh
```


## Citation
<!-- This work is published in XXX, and please cite related papers: -->

## Acknowledgement
We sincerely thank the two open source efforts [SOLiD](https://github.com/sparolab/SOLiD) and [DCL-SLAM](https://github.com/zhongshp/DCL-SLAM) for their contributions to the community.