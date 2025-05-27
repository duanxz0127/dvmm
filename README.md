# dvmm
The code will be released upon the acceptance of our paper 
> **DVMM: A Dual-View Combination Descriptor for Multi-Modal LiDARs Online Place Recognition**

![Figure_10](https://github.com/user-attachments/assets/5513adfc-1af1-412b-ba84-52ba450c9457)

## Introduction
DVMM is an online place recognition method for multi-modal LiDARs. This method introduces a dual-view combination descriptor, termed DVMM, by separately encoding radial and vertical scene information. The place recognition process consists of two stages: loop closure detection and verification. In the detection stage, point clouds are projected onto an adaptive grid and a 1D radial descriptor is generated via Gaussianweighted column summation. The radial descriptor is utilized to retrieve potential loop candidates through vector matching. In the verification stage, point clouds within a fixed height range are encoded as a binary occupancy image, which serves as the cross-section descriptor. Accurate loop closures are determined by performing image matching on the cross-section descriptors.
