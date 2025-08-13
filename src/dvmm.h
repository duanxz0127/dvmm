#pragma once
#ifndef DVMM_H
#define DVMM_H

#include <omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

typedef pcl::PointXYZI PointType;
const float MAX_VALUE = 99999.999;

struct DVMMInfo
{
    cv::Mat azimuthal;
    cv::Mat crosssection;
    int non_zero_count;
    int left_zero_count;
    float lidar_height;
};

struct DVMMCandidate
{
    DVMMInfo descriptor;
    int yaw_index_offset;
    int window_nonzeros;
    float similarity_score;
    int index_in_dataset;
};

struct DVMMResult
{
    int index;
    float difference;
    float delta_x;
    float delta_y;
    float delta_z;
    float delta_yaw;
};

class DVMMManager
{
   public:
    DVMMManager();
    ~DVMMManager();

    void loadYAMLParameters(std::string file_path);
    void initializeDVMM();
    DVMMInfo makeDVMM(pcl::PointCloud<PointType>::Ptr cloud);
    DVMMResult findNearestDescriptor(std::vector<DVMMInfo>& descriptors,
                                     DVMMInfo& query);

   private:
    // ************************ Preprocessing ************************//
    void filterPointCloudByRange(pcl::PointCloud<PointType>::Ptr cloud,
                                 pcl::PointCloud<PointType>::Ptr cloud_filtered,
                                 float min_range, float max_range);
    void filterPointcloudVoxelGrid(
        pcl::PointCloud<PointType>::Ptr cloud,
        pcl::PointCloud<PointType>::Ptr cloud_filtered, float leaf_size);
    void putPointcloudHorizon(pcl::PointCloud<PointType>::Ptr cloud,
                              pcl::PointCloud<PointType>::Ptr cloud_hor);

    // ******************** Descriptor Generation ********************//
    cv::Mat projectPointCloudToImageByRange(
        pcl::PointCloud<PointType>::Ptr cloud);
    cv::Mat makeOccupancyImage(pcl::PointCloud<PointType>::Ptr cloud);
    cv::Mat calculateWeightedSum(cv::Mat& image);
    cv::Mat extractNonZeroCols(cv::Mat& image);

    // ******************** Coarse Loop Detection *******************//
    DVMMCandidate coarseLoopDetection(DVMMInfo& query, DVMMInfo& reference);
    float calculateEuclideanDistance(const std::vector<double>& vec1,
                                     const std::vector<double>& vec2);

    // ******************** Fine Loop Verification ********************//
    DVMMResult fineLoopVerification(DVMMInfo& query, DVMMCandidate& candidate);
    float calculateBEVDifference(const cv::Mat& A, const cv::Mat& B,
                                 float& delta_x, float& delta_y);

   public:
    std::vector<int> vec_reference_index;
    float LOOP_THRESHOLD = 0.3;

   private:
    float GAUSSIAN_KERNEL_SIGMA_RATIO;
    float LIDAR_HEIGHT;
    float DOWN_SAMPLE_RESOLUTION;
    float MIN_RANGE, MAX_RANGE, MIN_HEIGHT, MAX_HEIGHT;
    float X_RESOLUTION, Y_RESOLUTION, YAW_RESOLUTION;

    int N_PHI, N_THETA, PHI_INTERVAL, THETA_INTERVAL;
    int OCCUPANCY_SIZE;
    int topK;

    bool query_shift;
    std::vector<double> GAUSSIAN_WEIGHT;
    std::vector<double> vec_vertical_bins_cum;
    int sum_vertical_bins = 0;
    int left_zeros = 0;
};

#endif  // !DVMM_H
