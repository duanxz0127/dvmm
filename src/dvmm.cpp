#include "dvmm.h"

using namespace std;
using namespace Eigen;

std::pair<int, bool> findInterval(const std::vector<double>& vec, double a)
{
    if (vec.empty())
    {
        throw std::invalid_argument("The input vector is empty.");
    }

    if (a < vec.front() || a > vec.back())
    {
        return {-1, false};
    }

    auto it = std::lower_bound(vec.begin(), vec.end(), a);

    int index = std::distance(vec.begin(), it);

    if (index > 0 && a == vec[index])
    {
        --index;
    }

    return {index, true};
}

DVMMManager::DVMMManager()
{
}

DVMMManager::~DVMMManager()
{
}

void DVMMManager::loadYAMLParameters(std::string file_path)
{
    YAML::Node config = YAML::LoadFile(file_path);
    if (config["topK"])
    {
        topK = config["topK"].as<int>();
    }
    if (config["PHI_INTERVAL"])
    {
        PHI_INTERVAL = config["PHI_INTERVAL"].as<int>();
    }
    if (config["THETA_INTERVAL"])
    {
        THETA_INTERVAL = config["THETA_INTERVAL"].as<int>();
    }
    if (config["GAUSSIAN_KERNEL_SIGMA_RATIO"])
    {
        GAUSSIAN_KERNEL_SIGMA_RATIO =
            config["GAUSSIAN_KERNEL_SIGMA_RATIO"].as<float>();
    }
    if (config["OCCUPANCY_SIZE"])
    {
        OCCUPANCY_SIZE = config["OCCUPANCY_SIZE"].as<int>();
    }
    if (config["MIN_RANGE"])
    {
        MIN_RANGE = config["MIN_RANGE"].as<float>();
    }
    if (config["MAX_RANGE"])
    {
        MAX_RANGE = config["MAX_RANGE"].as<float>();
    }

    X_RESOLUTION = 2 * MAX_RANGE / OCCUPANCY_SIZE;
    Y_RESOLUTION = 2 * MAX_RANGE / OCCUPANCY_SIZE;
    YAW_RESOLUTION = 360.0 / static_cast<float>(N_THETA);
}

void DVMMManager::initializeDVMM()
{
    DOWN_SAMPLE_RESOLUTION = 0.1;  // 0.1 m
    MIN_HEIGHT = 2.0;              // 2 m
    MAX_HEIGHT = 5.0;              // 5 m

    sum_vertical_bins = 0;
    vec_vertical_bins_cum.clear();

    const int vertical_blocks = 90 / PHI_INTERVAL;
    const float phi_interval_rad = DEG2RAD(PHI_INTERVAL);

    // min_projection for normalization
    float min_projection = sin(vertical_blocks * phi_interval_rad) -
                           sin((vertical_blocks - 1) * phi_interval_rad);

    vec_vertical_bins_cum.push_back(0.0f);

    for (int i = 0; i < vertical_blocks; ++i)
    {
        // current block's phi projection
        float phi_proj =
            sin((i + 1) * phi_interval_rad) - sin(i * phi_interval_rad);

        // normalization
        int bin_count = static_cast<int>(phi_proj / min_projection + 0.5f);
        float bin_res = phi_interval_rad / bin_count;

        sum_vertical_bins += bin_count;

        // push back the cumulative bins
        for (int j = 1; j <= bin_count; ++j)
        {
            vec_vertical_bins_cum.push_back(j * bin_res + i * phi_interval_rad);
        }
    }

    N_PHI = 2 * sum_vertical_bins;
    N_THETA = 360 / THETA_INTERVAL;

    // generate an 1-D Gaussian weight
    GAUSSIAN_WEIGHT.resize(N_PHI);
    float sum = 0.0;
    float mu = N_PHI / 2;
    float sigma = GAUSSIAN_KERNEL_SIGMA_RATIO * mu;

    // Gaussian weights
    for (int i = 0; i < N_PHI; ++i)
    {
        GAUSSIAN_WEIGHT[i] =
            std::exp(-std::pow(i - mu, 2) / (2 * std::pow(sigma, 2)));
        sum += GAUSSIAN_WEIGHT[i];
    }
    // Normalize weights
    for (int i = 0; i < N_PHI; ++i)
    {
        GAUSSIAN_WEIGHT[i] /= sum;
    }
}

DVMMInfo DVMMManager::makeDVMM(pcl::PointCloud<PointType>::Ptr cloud)
{
    if (cloud->empty())
    {
        throw std::invalid_argument("The input point cloud is empty.");
    }

    pcl::PointCloud<PointType>::Ptr cloud_filtered(
        new pcl::PointCloud<PointType>);
    filterPointCloudByRange(cloud, cloud_filtered, MIN_RANGE, MAX_RANGE);

    pcl::PointCloud<PointType>::Ptr cloud_voxel_filtered(
        new pcl::PointCloud<PointType>);
    filterPointcloudVoxelGrid(cloud_filtered, cloud_voxel_filtered,
                              DOWN_SAMPLE_RESOLUTION);

    pcl::PointCloud<PointType>::Ptr cloud_hor(new pcl::PointCloud<PointType>());
    putPointcloudHorizon(cloud_voxel_filtered, cloud_hor);

    for (int i = 0; i < cloud_hor->points.size(); i++)
    {
        cloud_hor->points[i].z = cloud_hor->points[i].z + LIDAR_HEIGHT;
    }

    // generate the adaptive projection range image
    cv::Mat gaussian_grid_full_range_image =
        projectPointCloudToImageByRange(cloud_hor);
    // calculate the weighted sum of the range image
    cv::Mat gaussian_grid_col_sum =
        calculateWeightedSum(gaussian_grid_full_range_image);
    // extract non-zero columns
    cv::Mat azimuthal_descriptor = extractNonZeroCols(gaussian_grid_col_sum);
    int none_zero_count = countNonZero(azimuthal_descriptor);
    int left_zero_count = left_zeros;

    // generate crosssection occupancy image
    cv::Mat crosssection_descriptor = makeOccupancyImage(cloud_hor);

    DVMMInfo descriptor;
    descriptor.azimuthal = azimuthal_descriptor;
    descriptor.crosssection = crosssection_descriptor;
    descriptor.non_zero_count = none_zero_count;
    descriptor.left_zero_count = left_zero_count;
    descriptor.lidar_height = LIDAR_HEIGHT;

    return descriptor;
}

DVMMCandidate DVMMManager::coarseLoopDetection(DVMMInfo& query,
                                               DVMMInfo& reference)
{
    cv::Mat desc_in_window, desc_at_back, desc_in_window_full,
        desc_at_back_full;
    int desc_in_window_left_zeros = 0;
    int desc_in_window_none_zeros = 0;
    int desc_at_back_left_zeros = 0;

    if (query.non_zero_count > reference.non_zero_count)
    {
        // HFOV of query is larger than reference
        desc_in_window = reference.azimuthal;
        desc_at_back = query.azimuthal;
        desc_in_window_left_zeros = reference.left_zero_count;
        desc_in_window_none_zeros = reference.non_zero_count;
        desc_at_back_left_zeros = query.left_zero_count;

        query_shift = false;
    }
    else
    {
        // HFOV of query is smaller than reference
        desc_in_window = query.azimuthal;
        desc_at_back = reference.azimuthal;
        desc_in_window_left_zeros = query.left_zero_count;
        desc_in_window_none_zeros = query.non_zero_count;
        desc_at_back_left_zeros = reference.left_zero_count;

        query_shift = true;
    }

    int desc_in_window_cols = desc_in_window.cols;
    int desc_at_back_cols = desc_at_back.cols;
    int sum_cols = desc_in_window.cols + desc_at_back.cols;
    vector<double> vec_in_window, vec_at_back, vec_similarity;
    for (int i = 0; i < desc_in_window_cols; i++)
    {
        vec_in_window.push_back(desc_in_window.at<float>(0, i));
    }

    for (int i = 0; i < sum_cols; i++)
    {
        if (i < desc_at_back_cols)
        {
            vec_at_back.push_back(desc_at_back.at<float>(0, i));
        }
        else if (i - desc_at_back_cols < desc_at_back_cols)
        {
            vec_at_back.push_back(
                desc_at_back.at<float>(0, i - desc_at_back_cols));
        }
    }

    for (int i = 0; i < desc_at_back_cols; i++)
    {
        vector<double> vec_at_back_sub(
            vec_at_back.begin() + i,
            vec_at_back.begin() + i + desc_in_window_cols);
        float euclidean_similarity =
            1 / calculateEuclideanDistance(vec_in_window, vec_at_back_sub);
        vec_similarity.push_back(euclidean_similarity);
    }

    auto max_it =
        std::max_element(vec_similarity.begin(), vec_similarity.end());
    float max_similarity = *max_it;
    int max_similarity_index = std::distance(vec_similarity.begin(), max_it);
    int abs_shift = max_similarity_index + desc_at_back_left_zeros -
                    desc_in_window_left_zeros;

    DVMMCandidate candidate;
    candidate.descriptor = reference;
    candidate.yaw_index_offset = abs_shift;
    candidate.window_nonzeros = desc_in_window_none_zeros;
    candidate.similarity_score = max_similarity;

    return candidate;
}

DVMMResult DVMMManager::fineLoopVerification(DVMMInfo& query,
                                             DVMMCandidate& candidate)
{
    //**************************************** calculate Card BEV image
    // difference
    //****************************************//
    cv::Mat bev_query_cmp;
    cv::Mat bev_reference_cmp;
    if (query_shift)
    {
        float small_fov = candidate.window_nonzeros * 360.0 / N_THETA;
        float rot_angle = candidate.yaw_index_offset * 360.0 / N_THETA;

        // ratate the cv::Mat by rot_angle
        cv::Mat rot_mat =
            cv::getRotationMatrix2D(cv::Point2f(query.crosssection.cols / 2,
                                                query.crosssection.rows / 2),
                                    rot_angle, 1.0);

        cv::warpAffine(query.crosssection, bev_query_cmp, rot_mat,
                       query.crosssection.size(), cv::INTER_NEAREST,
                       cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        bev_reference_cmp = candidate.descriptor.crosssection;
    }
    else
    {
        float small_fov = candidate.window_nonzeros * 360.0 / N_THETA;
        float rot_angle = -candidate.yaw_index_offset * 360.0 / N_THETA;
        // ratate the cv::Mat by rot_angle
        cv::Mat rot_mat =
            cv::getRotationMatrix2D(cv::Point2f(query.crosssection.cols / 2,
                                                query.crosssection.rows / 2),
                                    rot_angle, 1.0);

        cv::warpAffine(query.crosssection, bev_query_cmp, rot_mat,
                       query.crosssection.size(), cv::INTER_NEAREST,
                       cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        bev_reference_cmp = candidate.descriptor.crosssection;
    }

    float bev_diff = MAX_VALUE;
    float delta_x = 0;
    float delta_y = 0;
    bev_diff = calculateBEVDifference(bev_query_cmp, bev_reference_cmp, delta_x,
                                      delta_y);

    int yaw_index_shift =
        query_shift
            ? candidate.yaw_index_offset
            : -candidate.yaw_index_offset;  // shift > 0 means query is shifted
                                            // to right (anti-closck-wise)

    DVMMResult result;
    result.difference = bev_diff;
    result.delta_x = delta_x * X_RESOLUTION;
    result.delta_y = delta_y * Y_RESOLUTION;
    result.delta_z = candidate.descriptor.lidar_height - query.lidar_height;
    result.delta_yaw = static_cast<float>(yaw_index_shift) * YAW_RESOLUTION;

    return result;
}

DVMMResult DVMMManager::findNearestDescriptor(
    std::vector<DVMMInfo>& descriptors, DVMMInfo& query)
{
    int reference_count = descriptors.size();
    int candidate_count =
        (topK > 0 && topK < reference_count) ? topK : reference_count;

    vector<DVMMResult> vec_results(candidate_count);
    vector<DVMMResult> vec_topk_results(candidate_count);

    vector<DVMMCandidate> vec_candidates(reference_count);

    DVMMResult best_result;

    for (int i = 0; i < reference_count; i++)
    {
        DVMMInfo reference = descriptors[i];
        DVMMCandidate candidate = coarseLoopDetection(query, reference);
        candidate.index_in_dataset = i;
        vec_candidates[i] = candidate;
    }

    // sort the candidates by max_similarity
    sort(vec_candidates.begin(), vec_candidates.end(),
         [](const DVMMCandidate& a, const DVMMCandidate& b) {
             return a.similarity_score > b.similarity_score;
         });

    for (int i = 0; i < candidate_count; i++)
    {
        DVMMResult result = fineLoopVerification(query, vec_candidates[i]);
        result.index = vec_reference_index[vec_candidates[i].index_in_dataset];
        vec_results[i] = result;
    }

    sort(vec_results.begin(), vec_results.end(),
         [](const DVMMResult& a, const DVMMResult& b) {
             return a.difference < b.difference;
         });

    best_result = vec_results[0];
    return best_result;
}

cv::Mat DVMMManager::makeOccupancyImage(pcl::PointCloud<PointType>::Ptr cloud)
{
    cv::Mat desc_mat = cv::Mat::zeros(OCCUPANCY_SIZE, OCCUPANCY_SIZE, CV_8UC1);

    float pixel_size = (2 * MAX_RANGE) / OCCUPANCY_SIZE;

    for (const auto& point : cloud->points)
    {
        float x = point.x;
        float y = point.y;
        float z = point.z;

        float range = std::sqrt(x * x + y * y);

        if (z > MAX_HEIGHT || z < MIN_HEIGHT) continue;

        int u = static_cast<int>((x + MAX_RANGE) / pixel_size);
        int v = static_cast<int>((y + MAX_RANGE) / pixel_size);

        if (u >= 0 && u < OCCUPANCY_SIZE && v >= 0 && v < OCCUPANCY_SIZE)
        {
            desc_mat.at<uchar>(u, v) = 255;
        }
    }

    return desc_mat;
}

float DVMMManager::calculateEuclideanDistance(const std::vector<double>& vec1,
                                              const std::vector<double>& vec2)
{
    // Ensure the vectors have the same size
    if (vec1.size() != vec2.size())
    {
        std::cerr << "Error: Vectors must have the same size!" << std::endl;
        return -1.0;  // Return an error value
    }

    float sum = 0.0;
    for (size_t i = 0; i < vec1.size(); ++i)
    {
        sum += std::pow(vec1[i] - vec2[i], 2);
    }

    return std::sqrt(sum);
}

float DVMMManager::calculateBEVDifference(const cv::Mat& A, const cv::Mat& B,
                                          float& delta_x, float& delta_y)
{
    CV_Assert(A.size() == B.size() && A.type() == B.type());
    float bev_diff = 0.0;
    float bev_occ = 0.0;

    if (A.size() != B.size())
    {
        throw std::invalid_argument("Input images must have the same size.");
    }

    cv::Mat img1f, img2f;
    A.convertTo(img1f, CV_32F);
    B.convertTo(img2f, CV_32F);

    double response;
    cv::Point2d offset =
        cv::phaseCorrelate(img1f, img2f, cv::noArray(), &response);

    if (fabs(offset.x) > 3 || fabs(offset.y) > 3)
    {
        return 1.00;
    }

    // shift B by the shift values
    cv::Mat A_shifted;
    cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, offset.x, 0, 1, offset.y);
    cv::warpAffine(A, A_shifted, M, A.size());

    int same_count = 0;
    int none_zeros_A = 0;
    int none_zeros_B = 0;

    cv::Mat C;
    cv::bitwise_and(A_shifted, B, C);
    same_count = cv::countNonZero(C);
    none_zeros_A = cv::countNonZero(A);
    none_zeros_B = cv::countNonZero(B);

    if (none_zeros_B == 0)
    {
        return 1.00;
    }

    bev_diff =
        1 - static_cast<float>(same_count) / static_cast<float>(none_zeros_B);

    delta_x = offset.x;
    delta_y = offset.y;

    return bev_diff;
}

void DVMMManager::filterPointCloudByRange(
    pcl::PointCloud<PointType>::Ptr cloud,
    pcl::PointCloud<PointType>::Ptr cloud_filtered, float min_range,
    float max_range)
{
    for (size_t i = 0; i < cloud->size(); i++)
    {
        float range = sqrt(cloud->points[i].x * cloud->points[i].x +
                           cloud->points[i].y * cloud->points[i].y +
                           cloud->points[i].z * cloud->points[i].z);
        if (range < max_range && range > min_range)
        {
            cloud_filtered->push_back(cloud->points[i]);
        }
    }
}

void DVMMManager::filterPointcloudVoxelGrid(
    pcl::PointCloud<PointType>::Ptr cloud,
    pcl::PointCloud<PointType>::Ptr cloud_filtered, float leaf_size)
{
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_filtered);
}

void DVMMManager::putPointcloudHorizon(
    pcl::PointCloud<PointType>::Ptr cloud,
    pcl::PointCloud<PointType>::Ptr cloud_hor)
{
    pcl::PointCloud<PointType>::Ptr seeds(new pcl::PointCloud<PointType>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (size_t i = 0; i < cloud->size(); i++)
    {
        if (cloud->points[i].z < 1.0)
        {
            seeds->push_back(cloud->points[i]);
        }
    }
    // do ransac to estimate the ground plane
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    seg.setMaxIterations(100);
    seg.setInputCloud(seeds);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset."
                  << std::endl;
        return;
    }

    if (coefficients->values[2] < 0)
    {
        coefficients->values[0] = -coefficients->values[0];
        coefficients->values[1] = -coefficients->values[1];
        coefficients->values[2] = -coefficients->values[2];
        coefficients->values[3] = -coefficients->values[3];
    }

    // ************************************************** //
    pcl::PointCloud<PointType>::Ptr pc_no_ground(
        new pcl::PointCloud<PointType>);
    for (int i = 0; i < cloud->size(); ++i)
    {
        PointType tmp_point = cloud->at(i);
        float dis = coefficients->values[0] * tmp_point.x +
                    coefficients->values[1] * tmp_point.y +
                    coefficients->values[2] * tmp_point.z +
                    coefficients->values[3];
        if (fabs(dis) > 0.1)
        {
            pc_no_ground->push_back(tmp_point);
        }
    }

    if (pc_no_ground->size() > 0)
    {
        *cloud = *pc_no_ground;
    }
    // ************************************************** //

    Eigen::Vector3f ground_normal(coefficients->values[0],
                                  coefficients->values[1],
                                  coefficients->values[2]);
    ground_normal.normalize();

    Eigen::Vector3f reference_normal(0, 0, 1);

    Eigen::Vector3f rotation_axis = ground_normal.cross(reference_normal);
    rotation_axis.normalize();

    float cos_theta = ground_normal.dot(reference_normal);
    float angle = std::acos(cos_theta);

    Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();
    rotation_matrix.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(angle, rotation_axis).toRotationMatrix();

    pcl::transformPointCloud(*cloud, *cloud_hor, rotation_matrix);

    LIDAR_HEIGHT = coefficients->values[3];
}

cv::Mat DVMMManager::projectPointCloudToImageByRange(
    pcl::PointCloud<PointType>::Ptr cloud)
{
    cv::Mat des_1(N_PHI, N_THETA, CV_32FC1, cv::Scalar(0));
    cv::Mat des_2(N_PHI, N_THETA, CV_32FC1, cv::Scalar(0));

    for (int i = 0; i < cloud->size(); i++)
    {
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;
        float rho = sqrt(x * x + y * y);
        float theta = atan2(y, x);
        float phi = atan2(z, rho);

        if (phi > 0)
        {
            auto result = findInterval(vec_vertical_bins_cum, phi);
            size_t index = result.first;
            bool found = result.second;

            int row = sum_vertical_bins - index + 1;
            int col = (theta + M_PI) / (2 * M_PI) * (N_THETA);
            if (col == N_THETA) col--;
            if (row == N_PHI) row--;
            float range = sqrt(x * x + y * y + z * z);
            des_1.at<float>(row, col) += 1;
            des_2.at<float>(row, col) += range;
        }
        else
        {
            auto result = findInterval(vec_vertical_bins_cum, -phi);
            size_t index = result.first;
            bool found = result.second;

            int row = sum_vertical_bins + index;
            int col = (theta + M_PI) / (2 * M_PI) * (N_THETA);
            if (col == N_THETA) col--;
            if (row == N_PHI) row--;
            float range = sqrt(x * x + y * y + z * z);
            des_1.at<float>(row, col) += 1;
            des_2.at<float>(row, col) += range;
        }
    }

    des_2 = des_2 / des_1;

    // in case des_1 has zero values
    for (int i = 0; i < des_1.rows; i++)
    {
        for (int j = 0; j < des_1.cols; j++)
        {
            if (des_1.at<float>(i, j) == 0)
            {
                des_2.at<float>(i, j) = 0;
            }
        }
    }

    return des_2;
}

cv::Mat DVMMManager::calculateWeightedSum(cv::Mat& image)
{
    cv::Mat colsum = cv::Mat::zeros(1, N_THETA, CV_32FC1);

    for (int i = 0; i < N_THETA; i++)
    {
        double weighted_colsum = 0;
        for (int j = 0; j < N_PHI; j++)
        {
            // weighted_colsum += image.at<float>(j, i);
            weighted_colsum += GAUSSIAN_WEIGHT[j] * image.at<float>(j, i);
        }
        colsum.at<float>(0, i) = weighted_colsum;
    }
    return colsum;
}

cv::Mat DVMMManager::extractNonZeroCols(cv::Mat& image)
{
    left_zeros = 0;
    vector<int> non_zero_cols;
    bool left_flag = true;
    cv::Mat image_filtered;

    if (countNonZero(image.col(0)) > 0)
    {
        image_filtered = image;
    }
    else
    {
        for (int col = 0; col < image.cols; ++col)
        {
            if (countNonZero(image.col(col)) > 0)
            {
                non_zero_cols.push_back(col);
                left_flag = false;
            }
            else if (left_flag)
            {
                left_zeros++;
            }
        }

        for (size_t i = 0; i < non_zero_cols.size(); ++i)
        {
            if (i == 0)
                image_filtered = image.col(non_zero_cols[i]).clone();
            else
                hconcat(image_filtered, image.col(non_zero_cols[i]),
                        image_filtered);
        }
    }

    return image_filtered;
}
