#include <boost/filesystem.hpp>
#include "dvmm.h"

using namespace std;
using namespace Eigen;
namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        std::cerr
            << "Usage: " << argv[0]
            << " <query_pcd_directory> <reference_pcd_directory> <config_path>"
            << std::endl;
        return 1;
    }

    string query_directory = argv[1];
    string reference_directory = argv[2];
    string config_path = argv[3];

    DVMMManager dvmm_manager;
    vector<DVMMInfo> vec_dvmm_reference;
    dvmm_manager.loadYAMLParameters(config_path);
    dvmm_manager.initializeDVMM();

    for (const auto& entry : fs::directory_iterator(reference_directory))
    {
        const fs::path& filePath = entry.path();

        if (fs::is_regular_file(filePath) && filePath.extension() == ".pcd")
        {
            string reference_path = filePath.string();

            cout << "Processing: " << reference_path << endl;

            string reference_name = fs::path(reference_path).stem().string();
            dvmm_manager.vec_reference_index.push_back(
                atoi(reference_name.c_str()));

            pcl::PointCloud<pcl::PointXYZI>::Ptr reference_cloud(
                new pcl::PointCloud<pcl::PointXYZI>());
            pcl::io::loadPCDFile<pcl::PointXYZI>(reference_path,
                                                 *reference_cloud);
            DVMMInfo dvmm_reference = dvmm_manager.makeDVMM(reference_cloud);
            vec_dvmm_reference.push_back(dvmm_reference);
        }
    }

    for (const auto& entry : fs::directory_iterator(query_directory))
    {
        const fs::path& filePath = entry.path();

        if (fs::is_regular_file(filePath) && filePath.extension() == ".pcd")
        {
            string query_path = filePath.string();

            string query_name = fs::path(query_path).stem().string();
            dvmm_manager.vec_reference_index.push_back(
                atoi(query_name.c_str()));
            int query_index = std::stoi(query_name);

            pcl::PointCloud<pcl::PointXYZI>::Ptr query_cloud(
                new pcl::PointCloud<pcl::PointXYZI>());
            pcl::io::loadPCDFile<pcl::PointXYZI>(query_path, *query_cloud);
            DVMMInfo dvmm_query = dvmm_manager.makeDVMM(query_cloud);
            DVMMResult match_result = dvmm_manager.findNearestDescriptor(
                vec_dvmm_reference, dvmm_query);

            int match_index = match_result.index;
            double score = match_result.difference;
            double delta_x = match_result.delta_x;
            double delta_y = match_result.delta_y;
            double delta_z = match_result.delta_z;
            double delta_yaw = match_result.delta_yaw;
            int target_index = match_result.index;
        }
    }

    return 0;
}