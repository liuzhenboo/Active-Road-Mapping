#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Attitude.h>

// pcl库接口
namespace Utils_pcl
{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    float voxelSize);

pcl::IndicesPtr extractIndices(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    bool negative);

pcl::IndicesPtr concatenate(const pcl::IndicesPtr &indicesA, const pcl::IndicesPtr &indicesB);
pcl::IndicesPtr concatenate(const std::vector<pcl::IndicesPtr> &indices);

std::vector<pcl::IndicesPtr> extractClusters(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    float clusterTolerance,
    int minClusterSize,
    int maxClusterSize = std::numeric_limits<int>::max(),
    int *biggestClusterIndex = 0);

pcl::IndicesPtr normalFiltering(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    float angleMax,
    const Eigen::Vector4f &normal,
    int normalKSearch,
    const Eigen::Vector4f &viewpoint);

pcl::IndicesPtr radiusFiltering(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    float radiusSearch,
    int minNeighborsInRadius);

pcl::IndicesPtr cropBox(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    const Eigen::Vector4f &min,
    const Eigen::Vector4f &max,
    const Attitude &Attitude,
    bool negative);

pcl::IndicesPtr passThrough(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    const std::string &axis,
    float min,
    float max,
    bool negative = false);
} // namespace Utils_pcl
