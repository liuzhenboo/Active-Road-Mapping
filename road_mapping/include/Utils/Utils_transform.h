
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

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <Attitude.h>

namespace Utils_transform
{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const Attitude &transform);
Attitude transformFromTF(const tf::Transform &transform);

}