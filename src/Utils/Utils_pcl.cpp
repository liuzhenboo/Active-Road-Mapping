#include <Utils_pcl.h>
namespace Utils_pcl
{
pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    float voxelSize)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    if ((cloud->is_dense && cloud->size()) || (!cloud->is_dense && indices->size()))
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setLeafSize(voxelSize, voxelSize, voxelSize);
        filter.setInputCloud(cloud);
        if (indices->size())
        {
            filter.setIndices(indices);
        }
        filter.filter(*output);
    }
    else if (cloud->size() && !cloud->is_dense && indices->size() == 0)
    {
        // UWARN("Cannot voxelize a not dense (organized) cloud with empty indices! (input=%d pts). Returning empty cloud!", (int)cloud->size());
    }
    return output;
}

pcl::IndicesPtr extractIndices(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    bool negative)
{
    pcl::IndicesPtr output(new std::vector<int>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*output);
    return output;
}

pcl::IndicesPtr concatenate(const pcl::IndicesPtr &indicesA, const pcl::IndicesPtr &indicesB)
{
    pcl::IndicesPtr ind(new std::vector<int>(*indicesA));
    ind->resize(ind->size() + indicesB->size());
    unsigned int oi = (unsigned int)indicesA->size();
    for (unsigned int i = 0; i < indicesB->size(); ++i)
    {
        ind->at(oi++) = indicesB->at(i);
    }
    return ind;
}
pcl::IndicesPtr concatenate(const std::vector<pcl::IndicesPtr> &indices)
{
    //compute total size
    unsigned int totalSize = 0;
    for (unsigned int i = 0; i < indices.size(); ++i)
    {
        totalSize += (unsigned int)indices[i]->size();
    }
    pcl::IndicesPtr ind(new std::vector<int>(totalSize));
    unsigned int io = 0;
    for (unsigned int i = 0; i < indices.size(); ++i)
    {
        for (unsigned int j = 0; j < indices[i]->size(); ++j)
        {
            ind->at(io++) = indices[i]->at(j);
        }
    }
    return ind;
}
std::vector<pcl::IndicesPtr> extractClusters(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    float clusterTolerance,
    int minClusterSize,
    int maxClusterSize,
    int *biggestClusterIndex)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>(true));
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    // 设置聚类半径m
    ec.setClusterTolerance(clusterTolerance);
    //设置聚类需要的最小数目
    ec.setMinClusterSize(minClusterSize);
    //设置聚类需要的最大数目
    ec.setMaxClusterSize(maxClusterSize);
    ec.setInputCloud(cloud);

    if (indices->size())
    {
        ec.setIndices(indices);
        tree->setInputCloud(cloud, indices);
    }
    else
    {
        tree->setInputCloud(cloud);
    }
    //设置点云搜索机制
    ec.setSearchMethod(tree);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    int maxIndex = -1;
    unsigned int maxSize = 0;
    std::vector<pcl::IndicesPtr> output(cluster_indices.size());
    for (unsigned int i = 0; i < cluster_indices.size(); ++i)
    {
        output[i] = pcl::IndicesPtr(new std::vector<int>(cluster_indices[i].indices));

        if (maxSize < cluster_indices[i].indices.size())
        {
            maxSize = (unsigned int)cluster_indices[i].indices.size();
            maxIndex = i;
        }
    }
    if (biggestClusterIndex)
    {
        *biggestClusterIndex = maxIndex;
    }

    return output;
}

pcl::IndicesPtr normalFiltering(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    float angleMax,
    const Eigen::Vector4f &normal,
    int normalKSearch,
    const Eigen::Vector4f &viewpoint)
{
    pcl::IndicesPtr output(new std::vector<int>());

    if (cloud->size())
    {
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>(false));

        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        if (indices->size())
        {
            ne.setIndices(indices);
        }

        if (indices->size())
        {
            tree->setInputCloud(cloud, indices);
        }
        else
        {
            tree->setInputCloud(cloud);
        }
        ne.setSearchMethod(tree);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

        ne.setKSearch(normalKSearch);
        if (viewpoint[0] != 0 || viewpoint[1] != 0 || viewpoint[2] != 0)
        {
            ne.setViewPoint(viewpoint[0], viewpoint[1], viewpoint[2]);
        }

        ne.compute(*cloud_normals);

        output->resize(cloud_normals->size());
        int oi = 0; // output iterator
        for (unsigned int i = 0; i < cloud_normals->size(); ++i)
        {
            Eigen::Vector4f v(cloud_normals->at(i).normal_x, cloud_normals->at(i).normal_y, cloud_normals->at(i).normal_z, 0.0f);
            float angle = pcl::getAngle3D(normal, v);
            if (angle < angleMax)
            {
                output->at(oi++) = indices->size() != 0 ? indices->at(i) : i;
            }
        }
        output->resize(oi);
    }

    return output;
}

pcl::IndicesPtr radiusFiltering(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    float radiusSearch,
    int minNeighborsInRadius)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>(false));

    if (indices->size())
    {
        pcl::IndicesPtr output(new std::vector<int>(indices->size()));
        int oi = 0; // output iterator
        tree->setInputCloud(cloud, indices);
        for (unsigned int i = 0; i < indices->size(); ++i)
        {
            std::vector<int> kIndices;
            std::vector<float> kDistances;
            int k = tree->radiusSearch(cloud->at(indices->at(i)), radiusSearch, kIndices, kDistances);
            if (k > minNeighborsInRadius)
            {
                output->at(oi++) = indices->at(i);
            }
        }
        output->resize(oi);
        return output;
    }
    else
    {
        pcl::IndicesPtr output(new std::vector<int>(cloud->size()));
        int oi = 0; // output iterator
        tree->setInputCloud(cloud);
        for (unsigned int i = 0; i < cloud->size(); ++i)
        {
            std::vector<int> kIndices;
            std::vector<float> kDistances;
            int k = tree->radiusSearch(cloud->at(i), radiusSearch, kIndices, kDistances);
            if (k > minNeighborsInRadius)
            {
                output->at(oi++) = i;
            }
        }
        output->resize(oi);
        return output;
    }
}

pcl::IndicesPtr cropBox(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    const Eigen::Vector4f &min,
    const Eigen::Vector4f &max,
    const Attitude &Attitude,
    bool negative)
{
    // UASSERT(min[0] < max[0] && min[1] < max[1] && min[2] < max[2]);

    pcl::IndicesPtr output(new std::vector<int>);
    pcl::CropBox<pcl::PointXYZRGB> filter;
    filter.setNegative(negative);
    filter.setMin(min);
    filter.setMax(max);
    if (!Attitude.isNull() && !Attitude.isIdentity())
    {
        filter.setTransform(Attitude.toEigen3f());
    }
    filter.setInputCloud(cloud);
    filter.setIndices(indices);
    filter.filter(*output);
    return output;
}

pcl::IndicesPtr passThrough(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::IndicesPtr &indices,
    const std::string &axis,
    float min,
    float max,
    bool negative)
{
    // UASSERT_MSG(max > min, uFormat("cloud=%d, max=%f min=%f axis=%s", (int)cloud->size(), max, min, axis.c_str()).c_str());
    // UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

    pcl::IndicesPtr output(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZRGB> filter;
    filter.setNegative(negative);
    filter.setFilterFieldName(axis);
    filter.setFilterLimits(min, max);
    filter.setInputCloud(cloud);
    filter.setIndices(indices);
    filter.filter(*output);
    return output;
}

} // namespace Utils_pcl