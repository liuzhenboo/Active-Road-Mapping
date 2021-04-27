#include <PC_Segment.h>

PC_Segment::PC_Segment()
{
	preVoxelFiltering_ = true;
	cellSize_ = 0.04;
	projMapFrame_ = false;
	footprintLength_ = 0.0;
	footprintWidth_ = 0.0;
	footprintHeight_ = 0.0;
	minGroundHeight_ = -3.0;
	maxGroundHeight_ = 3.0;
	maxObstacleHeight_ = 2.0;
	normalsSegmentation_ = true;
	groundIsObstacle_ = false;
	maxGroundAngle_ = 0.785;
	clusterRadius_ = 0.1;
	flatObstaclesDetected_ = true;
	minClusterSize_ = 4;
	noiseFilteringRadius_;
	noiseFilteringMinNeighbors_ = 5;
	normalKSearch_ = 10;
	maxGroundHeight_ = 0.0;
}

PC_Segment::~PC_Segment()
{
}
void PC_Segment::pc_init(ros::NodeHandle &nh)
{
	nh.param("preVoxelFiltering", preVoxelFiltering_, preVoxelFiltering_);
	nh.param("projMapFrame", projMapFrame_, projMapFrame_);
	nh.param("normalsSegmentation", normalsSegmentation_, normalsSegmentation_);
	nh.param("groundIsObstacle", groundIsObstacle_, groundIsObstacle_);
	nh.param("flatObstaclesDetected", flatObstaclesDetected_,
			 flatObstaclesDetected_);
	nh.param("minClusterSize", minClusterSize_, minClusterSize_);
	nh.param("noiseFilteringMinNeighbors", cellSize_, cellSize_);
	nh.param("normalKSearch", normalKSearch_, normalKSearch_);

	// double to float
	nh.param("cellSize", cellSize, cellSize);
	nh.param("footprintLength", footprintLength, footprintLength);
	nh.param("footprintWidth", footprintWidth, footprintWidth);
	nh.param("footprintHeight", footprintHeight, footprintHeight);
	nh.param("maxGroundHeight", maxGroundHeight, maxGroundHeight);
	nh.param("maxGroundAngle", maxGroundAngle, maxGroundAngle);
	nh.param("minGroundHeight", minGroundHeight, minGroundHeight);
	nh.param("maxGroundHeight", maxGroundHeight, maxGroundHeight);

	nh.param("clusterRadius", clusterRadius, clusterRadius);
	nh.param("noiseFilteringRadius", noiseFilteringRadius, noiseFilteringRadius);
	nh.param("maxObstacleHeight", maxObstacleHeight, maxObstacleHeight);

	// new parameter
	nh.param("min_region", _min_region, _min_region);

	double2float();
}
void PC_Segment::double2float()
{
	cellSize_ = cellSize;
	footprintLength_ = footprintLength;
	footprintWidth_ = footprintWidth;
	footprintHeight_ = footprintHeight;
	minGroundHeight_ = minGroundHeight;
	maxGroundHeight_ = maxGroundHeight;
	maxObstacleHeight_ = maxObstacleHeight;
	maxGroundAngle_ = maxGroundAngle;
	clusterRadius_ = clusterRadius;
	noiseFilteringRadius_ = noiseFilteringRadius;
	maxGroundHeight_ = maxGroundHeight;
}

// ********************************************************************************
//点云分割接口
//*********************************************************************************
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PC_Segment::segmentCloud(
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudIn,
	const pcl::IndicesPtr &indicesIn,
	const Attitude &pose,
	const cv::Point3f &viewPoint,
	pcl::IndicesPtr &groundIndices,
	pcl::IndicesPtr &obstaclesIndices,
	pcl::IndicesPtr *flatObstacles)
{
	groundIndices.reset(new std::vector<int>);
	obstaclesIndices.reset(new std::vector<int>);
	if (flatObstacles)
	{
		flatObstacles->reset(new std::vector<int>);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::IndicesPtr indices(new std::vector<int>);

	// voxelize之后cloud与indice一一对应
	if (preVoxelFiltering_)
	{
		// voxelize to grid cell size
		cloud = Utils_pcl::voxelize(cloudIn, indicesIn, cellSize_);

		indices->resize(cloud->size());
		for (unsigned int i = 0; i < indices->size(); ++i)
		{
			indices->at(i) = i;
		}
	}
	else
	{
		cloud = cloudIn;
		if (indicesIn->empty() && cloud->is_dense)
		{
			indices->resize(cloud->size());
			for (unsigned int i = 0; i < indices->size(); ++i)
			{
				indices->at(i) = i;
			}
		}
		else
		{
			indices = indicesIn;
		}
	}

	// add pose rotation without yaw
	float roll, pitch, yaw;
	pose.getEulerAngles(roll, pitch, yaw);
	cloud = Utils_transform::transformPointCloud(cloud, Attitude(0, 0, projMapFrame_ ? pose.z() : 0, roll, pitch, 0));

	// // filter footprint
	// if (footprintLength_ > 0.0f || footprintWidth_ > 0.0f || footprintHeight_ > 0.0f)
	// {
	// 	indices = Utils_pcl::cropBox(
	// 		cloud,
	// 		indices,
	// 		Eigen::Vector4f(
	// 			footprintLength_ > 0.0f ? -footprintLength_ / 2.0f : std::numeric_limits<int>::min(),
	// 			footprintWidth_ > 0.0f && footprintLength_ > 0.0f ? -footprintWidth_ / 2.0f : std::numeric_limits<int>::min(),
	// 			0,
	// 			1),
	// 		Eigen::Vector4f(
	// 			footprintLength_ > 0.0f ? footprintLength_ / 2.0f : std::numeric_limits<int>::max(),
	// 			footprintWidth_ > 0.0f && footprintLength_ > 0.0f ? footprintWidth_ / 2.0f : std::numeric_limits<int>::max(),
	// 			footprintHeight_ > 0.0f && footprintLength_ > 0.0f && footprintWidth_ > 0.0f ? footprintHeight_ : std::numeric_limits<int>::max(),
	// 			1),
	// 		Attitude::getIdentity(), true);
	// }

	// 根据minGroundHeight_和maxGroundHeight_来更新indices，这时的indices是为了找路
	if (minGroundHeight_ != 0.0f || maxGroundHeight_ != 0.0f)
	{
		indices = Utils_pcl::passThrough(cloud, indices, "z",
										 minGroundHeight_ != 0.0f ? minGroundHeight_ : std::numeric_limits<int>::min(),
										 maxGroundHeight_ > 0.0f ? maxGroundHeight_ : std::numeric_limits<int>::max());
		// UDEBUG("indices after max obstacles height filtering = %d", (int)indices->size());
	}

	if (indices->size())
	{
		if (normalsSegmentation_ && !groundIsObstacle_)
		{
			// UDEBUG("normalKSearch=%d", normalKSearch_);
			// UDEBUG("maxGroundAngle=%f", maxGroundAngle_);
			// UDEBUG("Cluster radius=%f", clusterRadius_);
			// UDEBUG("flatObstaclesDetected=%d", flatObstaclesDetected_ ? 1 : 0);
			// UDEBUG("maxGroundHeight=%f", maxGroundHeight_);
			segmentObstaclesFromGround(
				cloud,
				indices,
				groundIndices,
				obstaclesIndices,
				normalKSearch_,
				maxGroundAngle_,
				clusterRadius_,
				minClusterSize_,
				flatObstaclesDetected_,
				maxGroundHeight_,
				flatObstacles,
				Eigen::Vector4f(viewPoint.x, viewPoint.y, viewPoint.z + (projMapFrame_ ? pose.z() : 0), 1));
			// UDEBUG("viewPoint=%f,%f,%f", viewPoint.x, viewPoint.y, viewPoint.z + (projMapFrame_ ? pose.z() : 0));
			//UWARN("Saving ground.pcd and obstacles.pcd");
			//pcl::io::savePCDFile("ground.pcd", *cloud, *groundIndices);
			//pcl::io::savePCDFile("obstacles.pcd", *cloud, *obstaclesIndices);
		}
		else
		{
			//UDEBUG("");
			// passthrough filter
			groundIndices = Utils_pcl::passThrough(cloud, indices, "z",
												   minGroundHeight_ != 0.0f ? minGroundHeight_ : std::numeric_limits<int>::min(),
												   maxGroundHeight_ != 0.0f ? maxGroundHeight_ : std::numeric_limits<int>::max());

			pcl::IndicesPtr notObstacles = groundIndices;
			if (indices->size())
			{
				notObstacles = Utils_pcl::extractIndices(cloud, indices, true);
				notObstacles = Utils_pcl::concatenate(notObstacles, groundIndices);
			}
			obstaclesIndices = Utils_pcl::extractIndices(cloud, notObstacles, true);
		}

		// UDEBUG("groundIndices=%d obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());

		// Do radius filtering after voxel filtering ( a lot faster)
		if (noiseFilteringRadius_ > 0.0 && noiseFilteringMinNeighbors_ > 0)
		{
			//UDEBUG("");
			if (groundIndices->size())
			{
				groundIndices = Utils_pcl::radiusFiltering(cloud, groundIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if (obstaclesIndices->size())
			{
				obstaclesIndices = Utils_pcl::radiusFiltering(cloud, obstaclesIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if (flatObstacles && (*flatObstacles)->size())
			{
				*flatObstacles = Utils_pcl::radiusFiltering(cloud, *flatObstacles, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}

			if (groundIndices->empty() && obstaclesIndices->empty())
			{
				// UWARN("Cloud (with %d points) is empty after noise "
				// 	  "filtering. Occupancy grid cannot be "
				// 	  "created.",
				// 	  (int)cloud->size());
			}
		}
	}
	return cloud;
}

//*********************************************************************************
void PC_Segment::segmentObstaclesFromGround(
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
	const pcl::IndicesPtr &indices,
	pcl::IndicesPtr &ground,
	pcl::IndicesPtr &obstacles,
	int normalKSearch,
	float groundNormalAngle,
	float clusterRadius,
	int minClusterSize,
	bool segmentFlatObstacles,
	float maxGroundHeight,
	pcl::IndicesPtr *flatObstacles,
	const Eigen::Vector4f &viewPoint)
{
	ground.reset(new std::vector<int>);
	obstacles.reset(new std::vector<int>);
	if (flatObstacles)
	{
		flatObstacles->reset(new std::vector<int>);
	}

	if (cloud->size())
	{
		// Find the ground
		pcl::IndicesPtr flatSurfaces = Utils_pcl::normalFiltering(
			cloud,
			indices,
			groundNormalAngle,
			Eigen::Vector4f(0, 0, 1, 0),
			normalKSearch,
			viewPoint);

		if (segmentFlatObstacles && flatSurfaces->size())
		{
			int biggestFlatSurfaceIndex;
			std::vector<pcl::IndicesPtr> clusteredFlatSurfaces = Utils_pcl::extractClusters(
				cloud,
				flatSurfaces,
				clusterRadius,
				minClusterSize,
				std::numeric_limits<int>::max(),
				&biggestFlatSurfaceIndex);

			// cluster all surfaces for which the centroid is in the Z-range of the bigger surface
			if (clusteredFlatSurfaces.size())
			{

				Eigen::Vector4f min, max;
				pcl::getMinMax3D(*cloud, *clusteredFlatSurfaces.at(biggestFlatSurfaceIndex), min, max);

				//TODO:道路一致性判断，建模概率模型，判断它是路的概率是多少．和语义ＳＬＡＭ的数据关联相似，计算关联上的概率．

				// 道路面积判断
				if ((max[0] - min[0]) * (max[1] - min[1]) > _min_region)
				{
					ground = clusteredFlatSurfaces.at(biggestFlatSurfaceIndex);
					std::cout << "当前主道路面积为:" << (max[0] - min[0]) * (max[1] - min[1]) << "m*m" << std::endl;
				}
				else
				{
					/* code */
					std::cout << "当前主道路面积为:" << (max[0] - min[0]) * (max[1] - min[1]) << "m*m,小于" << _min_region << "m*m,舍去!" << std::endl;
				}

				// if (maxGroundHeight == 0.0f || min[2] < maxGroundHeight)
				if (maxGroundHeight == 0.0f || min[2] < maxGroundHeight)
				{
					// 是否只要最大的聚类平面．
					if (false)
					{
						for (unsigned int i = 0; i < clusteredFlatSurfaces.size(); ++i)
						{
							if ((int)i != biggestFlatSurfaceIndex)
							{
								Eigen::Vector4f min, max;
								pcl::getMinMax3D(*cloud, *clusteredFlatSurfaces.at(i), min, max);
								// Eigen::Vector4f centroid(0, 0, 0, 1);
								// pcl::compute3DCentroid(*cloud, *clusteredFlatSurfaces.at(i), centroid);
								// if (maxGroundHeight == 0.0f || centroid[2] <= maxGroundHeight || centroid[2] <= max[2]) // epsilon
								if ((max[0] - min[0]) * (max[1] - min[1]) > _min_region && min[2] < maxGroundHeight)
								{
									ground = Utils_pcl::concatenate(ground, clusteredFlatSurfaces.at(i));
								}
								else if (flatObstacles)
								{
									// 平面障碍物
									*flatObstacles = Utils_pcl::concatenate(*flatObstacles, clusteredFlatSurfaces.at(i));
								}
							}
						}
					}
				}
				else
				{
					// reject ground!
					ground.reset(new std::vector<int>);
					if (flatObstacles)
					{
						*flatObstacles = flatSurfaces;
					}
				}
			}
		}
		else
		{
			ground = flatSurfaces;
		}

		if (ground->size() != cloud->size())
		{
			// Remove ground
			pcl::IndicesPtr notObstacles = ground;
			if (indices->size())
			{
				// indices是满足法线要求的点云索引
				// notObstacles = Utils_pcl::extractIndices(cloud, indices, true);
				// notObstacles = Utils_pcl::concatenate(notObstacles, ground);
			}
			// otherStuffIndices为去除地面之后的点云索引
			pcl::IndicesPtr otherStuffIndices = Utils_pcl::extractIndices(cloud, notObstacles, true);

			// 限制障碍物的最大高度
			// if (maxObstacleHeight_ != 0.0f)
			// {
			// 	otherStuffIndices = Utils_pcl::passThrough(cloud, otherStuffIndices, "z", maxObstacleHeight_, std::numeric_limits<float>::max());
			// }
			// If ground height is set, remove obstacles under it
			if (minGroundHeight_ != 0.0f || maxObstacleHeight_ != 0.0f)
			{
				otherStuffIndices = Utils_pcl::passThrough(cloud, otherStuffIndices, "z", minGroundHeight_, maxObstacleHeight_);
			}

			//Cluster remaining stuff (obstacles)
			if (otherStuffIndices->size())
			{
				std::vector<pcl::IndicesPtr> big_clusteredObstaclesSurfaces;
				std::vector<pcl::IndicesPtr> clusteredObstaclesSurfaces = Utils_pcl::extractClusters(
					cloud,
					otherStuffIndices,
					clusterRadius,
					minClusterSize);
				for (int k = 0; k < clusteredObstaclesSurfaces.size(); k++)
				{
					// 	TODO:障碍物判断
					Eigen::Vector4f min, max;
					pcl::getMinMax3D(*cloud, *clusteredObstaclesSurfaces.at(k), min, max);
					if ((max[2] - min[2]) > 0.4)
					{
						big_clusteredObstaclesSurfaces.push_back(clusteredObstaclesSurfaces.at(k));
					}
				}
				// merge indices
				obstacles = Utils_pcl::concatenate(big_clusteredObstaclesSurfaces);
			}
		}
	}
	else
	{
		std::cout << "有效点云为空！" << std::endl;
	}
}
