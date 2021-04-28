

#include <ros/ros.h>
#include <iostream>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>

#include <Utils_pcl.h>
#include <Utils_transform.h>

class PC_Segment
{
public:
	PC_Segment(/* args */);
	~PC_Segment();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudIn,
		const pcl::IndicesPtr &indicesIn,
		const Attitude &pose,
		const cv::Point3f &viewPoint,
		pcl::IndicesPtr &groundIndices,
		pcl::IndicesPtr &obstaclesIndices,
		pcl::IndicesPtr *flatObstacles);

	void segmentObstaclesFromGround(
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
		const Eigen::Vector4f &viewPoint);

	void pc_init(ros::NodeHandle &nh);
	void double2float();

private:
	bool preVoxelFiltering_ = true;
	bool projMapFrame_ = false;

	bool normalsSegmentation_ = true;
	bool groundIsObstacle_ = false;
	bool flatObstaclesDetected_ = true;
	int minClusterSize_ = 4;
	int noiseFilteringMinNeighbors_ = 5;
	int normalKSearch_ = 10;

	//float
	float cellSize_ = 0.04;
	float footprintLength_ = 0.0;
	float footprintWidth_ = 0.0;
	float footprintHeight_ = 0.0;
	float minGroundHeight_ = -3.0;
	float maxGroundHeight_ = 3.0;
	float maxObstacleHeight_ = 2.0;
	float maxGroundAngle_ = 0.785;
	float clusterRadius_ = 0.1;
	float noiseFilteringRadius_;

	// double
	double cellSize = 0.04;
	double footprintLength = 0.0;
	double footprintWidth = 0.0;
	double footprintHeight = 0.0;
	double minGroundHeight = -3.0;
	double maxGroundHeight = 3.0;
	double maxObstacleHeight = 2.0;
	double maxGroundAngle = 0.785;
	double clusterRadius = 0.1;
	double noiseFilteringRadius;

	// new parameter
	double _min_region = 2.0;
};