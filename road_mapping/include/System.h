
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <boost/mpl/bind.hpp>
#include <PC_Segment.h>
#include <Map.h>
#include <math.h>
#include <mutex>
#include <active_road_mapping/Transform_matrix.h>

// #define debug_rawground
// #define debug_rawobstacles
// #define debug_others

class System
{
public:
	System();
	~System();
	void callback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
	void mappoints_callback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
	void TransformTcw_callback(const active_road_mapping::Transform_matrix::ConstPtr &Tcw);
	void Init_parameter(ros::NodeHandle &);
	void run();
	void vis_map();
	void Sent2MapHandle(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr othersCloud);
	void Path_plan();
	void Generate_waypoints(Eigen::Vector3f start, Eigen::Vector3f goal, std::vector<Eigen::Vector3f> &onepath);
	int SelectBestWay(std::vector<std::vector<Eigen::Vector3f>> &waypoints);

private:
	std::string frameId_;
	std::string mapFrameId_;
	bool waitForTransform_;
	tf::TransformListener *tfListener_;
	bool mapFrameProjection_;
	double pointcloud_xu_ = 5.0;
	double pointcloud_xd_ = -5.0;
	double pointcloud_yu_ = 5.0;
	double pointcloud_yd_ = -5.0;
	double pointcloud_zu_ = 5.0;
	double pointcloud_zd_ = -5.0;

	int system_status_;

	ros::Publisher rawgroundPub_;
	ros::Publisher rawobstaclesPub_;
	ros::Publisher rawothersPub_;

	ros::Publisher groundPub_;
	ros::Publisher obstaclesPub_;
	ros::Publisher othersPub_;

	ros::Publisher localgroundPub_;
	ros::Publisher localobstaclesPub_;
	ros::Publisher localothersPub_;

	ros::Publisher circlePub_;
	ros::Publisher frontpointPub_;
	ros::Publisher pointwaysPub_;
	ros::Publisher bestwayPub_;

	ros::Subscriber cloudSub_;
	ros::Subscriber local_mappointsSub_;
	ros::Subscriber odomSub_;

	// 点云切割器
	PC_Segment PC_Processor_;

	Map map_;

	const sensor_msgs::PointCloud2ConstPtr cloudMsg_;
	Attitude pose_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud_;

	// 局部地图点（特征点）

	std::mutex Mutex_local_mappoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_mappoints;

	std::mutex Mutex_odometry;
	Attitude current_odom_Twc;
};
