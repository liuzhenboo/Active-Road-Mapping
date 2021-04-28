#include <iostream>
#include <ros/ros.h>
#include <set>
#include <Utils_transform.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <Cell.h>

enum Color
{
	COLOR_BLACK,
	COLOR_RED,
	COLOR_BLUE,
	COLOR_GREEN,
	COLOR_WHITE,
	COLOR_CYAN,
	COLOR_YELLOW,
	COLOR_MAGENTA,
};
class Map
{
public:
	Map();
	~Map();
	void init(ros::NodeHandle &nh);

	// 点云拼接融合
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_allmap();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_allObsmap();
	bool fusion(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pts);
	bool fusion_obs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pts);
	// 方格融合
	bool fusion_cell(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pts, uint8_t class_id);

	void addOnePoint(float x, float y, float z, uint8_t class_id);
	pcl::PointCloud<pcl::PointXYZ>::Ptr get_cellMap();
	pcl::PointCloud<pcl::PointXYZ>::Ptr get_localMap();
	pcl::PointCloud<pcl::PointXYZ>::Ptr get_ObscellMap();
	int Corrd2Id(float x, float y, float z);
	std::vector<float> Id2Corrd(int id);

	int Initialization_Newground(pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud);
	int Track(pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud);
	int Init_Clouds2Localmap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud, int Initializing);
	int Init_Fusion2Localmap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud);
	int Fusion2Localmap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud);
	void Add2Globalmap();
	void SetPose(Attitude pose);
	void SetInitFlag(int flag);
	pcl::PointCloud<pcl::PointXYZ>::Ptr get_obsMap();
	pcl::PointCloud<pcl::PointXYZ>::Ptr get_unsureMap();

private:
	// 点云　fusion
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_pc;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_pc_obs;
	// 方格fusion
	std::vector<Cell *> cellDataset_;

	//global map
	std::set<int> road_ids;
	std::set<int> obs_ids;
	//弃用
	std::set<int> unsure_ids;

	// local map
	std::map<int, Cell *> localmap_;
	std::set<int> localnew_id_;
	std::set<int> localupdate_id1_;
	std::set<int> localupdate_id2_;
	int InitializeFromScratch_;
	std::map<int, int> cell_olds;
	int current_old;
	int localmap_size_;

	// location
	Attitude pose_;

	double length = 30;
	double width = 30;
	double resolution = 0.2;
	int length_size;
	int width_size;
	int sum_size;
	int origin_id;
	int origin_id_x;
	int origin_id_y;
	double rd_x;
	double rd_y;
};