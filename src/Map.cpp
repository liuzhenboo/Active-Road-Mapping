
#include <Map.h>
#include <memory>
#include <iostream>
Map::Map()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZRGB>);

	all_pc = temp;
	all_pc_obs = temp1;
	length = 100.0;
	width = 100.0;
	resolution = 0.1;
	InitializeFromScratch_ = 1;
	current_old = 1;
	localmap_size_ = 5000;
}
Map::~Map()
{
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map::get_allmap()
{
	return all_pc;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map::get_allObsmap()
{
	return all_pc_obs;
}
bool Map::fusion(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pt)
{
	*all_pc = *all_pc + *current_pt;
	//*all_pc = *current_pt;

	return true;
}

bool Map::fusion_obs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pt)
{
	*all_pc_obs = *all_pc_obs + *current_pt;
	return true;
}

bool Map::fusion_cell(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pts, uint8_t class_id)
{
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> data = current_pts->points;
	for (unsigned int i = 0; i < data.size(); i++)
	{
		addOnePoint(data[i].x, data[i].y, data[i].z, class_id);
	}
	return true;
}

// 投射到格子iｄ号，更新状态量
void Map::addOnePoint(float x, float y, float z, uint8_t class_id)
{
	int current_id = Corrd2Id(x, y, z);
	// TODO get current_id...
	//std::cout << "ADD one point" << std::endl;
	if (current_id < sum_size)
	{ // 如果已经存在，那就融合高度；否则创建新的
		//std::cout << "current_id: " << current_id << std::endl;
		//std::cout << "sum_size: " << sum_size << std::endl;

		// 此id之前对应道路分类，障碍物分类，或没有分类
		if (road_ids.find(current_id) != road_ids.end())
		{
			//std::cout << "ADD one point1" << std::endl;

			uint8_t fusion_classid = cellDataset_[current_id]->Update(class_id, z);
			if (fusion_classid == 2)
			{
				road_ids.erase(current_id);
				obs_ids.insert(current_id);
			}
			else if (fusion_classid == 0)
			{
				road_ids.erase(current_id);
				delete cellDataset_[current_id];
			}
		}
		else if (obs_ids.find(current_id) != obs_ids.end())
		{
			uint8_t fusion_classid = cellDataset_[current_id]->Update(class_id, z);
			if (fusion_classid == 1)
			{
				obs_ids.erase(current_id);
				road_ids.insert(current_id);
			}
			else if (fusion_classid == 0)
			{
				obs_ids.erase(current_id);
				delete cellDataset_[current_id];
			}
		}
		else if (unsure_ids.find(current_id) != unsure_ids.end())
		{
			uint8_t fusion_classid = cellDataset_[current_id]->Update(class_id, z);
			if (fusion_classid == 1)
			{
				unsure_ids.erase(current_id);
				road_ids.insert(current_id);
			}
			else if (fusion_classid == 2)
			{
				unsure_ids.erase(current_id);
				obs_ids.insert(current_id);
			}
			else if (fusion_classid == 0)
			{
				unsure_ids.erase(current_id);
				delete cellDataset_[current_id];
			}
		}
		else
		{
			//std::cout << "ADD one point2" << std::endl;

			cellDataset_[current_id] = new Cell(class_id, z);
			unsure_ids.insert(current_id);
		}
	}
	else
	{
		std::cout << "超出地图！" << std::endl;
	}
	// debug
	std::cout << obs_ids.size() << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Map::get_cellMap()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vis(new pcl::PointCloud<pcl::PointXYZ>);
	std::set<int>::iterator iter = road_ids.begin();
	while (iter != road_ids.end())
	{
		// 因为同一个格子上方的点云数目可能有多个，所以road_ids中可能存在obs_ids中的点．
		if (cellDataset_[*iter]->GetState() == 1)
		{
			std::vector<float> current_ground_point = Id2Corrd(*iter);
			pcl::PointXYZ pt;
			pt.x = current_ground_point[0];
			pt.y = current_ground_point[1];
			pt.z = current_ground_point[2];

			cloud_vis->push_back(pt);
		}
		iter++;
	}
	return cloud_vis;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr Map::get_ObscellMap()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vis(new pcl::PointCloud<pcl::PointXYZ>);
	std::set<int>::iterator iter = obs_ids.begin();
	while (iter != obs_ids.end())
	{
		/* code */
		std::vector<float> current_ground_point = Id2Corrd(*iter);
		pcl::PointXYZ pt;
		pt.x = current_ground_point[0];
		pt.y = current_ground_point[1];
		pt.z = current_ground_point[2];

		cloud_vis->push_back(pt);
		iter++;
	}
	return cloud_vis;
}
std::vector<float> Map::Id2Corrd(int id)
{
	std::vector<float> reasult;
	reasult.push_back(((id + 1) / length_size - origin_id_x) * resolution);
	reasult.push_back(((id + 1) % length_size - origin_id_y) * resolution);
	reasult.push_back(cellDataset_[id]->GetZ(id));
	return reasult;
}
int Map::Corrd2Id(float x, float y, float z)
{
	int ans = static_cast<int>((x - rd_x) / resolution + 1) * length_size + static_cast<int>((y - rd_y) / resolution);
	return ans;
}
void Map::init(ros::NodeHandle &nh)
{
	length_size = length / resolution;
	width_size = width / resolution;
	sum_size = length_size * width_size;
	cellDataset_.resize(sum_size);
	origin_id = static_cast<int>(width_size / 2) * length_size + static_cast<int>(length_size / 2) - 1;

	// 以右下角为原点(1,1)，便于id2cord...，
	origin_id_x = static_cast<int>(width_size / 2) + 1;
	origin_id_y = static_cast<int>(length_size / 2);

	rd_x = -1.0 * origin_id_x * resolution;
	rd_y = -1.0 * origin_id_y * resolution;

	nh.param("localmap_size", localmap_size_, localmap_size_);
}
int Map::Initialization_Newground(pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud)
{
	// 已经开始初始化,初始化正在进行时．．．
	if (!InitializeFromScratch_)
	{
		// fusion localmap
		return Init_Clouds2Localmap(clouds, obstaclesCloud, InitializeFromScratch_);
	}
	else
	{
		// clean localmap,start new ...
		localmap_.clear();
		localnew_id_.clear();
		return Init_Clouds2Localmap(clouds, obstaclesCloud, InitializeFromScratch_);
	}
}
int Map::Track(pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud)
{
	return 1;
	int id = Corrd2Id(pose_.x(), pose_.y(), pose_.z());
	if (obs_ids.find(id) != obs_ids.end())
	{
		return 0;
	}

	return 1;
	// 跟踪方法：计算匹配的总体效果（计算匹配上的每一个cell的高度差，计算平均高度差，高度差要小于阈值）

	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> data = groundCloud->points;
	for (unsigned int i = 0; i < data.size(); i++)
	{
		int current_id = Corrd2Id(data[i].x, data[i].y, data[i].z);
		if (localmap_.find(current_id) != localmap_.end())
		{
			localmap_[current_id]->Update(1, data[i].z);
		}
		else
		{
			localmap_[current_id] = new Cell(1, data[i].z);
			localnew_id_.insert(current_id);
		}
		//AddOnePoint2Localmap(data[i].x, data[i].y, data[i].z);
	}
	return 0;
}
int Map::Init_Clouds2Localmap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud, int InitializeFromScratch)
{
	if (InitializeFromScratch)
	{
		std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> data = clouds->points;
		for (unsigned int i = 0; i < data.size(); i++)
		{
			int current_id = Corrd2Id(data[i].x, data[i].y, data[i].z);
			localmap_[current_id] = new Cell(1, data[i].z);
			//AddOnePoint2Localmap(data[i].x, data[i].y, data[i].z);
		}
		std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> obstaclesdata = obstaclesCloud->points;
		for (unsigned int i = 0; i < obstaclesdata.size(); i++)
		{
			int current_id = Corrd2Id(obstaclesdata[i].x, obstaclesdata[i].y, obstaclesdata[i].z);
			localmap_[current_id] = new Cell(2, obstaclesdata[i].z);
			//AddOnePoint2Localmap(data[i].x, data[i].y, data[i].z);
		}
		InitializeFromScratch_ = 0;
		return 0;
	}
	else
	{
		int flag = Init_Fusion2Localmap(clouds, obstaclesCloud);
		if (flag == 0) //连续性验证失败
		{
			InitializeFromScratch_ = 1;
			return 0;
		}
		else if (flag == 1)
			// 连续性验证成功，但是还未初始化完成
			return 0;
		else
			//初始化成功
			return 1;
	}
}
// TODO:这里认为两次融合必定成功，没有做进一步细致的判断
int Map::Init_Fusion2Localmap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud)
{
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> data = clouds->points;
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> obstaclesdata = obstaclesCloud->points;

	for (unsigned int i = 0; i < data.size(); i++)
	{
		int current_id = Corrd2Id(data[i].x, data[i].y, data[i].z);
		if (localmap_.find(current_id) != localmap_.end())
		{
			localmap_[current_id]->Update(1, data[i].z);
		}
		else
		{
			localmap_[current_id] = new Cell(1, data[i].z);
		}
		//AddOnePoint2Localmap(data[i].x, data[i].y, data[i].z);
	}
	for (unsigned int i = 0; i < obstaclesdata.size(); i++)
	{
		int current_id = Corrd2Id(obstaclesdata[i].x, obstaclesdata[i].y, obstaclesdata[i].z);
		if (localmap_.find(current_id) != localmap_.end())
		{
			localmap_[current_id]->Update(2, data[i].z);
		}
		else
		{
			localmap_[current_id] = new Cell(2, data[i].z);
		}
		//AddOnePoint2Localmap(data[i].x, data[i].y, data[i].z);
	}

	// 将局部地图的放入全局地图
	for (auto &lm : localmap_)
	{
		cellDataset_[lm.first] = lm.second;
		if (lm.second->GetState() == 1)
		{
			road_ids.insert(lm.first);
		}
		else if (lm.second->GetState() == 2)
		{
			obs_ids.insert(lm.first);
		}
		else
		{
		}
		cell_olds[current_old++] = lm.first;
	}

	localnew_id_.clear();
	return 2;
}

int Map::Fusion2Localmap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud)
{
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> data = clouds->points;
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> obstaclesdata = obstaclesCloud->points;

	//std::map<int, Cell *> temp_localmap_;
	for (unsigned int i = 0; i < data.size(); i++)
	{
		int current_id = Corrd2Id(data[i].x, data[i].y, data[i].z);
		if (localmap_.find(current_id) != localmap_.end())
		{
			// 标志位判断在融合数据时候，当前的cell的状态是否变过，变化的方向是什么；
			// 故，这里的 localupdate_id1_与localupdate_id２_的元素可能存在相同的值
			int flag = localmap_[current_id]->GetState();
			localmap_[current_id]->Update(1, data[i].z);
			int flag1 = localmap_[current_id]->GetState();
			if (flag == 1 && flag1 == 2)
			{
				localupdate_id1_.insert(current_id);
			}
			else if (flag == 2 && flag1 == 1)
			{
				localupdate_id2_.insert(current_id);
			}
			//temp_localmap_[current_id] = localmap_[current_id];
		}
		else
		{
			localmap_[current_id] = new Cell(1, data[i].z);
			localnew_id_.insert(current_id);
			//temp_localmap_[current_id] = localmap_[current_id];
			cell_olds[current_old++] = current_id;
		}
		//AddOnePoint2Localmap(data[i].x, data[i].y, data[i].z);
		//localmap_ = temp_localmap_;
	}
	for (unsigned int i = 0; i < obstaclesdata.size(); i++)
	{
		int current_id = Corrd2Id(obstaclesdata[i].x, obstaclesdata[i].y, obstaclesdata[i].z);
		if (localmap_.find(current_id) != localmap_.end())
		{
			int flag = localmap_[current_id]->GetState();
			localmap_[current_id]->Update(2, obstaclesdata[i].z);
			int flag1 = localmap_[current_id]->GetState();
			//temp_localmap_[current_id] = localmap_[current_id];
			if (flag == 1 && flag1 == 2)
			{
				localupdate_id1_.insert(current_id);
			}
			else if (flag == 2 && flag1 == 1)
			{
				localupdate_id2_.insert(current_id);
			}
		}
		else
		{
			localmap_[current_id] = new Cell(2, obstaclesdata[i].z);
			localnew_id_.insert(current_id);
			//temp_localmap_[current_id] = localmap_[current_id];
			cell_olds[current_old++] = current_id;
		}
		//AddOnePoint2Localmap(data[i].x, data[i].y, data[i].z);
		//localmap_ = temp_localmap_;
	}
	return 0;
}
void Map::Add2Globalmap()
{
	for (auto &lm : localnew_id_)
	{
		cellDataset_[lm] = localmap_[lm];
		if (localmap_[lm]->GetState() == 1)
		{
			road_ids.insert(lm);
		}
		else if (localmap_[lm]->GetState() == 2)
		{
			obs_ids.insert(lm);
		}
		else
		{
			//unsure_ids.insert(lm);
		}

		// if (!cellDataset_[lm])
		// {
		// 	cellDataset_[lm] = localmap_[lm];
		// 	road_ids.insert(lm);
		// }
		// else
		// {
		// 	cellDataset_[lm]->Update(1, localmap_[lm]->GetZ(1));
		// 	delete localmap_[lm];
		// 	localmap_[lm] = cellDataset_[lm];
		// }
	}
	for (auto &lm : localupdate_id1_)
	{
		// if (cellDataset_[lm]->GetState() != 2)
		// 	std::cout << "wrong2" << std::endl;
		road_ids.erase(lm);
		obs_ids.insert(lm);
	}
	for (auto &lm : localupdate_id2_)
	{
		obs_ids.erase(lm);
		// if (cellDataset_[lm]->GetState() != 1)
		// 	std::cout << "wrong1" << std::endl;
		road_ids.insert(lm);
	}
	localnew_id_.clear();
	localupdate_id1_.clear();
	localupdate_id2_.clear();

	if (cell_olds.size() > localmap_size_)
	{
		int sum = cell_olds.size() - localmap_size_;
		std::map<int, int>::iterator itr = cell_olds.begin();
		for (int i = 0; i < sum; i++)
		{
			// if (localmap_[itr->second]->GetState() == 2)
			// 	unsure_ids.insert(itr->second);
			localmap_.erase(itr->second);
			cell_olds.erase(itr++);
		}
	}
}
pcl::PointCloud<pcl::PointXYZ>::Ptr Map::get_localMap()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vis(new pcl::PointCloud<pcl::PointXYZ>);
	std::map<int, Cell *>::iterator iter = localmap_.begin();
	while (iter != localmap_.end())
	{
		// 显示时，局部地图不显示障碍物
		if (iter->second->GetState() != 2)
		{
			std::vector<float> current_ground_point = Id2Corrd(iter->first);
			pcl::PointXYZ pt;
			pt.x = current_ground_point[0];
			pt.y = current_ground_point[1];
			pt.z = current_ground_point[2];

			cloud_vis->push_back(pt);
		}
		iter++;
	}
	return cloud_vis;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr Map::get_obsMap()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vis(new pcl::PointCloud<pcl::PointXYZ>);
	std::set<int>::iterator iter = obs_ids.begin();
	while (iter != obs_ids.end())
	{
		if (cellDataset_[*iter]->GetState() == 2)
		{
			std::vector<float> current_obs_point = Id2Corrd(*iter);
			pcl::PointXYZ pt;
			pt.x = current_obs_point[0];
			pt.y = current_obs_point[1];
			pt.z = current_obs_point[2];

			cloud_vis->push_back(pt);
		}
		iter++;
	}
	return cloud_vis;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Map::get_unsureMap()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vis(new pcl::PointCloud<pcl::PointXYZ>);
	std::set<int>::iterator iter = unsure_ids.begin();
	while (iter != unsure_ids.end())
	{
		/* code */
		std::vector<float> current_unsure_point = Id2Corrd(*iter);
		pcl::PointXYZ pt;
		pt.x = current_unsure_point[0];
		pt.y = current_unsure_point[1];
		pt.z = current_unsure_point[2];

		cloud_vis->push_back(pt);
		iter++;
	}
	return cloud_vis;
}
void Map::SetPose(Attitude pose)
{
	pose_ = pose;
}
void Map::SetInitFlag(int flag)
{
	InitializeFromScratch_ = 1;
}
