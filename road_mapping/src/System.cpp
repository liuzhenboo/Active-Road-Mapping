
#include <System.h>

System::System()
{
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
	system_status_ = 0;
}
System::~System()
{
}
void System::Init_parameter(ros::NodeHandle &nh)
{

	PC_Processor_.pc_init(nh);
	map_.init(nh);
	tfListener_ = new tf::TransformListener();
	mapFrameProjection_ = false;
	nh.param("frame_id", frameId_, frameId_);
	nh.param("map_frame_id", mapFrameId_, mapFrameId_);
	nh.param("wait_for_transform", waitForTransform_, waitForTransform_);
	nh.param("projMapFrame", mapFrameProjection_, mapFrameProjection_);
	nh.param("pointcloud_xu", pointcloud_xu_, pointcloud_xu_);
	nh.param("pointcloud_xd", pointcloud_xd_, pointcloud_xd_);
	nh.param("pointcloud_yu", pointcloud_yu_, pointcloud_yu_);
	nh.param("pointcloud_yd", pointcloud_yd_, pointcloud_yd_);
	nh.param("pointcloud_zu", pointcloud_zu_, pointcloud_zu_);
	nh.param("pointcloud_zd", pointcloud_zd_, pointcloud_zd_);
	if (mapFrameProjection_ && mapFrameId_.empty())
	{
		ROS_ERROR("mav_find_road: Parameter mapFrameProjection is true but map_frame_id is not set!");
	}

	ROS_INFO("Starting...");

	// 需要订阅传感器原始点云数据
	// cloudSub_ = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 2, boost::bind(&System::callback, this, _1));
	cloudSub_ = nh.subscribe<sensor_msgs::PointCloud2>("/mono_Inertial/LocalaMap", 2, boost::bind(&System::callback, this, _1));
	// 需要订阅SLAM系统的局部地图点
	local_mappointsSub_ = nh.subscribe<sensor_msgs::PointCloud2>("/mono_Inertial/LocalaMap", 2, boost::bind(&System::mappoints_callback, this, _1));
	// 里程计回调函数

	odomSub_ = nh.subscribe<active_road_mapping::Transform_matrix>("/mono_Inertial/orbslam3_Tcw", 2, boost::bind(&System::TransformTcw_callback, this, _1));

	// 点云分割的结果，发布道路与障碍物点云，debug
	rawgroundPub_ = nh.advertise<sensor_msgs::PointCloud2>("raw_ground", 1);
	rawobstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("raw_obstacles", 1);
	rawothersPub_ = nh.advertise<sensor_msgs::PointCloud2>("raw_others", 1);

	// 根据建图的结果，发布不同的点云数据，进行可视化．
	groundPub_ = nh.advertise<sensor_msgs::PointCloud2>("global_ground", 1);
	obstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("global_obstacles", 1);
	othersPub_ = nh.advertise<sensor_msgs::PointCloud2>("global_others", 1);

	localgroundPub_ = nh.advertise<sensor_msgs::PointCloud2>("localground", 1);
	localobstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("localobstacles", 1);
	localothersPub_ = nh.advertise<sensor_msgs::PointCloud2>("localothers", 1);

	// 发布圆形范围
	circlePub_ = nh.advertise<sensor_msgs::PointCloud2>("circle", 10);
	frontpointPub_ = nh.advertise<sensor_msgs::PointCloud2>("frontpoint", 10);
	pointwaysPub_ = nh.advertise<sensor_msgs::PointCloud2>("pointways", 10);
	bestwayPub_ = nh.advertise<sensor_msgs::PointCloud2>("bestway", 10);
}

void System::run()
{
	ros::Rate rate(10);
	bool status = ros::ok();
	while (status)
	{
		ros::spinOnce();
		status = ros::ok();
		rate.sleep();
	}
}
void System::TransformTcw_callback(const active_road_mapping::Transform_matrix::ConstPtr &Tcw)
{
	{
		std::unique_lock<std::mutex> lock(Mutex_odometry);
		Attitude temp(Tcw->T00, Tcw->T01, Tcw->T02, Tcw->T03,
					  Tcw->T10, Tcw->T11, Tcw->T12, Tcw->T13,
					  Tcw->T20, Tcw->T21, Tcw->T22, Tcw->T23);
		current_odom_Twc = temp.inverse();
		std::cout << "debug_current_odom_Twc:" << Tcw->T03 << " " << Tcw->T13 << " " << Tcw->T23 << std::endl;
	}
}
void System::mappoints_callback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*cloudMsg, *inputCloud);
	{
		std::unique_lock<std::mutex> lock(Mutex_local_mappoints);
		local_mappoints = inputCloud;
	}
}
void System::callback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{

	ros::WallTime time = ros::WallTime::now();
	// if (groundPub_.getNumSubscribers() == 0 && obstaclesPub_.getNumSubscribers() == 0)
	// {
	// 	// 无人订阅,则不进行处理.
	// 	ROS_INFO("no one wants the results!");
	// 	return;
	// }

	// tf树获得localTransform: 点云坐标系与机体系frameId(base_link)的坐标变换
	//Attitude localTransform = Attitude::getIdentity();
	// try
	// {
	// 	if (waitForTransform_)
	// 	{
	// 		if (!tfListener_->waitForTransform(frameId_, cloudMsg->header.frame_id, ros::Time(0), ros::Duration(3)))
	// 		{
	// 			ROS_ERROR("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), cloudMsg->header.frame_id.c_str());
	// 			return;
	// 		}
	// 	}
	// 	tf::StampedTransform tmp;
	// 	tfListener_->lookupTransform(frameId_, cloudMsg->header.frame_id, ros::Time(0), tmp);
	// 	localTransform = Utils_transform::transformFromTF(tmp);
	// }
	// catch (tf::TransformException &ex)
	// {
	// 	ROS_ERROR("%s", ex.what());
	// 	return;
	// }

	// // tf树获得pose: frameId(base_link)在mapFrameId中的姿态
	// Attitude pose = Attitude::getIdentity();
	// if (!mapFrameId_.empty())
	// {
	// 	try
	// 	{
	// 		if (waitForTransform_)
	// 		{
	// 			if (!tfListener_->waitForTransform(mapFrameId_, frameId_, ros::Time(0), ros::Duration(3)))
	// 			{
	// 				ROS_ERROR("Could not get transform from %s to %s after 1 second!", mapFrameId_.c_str(), frameId_.c_str());
	// 				return;
	// 			}
	// 		}
	// 		tf::StampedTransform tmp;
	// 		tfListener_->lookupTransform(mapFrameId_, frameId_, ros::Time(0), tmp);
	//		pose = Utils_transform::transformFromTF(tmp);
	// 		pose_ = pose;
	// 		map_.SetPose(pose);
	// 	}
	// 	catch (tf::TransformException &ex)
	// 	{
	// 		ROS_ERROR("%s", ex.what());
	// 		return;
	// 	}
	// }

	// 如果不使用tf树，使用里程计
	// 这里的localTransform应该为无人机里程计位置与原始点云参考坐标系的变换
	Attitude localTransform = Attitude::getIdentity();
	Attitude pose = Attitude::getIdentity();
	{
		std::unique_lock<std::mutex> lock(Mutex_odometry);
		pose = current_odom_Twc;
		pose_ = pose;
		map_.SetPose(pose);
	}
	// 去除无效点云数据
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud0(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*cloudMsg, *inputCloud0);
	if (inputCloud0->isOrganized())
	{
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*inputCloud0, *inputCloud0, indices);
	}

	// 主要的点云变量
	pcl::IndicesPtr ground, obstacles, others;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr othersCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloudWithoutFlatSurfaces(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_pt(new pcl::PointCloud<pcl::PointXYZRGB>);

	// 核心逻辑,对inputCloud进行处理
	if (inputCloud0->size())
	{
		// 转换到base_link坐标系,
		inputCloud0 = Utils_transform::transformPointCloud(inputCloud0, localTransform);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		debug_pt = inputCloud;
		///  保留想要范围内的点云
		for (int i = 0; i < inputCloud0->points.size(); i++)
		{
			pcl::PointXYZRGB pt;
			pt = inputCloud0->points[i];
			if (pt.x < pointcloud_xu_ && pt.x > pointcloud_xd_ && pt.y < pointcloud_yu_ && pt.y > pointcloud_yd_ && pt.z > pointcloud_zd_ && pt.z < pointcloud_zu_)
				inputCloud->points.push_back(pt);
		}
		inputCloud->width = inputCloud->points.size();
		inputCloud->height = 1;
		inputCloud->is_dense = true;

		if (!inputCloud->size())
			std::cout << "点云切块后为空！" << std::endl;

		pcl::IndicesPtr flatObstacles(new std::vector<int>);

		// 点云分割
		// ground为道路点云
		// obstacles为影响行军的障碍物点云
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = PC_Processor_.segmentCloud(
			inputCloud,
			pcl::IndicesPtr(new std::vector<int>),
			pose,
			//cv::Point3f(localTransform.x(), localTransform.y(), localTransform.z()),
			cv::Point3f(0, 0, 0),
			ground,
			obstacles,
			&flatObstacles);

		//debug
		// if (cloud->size())
		// {
		// 	std::cout << "非空" << std::endl;
		// }

		// ground与obstacles必须有一个不为空
		if (cloud->size() && ((ground.get() && ground->size()) || (obstacles.get() && obstacles->size())))
		{
			// 对道路点云索引处理
			// 从索引ground得到道路真实数据groundCloud
			if (ground.get() && ground->size())
			{
				pcl::copyPointCloud(*cloud, *ground, *groundCloud);
			}

			// 对障碍物点云索引处理
			if (obstacles.get() && obstacles->size())
			{
				obstaclesCloud->resize(obstacles->size());
				for (unsigned int i = 0; i < obstacles->size(); ++i)
				{
					obstaclesCloud->points[i] = cloud->at(obstacles->at(i));
				}
			}

			if (!localTransform.isIdentity() || !pose.isIdentity())
			{
				//transform back in topic frame for 3d clouds and base frame for 2d clouds
				// 在分割的函数中对点云做了一些坐标变换,这里恢复原来的坐标
				float roll, pitch, yaw;
				pose.getEulerAngles(roll, pitch, yaw);
				Attitude t = Attitude(0, 0, mapFrameProjection_ ? pose.z() : 0, roll, pitch, 0);

				if (groundCloud->size())
				{
					groundCloud = Utils_transform::transformPointCloud(groundCloud, t.inverse());
					Attitude t1 = Attitude(pose.x(), pose.y(), pose.z(), roll, pitch, yaw);
					groundCloud = Utils_transform::transformPointCloud(groundCloud, t1);
					//debug_pt = Utils_transform::transformPointCloud(debug_pt, t1);
				}
				//t = (t * localTransform).inverse();

				if (obstaclesCloud->size())
				{
					obstaclesCloud = Utils_transform::transformPointCloud(obstaclesCloud, t.inverse());
					Attitude t2 = Attitude(pose.x(), pose.y(), pose.z(), roll, pitch, yaw);
					obstaclesCloud = Utils_transform::transformPointCloud(obstaclesCloud, t2);
				}
				else
				{
					std::cout << "没有检测到障碍物！" << std::endl;
				}
			}
		}
		else
		{
			ROS_INFO("no ground pointcloud and obstacles pointcloud!");
		}
	}
	else
	{
		ROS_WARN("mav_find_road: Input cloud is empty! (%d x %d, is_dense=%d)", cloudMsg->width, cloudMsg->height, cloudMsg->is_dense ? 1 : 0);
	}

#ifdef debug_rawground
	if (rawgroundPub_.getNumSubscribers())
	{
		sensor_msgs::PointCloud2 rosCloud;
		pcl::toROSMsg(*groundCloud, rosCloud);
		rosCloud.header.stamp = cloudMsg->header.stamp;
		rosCloud.header.frame_id = mapFrameId_;
		//publish the message
		rawgroundPub_.publish(rosCloud);
	}
#endif

#ifdef debug_rawobstacles
	if (rawobstaclesPub_.getNumSubscribers())
	{
		sensor_msgs::PointCloud2 rosCloud;
		pcl::toROSMsg(*obstaclesCloud, rosCloud);
		rosCloud.header.stamp = cloudMsg->header.stamp;
		rosCloud.header.frame_id = mapFrameId_;
		//publish the message
		rawobstaclesPub_.publish(rosCloud);
	}
#endif

#ifdef debug_others
	if (rawothersPub_.getNumSubscribers())
	{
		sensor_msgs::PointCloud2 rosCloud;
		pcl::toROSMsg(*othersCloud, rosCloud);
		rosCloud.header.stamp = cloudMsg->header.stamp;
		rosCloud.header.frame_id = mapFrameId_;
		//publish the message
		rawothersPub_.publish(rosCloud);
	}
#endif

	ROS_DEBUG("road detect cost = %f s", (ros::WallTime::now() - time).toSec());
	Sent2MapHandle(groundCloud, obstaclesCloud, othersCloud);

	Path_plan();
}
void System::Path_plan()
{
	float x = pose_.x();
	float y = pose_.y();
	float z = pose_.z();
	pcl::PointCloud<pcl::PointXYZ> cloud_vis;
	float sum = 100;
	float r = 1;
	for (int k = 0; k < sum; k++)
	{
		pcl::PointXYZ pt_inf;
		pt_inf.x = x + r * std::cos((2 * M_PI / sum) * k);
		pt_inf.y = y + r * std::sin((2 * M_PI / sum) * k);
		pt_inf.z = z;
		// 心型线
		// TODO:各种形状来玩耍啊！
		// pt_inf.x = x + r * (2.0 * std::cos((2 * M_PI / sum) * k) - std::cos((2 * M_PI / sum) * 2 * k));
		// pt_inf.y = y + r * (2.0 * std::sin((2 * M_PI / sum) * k) - std::sin((2 * M_PI / sum) * 2 * k));
		// pt_inf.z = z;
		cloud_vis.points.push_back(pt_inf);
	}

	sensor_msgs::PointCloud2 map_vis;
	pcl::toROSMsg(cloud_vis, map_vis);
	map_vis.header.frame_id = mapFrameId_;
	circlePub_.publish(map_vis);

	// 找到飞机的正前方，画出来；
	// 435i相机：x往前，y左，z上
	// 对于机体坐标系中的点(1, 0, 0),找到其在map坐标系的坐标
	// 先看看yaw角度是多少
	float roll, pitch, yaw;
	pose_.getEulerAngles(roll, pitch, yaw);
	pcl::PointCloud<pcl::PointXYZ> cloud_vis1;
	sum = 2;
	for (int k = 0; k < sum; k++)
	{
		pcl::PointXYZ pt_inf;
		pt_inf.x = x + r * std::cos(yaw + std::pow(-1, k) * M_PI * 5.0 / 12.0 + 0.5 * M_PI);
		pt_inf.y = y + r * std::sin(yaw + std::pow(-1, k) * M_PI * 5.0 / 12.0 + 0.5 * M_PI);
		pt_inf.z = z;
		cloud_vis1.points.push_back(pt_inf);
	}
	sensor_msgs::PointCloud2 map_vis1;
	pcl::toROSMsg(cloud_vis1, map_vis1);
	map_vis1.header.frame_id = mapFrameId_;

	frontpointPub_.publish(map_vis1);

	// 开始生成航路点
	//在120度的视角下，每15度做一个目标点，共可以得到9条路线
	Eigen::Vector3f start(x, y, z);
	size_t way_count = 9;
	std::vector<std::vector<Eigen::Vector3f>> waypoints(way_count);
	for (size_t i = 0; i < way_count; i++)
	{
		float x_g = x + r * std::cos(yaw - M_PI * 1.0 / 3.0 + i * M_PI * 2.0 / (3.0 * (way_count - 1)) + 0.5 * M_PI);
		float y_g = y + r * std::sin(yaw - M_PI * 1.0 / 3.0 + i * M_PI * 2.0 / (3.0 * (way_count - 1)) + 0.5 * M_PI);
		float z_g = z;
		Eigen::Vector3f goal(x_g, y_g, z_g);
		Generate_waypoints(start, goal, waypoints[i]);
	}

	// path可视化
	pcl::PointCloud<pcl::PointXYZ> cloud_vis2;
	sum = waypoints.size();
	for (int j = 0; j < sum; j++)
	{
		for (int j1 = 0; j1 < waypoints[j].size(); j1++)
		{
			pcl::PointXYZ pt_inf;
			pt_inf.x = waypoints[j][j1](0);
			pt_inf.y = waypoints[j][j1](1);
			pt_inf.z = waypoints[j][j1](2);
			cloud_vis2.points.push_back(pt_inf);
		}
	}
	sensor_msgs::PointCloud2 map_vis2;
	pcl::toROSMsg(cloud_vis2, map_vis2);
	map_vis2.header.frame_id = mapFrameId_;

	pointwaysPub_.publish(map_vis2);

	// 选择最好的路径
	int id = SelectBestWay(waypoints);
	pcl::PointCloud<pcl::PointXYZ> cloud_vis3;
	for (int j = 0; j < waypoints[id].size(); j++)
	{
		pcl::PointXYZ pt_inf;
		pt_inf.x = waypoints[id][j](0);
		pt_inf.y = waypoints[id][j](1);
		pt_inf.z = waypoints[id][j](2);
		cloud_vis3.points.push_back(pt_inf);
	}
	sensor_msgs::PointCloud2 map_vis3;
	pcl::toROSMsg(cloud_vis3, map_vis3);
	map_vis3.header.frame_id = mapFrameId_;

	bestwayPub_.publish(map_vis3);
}

// TODO:路径评判函数
int System::SelectBestWay(std::vector<std::vector<Eigen::Vector3f>> &waypoints)
{
	// 根据orbslam3局部地图信息，选择路线
	{
		std::unique_lock<std::mutex> lock(Mutex_local_mappoints);

		//local_mappoints;
	}

	return 4;
}

void System::Generate_waypoints(Eigen::Vector3f start, Eigen::Vector3f goal, std::vector<Eigen::Vector3f> &onepath)
{
	// 方案一：直线
	float x = goal(0) - start(0);
	float y = goal(1) - start(1);
	size_t sum = 30;
	for (size_t i = 0; i < sum; i++)
	{
		Eigen::Vector3f temp((start(0) + x * i / sum), (start(1) + y * i / sum), start(2));
		onepath.push_back(temp);
	}

	// 方案二：考虑到障碍物以及行军道路信息
	// TODO...
}
void System::Sent2MapHandle(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr othersCloud)
{
	ros::WallTime time1 = ros::WallTime::now();

	// map_.Track(groundCloud);
	// map_.Fusion2Localmap(groundCloud, obstaclesCloud);
	// map_.Add2Globalmap();
	// std::cout << "fusion_cell !!!" << std::endl;
	// vis_map();

	if (system_status_ == 0)
	{
		system_status_ = map_.Initialization_Newground(groundCloud, obstaclesCloud);
		if (system_status_ == 1)
		{
			std::cout << "初始化成功！" << std::endl;
		}
		else
		{
			//std::cout << "正在初始化．．．"　<< std::endl;
		}
	}
	// 初始化成功
	else
	{
		// 跟踪成功
		if (map_.Track(groundCloud))
		{
			map_.Fusion2Localmap(groundCloud, obstaclesCloud);
			map_.Add2Globalmap();
			std::cout << "fusion_cell" << std::endl;
			vis_map();
		}
		else
		{
			//std::cout << "跟踪失败！重新初始化！！"　<< std::endl;
			system_status_ = 0;
			map_.SetInitFlag(1);
		}
	}

	ROS_DEBUG("map fusion cost = %f s", (ros::WallTime::now() - time1).toSec());
	obstaclesCloud_ = groundCloud;
}

void System::vis_map()
{
	if (groundPub_.getNumSubscribers())
	{
		sensor_msgs::PointCloud2 rosCloud;
		sensor_msgs::PointCloud2 roslocalCloud;
		sensor_msgs::PointCloud2 rosobsCloud;
		sensor_msgs::PointCloud2 rosunsureCloud;

		pcl::PointCloud<pcl::PointXYZ>::Ptr all_groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr local_groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr obsCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr unsureCloud(new pcl::PointCloud<pcl::PointXYZ>);

		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_groundCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		//all_groundCloud = map_.get_allmap();
		all_groundCloud = map_.get_cellMap();
		local_groundCloud = map_.get_localMap();
		obsCloud = map_.get_obsMap();
		unsureCloud = map_.get_unsureMap();

		pcl::toROSMsg(*all_groundCloud, rosCloud);
		pcl::toROSMsg(*local_groundCloud, roslocalCloud);
		pcl::toROSMsg(*obsCloud, rosobsCloud);
		pcl::toROSMsg(*obstaclesCloud_, rosunsureCloud);

		//rosCloud.header = cloudMsg->header;
		//rosCloud.header.stamp = cloudMsg_->header.stamp;
		rosCloud.header.frame_id = mapFrameId_;
		roslocalCloud.header.frame_id = mapFrameId_;
		rosobsCloud.header.frame_id = mapFrameId_;
		rosunsureCloud.header.frame_id = mapFrameId_;

		//publish the message
		groundPub_.publish(rosCloud);
		obstaclesPub_.publish(rosobsCloud);
		//othersPub_.publish();

		localgroundPub_.publish(roslocalCloud);
		//localobstaclesPub_.publish();
		//localothersPub_.publish();

		//unsurePub_.publish(rosunsureCloud);
	}
	// map_.fusion_cell(obstaclesCloud, 2);

	// if (obstaclesPub_.getNumSubscribers())
	// {
	// 	sensor_msgs::PointCloud2 rosCloud;
	// 	pcl::PointCloud<pcl::PointXYZ>::Ptr all_obsCloud(new pcl::PointCloud<pcl::PointXYZ>);
	// 	all_obsCloud = map_.get_ObscellMap();
	// 	pcl::toROSMsg(*all_obsCloud, rosCloud);
	// 	rosCloud.header.stamp = cloudMsg_->header.stamp;
	// 	rosCloud.header.frame_id = mapFrameId_;
	// 	//publish the message
	// 	obstaclesPub_.publish(rosCloud);
	// }
}