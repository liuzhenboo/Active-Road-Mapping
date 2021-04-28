/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"
#include "../include/ImuTypes.h"

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

// pcl
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
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

// ros message
#include <active_road_mapping/Transform_matrix.h>

using namespace std;

class ImuGrabber
{
public:
  ImuGrabber(){};
  void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

  queue<sensor_msgs::ImuConstPtr> imuBuf;
  std::mutex mBufMutex;
};

class ImageGrabber
{
public:
  ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, const bool bClahe) : mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe) {}

  void GrabImage(const sensor_msgs::ImageConstPtr &msg);
  cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
  void SyncWithImu();

  queue<sensor_msgs::ImageConstPtr> img0Buf;
  std::mutex mBufMutex;

  ORB_SLAM3::System *mpSLAM;
  ImuGrabber *mpImuGb;

  const bool mbClahe;
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

ros::Publisher LocalaMapPub_;
ros::Publisher tranform_Tcw_pub;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mono_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  LocalaMapPub_ = n.advertise<sensor_msgs::PointCloud2>("LocalaMap", 10);
  tranform_Tcw_pub = n.advertise<active_road_mapping::Transform_matrix>("orbslam3_Tcw", 50);

  bool bEqual = false;
  if (argc < 3 || argc > 4)
  {
    cerr << endl
         << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }

  if (argc == 4)
  {
    std::string sbEqual(argv[3]);
    if (sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM, &imugb, bEqual); // TODO

  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
  ros::Subscriber sub_img0 = n.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage, &igb);

  std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

  ros::spin();

  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutex.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  if (cv_ptr->image.type() == 0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
  while (1)
  {

    cv::Mat im;
    double tIm = 0;
    if (!img0Buf.empty() && !mpImuGb->imuBuf.empty())
    {
      tIm = img0Buf.front()->header.stamp.toSec();
      if (tIm > mpImuGb->imuBuf.back()->header.stamp.toSec())
        continue;
      {
        this->mBufMutex.lock();
        im = GetImage(img0Buf.front());
        img0Buf.pop();
        this->mBufMutex.unlock();
      }

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if (!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if (mbClahe)
        mClahe->apply(im, im);

      mpSLAM->TrackMonocular(im, tIm, vImuMeas);
    }
    //================================================================
    {
      /*-----------------------------------------------------------------
  发布tf树，以及局部地图点(点云)
-----------------------------------------------------------------*/
      ORB_SLAM3::Tracking *myTracker = mpSLAM->Get_track();

      // (1) 发布定位信息，tf树，主要是T^M_B B->M，T^B_P  P->B
      cv::Mat cvTcw = cv::Mat::eye(4, 4, CV_32F);
      cvTcw = myTracker->mCurrentFrame.mTcw.clone();

      if (cvTcw.rows == 4 && cvTcw.cols == 4)
      {
        active_road_mapping::Transform_matrix Tcw;
        Tcw.header.stamp = ros::Time::now();
        Tcw.T00 = cvTcw.at<float>(0, 0);
        Tcw.T01 = cvTcw.at<float>(0, 1);
        Tcw.T02 = cvTcw.at<float>(0, 2);
        Tcw.T10 = cvTcw.at<float>(1, 0);
        Tcw.T11 = cvTcw.at<float>(1, 1);
        Tcw.T12 = cvTcw.at<float>(1, 2);
        Tcw.T20 = cvTcw.at<float>(2, 0);
        Tcw.T21 = cvTcw.at<float>(2, 1);
        Tcw.T22 = cvTcw.at<float>(2, 2);
        Tcw.T03 = cvTcw.at<float>(0, 3);
        Tcw.T13 = cvTcw.at<float>(1, 3);
        Tcw.T23 = cvTcw.at<float>(2, 3);
        tranform_Tcw_pub.publish(Tcw);

        // (2) 发布点云
        pcl::PointCloud<pcl::PointXYZ> localmap;
        std::vector<ORB_SLAM3::MapPoint *> *LocalMapPoints;
        LocalMapPoints = myTracker->Get_LocalMap();

        for (int j = 0; j < LocalMapPoints->size(); j++)
        {
          pcl::PointXYZ pt_inf;
          if ((*LocalMapPoints)[j] == nullptr)
            continue;
          if ((*LocalMapPoints)[j]->GetWorldPos().rows != 3 || (*LocalMapPoints)[j]->GetWorldPos().cols != 1)
            continue;
          pt_inf.x = (*LocalMapPoints)[j]->GetWorldPos().at<float>(0, 0);
          pt_inf.y = (*LocalMapPoints)[j]->GetWorldPos().at<float>(1, 0);
          pt_inf.z = (*LocalMapPoints)[j]->GetWorldPos().at<float>(2, 0);
          localmap.points.push_back(pt_inf);
        }

        sensor_msgs::PointCloud2 localmap_ros;
        pcl::toROSMsg(localmap, localmap_ros);
        localmap_ros.header.frame_id = "world";
        LocalaMapPub_.publish(localmap_ros);
      }
      //=================================================================
    }
    std::chrono::milliseconds tSleep(1);
    //std::this_thread::sleep_for(tSleep);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}
