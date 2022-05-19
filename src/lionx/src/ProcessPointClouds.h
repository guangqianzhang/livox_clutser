//
// Created by zgq on 2022/5/14.
//

#ifndef TEST5_PROCESSPOINTCLOUDS_H
#define TEST5_PROCESSPOINTCLOUDS_H
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <unordered_set>
#include "Box.h"
#include "KDTree.h"
#include "ros/ros.h"
#include <std_msgs/Header.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
template<typename PointT>
class ProcessPointClouds {
private:
std::string __APP_NAME__;
    ros::NodeHandle nh_;
    ros::Subscriber lionx_sub_;
    ros::Publisher lionx_msg_pub_;
     ros::Publisher pub_bounding_boxs_;

     std::string lidar_topic_;
     std::string lidar_box_topic_;

  std::vector<double> seg_distance_, cluster_distance_;

  std_msgs::Header point_cloud_header_;
    struct Detected_Obj
  {
    jsk_recognition_msgs::BoundingBox bounding_box_;
        
    Box box_msg_;
    double size;

  };
public:
    //constructor
    ProcessPointClouds(ros::NodeHandle &nh);
    //deconstructor
    ~ProcessPointClouds();
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);
    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);
    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                      float filterRes, Eigen::Vector4f minPoint,
                                                      Eigen::Vector4f maxPoint);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers,
                                                                                                           typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                                         int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, 
                                                    int minSize, int maxSize,std::vector<Detected_Obj> &obj_list);
    double numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);



    void clusterHelper(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize);
 void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);
};


#endif //TEST5_PROCESSPOINTCLOUDS_H
