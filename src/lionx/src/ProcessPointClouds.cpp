//
// Created by zgq on 2022/5/14.
//
#include "glog/logging.h"
#include "ProcessPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds(ros::NodeHandle &nh):nh_(nh) {

    __APP_NAME__ = ros::this_node::getName();
        ros::NodeHandle private_node_handle("~");
        private_node_handle.param<std::string>("lidar_box_topic", lidar_box_topic_, "");
        LOG(INFO) << __APP_NAME__ << "lidar_box_topic:" << lidar_box_topic_.c_str();

        private_node_handle.param<std::string>("lidar_topic", lidar_topic_, "/kitti/velo/pointcloud");
        LOG(INFO) << __APP_NAME__ << "lidar_topic:" << lidar_topic_.c_str();

lionx_sub_=nh_.subscribe(lidar_topic_,100,&ProcessPointClouds::point_cb,this);
pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(lidar_box_topic_, 5);

ros::spin();
}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}
template<typename PointT>
void ProcessPointClouds<PointT>::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
   typename pcl::PointCloud<PointT>::Ptr current_pc_ptr(new pcl::PointCloud<PointT>);
   typename pcl::PointCloud<PointT>::Ptr filtered_pc_ptr(new pcl::PointCloud<PointT>);

    point_cloud_header_ = in_cloud_ptr->header;
    LOG(INFO)<<"cloud header:"<<point_cloud_header_;
    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    // down sampling the point cloud before cluster
    float filterRes = 0.4;
    Eigen::Vector4f minpoint(-10, -6.5, -2, 1);
    Eigen::Vector4f maxpoint(30, 6.5, 1, 1);
    //下采样 VoxelGrid
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered = this->FilterCloud(current_pc_ptr,0.3,
                                                                                       minpoint,
                                                                                       maxpoint);
    //分割地面 ransac3d
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>segmentCloud
            = this->SegmentPlane(cloudFiltered, 50, 0.3);
    //聚类
    std::vector<Detected_Obj> global_obj_list;
   std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters = this->Clustering(segmentCloud.first, 0.53, 10, 500,global_obj_list);
    LOG(INFO)<<"cloud obj numbers: "<<cloudClusters.size();


    jsk_recognition_msgs::BoundingBoxArray bbox_array;


    for (size_t i = 0; i < global_obj_list.size(); i++)
    {
        bbox_array.boxes.push_back(global_obj_list[i].bounding_box_);
    }
    bbox_array.header = point_cloud_header_;

    pub_bounding_boxs_.publish(bbox_array);

}

template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
                                               boost::filesystem::directory_iterator{});
    sort(paths.begin(),paths.end());
    return paths;
}
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                              float filterRes, Eigen::Vector4f minPoint,
                                                                              Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*cloudFiltered);

    // interesting region
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    ///remove ego car roof points	//提取车身周围范围内的所有的点，并将提取到的所有点保存在indices中
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6,1.7,-0.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);/// 默认false，提取索引内的点 remains ；true，提取索引外的点 move
    extract.filter(*cloudRegion);




    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    LOG(INFO) << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>  ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename  pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    for(int index : inliers-> indices)
        planeCloud -> points.push_back(cloud -> points[index]);
    //Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                                                                 int maxIterations,
                                                                                                                                 float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    //  TODO:: Fill in this function to find inliers for the cloud.
   /*
    * // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    // pcl::SACSegmentation<PointT> seg;
    // seg.setOptimizeCoefficients (true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);
    // seg.setInputCloud(cloud);
    // seg.segment(*inliers, *coefficients);
    // if(inliers->indices.size()==0){
    //     std::cout<<"could not find a plane model for the given dataset."<<std::endl;
    // }
*/
//ransac3d
///    unordered_set 无序 set 容器
    std::unordered_set<int> inliersResult;

    srand(time(NULL));//suijishu
    while(maxIterations--){
        std::unordered_set<int> inliers;
        while(inliers.size()<3)
        {
//            if(!cloud->points.size())continue;
            inliers.insert(rand()%(cloud->points.size()));}

        float x1, y1,z1, x2, y2,z2, x3, y3,z3;
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;
        float i = (y2 - y1)*(z3 - z1) - (z2-z1)*(y3 - y1);
        float j = (z2 - z1)*(x3 - x1) - (x2-x1)*(z3 - z1);
        float k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3-x1);
        float D = -(i*x1 + j*y1 + k*z1);
        for(int index=0;index< cloud->points.size();index++){
            if(inliers.count(index)>0)
                continue;
            PointT  point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;
            float temp=sqrt(i*i + j*j + k*k);

            float d = fabs(i*x4 + j*y4 + k*z4 +D)/sqrt(i*i + j*j + k*k);
               if(d< distanceThreshold)
                   inliers.insert(index);



        }
        if(inliers.size() > inliersResult.size()){
            inliersResult = inliers;
        }

    }

    pcl::PointIndices::Ptr inliers2(new pcl::PointIndices());
    for(int index=0; index< cloud->points.size();index++){
        //  PointT  point = cloud->points[index];
        if(inliersResult.count(index)){
            inliers2-> indices.push_back(index);
        }

    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    LOG(INFO) << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers2,cloud);
    return segResult;
}
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                     float clusterTolerance, int minSize, int maxSize,std::vector<Detected_Obj> &obj_list)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;//保存分割后的所有类 每一类为一个点云

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;// 欧式聚类对象
    ec.setClusterTolerance(clusterTolerance); //设置近邻搜索半径
    ec.setMinClusterSize(minSize);//设置一个类需要的最小的点数
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);//设置搜索方法
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);// 得到所有类别的索引 clusterIndices
    for(pcl::PointIndices getIndices: cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for(int index : getIndices.indices)		// For each point indice in each cluster
            cloudCluster-> points.push_back(cloud->points[index]);
        cloudCluster -> width = cloudCluster -> points.size();
        cloudCluster -> height = 1;
        cloudCluster -> is_dense = true;

        clusters.push_back(cloudCluster);
        
        Detected_Obj obj_info;
        obj_info.size=this->numPoints(cloudCluster);
        obj_info.box_msg_=this->BoundingBox(cloudCluster);
        LOG(INFO)<<"obj size:"<<obj_info.size;
        LOG(INFO)<<"obj BOx:"<<obj_info.box_msg_.centroid_;
          //calculate bounding box
        double length_ = obj_info.box_msg_.max_point_.x - obj_info.box_msg_.min_point_.x;
        double width_ = obj_info.box_msg_.max_point_.y - obj_info.box_msg_.min_point_.y;
        double height_ = obj_info.box_msg_.max_point_.z - obj_info.box_msg_.min_point_.z;

        obj_info.bounding_box_.header = point_cloud_header_;

        obj_info.bounding_box_.pose.position.x = obj_info.box_msg_.min_point_.x + length_ / 2;
        obj_info.bounding_box_.pose.position.y = obj_info.box_msg_.min_point_.y + width_ / 2;
        obj_info.bounding_box_.pose.position.z = obj_info.box_msg_.min_point_.z + height_ / 2;

        obj_info.bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
        obj_info.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
        obj_info.bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);
        LOG(INFO)<<"obj pose:"<<obj_info.bounding_box_;
        obj_list.push_back(obj_info);
        
    }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    LOG(INFO) << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
template<typename PointT>
double ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{

    
    return cloud->points.size();
}
template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    Box box;
    pcl::getMinMax3D(*cluster, box.min_point_, box.max_point_);

    Eigen::Vector4f centroid;  //质心 
    pcl::compute3DCentroid(*cluster,box.centroid_); // 计算质心

    return box;
}
template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}
///no used
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
    processed[idx] = true;
    cluster.push_back(idx);
    std::vector<int> nearest = tree->search(cloud->points[idx], distanceTol);
    for(auto id : nearest)
    {
        if(!processed[id])
            clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> processed(cloud->points.size(),false);
    for(size_t idx=0;idx< cloud->points.size(); ++idx){
        if(processed[idx] == false){
            std::vector<int> cluster_idx;
            typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
            clusterHelper(idx,cloud, cluster_idx, processed, tree, distanceTol);
            if(cluster_idx.size() >= minSize && cluster_idx.size() <= maxSize){
                for(int i=0; i<cluster_idx.size(); i++){
                    PointT point;
                    point = cloud->points[cluster_idx[i]];
                    cloudCluster->points.push_back(point);
                }
                cloudCluster->width = cloudCluster->points.size();
                cloudCluster->height = 1;
                clusters.push_back(cloudCluster);
            }else{
                for(int i=1;i<cluster_idx.size();i++){
                    processed[cluster_idx[i]] = false;
                }
            }
        }
    }
    return clusters;
}