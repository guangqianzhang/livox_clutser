//
// Created by zgq on 2022/5/14.
//

#ifndef TEST5_BOX_H
#define TEST5_BOX_H
#include <Eigen/Core>
#include <Eigen/Geometry>
struct Box
{
pcl::PointXYZ min_point_;
    pcl::PointXYZ max_point_;

   Eigen::Vector4f centroid_;
};

#endif //TEST5_BOX_H
