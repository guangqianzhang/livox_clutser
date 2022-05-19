//
// Created by zgq on 2022/5/14.
//

#ifndef TEST5_RENDER_H
#define TEST5_RENDER_H
#include <pcl/visualization/pcl_visualizer.h>
#include "Box.h"
enum CameraAngle
{
    XY, TopDown, Side, FPS
};
struct Color
{

    float r, g, b;

    Color(float setR, float setG, float setB)
            : r(setR), g(setG), b(setB)
    {}
};

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color = Color(1,1,1));
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color = Color(-1,-1,-1));
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color = Color(1,0,0), float opacity=1);
/////////////////////////*************************//////////////////
struct Vect3
{

    double x, y, z;

    Vect3(double setX, double setY, double setZ)
            : x(setX), y(setY), z(setZ)
    {}

    Vect3 operator+(const Vect3& vec)
    {
        Vect3 result(x+vec.x,y+vec.y,z+vec.z);
        return result;
    }
};
struct Car
{

    // units in meters
    Vect3 position, dimensions;

    std::string name;
    Color color;

    Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, std::string setName)
            : position(setPosition), dimensions(setDimensions), color(setColor), name(setName)
    {}

    void render(pcl::visualization::PCLVisualizer::Ptr& viewer)
    {
        // render bottom of car
        //viewer->addCube 向视图中添加一个立方体模型
        viewer->addCube(position.x-dimensions.x/2, position.x+dimensions.x/2, position.y-dimensions.y/2, position.y+dimensions.y/2, position.z, position.z+dimensions.z*2/3, color.r, color.g, color.b, name);
        // setShapeRenderingProperties 设置格子的属性
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name);
        // render top of car
        viewer->addCube(position.x-dimensions.x/4, position.x+dimensions.x/4, position.y-dimensions.y/2, position.y+dimensions.y/2, position.z+dimensions.z*2/3, position.z+dimensions.z, color.r, color.g, color.b, name+"Top");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name+"Top");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name+"Top");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name+"Top");
    }

    // collision helper function
    bool inbetween(double point, double center, double range)
    {
        return (center-range <= point) && (center+range >= point);
    }

    bool checkCollision(Vect3 point)
    {
        return (inbetween(point.x,position.x,dimensions.x/2)&&inbetween(point.y,position.y,dimensions.y/2)&&inbetween(point.z,position.z+dimensions.z/3,dimensions.z/3))||
               (inbetween(point.x,position.x,dimensions.x/4)&&inbetween(point.y,position.y,dimensions.y/2)&&inbetween(point.z,position.z+dimensions.z*5/6,dimensions.z/6));

    }
};
void renderHighway(pcl::visualization::PCLVisualizer::Ptr& viewer);



#endif //TEST5_RENDER_H
