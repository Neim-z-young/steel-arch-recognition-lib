//
// Created by oyoungy on 2020/3/25.
//


//std
#include<iostream>

#include <math.h>

//pcd
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

//user lib
#include "calibratingTool.h"

int main(int, char **argv) {
    std::string filename = argv[1];
    std::cout << "Reading " << filename << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *clustered_color_cloud) == -1)
        // load the file
    {
        PCL_ERROR ("Couldn't read file");
        return (-1);
    }
    std::cout << "Loaded " << clustered_color_cloud->points.size() << " points." << std::endl;

    float multiple = 2500, radius = 0.3;
    int min_pts = 200;

    std::cout << "Clustered points size is: " << (*clustered_color_cloud).points.size() << std::endl;

    float x = 0, y = 0, z = 0;
    for(size_t i=0; i<clustered_color_cloud->points.size(); i++){
        x+=clustered_color_cloud->points[i].x;
        y+=clustered_color_cloud->points[i].y;
        z+=clustered_color_cloud->points[i].z;
    }
    x/=clustered_color_cloud->points.size();
    y/=clustered_color_cloud->points.size();
    z/=clustered_color_cloud->points.size();

//    pcl::visualization::CloudViewer viewer("Cloud Viewer");//创建viewer对象

    pcl::visualization::PCLVisualizer visualizer("Cloud visualizer");
    visualizer.addPointCloud(clustered_color_cloud);
    visualizer.addCoordinateSystem(10000, x, y, z);
    //visualizer.initCameraParameters ();

    while(!visualizer.wasStopped()){
        visualizer.spinOnce(100);
    }

//    viewer.showCloud(clustered_color_cloud);


//    while (!viewer.wasStopped())//要想让自己所创窗口一直显示，则加上 while (!viewer.wasStopped()){ };即可.
//    {}

    return 0;
}
