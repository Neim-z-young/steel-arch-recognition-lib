//
// Created by oyoungy on 2020/1/6.
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
#include "pclDBSCAN.h"

int main(int, char **argv) {
    std::string filename = argv[1];
    std::cout << "Reading " << filename << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
        // load the file
    {
        PCL_ERROR ("Couldn't read file");
        return (-1);
    }
    std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

    float multiple = 2500, radius = 0.3;
    int min_pts = 200;

    std::cout << "After voxellization size is: " << (*cloud).points.size() << std::endl;

    //点云过滤
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> removal;


    pcl::visualization::CloudViewer viewer("Cloud Viewer");//创建viewer对象
    //blocks until the cloud is actually rendered
//    viewer.showCloud(cloud);//将pcb与viewer对象联系起来
    viewer.showCloud(cloud);

    while (!viewer.wasStopped())//要想让自己所创窗口一直显示，则加上 while (!viewer.wasStopped()){ };即可.
    {
    }

    return 0;
}
