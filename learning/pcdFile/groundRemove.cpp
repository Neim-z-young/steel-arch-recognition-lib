//
// Created by oyoungy on 2020/3/30.
//

//
// Created by oyoungy on 2020/3/25.
//

//std
#include<iostream>

#include <math.h>

//pcl
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
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

//user lib
#include "../designLib/tunnelTool.h"
#include "groundHelper.h"
using designSpace::_PARAM_;

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


    std::cout << "Clustered points size is: " << (*clustered_color_cloud).points.size() << std::endl;

    boost::shared_ptr<std::vector<int>> cluster_indices(new std::vector<int>(clustered_color_cloud->points.size()));
    for(size_t i=0; i<clustered_color_cloud->points.size(); i++){
        (*cluster_indices)[i] = i;
    }

    time_t start, end;


    designSpace::GroundRemoval<pcl::PointXYZRGB> groundRemoval;
    pcl::PointIndices::Ptr remain_indices_ptr(new pcl::PointIndices);
    groundRemoval.setAxis('z');
    groundRemoval.setGroundHeight(_PARAM_.GROUND_HEIGHT_);
    groundRemoval.setInputCloud(clustered_color_cloud);
    groundRemoval.setIndices(cluster_indices);


    start = time(nullptr);
    std::cout << "start to remove ground: "<<std::endl;
    groundRemoval.remove(*remain_indices_ptr);
    end = time(nullptr);
    std::cout << "finish to remove ground. time: "<<(end-start)<<" second"<<std::endl;

    std::cout<<"remain size: "<<remain_indices_ptr->indices.size()<<std::endl;


//    pcl::visualization::CloudViewer viewer("Cloud Viewer");//创建viewer对象

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*clustered_color_cloud, remain_indices_ptr->indices, *ground_removed_cloud);

    pcl::visualization::PCLVisualizer visualizer("Cloud visualizer");

    visualizer.addPointCloud(ground_removed_cloud);
    //visualizer.initCameraParameters ();

    while (!visualizer.wasStopped()) {
        visualizer.spinOnce(100);
    }

//    viewer.showCloud(clustered_color_cloud);


//    while (!viewer.wasStopped())//要想让自己所创窗口一直显示，则加上 while (!viewer.wasStopped()){ };即可.
//    {}

    std::string path, name;
    designSpace::TunnelParameter::getPcdFileNameAndPath(filename, name, path);
    std::string out_file = path+"groundRemoved_"+name;
    pcl::io::savePCDFileASCII(out_file, *ground_removed_cloud);
    std::cout<<"write rgb points to "<<out_file<<std::endl;

    return 0;
}
