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
#include "../designLib/tunnelTool.h"
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

    boost::shared_ptr<std::vector<int>> clustered_indices(new std::vector<int>(clustered_color_cloud->points.size()));
    for(size_t i=0; i<clustered_color_cloud->points.size(); i++){
        (*clustered_indices)[i] = i;
    }
    time_t start, end;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr calibrated_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    designSpace::Calibration<pcl::PointXYZRGB> calibration;
    calibration.setInputCloud(clustered_color_cloud);
    calibration.setIndices(clustered_indices);
    calibration.setProjectionGridSize(_PARAM_.ARCH_STEEL_THICKNESS_);
    calibration.setStartAngle(-50.f);
    calibration.setAngleStep(3.f);
    calibration.setEndAngle(50.f);

    start = time(nullptr);
    std::cout << "start to calibrate: "<<std::endl;
    calibration.calibrate(calibrated_cloud);
    end = time(nullptr);

    std::cout << "finish to calibrate. time: "<<(end-start)<<" second"<<std::endl;


    float x = 0, y = 0, z = 0;
    for(size_t i=0; i<calibrated_cloud->points.size(); i++){
        x+=calibrated_cloud->points[i].x;
        y+=calibrated_cloud->points[i].y;
        z+=calibrated_cloud->points[i].z;
    }
    x/=calibrated_cloud->points.size();
    y/=calibrated_cloud->points.size();
    z/=calibrated_cloud->points.size();

//    pcl::visualization::CloudViewer viewer("Cloud Viewer");//创建viewer对象

    std::string path, name;
    designSpace::TunnelParameter::getPcdFileNameAndPath(filename, name, path);
    std::string out_file = path+"calibrated_"+name;
    pcl::io::savePCDFileASCII(out_file, *calibrated_cloud);

    std::cout<<"write points to "<<out_file<<std::endl;

    pcl::visualization::PCLVisualizer visualizer("Cloud visualizer");
    visualizer.addPointCloud(calibrated_cloud);
    visualizer.addCoordinateSystem(10000, x, y, z);
    //visualizer.initCameraParameters ();

    while(!visualizer.wasStopped()){
        visualizer.spinOnce(100);
    }


    calibration.plotAngleEvaluateValue();
//    viewer.showCloud(clustered_color_cloud);


//    while (!viewer.wasStopped())//要想让自己所创窗口一直显示，则加上 while (!viewer.wasStopped()){ };即可.
//    {}

    return 0;
}
