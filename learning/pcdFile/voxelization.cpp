//
// Created by oyoungy on 2020/3/23.
//
#include <math.h>

//pcd
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/filters/voxel_grid.h>

//userlib
#include "../designLib/tunnelTool.h"
using designSpace::_PARAM_;

int main(int, char** argv)
{
    std::string filename = argv[1];
    std::cout << "Reading " << filename << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile <pcl::PointXYZ> (filename, *cloud) == -1)
        // load the file
    {
        PCL_ERROR ("Couldn't read file");
        return (-1);
    }
    std::cout << "Loaded " << cloud->points.size () << " points." << std::endl;

    //体素滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelFilterCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);

    voxelGrid.setLeafSize(_PARAM_.VOXEL_SIZE_, _PARAM_.VOXEL_SIZE_, _PARAM_.VOXEL_SIZE_);
    voxelGrid.filter(*voxelFilterCloud);

    std::cout<<"before voxelization size is: "<<(*cloud).points.size()<<std::endl;
    std::cout << "Voxel size is: " << _PARAM_.VOXEL_SIZE_ << std::endl;
    std::cout<<"After voxellization size is: "<<(*voxelFilterCloud).points.size()<<std::endl;

    std::string path, name;
    designSpace::TunnelParameter::getPcdFileNameAndPath(filename, name, path);
    std::string out_file = path+"voxelization_"+name;
    pcl::io::savePCDFileASCII(out_file, *voxelFilterCloud);

    std::cout<<"write points to "<<out_file<<std::endl;

    //show
    pcl::visualization::PCLVisualizer visualizer("Cloud visualizer");
    visualizer.addPointCloud(voxelFilterCloud);
    while (!visualizer.wasStopped()) {
        visualizer.spinOnce(100);
    }
}
