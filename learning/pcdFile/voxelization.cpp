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
    float sensorResolution = 0.05, scanSize = 30, multiple = 2500;
    float voxelSize =multiple*scanSize*tan(M_PI/180*0.05);
    voxelGrid.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxelGrid.filter(*voxelFilterCloud);

    std::cout<<"before voxelization size is: "<<(*cloud).points.size()<<std::endl;
    std::cout<<"Voxel size is: "<<voxelSize<<std::endl;
    std::cout<<"After voxellization size is: "<<(*voxelFilterCloud).points.size()<<std::endl;

    std::string out_file = "/home/oyoungy/Documents/DATA/xyz_voxelization.pcd";
    pcl::io::savePCDFileASCII(out_file, *voxelFilterCloud);

    std::cout<<"write points to "<<out_file<<std::endl;
}
