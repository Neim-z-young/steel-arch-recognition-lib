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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

//user lib
#include "pclDBSCAN.h"
#include "../designLib/tunnelTool.h"
using designSpace::_PARAM_;

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



    std::cout << "After voxellization size is: " << (*cloud).points.size() << std::endl;

    //聚类参数

    std::cout << "start cluster..." << std::endl;
    designSpace::DBSCAN<pcl::PointXYZ> dbscan;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    dbscan.setInputCloud(cloud);
    dbscan.setRadius(_PARAM_.RADIUS_DBSCAN_);
    dbscan.setTree(tree);
    dbscan.setMinPtsPerCoreObject(_PARAM_.MIN_PTS_);
    dbscan.extract(cluster_indices);

    std::cout << "cluster size: " << cluster_indices.size() << std::endl;
    size_t total_size = 0, max_cluster_inx = 0, max_size = 0;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
        size_t size = cluster_indices[i].indices.size();
        if (max_size < size) {
            max_size = size;
            max_cluster_inx = i;
        }
        total_size += size;
    }
    std::cout << "total clustered point: " << total_size << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *color_cloud);
    //对所有聚类着色
//    for(const pcl::PointIndices& indices:cluster_indices){
//        int size = static_cast<int>(indices.indices.size());
//        if(size>=1){
//            int R = random()%256;
//            int G = random()%256;
//            int B = random()%256;
//            for(const int &i:indices.indices){
//                (*color_cloud).points[i].r = R;
//                (*color_cloud).points[i].g = G;
//                (*color_cloud).points[i].b = B;
//            }
//        }
//    }

    //提取出最大聚类点云并着色
    int R = random() % 256;
    int G = random() % 256;
    int B = random() % 256;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    clustered_color_cloud->resize(max_size);
    for(size_t i=0; i<max_size; i++){
        clustered_color_cloud->points[i].x = (*color_cloud).points[cluster_indices[max_cluster_inx].indices[i]].x;
        clustered_color_cloud->points[i].y = (*color_cloud).points[cluster_indices[max_cluster_inx].indices[i]].y;
        clustered_color_cloud->points[i].z = (*color_cloud).points[cluster_indices[max_cluster_inx].indices[i]].z;
        clustered_color_cloud->points[i].r = R;
        clustered_color_cloud->points[i].g = G;
        clustered_color_cloud->points[i].b = B;
    }

//    pcl::visualization::CloudViewer viewer("Cloud Viewer");//创建viewer对象
//    viewer.showCloud(clustered_color_cloud);
//
//
//    while (!viewer.wasStopped())//要想让自己所创窗口一直显示，则加上 while (!viewer.wasStopped()){ };即可.
//    {}

    std::cout<<"max cluster size: "<<clustered_color_cloud->points.size()<<std::endl;

    std::string path, name;
    designSpace::TunnelParameter::getPcdFileNameAndPath(filename, name, path);
    std::string out_file = path+"rgb_clustered_"+name;
    pcl::io::savePCDFileASCII(out_file, *clustered_color_cloud);
    std::cout<<"write rgb points to "<<out_file<<std::endl;

    //show
    pcl::visualization::PCLVisualizer visualizer("Cloud visualizer");
    visualizer.addPointCloud(clustered_color_cloud);
    while (!visualizer.wasStopped()) {
        visualizer.spinOnce(100);
    }
    return 0;
}
