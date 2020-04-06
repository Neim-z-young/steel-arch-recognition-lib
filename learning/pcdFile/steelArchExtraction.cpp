//
// Created by oyoungy on 2020/4/1.
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

//user lib
#include "steelArchHelper.h"

int main(int, char **argv) {
    std::string filename = argv[1];
    std::cout << "Reading " << filename << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rockface_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *rockface_cloud) == -1)
        // load the file
    {
        PCL_ERROR ("Couldn't read file");
        return (-1);
    }
    std::cout << "Loaded " << rockface_cloud->points.size() << " points." << std::endl;

    float multiple = 2500, radius = 0.3;
    int min_pts = 200;

    std::cout << "Clustered points size is: " << (*rockface_cloud).points.size() << std::endl;


    time_t start, end;

    float x = 0, y = 0, z = 0, min_x = FLT_MAX, max_x = FLT_MIN, min_y = FLT_MAX, max_y = FLT_MIN, min_z = FLT_MAX, max_z = FLT_MIN;
    for (size_t i = 0; i < rockface_cloud->points.size(); i++) {
        min_x = fmin(min_x, rockface_cloud->points[i].x);
        min_y = fmin(min_y, rockface_cloud->points[i].y);
        min_z = fmin(min_z, rockface_cloud->points[i].z);

        max_x = fmax(max_x, rockface_cloud->points[i].x);
        max_y = fmax(max_y, rockface_cloud->points[i].y);
        max_z = fmax(max_z, rockface_cloud->points[i].z);

        x += rockface_cloud->points[i].x;
        y += rockface_cloud->points[i].y;
        z += rockface_cloud->points[i].z;
    }
    //重心
    x /= rockface_cloud->points.size();
    y /= rockface_cloud->points.size();
    z /= rockface_cloud->points.size();

    std::cout << "center point is: x:" << x << " y:" << y << " z:" << z << std::endl;

    std::cout << "x axis range is: " <<max_x - min_x<< std::endl;
    std::cout << "y axis range is: " <<max_y - min_y<< std::endl;
    std::cout << "z axis range is: " <<max_z - min_z<< std::endl;


    boost::shared_ptr<std::vector<int>> rockface_indices(new std::vector<int>(rockface_cloud->points.size()));
    for(size_t i=0; i<rockface_cloud->points.size(); i++){
        (*rockface_indices)[i] = i;
    }

    //计算点云法向
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_ne(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setInputCloud(rockface_cloud);
    ne.setIndices(rockface_indices);
    ne.setViewPoint(6012.96, -81.7, 28.68);
    tree_ne->setInputCloud(rockface_cloud, rockface_indices);
    ne.setSearchMethod(tree_ne);
    ne.setKSearch(40);
//    ne.compute(*normals);


    std::cout << "setting parameter..."<<std::endl;
    //TODO 参数设置
    float Wa = 1*1000, radius_d = 0.5f * Wa, segment_length = 0.1f * Wa, arch_thick = 1000*0.2;
    int k = 40;
    designSpace::SteelArchExtraction<pcl::PointXYZRGB> steelArchExtraction;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    steelArchExtraction.setInputCloud(rockface_cloud);
    steelArchExtraction.setIndices(rockface_indices);
    steelArchExtraction.setK(k);
    steelArchExtraction.setRadius(radius_d);
    steelArchExtraction.setTree(tree);
    steelArchExtraction.setArchThickness(arch_thick);
    steelArchExtraction.setSteelArchGap(Wa);
    steelArchExtraction.setStartArchGap(0.);
    steelArchExtraction.setViewPoint(x, y, z);

    pcl::PointIndices::Ptr steel_arch_indices_ptr(new pcl::PointIndices);
    std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> steel_arch_points;
    start = time(nullptr);
    std::cout << "start to extract steel arch: "<<std::endl;
    steelArchExtraction.extract(steel_arch_points);
    end = time(nullptr);
    std::cout << "finish to extract steel arch. time: "<<(end-start)<<" second"<<std::endl;

    std::cout << "steel arch point number: "<<steel_arch_points.size()<<std::endl;

    std::cout << "start to calculate normals"<<std::endl;
//    steelArchExtraction.calculateNormal(*normals);
    std::cout << "finish calculate normals, normal size: "<<normals->size()<<std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr steel_arch_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //提取出最大聚类点云并着色
    int R = 0xea;
    int G = 0x62;
    int B = 0x46;

    for(pcl::PointXYZRGB& point:steel_arch_points){
        point.r = R;
        point.g = G;
        point.b = B;
        steel_arch_cloud->push_back(point);
        //rockface_cloud->push_back(point);
    }


//    pcl::visualization::CloudViewer viewer("Cloud Viewer");//创建viewer对象

    pcl::visualization::PCLVisualizer visualizer("Cloud visualizer");
    visualizer.addCoordinateSystem(10000, x, y, z);

//    visualizer.addPointCloud(rockface_cloud);
    //visualizer.initCameraParameters ();

    visualizer.addPointCloud(steel_arch_cloud, "steel");

    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "steel");

//    visualizer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(rockface_cloud, normals, 40, 500, "normal");

    while (!visualizer.wasStopped()) {
        visualizer.spinOnce(100);
    }


    return 0;
}
