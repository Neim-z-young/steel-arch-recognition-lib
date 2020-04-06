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
#include "rockfaceHelper.h"

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


    time_t start, end;

    float x = 0, y = 0, z = 0, min_x = FLT_MAX, max_x = FLT_MIN, min_y = FLT_MAX, max_y = FLT_MIN, min_z = FLT_MAX, max_z = FLT_MIN;
    for (size_t i = 0; i < clustered_color_cloud->points.size(); i++) {
        min_x = fmin(min_x, clustered_color_cloud->points[i].x);
        min_y = fmin(min_y, clustered_color_cloud->points[i].y);
        min_z = fmin(min_z, clustered_color_cloud->points[i].z);

        max_x = fmax(max_x, clustered_color_cloud->points[i].x);
        max_y = fmax(max_y, clustered_color_cloud->points[i].y);
        max_z = fmax(max_z, clustered_color_cloud->points[i].z);

        x += clustered_color_cloud->points[i].x;
        y += clustered_color_cloud->points[i].y;
        z += clustered_color_cloud->points[i].z;
    }
    //重心
    x /= clustered_color_cloud->points.size();
    y /= clustered_color_cloud->points.size();
    z /= clustered_color_cloud->points.size();

    std::cout << "center point is: x:" << x << " y:" << y << " z:" << z << std::endl;

    std::cout << "x axis range is: " <<max_x - min_x<< std::endl;
    std::cout << "y axis range is: " <<max_y - min_y<< std::endl;
    std::cout << "z axis range is: " <<max_z - min_z<< std::endl;


    //TODO 参数设置
    float Wa = 1*1000, radius_d = 0.3f * Wa, segment_length = 0.1f * Wa;
    int k = 20;
    designSpace::RockfaceExtraction<pcl::PointXYZRGB> rockfaceExtraction;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    rockfaceExtraction.setInputCloud(clustered_color_cloud);
    rockfaceExtraction.setTree(tree);
    rockfaceExtraction.setAxis('x');
    rockfaceExtraction.setRadius(radius_d);
    rockfaceExtraction.setK(k);
    rockfaceExtraction.setSegmentLength(segment_length);


    pcl::PointIndices::Ptr rockface_indices_ptr(new pcl::PointIndices);

    start = time(nullptr);
    std::cout << "start to extract rockface: "<<std::endl;
    int seg_inx = rockfaceExtraction.extract(*rockface_indices_ptr);
    end = time(nullptr);

    std::cout << "finish to extract rockface. time: "<<(end-start)<<" second"<<std::endl;
    designSpace::RockfaceExtraction<pcl::PointXYZRGB>::IndicesConstPtr m1_indices =
            rockfaceExtraction.getSegmentIndices()[seg_inx];

    std::cout<<"extracted size: "<<rockface_indices_ptr->indices.size()<<std::endl;

    std::string out_file = "/home/oyoungy/Documents/DATA/rgb_xyz_clustered_rockface.pcd";
    pcl::io::savePCDFile(out_file, *clustered_color_cloud, rockface_indices_ptr->indices);
    std::cout<<"write rgb points to "<<out_file<<std::endl;

//    random();
    int R = 135;
    int G = 10;
    int B = 200;
    for(int t:rockface_indices_ptr->indices){
        clustered_color_cloud->points[t].r = R;
        clustered_color_cloud->points[t].g = G;
        clustered_color_cloud->points[t].b = B;
    }
    //找到曲率查分变化最大的段
     R = 10;
     G = 10;
     B = 200;
    for(int t:*m1_indices){
        clustered_color_cloud->points[t].r = R;
        clustered_color_cloud->points[t].g = G;
        clustered_color_cloud->points[t].b = B;
    }


//    pcl::visualization::CloudViewer viewer("Cloud Viewer");//创建viewer对象

    pcl::visualization::PCLVisualizer visualizer("Cloud visualizer");
    visualizer.addCoordinateSystem(10000, x, y, z);

    visualizer.addPointCloud(clustered_color_cloud);
    //visualizer.initCameraParameters ();

    while (!visualizer.wasStopped()) {
        visualizer.spinOnce(100);
    }

    rockfaceExtraction.plotCurvatureRelation();

    rockfaceExtraction.plotHeightRelation();

//    viewer.showCloud(clustered_color_cloud);


//    while (!viewer.wasStopped())//要想让自己所创窗口一直显示，则加上 while (!viewer.wasStopped()){ };即可.
//    {}

    return 0;
}
