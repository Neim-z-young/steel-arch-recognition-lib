//
// Created by oyoungy on 2020/4/18.
//
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
#include "../designLib/tunnelTool.h"
#include "../designLib/PCATool.h"
using designSpace::_PARAM_;

int main(int, char **argv) {
    std::string filename = argv[1];
    std::cout << "Reading " << filename << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud) == -1)
        // load the file
    {
        PCL_ERROR ("Couldn't read file");
        return (-1);
    }
    std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

    float multiple = 2500, radius = 0.3;
    int min_pts = 200;

    std::cout << "Clustered points size is: " << (*cloud).points.size() << std::endl;


    time_t start, end;

    float x = 0, y = 0, z = 0, min_x = FLT_MAX, max_x = FLT_MIN, min_y = FLT_MAX, max_y = FLT_MIN, min_z = FLT_MAX, max_z = FLT_MIN;
    for (size_t i = 0; i < cloud->points.size(); i++) {
        min_x = fmin(min_x, cloud->points[i].x);
        min_y = fmin(min_y, cloud->points[i].y);
        min_z = fmin(min_z, cloud->points[i].z);

        max_x = fmax(max_x, cloud->points[i].x);
        max_y = fmax(max_y, cloud->points[i].y);
        max_z = fmax(max_z, cloud->points[i].z);

        x += cloud->points[i].x;
        y += cloud->points[i].y;
        z += cloud->points[i].z;
    }
    //重心
    x /= cloud->points.size();
    y /= cloud->points.size();
    z /= cloud->points.size();

    std::cout << "center point is: x:" << x << " y:" << y << " z:" << z << std::endl;

    std::cout << "x axis range is: " <<max_x - min_x<< std::endl;
    std::cout << "y axis range is: " <<max_y - min_y<< std::endl;
    std::cout << "z axis range is: " <<max_z - min_z<< std::endl;


    boost::shared_ptr<std::vector<int>> rockface_indices(new std::vector<int>(cloud->points.size()));
    for(size_t i=0; i < cloud->points.size(); i++){
        (*rockface_indices)[i] = i;
    }

    //计算点云法向
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_ne(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setInputCloud(cloud);
    ne.setIndices(rockface_indices);
    ne.setViewPoint(x, y, z);
    tree_ne->setInputCloud(cloud, rockface_indices);
    ne.setSearchMethod(tree_ne);
    ne.setRadiusSearch(_PARAM_->RADIUS_FOR_C_N_*0.4);
//    ne.setKSearch(_PARAM_->K_FOR_C_N_);
//    ne.compute(*normals);

    designSpace::PCAEstimate<pcl::PointXYZRGB> pcaEstimate;
    pcaEstimate.setInputCloud(cloud);
    pcaEstimate.setTree(tree_ne);
    pcaEstimate.setK(_PARAM_->K_FOR_C_N_);
    pcaEstimate.setRadius(_PARAM_->RADIUS_FOR_C_N_);

    int it = 1000;
    start = time(nullptr);
    for(pcl::PointXYZRGB point : cloud->points){
        float curv = pcaEstimate.calculateCurvature(point);
        if(it--<=0){
            break;
        }
    }
    end = time(nullptr);

    std::cout << "finish to calculate 1000 times  curvature. time: "<<(end-start)<<" second"<<std::endl;


    std::cout << "start to calculate normals"<<std::endl;
//    steelArchExtraction.calculateNormal(*normals);
    std::cout << "finish calculate normals, normal size: "<<normals->size()<<std::endl;



//    pcl::visualization::CloudViewer viewer("Cloud Viewer");//创建viewer对象

    pcl::visualization::PCLVisualizer visualizer("Cloud visualizer");

    visualizer.addPointCloud(cloud);


//    visualizer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(rockface_cloud, normals, 40, 500, "normal");

    while (!visualizer.wasStopped()) {
        visualizer.spinOnce(100);
    }


    return 0;
}

