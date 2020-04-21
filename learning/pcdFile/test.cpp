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
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>

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
//    for(pcl::PointXYZRGB point : cloud->points){
//        float curv = pcaEstimate.calculateCurvature(point);
//        if(it--<=0){
//            break;
//        }
//    }
    end = time(nullptr);

    std::cout << "finish to calculate 1000 times  curvature. time: "<<(end-start)<<" second"<<std::endl;


    std::cout << "start to calculate normals"<<std::endl;
//    steelArchExtraction.calculateNormal(*normals);
    std::cout << "finish calculate normals, normal size: "<<normals->size()<<std::endl;


    //harris3d关键点识别
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_out->height = 1;
    cloud_out->width =400;
    cloud_out->resize(cloud_out->height * cloud_out->width);
    cloud_out->clear();
    pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI, pcl::Normal> keypoint3D;
    keypoint3D.setInputCloud(cloud);
    keypoint3D.setRadius(_PARAM_->ARCH_STEEL_THICKNESS_);
    keypoint3D.setSearchMethod(tree_ne);
    keypoint3D.setNonMaxSupression(true);
    keypoint3D.setThreshold(0.0005f);

    std::cout << "start to compute keypoint"<<std::endl;
    start = time(nullptr);
    keypoint3D.compute(*cloud_out);
    end = time(nullptr);

    std::cout << "finish to compute. time: "<<(end-start)<<" second"<<std::endl;
    pcl::copyPointCloud(*cloud_out, *rgb_cloud_out);
    std::cout << "size: "<<rgb_cloud_out->size()<<std::endl;

    int R = 0xea;
    int G = 0x62;
    int B = 0x46;
    for(pcl::PointXYZRGB& point: rgb_cloud_out->points){
        point.r = R;
        point.g = G;
        point.b = B;
    }

    //narf keypoint
    pcl::RangeImage::Ptr rangeImage(new pcl::RangeImage);
    rangeImage->createFromPointCloud(*cloud,
            pcl::deg2rad (0.05f),
            pcl::deg2rad (360.f),
            pcl::deg2rad (180.f),
            Eigen::Affine3f::Identity ());
    // 提取NARF关键点
    pcl::RangeImageBorderExtractor range_image_border_extractor;//创建深度图像的边界提取器，用于提取NARF关键点
    pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor, 100);//创建NARF对象
    narf_keypoint_detector.setRangeImage (&(*rangeImage));//设置点云对应的深度图
//    narf_keypoint_detector.getParameters ().support_size = 100.f;// 感兴趣点的尺寸（球面的直径）

    pcl::PointCloud<int>::Ptr keypoint_indices(new pcl::PointCloud<int>);
    narf_keypoint_detector.compute(*keypoint_indices);
    std::cout << "Found size: "<<keypoint_indices->points.size ()<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr narf_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
    size_t size = (*keypoint_indices).points.size();
    narf_keypoints->resize(size);
    int i = 0;
    for(const int& index:(*keypoint_indices).points){
        narf_keypoints->points[i].getVector3fMap() = rangeImage->points[index].getVector3fMap();
        i++;
    }

//    pcl::visualization::CloudViewer viewer("Cloud Viewer");//创建viewer对象

    pcl::visualization::PCLVisualizer visualizer("Cloud visualizer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (rangeImage, 100, 0, 0);
    visualizer.addPointCloud (rangeImage, range_image_color_handler, "range image");//添加点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> narf_color_handler (narf_keypoints, 0, 255, 0);
    visualizer.addPointCloud (narf_keypoints, narf_color_handler, "narf image");//添加点云
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "narf image");


//    visualizer.addPointCloud(cloud);
//    visualizer.addPointCloud(rgb_cloud_out, "key point");
//    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "key point");


//    visualizer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(rockface_cloud, normals, 40, 500, "normal");

    while (!visualizer.wasStopped()) {
        visualizer.spinOnce(100);
    }


    return 0;
}

