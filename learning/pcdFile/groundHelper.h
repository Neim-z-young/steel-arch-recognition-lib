//
// Created by oyoungy on 2020/3/30.
//

#ifndef PCLDEMO_GROUNDHELPER_H
#define PCLDEMO_GROUNDHELPER_H

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
#include <pcl/features/normal_3d.h>

namespace designSpace {
/**
 地面去除步骤
 使用时应先调用setInputCloud()和setIndices()
 * @tparam PointT
 */
    template<typename PointT>
    class GroundRemoval : public pcl::PCLBase<PointT> {
        using BasePCLBase = pcl::PCLBase<PointT>;
    public:
        using PointCloud = pcl::PointCloud<PointT>;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;

        using KdTree = pcl::search::Search<PointT>;
        using KdTreePtr = typename KdTree::Ptr;

        using PointIndicesPtr = pcl::PointIndices::Ptr;
        using PointIndicesConstPtr = pcl::PointIndices::ConstPtr;

        using IndicesPtr = boost::shared_ptr<std::vector<int> >;
        using IndicesConstPtr = boost::shared_ptr<const std::vector<int> >;

        //custom


        GroundRemoval() : ground_height_(1),
                          axis_('z'){}

        float getGroundHeight() const {
            return ground_height_;
        }

        void setGroundHeight(float groundHeight) {
            ground_height_ = groundHeight;
        }

        char getAxis() const {
            return axis_;
        }

        void setAxis(char axis) {
            axis_ = axis;
        }

        void remove(pcl::PointIndices &surface_indices) {
            if (!initCompute() ||
                (input_ && input_->points.empty()) ||
                (indices_ && indices_->empty())) {
                return;
            }
            float min_x = FLT_MAX, max_x = FLT_MIN, min_y = FLT_MAX, max_y = FLT_MIN, min_z = FLT_MAX, max_z = FLT_MIN;
            for (size_t i = 0; i < indices_->size(); i++) {
                min_x = fmin(min_x, input_->points[(*indices_)[i]].x);
                min_y = fmin(min_y, input_->points[(*indices_)[i]].y);
                min_z = fmin(min_z, input_->points[(*indices_)[i]].z);

                max_x = fmax(max_x, input_->points[(*indices_)[i]].x);
                max_y = fmax(max_y, input_->points[(*indices_)[i]].y);
                max_z = fmax(max_z, input_->points[(*indices_)[i]].z);

            }
            surface_indices.indices.reserve(indices_->size());
            switch(axis_){
                case 'z':{
                    for (size_t i = 0; i < indices_->size(); i++) {
                        if(input_->points[(*indices_)[i]].z >= min_z+ground_height_){
                            surface_indices.indices.push_back((*indices_)[i]);
                        }
                    }
                    break;
                }
                default:
                    break;
            }
        }



    protected:
        // Members derived from the base class
        using BasePCLBase::input_;
        using BasePCLBase::indices_;
        using BasePCLBase::initCompute;
        using BasePCLBase::deinitCompute;


    private:
        float ground_height_;
        char axis_;

    };
}

#endif //PCLDEMO_GROUNDHELPER_H
