//
// Created by oyoungy on 2020/4/17.
//

#ifndef PCLDEMO_PCAESTIMATE_H
#define PCLDEMO_PCAESTIMATE_H
//std
#include<iostream>

#include <math.h>

//pcl
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/io/io.h>
#include <pcl/search/kdtree.h>

//Eigen3
#include <Eigen/Dense>

namespace designSpace {

    template<typename PointT>
    class PCAEstimate : public pcl::PCLBase<PointT> {
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

        PCAEstimate():radius_(0.f),
                      k_(0),
                      tree_(){}

        const KdTreePtr &getTree() const {
            return tree_;
        }

        void setTree(const KdTreePtr &tree) {
            tree_ = tree;
        }

        double getRadius() const {
            return radius_;
        }

        void setRadius(double radius) {
            radius_ = radius;
        }

        int getK() const {
            return k_;
        }

        void setK(int k) {
            k_ = k;
        }

        //通过主成分分析计算曲率
        float calculateCurvature(PointT point){
            tree_->setInputCloud(input_, indices_);
            std::vector<int> k_indices(k_);
            std::vector<float> k_sqr_distances(k_);
            tree_->radiusSearch(point, radius_, k_indices, k_sqr_distances);
            float sqr_radius = radius_ * radius_;
            if (k_sqr_distances[k_sqr_distances.size() - 1] > sqr_radius) {
                //TODO fix it
                //something wrong
                std::cerr << "warning!! distance is larger than radius , squared distance is: "
                          << k_sqr_distances[k_sqr_distances.size() - 1] << std::endl;
            }
            Eigen::Vector3f centroid(0, 0, 0);
            for(const int& index : k_indices){
                centroid(0) += input_->points[index].x;
                centroid(1) += input_->points[index].y;
                centroid(2) += input_->points[index].z;
            }
            centroid /= k_indices.size();

            //covariance matrix
            Eigen::Matrix3f cov_mat;
            cov_mat<<0, 0, 0,
                     0, 0, 0,
                     0, 0, 0;
            for(const int& index : k_indices){
                Eigen::Vector3f tmp(input_->points[index].x, input_->points[index].y, input_->points[index].z);
                float xx = tmp(0)-centroid(0);
                float yy = tmp(1)-centroid(1);
                float zz = tmp(2)-centroid(2);
                cov_mat(0,0) += xx*xx;
                cov_mat(0,1) += xx*yy;
                cov_mat(0,2) += xx*zz;

                cov_mat(1,0) += xx*yy;
                cov_mat(1,1) += yy*yy;
                cov_mat(1,2) += yy*zz;

                cov_mat(2,0) += xx*zz;
                cov_mat(2,1) += yy*zz;
                cov_mat(2,2) += zz*zz;
            }
            cov_mat /= k_indices.size();

            cov_mat = cov_mat.transpose()*cov_mat;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(cov_mat);
            if (eigensolver.info() != Eigen::Success) abort();
            eigensolver.eigenvalues();
            cov_mat.trace();
            float curv = std::abs(eigensolver.eigenvalues()(0)/cov_mat.trace());
            return curv;
        }

    protected:
        // Members derived from the base class
        using BasePCLBase::input_;
        using BasePCLBase::indices_;
        using BasePCLBase::initCompute;
        using BasePCLBase::deinitCompute;

    private:
        KdTreePtr tree_;

        /** judge distance less than radius. using euclidean distance */
        double radius_;

        /** nearest k points to compute curvature. */
        int k_;

    };
}

#endif //PCLDEMO_PCAESTIMATE_H
