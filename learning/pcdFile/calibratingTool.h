//
// Created by oyoungy on 2020/3/25.
//

#ifndef PCLDEMO_CALIBRATINGTOOL_H
#define PCLDEMO_CALIBRATINGTOOL_H

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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/common/common.h>

#include <Eigen/Dense>

//user lib
#include "../designLib/tunnelTool.h"
#include "groundHelper.h"

using designSpace::_PARAM_;
namespace designSpace {

    /**
 将点云绕某一轴旋转theta度数
 通过旋转矩阵计算
 */

    //内部类
    template<typename PointT>
    class PlaneGrid : protected pcl::VoxelGrid<PointT> {
    public:
        using BasePCLBase = pcl::PCLBase<PointT>;
        using VoxelBase = pcl::VoxelGrid<PointT>;

        using BasePCLBase::setInputCloud;
        using BasePCLBase::setIndices;
        using VoxelBase::setLeafSize;


        PlaneGrid() : plane_("yoz") {}

        const std::string &getPlane() const {
            return plane_;
        }

        void setPlane(const std::string &plane) {
            plane_ = plane;
        }

        //评估点云在某一坐标平面上的投影大小
        float evaluateByVoxel() {
            // Has the input dataset been set already?
            if (!initCompute() ||
                (input_ && input_->points.empty()) ||
                (indices_ && indices_->empty())) {
                assert(input_ != nullptr);
                return 0.;
            }

            Eigen::Vector4f min_p, max_p;
            // Get the minimum and maximum dimensions
            if (!filter_field_name_.empty()) // If we don't want to process the entire cloud...
                pcl::getMinMax3D<PointT>(input_, *indices_, filter_field_name_, static_cast<float> (filter_limit_min_),
                                         static_cast<float> (filter_limit_max_), min_p, max_p, filter_limit_negative_);
            else
                pcl::getMinMax3D<PointT>(*input_, *indices_, min_p, max_p);

            // Check that the leaf size is not too small, given the size of the data
            std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
            std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
            std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

            if ((dx * dy * dz) > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
                return -1;
            }

            // Compute the minimum and maximum bounding box values
            min_b_[0] = static_cast<int> (std::floor(min_p[0] * inverse_leaf_size_[0]));
            max_b_[0] = static_cast<int> (std::floor(max_p[0] * inverse_leaf_size_[0]));
            min_b_[1] = static_cast<int> (std::floor(min_p[1] * inverse_leaf_size_[1]));
            max_b_[1] = static_cast<int> (std::floor(max_p[1] * inverse_leaf_size_[1]));
            min_b_[2] = static_cast<int> (std::floor(min_p[2] * inverse_leaf_size_[2]));
            max_b_[2] = static_cast<int> (std::floor(max_p[2] * inverse_leaf_size_[2]));

            // Compute the number of divisions needed along all axis
            div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
            div_b_[3] = 0;

            // Set up the division multiplier
            divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

            // Storage for mapping leaf and pointcloud indexes
            std::unordered_map<int, int> voxel_map;
            voxel_map.reserve(indices_->size());

            // process the entire cloud
            // First pass: go over all points and insert them into the index_vector vector
            // with calculated idx. Points with the same idx value will contribute to the
            // same point of resulting CloudPoint
            for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it) {
                if (!input_->is_dense)
                    // Check if the point is invalid
                    if (!std::isfinite(input_->points[*it].x) ||
                        !std::isfinite(input_->points[*it].y) ||
                        !std::isfinite(input_->points[*it].z))
                        continue;

                int ijk0 = static_cast<int> (std::floor(input_->points[*it].x * inverse_leaf_size_[0]) -
                                             static_cast<float> (min_b_[0]));
                int ijk1 = static_cast<int> (std::floor(input_->points[*it].y * inverse_leaf_size_[1]) -
                                             static_cast<float> (min_b_[1]));
                int ijk2 = static_cast<int> (std::floor(input_->points[*it].z * inverse_leaf_size_[2]) -
                                             static_cast<float> (min_b_[2]));
                if (plane_.compare("yoz") == 0) {
                    // Compute the centroid leaf index
                    //与x轴坐标无关
                    int idx = ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
                    voxel_map[idx] += 1;
                } else if (plane_.compare("xoy") == 0) {
                    // Compute the centroid leaf index
                    //与z轴坐标无关
                    int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1];
                    voxel_map[idx] += 1;
                } else { //xoz
                    // Compute the centroid leaf index
                    //与y轴坐标无关
                    int idx = ijk0 * divb_mul_[0] + ijk2 * divb_mul_[2];
                    voxel_map[idx] += 1;
                }
            }

            float f_value = 0.;
            for (auto pair:voxel_map) {
                f_value += float(pair.second * pair.second);
            }
            return f_value / (voxel_map.size()*voxel_map.size());  //修改评估策略
        }

        // Members derived from the base class
        using VoxelBase::input_;
        using VoxelBase::indices_;
        using VoxelBase::initCompute;
        using VoxelBase::deinitCompute;
        using VoxelBase::filter_field_name_;
        using VoxelBase::filter_limit_min_;
        using VoxelBase::filter_limit_max_;
        using VoxelBase::inverse_leaf_size_;
        using VoxelBase::min_b_;
        using VoxelBase::max_b_;
        using VoxelBase::div_b_;
        using VoxelBase::divb_mul_;
        using VoxelBase::filter_limit_negative_;

    private:
        std::string plane_;
    };


    template<typename PointT>
    class Calibration : public pcl::PCLBase<PointT> {
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
        using Segments = std::vector<IndicesPtr>;
        using SegmentArgs = std::vector<float>;

        Calibration() : rotating_axis_('z'),
                        projection_plane_("yoz"),
                        projection_grid_size_(0.),
                        rotate_mat_(),
                        planeGrid_(),
                        evaluate_value_(),
                        angle_step_(3.f),
                        start_angle_(0.f),
                        end_angle_(360.f),
                        calibrated_cloud_(new pcl::PointCloud<PointT>),
                        ground_height_(0.f) {}

        char getRotatingAxis() const {
            return rotating_axis_;
        }

        void setRotatingAxis(char rotatingAxis) {
            rotating_axis_ = rotatingAxis;
        }

        const std::string &getProjectionPlane() const {
            return projection_plane_;
        }

        void setProjectionPlane(const std::string &projectionPlane) {
            projection_plane_ = projectionPlane;
        }

        float getProjectionGridSize() const {
            return projection_grid_size_;
        }

        void setProjectionGridSize(float projectionGridSize) {
            projection_grid_size_ = projectionGridSize;
        }

        float getAngleStep() const {
            return angle_step_;
        }

        void setAngleStep(float angleStep) {
            angle_step_ = angleStep;
        }

        float getStartAngle() const {
            return start_angle_;
        }

        void setStartAngle(float startAngle) {
            start_angle_ = startAngle;
        }

        float getEndAngle() const {
            return end_angle_;
        }

        void setEndAngle(float endAngle) {
            end_angle_ = endAngle;
        }

        float getGroundHeight() const {
            return ground_height_;
        }

        void setGroundHeight(float groundHeight) {
            ground_height_ = groundHeight;
        }

        //使用旋转投影密度方差法(RPDV)
        void calibrate(PointCloudPtr &cloudPtr) {

            if (!initCompute() ||
                (input_ && input_->points.empty()) ||
                (indices_ && indices_->empty())) {
                return;
            }
            initialCalibrate();

            //TODO 设定旋转轴
            settingRotateAxis();

            //调整其他轴
            float best_angle = settingOtherAxis();
            rotateAngle(best_angle, rotating_axis_);
            cloudPtr = calibrated_cloud_;
        }

        void plotAngleEvaluateValue() {
            pcl::visualization::PCLPlotter::Ptr plot(new pcl::visualization::PCLPlotter);
            plot->setBackgroundColor(1, 1, 1);
            plot->setTitle("Evaluate rotate angle value");
            plot->setXTitle("angle");
            plot->setYTitle("evaluate value");
            size_t size = evaluate_value_.size();
            std::vector<double> X_value, evaluate_Y_value;
            X_value.reserve(size);
            evaluate_Y_value.reserve(size);
            for (size_t i = 0; i < size; i++) {
                X_value.push_back(start_angle_ + i * angle_step_);
                evaluate_Y_value.push_back(evaluate_value_[i]);

            }
            if (size > 0) {
                plot->addPlotData(X_value, evaluate_Y_value, "angle evaluate value", vtkChart::LINE);
                plot->plot();
            }
        }

    protected:
        // Members derived from the base class
        using BasePCLBase::input_;
        using BasePCLBase::indices_;
        using BasePCLBase::initCompute;
        using BasePCLBase::deinitCompute;

        void initialCalibrate() {
            calibrated_cloud_->resize(input_->size());
            pcl::copyPointCloud(*input_, *calibrated_cloud_);
        }

        void settingRotateAxis() {
            assert(calibrated_cloud_->size() == input_->size());
            GroundRemoval<PointT> groundRemoval;
            groundRemoval.setInputCloud(calibrated_cloud_);
            groundRemoval.setIndices(indices_);
            groundRemoval.setGroundHeight(ground_height_);
            groundRemoval.setAxis(rotating_axis_);
            pcl::PointIndices tmp_indices;
            groundRemoval.remove(tmp_indices);
            if (rotating_axis_ == 'z') {
                //使地面与xoy面平行
                IndicesConstPtr ground_indices = groundRemoval.getGroundIndices();
                float nx, ny, nz;  //平面的法向量
                float curvature;
                pcl::NormalEstimation<PointT, pcl::Normal> ne;
                ne.computePointNormal(*calibrated_cloud_, *ground_indices, nx, ny, nz, curvature);

                PointT p(calibrated_cloud_->points[(*ground_indices)[0]]);
                float x = 0 - p.x, y = 0 - p.y, z = 0 - p.z; //TODO 修改视点
                if (nx * x + ny * y + nz * z < 0) {
                    nx = -nx;
                    ny = -ny;
                    nz = -nz;
                }
                float angle_x = std::atan2(ny, nz) * 180 / M_PI, angle_y = std::atan2(nx, nz) * 180 / M_PI;
                //将平面法向旋转至与z轴（0,0,1）平行
                rotateAngle(angle_x, 'x');
                rotateAngle(angle_y, 'y');
            }
        }

        float settingOtherAxis() {
            float angle = start_angle_;
            evaluate_value_.resize(std::floor((end_angle_ - start_angle_) / angle_step_));
            for (; angle < end_angle_; angle += angle_step_) {
                rotateAngle(angle, rotating_axis_);
                evaluate_value_[std::floor((angle - start_angle_) / angle_step_)] = evaluate();
            }
            float max_f = -FLT_MAX, best_angle = start_angle_;
            for (size_t i = 0; i < evaluate_value_.size(); i++) {
                if (evaluate_value_[i] > max_f) {
                    max_f = evaluate_value_[i];
                    best_angle = i;
                }
            }
            best_angle = start_angle_ + best_angle * angle_step_;
            return best_angle;
        }

        void rotateAngle(float angle, char axis) {
            assert(calibrated_cloud_->size() == input_->size());
            float u, v, w;
            if (axis == 'z') {
                u = 0.f;
                v = 0.f;
                w = 1.f;
            } else if (axis == 'y') {
                u = 0.f;
                v = 1.f;
                w = 0.f;
            } else {//x
                u = 1.f;
                v = 0.f;
                w = 0.f;
            }
            float cos = std::cos(angle * M_PI / 180);
            float sin = std::sin(angle * M_PI / 180);
            rotate_mat_ << u * u * (1 - cos) + cos, u * v * (1 - cos) - w * sin, u * w * (1 - cos) + v * sin,
                    u * v * (1 - cos) + w * sin, v * v * (1 - cos) + cos, v * w * (1 - cos) - u * sin,
                    u * w * (1 - cos) - v * sin, v * w * (1 - cos) + u * sin, w * w * (1 - cos) + cos;
            for (const int &index:*indices_) {
                calibrated_cloud_->points[index] = multiplyByMatrix(input_->points[index], rotate_mat_);
            }

        }

        inline PointT multiplyByMatrix(const PointT &origin, const Eigen::Matrix3f &mat) {
            PointT t(origin);
            Eigen::Vector3f res(origin.x, origin.y, origin.z);
            res = mat * res;
            t.x = res(0);
            t.y = res(1);
            t.z = res(2);
            return t;
        }

        float evaluate() {
            planeGrid_.setPlane(projection_plane_);
            planeGrid_.setInputCloud(calibrated_cloud_);
            planeGrid_.setIndices(indices_);
            planeGrid_.setLeafSize(projection_grid_size_, projection_grid_size_, projection_grid_size_);
            return planeGrid_.evaluateByVoxel();
        }

    private:

        //要旋转的轴
        char rotating_axis_;

        std::string projection_plane_;

        Eigen::Matrix3f rotate_mat_;

        float projection_grid_size_;

        PointCloudPtr calibrated_cloud_;

        PlaneGrid<PointT> planeGrid_;

        std::vector<float> evaluate_value_;
        float angle_step_;
        float start_angle_;
        float end_angle_;

        float ground_height_;
    };
}


#endif //PCLDEMO_CALIBRATINGTOOL_H
