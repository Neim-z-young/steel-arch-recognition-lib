//
// Created by oyoungy on 2020/3/27.
//

#ifndef PCLDEMO_ROCKFACEHELPER_H
#define PCLDEMO_ROCKFACEHELPER_H

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
#include <pcl/visualization/pcl_plotter.h>

//user lib
#include "../designLib/tunnelTool.h"
using designSpace::_PARAM_;
namespace designSpace {
    //Wd = [ voxel_size, Wa] x轴切片大小
//Pi 点云沿x轴的段序列
    template<typename PointT>
    class RockfaceExtraction : public pcl::PCLBase<PointT> {
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

        RockfaceExtraction() : segment_length_(1),
                               axis_('x'),
                               radius_(0.),
                               k_(0),
                               segment_indices_(),
                               curvatures_(),
                               heights_(),
                               rock_and_work_face_indices_(new std::vector<int>()),
                               max_m1_(0),
                               min_m1_(0){ }

        float getSegmentLength() const {
            return segment_length_;
        }

        void setSegmentLength(float segmentLength) {
            segment_length_ = segmentLength;
        }

        char getAxis() const {
            return axis_;
        }

        void setAxis(char axis) {
            axis_ = axis;
        }

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

        const Segments &getSegmentIndices() const {
            return segment_indices_;
        }

        const SegmentArgs &getCurvatures() const {
            return curvatures_;
        }

        //必须在提取操作结束后调用，计算可能的初始化点的起始x轴偏移（在钢拱识别时需要用）
        float calculateStartShift(){
            int segment = max_m1_+1;
            for(size_t i = segment+1; i<curvatures_.size()-1; i++){
                if(curvatures_[i]>=curvatures_[segment]){
                    segment = i;
                } else{
                    break;
                }
            }
            float shift = (segment - max_m1_ - 1)*segment_length_;
            return shift>FLT_EPSILON?shift:0.f;
        }

        //提取出岩石表面
        void extract(pcl::PointIndices &rockface_point_indices, int& m1, int& m2) {
            if (!initCompute() ||
                (input_ && input_->points.empty()) ||
                (indices_ && indices_->empty())) {
                return;
            }
            cloudSegmentAloneAxis(segment_indices_);

            size_t size = segment_indices_.size();
            curvatures_.clear();
            heights_.clear();
            curvatures_.reserve(size);
            heights_.reserve(size);
            for (size_t i = 0; i < size; i++) {
                if((*segment_indices_[i]).size()>0) {
                    curvatures_.push_back(
                            calculateAvgCurvature(segment_indices_[i], ceil(segment_indices_[i]->size() / 10.f)));
                    heights_.push_back(
                            calculateAvgHeight(segment_indices_[i], ceil(segment_indices_[i]->size() / 10.f)));
                } else{
                    assert(i>0);
                    curvatures_.push_back(curvatures_[i-1]);
                    heights_.push_back(heights_[i-1]);
                }
            }

            //DASST(Differential Analysis method for the Section Sequences of the Tunnel point cloud)
            max_m1_ = removeConcrete(*rock_and_work_face_indices_, true);

            min_m1_ = removeWorkface(max_m1_+1, rockface_point_indices.indices);

            //返回混凝土表面、工作表面与岩石表面的分隔段
            m1 = max_m1_;
            m2 = min_m1_;
        }

        int removeConcrete(std::vector<int> &rock_and_work_face_indices, bool direct_strategy) {
            //防止错误调用
            assert(curvatures_.size()!=0);

            int step = _PARAM_->CONCRETE_FACE_STEP_; //3*Wa/Wd
            int max_m1 = 0;
            float max_g_vaue = -FLT_MAX;
            size_t size = curvatures_.size();
            for (size_t m1 = 0; m1 < size - step; m1++) {
                float tmp = g_function(curvatures_, m1, step);
                if (max_g_vaue < tmp) {
                    max_m1 = m1;
                    max_g_vaue = tmp;
                }
            }
            //calculate avg shotcrete surface curvature
            float avg_curvature = 0, threshold = 0;
            for (int i = 0; i < max_m1; i++) {
                avg_curvature += curvatures_[i];
            }
            avg_curvature /= float(max_m1);
            threshold = avg_curvature * 3.f;

            rock_and_work_face_indices.reserve(indices_->size()/30); //TODO fix it by using indices_

            if (direct_strategy) { //直接筛选
                for (size_t i = max_m1; i < size; i++) {
                    for (size_t t = 0; t < segment_indices_[i]->size(); t++) {
                        rock_and_work_face_indices.push_back((*segment_indices_[i])[t]);
                    }
                }
            } else {
                //采用曲率阈值法筛选
                for (size_t i = 0; i < size; i++) {
                    if (curvatures_[i] > threshold) {
                        for (size_t t = 0; t < segment_indices_[i]->size(); t++) {
                            rock_and_work_face_indices.push_back((*segment_indices_[i])[t]);
                        }
                    }
                }
            }

            return max_m1;
        }

        int removeWorkface(int start_segment, std::vector<int>& rockface_indices){
            assert(heights_.size()!=0);

            int step = _PARAM_->WORK_FACE_STEP_; //差分步数
            int min_m1 = start_segment;
            float min_g_vaue = FLT_MAX;
            size_t size = heights_.size();
            for (size_t m1 = start_segment; m1 < size - step; m1++) {
                float tmp = g_function(heights_, m1, step);
                if (min_g_vaue > tmp) {
                    min_m1 = m1;
                    min_g_vaue = tmp;
                }
            }
            //calculate avg rockface height
            float avg_height = 0, threshold = 0;
            assert(start_segment!=min_m1);
            for (int i = start_segment; i < min_m1; i++) {
                avg_height += heights_[i];
            }
            avg_height /= float(min_m1 - start_segment);
            threshold = _PARAM_->ARCH_STEEL_THICKNESS_*2.f; //TODO 阈值设定

            //采用高度阈值法筛选
            for (size_t i = start_segment; i < min_m1; i++) {
                if (heights_[i] <=  avg_height + threshold && heights_[i] >= avg_height - threshold) {
                    for (const int& t : (*segment_indices_[i])) {
                        rockface_indices.push_back(t);
                    }
                }
            }
            return min_m1;
        }

        void plotCurvatureRelation() {
            pcl::visualization::PCLPlotter::Ptr plot(new pcl::visualization::PCLPlotter);
            plot->setBackgroundColor(1, 1, 1);
            plot->setTitle("Evaluate Tunnel Section Curvature Relation");
            plot->setXTitle("section");
            plot->setYTitle("curvature");
            size_t seg_size = curvatures_.size();
            std::vector<double> X_value, curvature_Y_value, g_Y_value;
            X_value.reserve(seg_size);
            curvature_Y_value.reserve(seg_size);
            g_Y_value.reserve(seg_size);
            for (size_t i = 0; i < seg_size; i++) {
                X_value.push_back(i);
                curvature_Y_value.push_back(curvatures_[i]);
                int step = _PARAM_->CONCRETE_FACE_STEP_; //修改步数
                if (i + step < seg_size) {
                    g_Y_value.push_back(g_function(curvatures_, i, step));
                } else {
                    g_Y_value.push_back(0.);
                }
            }
            plot->addPlotData(X_value, curvature_Y_value, "section curvature", vtkChart::LINE);
            plot->addPlotData(X_value, g_Y_value, "curvature difference", vtkChart::LINE);
            plot->plot();
        }

        void plotHeightRelation() {
            pcl::visualization::PCLPlotter::Ptr plot(new pcl::visualization::PCLPlotter);
            plot->setBackgroundColor(1, 1, 1);
            plot->setTitle("Evaluate Tunnel Section Height Relation");
            plot->setXTitle("section");
            plot->setYTitle("height");
            size_t seg_size = heights_.size();
            int start_seg = max_m1_ + 1;
            std::vector<double> X_value, height_Y_value, g_Y_value;
            X_value.reserve(seg_size - start_seg);
            height_Y_value.reserve(seg_size - start_seg);
            g_Y_value.reserve(seg_size - start_seg);
            for (size_t i = start_seg; i < seg_size; i++) {
                X_value.push_back(i);
                height_Y_value.push_back(heights_[i]);
                int step = _PARAM_->WORK_FACE_STEP_; //修改步数
                if (i + step < seg_size) {
                    g_Y_value.push_back(g_function(heights_, i, step));
                } else {
                    g_Y_value.push_back(0.);
                }
            }
            plot->addPlotData(X_value, height_Y_value, "section height", vtkChart::LINE);
            plot->addPlotData(X_value, g_Y_value, "height difference", vtkChart::LINE);
            plot->plot();
        }

    protected:
        // Members derived from the base class
        using BasePCLBase::input_;
        using BasePCLBase::indices_;
        using BasePCLBase::initCompute;
        using BasePCLBase::deinitCompute;

        //将点云沿轴分段
        void cloudSegmentAloneAxis(Segments &seg_indices) {
            float x = 0, y = 0, z = 0, min_x = FLT_MAX, max_x = -FLT_MAX, min_y = FLT_MAX, max_y = -FLT_MAX, min_z = FLT_MAX, max_z = -FLT_MAX;
            for (const int& index:*indices_) {
                min_x = fmin(min_x, input_->points[index].x);
                min_y = fmin(min_y, input_->points[index].y);
                min_z = fmin(min_z, input_->points[index].z);

                max_x = fmax(max_x, input_->points[index].x);
                max_y = fmax(max_y, input_->points[index].y);
                max_z = fmax(max_z, input_->points[index].z);
            }
            switch (axis_) {
                case 'x': {
                    int k = ceil((max_x - min_x) / segment_length_);
                    seg_indices.reserve(k);
                    for (int i = 0; i < k; i++) {
                        seg_indices.emplace_back(new std::vector<int>);
                    }
                    for (const int& inx:*indices_) {
                        int index = floor((input_->points[inx].x - min_x) / segment_length_);
                        seg_indices[index]->push_back(inx);
                    }
                    break;
                }
                default:
                    break; //TODO extend to y, z axes
            }
        }

        //计算一段点云的平均曲率
        float calculateAvgCurvature(const IndicesConstPtr &segment_indices, int k_samples) {
            float total_curvature = 0;
            size_t size = segment_indices->size();
            for (int i = 0; i < k_samples; i++) {
                int index = random() % size;
                total_curvature += calculateCurvature(segment_indices, index);
            }
            return total_curvature / float(k_samples);
        }

        //计算点云中一个点的曲率  index是原始点云中在segment段中点的下标
        float calculateCurvature(const IndicesConstPtr &segment_indices, int segment_point_index) {

            tree_->setInputCloud(input_, segment_indices);
            std::vector<int> k_indices(k_);
            std::vector<float> k_sqr_distances(k_);
            tree_->nearestKSearch(segment_point_index, k_, k_indices, k_sqr_distances);
            float sqr_radius = radius_ * radius_;
            if (k_sqr_distances[k_sqr_distances.size() - 1] > sqr_radius) {
                //TODO fix it
                //something wrong
                std::cerr << "warning!! distance is larger than radius , squared distance is: "
                          << k_sqr_distances[k_sqr_distances.size() - 1] << std::endl;
            }

            float nx, ny, nz;  //平面的法向量
            float curvature;
            pcl::NormalEstimation<PointT, pcl::Normal> ne;
            ne.computePointNormal(*input_, k_indices, nx, ny, nz, curvature);
            return curvature;
        }

        //计算一段点云的平均高度
        float calculateAvgHeight(const IndicesConstPtr &segment_indices, int k_samples) {
            float total_height = 0;
            size_t size = segment_indices->size();
            bool sampling = false;
            if(sampling){
                for (int i = 0; i < k_samples; i++) {
                    int index = random() % size;
                    total_height += input_->points[(*segment_indices)[index]].z;
                }
            } else{
                float max_height = -FLT_MAX, min_height = FLT_MAX;
                int max_inx = -1;
                for (const int& index:(*segment_indices)) {
                    if(max_height < input_->points[index].z){
                        max_height = input_->points[index].z;
                        max_inx = index;
                    }
                    if(min_height > input_->points[index].z){
                        min_height = input_->points[index].z;
                    }
                }
                //计算最高点附件其他点的平均高度来估算该段的高度
                std::vector<int> k_indices;
                std::vector<float> k_sqr_distances;
                assert(max_inx >= 0);
                tree_->setInputCloud(input_, segment_indices);
                tree_->radiusSearch(input_->points[max_inx], radius_, k_indices, k_sqr_distances);
                for(const int& index:k_indices){
                    total_height += (input_->points[index].z - min_height);
                }
//                total_height = max_height - min_height;
                k_samples = k_indices.size();
            }
            //TODO search max z value


            return total_height / float(k_samples);
        }

        //try to calculate difference
        float g_function(const SegmentArgs &segment_arg, int i, int step) {
            if (i >= segment_arg.size() - step) {
                //TODO fix it
                //something wrong
                std::cerr << "warning!! index out of bound" << std::endl;
            }
            float g_value = 0;
            for (int k = 1; k < step && i + k < segment_arg.size(); k++) {
                g_value += (segment_arg[i + k] - segment_arg[i]);
            }
            return g_value;
        }

    private:
        /** \brief A pointer to the spatial search object. */
        KdTreePtr tree_;

        /** judge distance less than radius. using euclidean distance */
        double radius_;

        /** nearest k points to compute curvature. */
        int k_;

        float segment_length_;
        char axis_;

        int max_m1_;  //用来标识混凝土表面与岩石表面的差分最值段
        int min_m1_;  //用来标识岩石表面与工作表面的差分最值段

        Segments segment_indices_;
        SegmentArgs curvatures_;
        SegmentArgs heights_;
        IndicesPtr rock_and_work_face_indices_;

    };
}

#endif //PCLDEMO_ROCKFACEHELPER_H
