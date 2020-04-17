//
// Created by oyoungy on 2020/4/4.
//

#ifndef PCLDEMO_STEELARCHHELPER_H
#define PCLDEMO_STEELARCHHELPER_H

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

//eigen
#include <Eigen/Dense>

//user lib
#include "../designLib/tunnelTool.h"
using designSpace::_PARAM_;

namespace designSpace {
    //Wd = [ voxel_size, Wa] x轴切片大小
//Pi 点云沿x轴的段序列

    template<typename PointT>
    class CustomPoint {
    public:
        CustomPoint() : index_(-1){ }

        PointT point;
        int index_;
    };

    /**
        提取前必须使用setInputCloud及setIndices方法设置点云参数。

     * @tparam PointT
     */
    template<typename PointT>
    class SteelArchExtraction : public pcl::PCLBase<PointT> {
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

        SteelArchExtraction() :
                axis_('x'),
                radius_(0.),
                k_(0),
                steel_arch_gap_(1.),
                arch_thickness_(1.),
                start_arch_gap_(1.),
                initial_points_(),
                start_angle_(0.),
                end_angle_(0.),
                tmp_growing_directions_(),
                seed_points_(),
                optimal_points_(),
                view_point_(),
                normals_(new pcl::PointCloud<pcl::Normal>()),
                estimated_tunnel_radius_(1.f){}


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

        float getSteelArchGap() const {
            return steel_arch_gap_;
        }

        void setSteelArchGap(float steelArchGap) {
            steel_arch_gap_ = steelArchGap;
        }

        float getArchThickness() const {
            return arch_thickness_;
        }

        void setArchThickness(float archThickness) {
            arch_thickness_ = archThickness;
        }

        float getStartArchGap() const {
            return start_arch_gap_;
        }

        void setStartArchGap(float startArchGap) {
            start_arch_gap_ = startArchGap;
        }

        void setViewPoint(float x, float y, float z){
            view_point_.x = x;
            view_point_.y = y;
            view_point_.z = z;
        }

        void calculateNormal(pcl::PointCloud<pcl::Normal>& normals){
            if (!initCompute() ||
                (input_ && input_->points.empty()) ||
                (indices_ && indices_->empty())) {
                return;
            }

            tree_->setInputCloud(input_, indices_);

            normals.clear();
            normals.resize(indices_->size());
            for(int i=0; i<indices_->size(); i++){
                PointT n_point = calculateNormalVector(input_->points[(*indices_)[i]], true);
                normals.points[i].normal_x = n_point.x;
                normals.points[i].normal_y = n_point.y;
                normals.points[i].normal_z = n_point.z;
            }
        }

        void extract(std::vector<PointT, Eigen::aligned_allocator<PointT>>& steel_arch_points) {
            if (!initCompute() ||
                (input_ && input_->points.empty()) ||
                (indices_ && indices_->empty())) {
                return;
            }

            tree_->setInputCloud(input_, indices_);

            //计算点云法向量
            preCalculateNormals();

            //计算初始化点
            initialCalculate();

            //设置初始容量/大小
            size_t arch_number = initial_points_.size();
            tmp_growing_directions_.resize(arch_number);
            seed_points_.resize(arch_number);

            int assume_number = 20;
            optimal_points_.reserve(assume_number * arch_number);
            optimal_points_.clear();

            int index = 0;
            //对每一个初始化点，执行生长算法
            for (PointT initial:initial_points_) {

                //Initail seed L1



                int optimal_index = findNearestPointIndex(initial);
                //O1(k) = L1(k)
                PointT optimal = input_->points[optimal_index];
                //所有的最优点都是通过插值法插入到原始点云图中的
                optimal_points_.push_back(optimal);

                //迭代次数约为隧道内周长/生长步长
                int it = 2*M_PI*estimated_tunnel_radius_/(2*arch_thickness_);
                bool grow = true;
                //迭代生长
                while (grow) {
                    //New seed Si
                    //update vi
                    tmp_growing_directions_[index] = calculateDirection(optimal);
                    //update seedi
                    seed_points_[index].x = optimal.x + tmp_growing_directions_[index].x;
                    seed_points_[index].y = optimal.y + tmp_growing_directions_[index].y;
                    seed_points_[index].z = optimal.z + tmp_growing_directions_[index].z;

                    //estimate the normal vector of the seed
                    PointT normal_of_seed = calculateNormalVector(seed_points_[index], false);


                    //寻找可能的钢拱点
                    std::vector<int> k_indices;
                    std::vector<float> k_sqr_distances;
                    float voxelSize = _PARAM_->VOXEL_SIZE_; //使用常数
                    size_t max_salient_points = ceil(2 * arch_thickness_ / voxelSize);
                    //搜索半径有区别
                    tree_->radiusSearch(seed_points_[index], 2*arch_thickness_, k_indices, k_sqr_distances, 5*max_salient_points);

                    size_t size = k_indices.size();
                    if(size>0){
                        //TODO bind可能用错
                        std::function<bool(int, int)> comp = std::bind(&SteelArchExtraction<PointT>::compIndexByLNSWithSeed, this,
                                                                       std::placeholders::_1, std::placeholders::_2,
                                                                       seed_points_[index], normal_of_seed);
                        std::sort(k_indices.begin(), k_indices.end(), comp);

                        float sum_x = 0., sum_y = 0., sum_z = 0.;
                        size_t tmp_size = std::min(size, max_salient_points);
                        for(size_t i = 0; i<tmp_size; i++){
                            sum_x += input_->points[k_indices[i]].x;
                            sum_y += input_->points[k_indices[i]].y;
                            sum_z += input_->points[k_indices[i]].z;
                        }
                        optimal.x = sum_x/tmp_size;
                        optimal.y = sum_y/tmp_size;
                        optimal.z = sum_z/tmp_size;
                    } else{
                        optimal = seed_points_[index];
                    }

                    //排除不在点云上的点
                    if(initial_points_[index].z <= optimal.z){
                        optimal_points_.push_back(optimal);
                    }
                    if(start_angle_>end_angle_ && calculateAndConvertAngle(optimal.z, optimal.y) <= end_angle_){
                        //TODO 终止条件
                        grow = false;
                    } else if(start_angle_<end_angle_ && calculateAndConvertAngle(optimal.z, optimal.y) >= end_angle_){
                        grow = false;
                    }

                    if(it-- < 0){
                        grow = false;
                    }
                }

                index++;
            }
            steel_arch_points.insert(steel_arch_points.begin(), optimal_points_.begin(), optimal_points_.end());

        }


    protected:
        // Members derived from the base class
        using BasePCLBase::input_;
        using BasePCLBase::indices_;
        using BasePCLBase::initCompute;
        using BasePCLBase::deinitCompute;

        void preCalculateNormals(){
            //计算点云法向
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
            //计算输入点云中所有点的法向量
            ne.setInputCloud(input_);
            ne.setViewPoint(view_point_.x, view_point_.y, view_point_.z);
            ne.setSearchMethod(tree_);
            ne.setKSearch(k_);
            normals_->clear();
            ne.compute(*normals_);
        }

        //计算点云的初始化点及起止角度
        void initialCalculate(){
            assert(indices_!= nullptr);

            PointT s_point = input_->points[(*indices_)[0]];

            float min_x = FLT_MAX, min_z = FLT_MAX, max_x = FLT_MIN, max_y = -FLT_MAX, min_y = FLT_MAX;
            for(const int& index : *indices_){
                const PointT& tmp = input_->points[index];
                if(measureStartPoint(s_point)>measureStartPoint(tmp)){
                    s_point = tmp;
                }
                if(min_x>tmp.x){
                    min_x = tmp.x;
                }
                if(max_x<tmp.x){
                    max_x = tmp.x;
                }
                if(min_y>tmp.y){
                    min_y = tmp.y;
                }
                if(max_y<tmp.y){
                    max_y = tmp.y;
                }
                if(min_z>tmp.z){
                    min_z = tmp.z;
                }
            }
            estimated_tunnel_radius_ = (max_y - min_y)/2;
            start_angle_ = calculateAndConvertAngle(s_point.z, s_point.y);
            end_angle_ = calculateAndConvertAngle(s_point.z, -s_point.y);

            int arch_number = 1 + floor((max_x - min_x - start_arch_gap_)/steel_arch_gap_);
            initial_points_.resize(arch_number);
            for(size_t i = 0; i<arch_number; i++){
                initial_points_[i].x = min_x + start_arch_gap_ + steel_arch_gap_*i;
                initial_points_[i].y = min_z/tan(start_angle_);
                initial_points_[i].z = min_z;
            }
        }

        //TODO atan2得到的弧度是(-PI, PI)，因此我们要分辨出起止弧度的大小，就要改变弧度取值范围，使其变为(-PI/2, 3PI/2)
        inline float calculateAndConvertAngle(const float& a1, const float& a2){
            float arch = atan2(a1, a2);
            if(arch < -M_PI/2){
                arch += (M_PI*2);
            }
            return arch;
        }

        inline float measureStartPoint(const PointT& t){
            float tmp =sqrt((t.y*t.y) + (t.z*t.z));
            return t.z*tmp;
        }

        //计算算法在某点的生长方向
        PointT calculateDirection(const PointT& point) {
            PointT normal = calculateNormalVector(point, false), direction;

            Eigen::Vector3f n(normal.x, normal.y, normal.z);
            Eigen::Vector3f vx(1, 0, 0);
            if(start_angle_<end_angle_){
                vx(0) = -vx(0);
            }
            Eigen::Vector3f tmp_d = vx.cross(n);
            //TODO 修改生长步长，不应太大
            tmp_d = 2 * arch_thickness_ * (tmp_d / tmp_d.norm());

            direction.x = tmp_d(0);
            direction.y = tmp_d(1);
            direction.z = tmp_d(2);

            return direction;
        }

        //计算点云中一个点的法向量
        PointT calculateNormalVector(const PointT& point, bool custom) {

            if(custom){
                std::vector<int> k_indices(k_);
                std::vector<float> k_sqr_distances(k_);
                int index = findNearestPointIndex(point);
                tree_->nearestKSearch(input_->points[index], k_, k_indices, k_sqr_distances);
                float sqr_radius = radius_ * radius_;
                if (k_sqr_distances[k_sqr_distances.size() - 1] > sqr_radius) {
                    //TODO fix it
                    //something wrong
                    std::cerr << "warning!! distance is larger than radius , squared distance is: "
                              << k_sqr_distances[k_sqr_distances.size() - 1] << std::endl;
                }

                PointT n_point;  //平面的法向量
                float curvature;
                pcl::NormalEstimation<PointT, pcl::Normal> ne;
                ne.computePointNormal(*input_, k_indices, n_point.x, n_point.y, n_point.z, curvature);

                //根据视点调整法向量的方向，使其朝向视点
                //公式：n*(v-p)>0
                PointT tmp;
                tmp.x = view_point_.x - point.x;
                tmp.y = view_point_.y - point.y;
                tmp.z = view_point_.z - point.z;

                if(dotForPointT(n_point, tmp)<0){
                    n_point.x = -n_point.x;
                    n_point.y = -n_point.y;
                    n_point.z = -n_point.z;
                }
                return n_point;
            }else{
                int index = findNearestPointIndex(point);
                pcl::Normal normal = normals_->points[index];
                PointT n_point;
                n_point.x = normal.normal_x;
                n_point.y = normal.normal_y;
                n_point.z = normal.normal_z;
                return n_point;
            }

        }

        //找到点云中离某点最近的点的下标
        inline int findNearestPointIndex(PointT point){
            assert(tree_!= nullptr);

            int k = 1;
            std::vector<int> k_indices(k);
            std::vector<float> k_sqr_distances(k);
            tree_->nearestKSearch(point, k, k_indices, k_sqr_distances);
            return k_indices[0];
        }

        //通过函数模版传入sort函数中
        bool compIndexByLNSWithSeed(const int &a, const int &b, const PointT& seed, const PointT& normal_vector_of_seed){
            return calculateLocalNormalSaliency(input_->points[a], seed, normal_vector_of_seed) >
                    calculateLocalNormalSaliency(input_->points[b], seed, normal_vector_of_seed);
        }

        //计算排序所需参数
        float calculateLocalNormalSaliency(const PointT& c, const PointT& seed, const PointT& normal_vector_of_seed) {
            PointT tmp;
            tmp.x = c.x - seed.x;
            tmp.y = c.y - seed.y;
            tmp.z = c.z - seed.z;
            return dotForPointT(tmp, normal_vector_of_seed);
        }

        float dotForPointT(const PointT& p1, const PointT& p2) {
            return p1.x * p2.x
                   + p1.y * p2.y
                   + p1.z * p2.z;
        }


    private:
        /** \brief A pointer to the spatial search object. */
        KdTreePtr tree_;

        /** 搜索半径 */
        double radius_;

        /** nearest k points to compute curvature. */
        int k_;

        /** 钢拱间隔 */
        float steel_arch_gap_;

        /** 钢拱厚度 */
        float arch_thickness_;

        /** 初始估计钢拱位置相对于起点的距离 */
        float start_arch_gap_;

        float estimated_tunnel_radius_;

        /** 初始化点 */
        std::vector<PointT, Eigen::aligned_allocator<PointT>> initial_points_;

        //生长起止角度，范围在(-PI/2, 3PI/2)
        float start_angle_;
        float end_angle_;

        /** 生长方向 */
        std::vector<PointT, Eigen::aligned_allocator<PointT>> tmp_growing_directions_;

        std::vector<PointT, Eigen::aligned_allocator<PointT>> seed_points_;

        std::vector<PointT, Eigen::aligned_allocator<PointT>> optimal_points_;

        PointT view_point_;

        pcl::PointCloud<pcl::Normal>::Ptr normals_;

        char axis_;


    };

}

#endif //PCLDEMO_STEELARCHHELPER_H
