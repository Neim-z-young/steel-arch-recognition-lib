//
// Created by oyoungy on 2020/3/23.
//

#ifndef PCLDEMO_PCLDBSCAN_H
#define PCLDEMO_PCLDBSCAN_H

#include <pcl/pcl_base.h>
#include <pcl/search/pcl_search.h>

namespace designSpace{
    /**
     * Density-Based Spatial Clustering of Application with Noise
     */
template <typename PointT>
class DBSCAN : public pcl::PCLBase<PointT> {
    using BasePCLBase = pcl::PCLBase<PointT>;

public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    using KdTree = pcl::search::Search<PointT>;
    using KdTreePtr = typename KdTree::Ptr;

    using PointIndicesPtr = pcl::PointIndices::Ptr;
    using PointIndicesConstPtr = pcl::PointIndices::ConstPtr;

    DBSCAN(): tree_(),
              radius_(0),
              min_pts_per_core_object(1),
              max_cluster_inx_(-1)
    {}

    void extract(std::vector<pcl::PointIndices> &clusters) {
        if (!initCompute () ||
            (input_   && input_->points.empty ()) ||
            (indices_ && indices_->empty ()))
        {
            clusters.clear ();
            return;
        }

        clusters.clear();
        tree_->setInputCloud(input_, indices_);

        std::vector<bool> processed ((*input_).points.size (), false);
        std::vector<bool> clustered ((*input_).points.size (), false);
        /**
         * 处理点云中的所有点
         */
        for(const int &index:(*indices_)){
            if(processed[index])
                continue;
            processed[index] = true; //标记为已处理

            std::vector<int> radius_indices;
            //查找半径内的所有点. 计算点的邻域
            regionQuery((*input_).points[index], radius_indices);
            if(static_cast<int>(radius_indices.size()) < min_pts_per_core_object){ //非核心点
                continue;
            } else{//作为核心点

                pcl::PointIndicesPtr indices(new pcl::PointIndices());
                indices->indices.reserve(radius_indices.size());
                for(const int& i:radius_indices){  //调整有效点
                    if(!clustered[i]){
                        indices->indices.push_back(i);
                        clustered[i] = true;
                    }
                }

                clustered[index] = true;
                expandCluster(*indices, processed, clustered);
                clusters.push_back(*indices);
            }
        }
        //TODO 找到最大聚类
        int max_size = 0;
        max_cluster_inx_ = 0;
        for (size_t i = 0; i < clusters.size(); i++) {
            size_t size = clusters[i].indices.size();
            if (max_size < size) {
                max_size = size;
                max_cluster_inx_ = i;
            }
        }
    }

    void printMsg() {
        std::cout<<"cluster is good"<<std::endl;
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

    int getMinPtsPerCoreObject() const {
        return min_pts_per_core_object;
    }

    void setMinPtsPerCoreObject(int minPtsPerCoreObject) {
        min_pts_per_core_object = minPtsPerCoreObject;
    }

    int getMaxClusterInx() const {
        return max_cluster_inx_;
    }

protected:
    // Members derived from the base class
    using BasePCLBase::input_;
    using BasePCLBase::indices_;
    using BasePCLBase::initCompute;
    using BasePCLBase::deinitCompute;

    /** \brief A pointer to the spatial search object. */
    KdTreePtr tree_;

    /** spatial search radius. using euclidean distance */
    double radius_;

    /** min points to obtain core object. */
    int min_pts_per_core_object;

    int max_cluster_inx_;

    /** \brief Class getName method. */
    virtual std::string getClassName () const { return ("DBSCAN_cluster"); }

    void regionQuery(PointT point, std::vector<int>& radius_indices) {

        std::vector<float> radius_sqrt_dist;
        radius_indices.clear();
        tree_->radiusSearch(point, radius_,  radius_indices, radius_sqrt_dist);
    }

    void expandCluster(pcl::PointIndices& indices, std::vector<bool> &processed,
                                       std::vector<bool> &clustered) {
        int index = 0;
        while (index < static_cast<int>(indices.indices.size())){
//            if(clustered[index]){  //该点已聚类，则继续循环
//                index++;
//                continue;
//            }
//            clustered[index] = true;
            int p_inx = indices.indices[index];
            if(!processed[p_inx]){  //未进行领域探寻，则需判断该点是否为核心点
                processed[p_inx] = true;
                std::vector<int> tmp_radius_indices;
                regionQuery((*input_).points[p_inx], tmp_radius_indices);
                if(static_cast<int>(tmp_radius_indices.size())>=min_pts_per_core_object){ //是核心点，则扩充该类别
                    for(const int &i:tmp_radius_indices){
                        if(!clustered[i]){
                            indices.indices.push_back(i);
                            clustered[i] = true;
                        }
                    }
                }
            }
            index++;
        }
    }
};
}


#endif //PCLDEMO_PCLDBSCAN_H