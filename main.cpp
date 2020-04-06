#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char* argv[]){
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud);

    while (!viewer.wasStopped())
    {
    }
}
