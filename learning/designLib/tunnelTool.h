//
// Created by oyoungy on 2020/4/7.
//

#ifndef PCLDEMO_TUNNELTOOL_H
#define PCLDEMO_TUNNELTOOL_H

#include <math.h>
#include <string>

namespace designSpace {
    class TunnelParameter {

    public:
        //简单的文件名分割函数
        static void getPcdFileNameAndPath(const std::string &file, std::string &name, std::string &path) {
            size_t index;
            for (index = file.size() - 1; index >= 0; index--) {
                if (file[index] == '/')
                    break;
            }
            path = file.substr(0, index + 1);
            name = file.substr(index + 1);
        }

        TunnelParameter(float arch_steel_gap, float arch_steel_thickness, float scan_size, float sensor_resolution) :
                SCALE_FACTOR_(2.5f),
                MULTIPLE_(1000.f),
                ARCH_STEEL_GAP_(arch_steel_gap * MULTIPLE_),
                ARCH_STEEL_THICKNESS_(arch_steel_thickness * MULTIPLE_),
                SCAN_SIZE_(scan_size),
                SENSOR_RESOLUTION_(sensor_resolution),
                VOXEL_SIZE_(SCALE_FACTOR_ * MULTIPLE_ * SCAN_SIZE_ * tan(M_PI / 180 * SENSOR_RESOLUTION_)),
                SEGMENT_LENGTH_(0.1f * ARCH_STEEL_GAP_),
                RADIUS_FOR_C_N_(ARCH_STEEL_GAP_),
                K_FOR_C_N_(3*RADIUS_FOR_C_N_/VOXEL_SIZE_),
                RADIUS_DBSCAN_(2.f*ARCH_STEEL_THICKNESS_), //0.3f * SCALE_FACTOR_ * MULTIPLE_
                MIN_PTS_(2*RADIUS_DBSCAN_/VOXEL_SIZE_),  //200
                CONCRETE_FACE_STEP_(3 * ARCH_STEEL_GAP_ / SEGMENT_LENGTH_),
                WORK_FACE_STEP_(1 * ARCH_STEEL_GAP_ / SEGMENT_LENGTH_),
                GROUND_HEIGHT_(0.7f * MULTIPLE_){}

        //放大倍数
        const float MULTIPLE_;

        const float ARCH_STEEL_GAP_;
        const float ARCH_STEEL_THICKNESS_;
        const float SEGMENT_LENGTH_;

        //体素化
        const float SENSOR_RESOLUTION_;
        const float SCAN_SIZE_;
        const float SCALE_FACTOR_;
        const float VOXEL_SIZE_;

        //计算曲率及法向量所需参数
        const float RADIUS_FOR_C_N_;
        const int K_FOR_C_N_;

        //DBSCAN
        const float RADIUS_DBSCAN_;
        const int MIN_PTS_;

        //CALIBRATION

        //移除地面
        const float GROUND_HEIGHT_;

        //岩石表面提取

        const int CONCRETE_FACE_STEP_;
        const int WORK_FACE_STEP_;


        //钢拱提取

    };

    TunnelParameter _PARAM_(1, 0.2, 30, 0.05);
};

#endif //PCLDEMO_TUNNELTOOL_H
