add_executable(spin_images pcdFile/example_spin_images.cpp)
target_link_libraries(spin_images
        ${VTK_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(show_pcd_file pcdFile/show.cpp pcdFile/pclDBSCAN.cpp)
target_link_libraries(show_pcd_file
        ${VTK_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(voxelization pcdFile/voxelization.cpp designLib/tunnelTool.h designLib/tunnelTool.cpp)
target_link_libraries(voxelization
        ${VTK_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(cluster pcdFile/clusterByDBSCAN.cpp pcdFile/pclDBSCAN.cpp designLib/tunnelTool.h designLib/tunnelTool.cpp)
target_link_libraries(cluster
        ${VTK_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(calibration pcdFile/calibration.cpp pcdFile/calibratingTool.h designLib/tunnelTool.h designLib/tunnelTool.cpp)
target_link_libraries(calibration
        ${VTK_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(rockface_extraction pcdFile/rockfaceExtraction.cpp pcdFile/rockfaceHelper.h designLib/tunnelTool.h designLib/tunnelTool.cpp)
target_link_libraries(rockface_extraction
        ${VTK_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(ground_remove pcdFile/groundRemove.cpp pcdFile/groundHelper.h designLib/tunnelTool.h designLib/tunnelTool.cpp)
target_link_libraries(ground_remove
        ${VTK_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(steel_arch_extraction pcdFile/steelArchExtraction.cpp pcdFile/steelArchHelper.h designLib/tunnelTool.h designLib/tunnelTool.cpp)
target_link_libraries(steel_arch_extraction
        ${VTK_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Eigen3_LIBRARIES}
        )