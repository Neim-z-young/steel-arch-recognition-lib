cmake_minimum_required(VERSION 3.15)
project(pclDemo)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
link_directories(${Eigen3_LIBRARY_DIRS})
#include_directories("/usr/include/eigen3")#很奇怪的是Eigen只有头文件没有库文件，所以只能通过这种方式导入即可使用Eigen

# pcl
find_package( PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})

#Boost
#set(BOOST_ROOT "/usr/local/boost") #boost库
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

#VTK
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})

add_subdirectory(learning)

add_executable(pclDemo main.cpp)

target_link_libraries(pclDemo
        #"/usr/local/lib/libpcl_visualization.so"
        #libboost_system.so libboost_filesystem.so
        ${VTK_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        )