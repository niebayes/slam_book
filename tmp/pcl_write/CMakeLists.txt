# cmake_minimum_required(VERSION 3.0)
# project(PCLWrite)

# # link_directories() will aplly only to targets creater after it is called
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
# add_executable(pcl_write pcl_write.cc)
# target_link_libraries(pcl_write ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(MY_GRAND_PROJECT)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcd_write_test pcd_write.cc)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})