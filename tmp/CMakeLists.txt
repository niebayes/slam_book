cmake_minimum_required(VERSION 2.8)
project(hello_cmake)

# TODO 设定库文件存放的位置 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_STANDARD 11)

# sophus
# find_package(Sophus REQUIRED)
# include_directories(${Sophus_INCLUDE_DIRS})
include_directories("/Users/bayes/slam_lib/Sophus/")

# eigen
# No need to link the lib, only include the path of the header files.
include_directories("/usr/local/Cellar/eigen/3.3.7/include/eigen3/")

# TODO 指定需要编译的库文件
# add_library(helloSLAM include/libHelloSLAM.cc)

# TODO 指定需要编译的可执行文件
add_executable(main src/main.cc)

# TODO 将编译好的库文件链接到编译得到的可执行文件 
# target_link_libraries(main ${Sophus_LIBRARIES})