# cmake_minimum_required(VERSION 3.0)
project(my_ch6)
# find_package(Eigen3 REQUIRED)
# find_package(OpenCV REQUIRED)
# include_directories(BEFORE
#     ${EIGEN_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIRS}
# )

# handwriting_gauss_newton.cc
# add_executable(handwriting_gauss_newton handwriting_gauss_newton.cc)
# target_link_libraries(handwriting_gauss_newton
#     ${OpenCV_LIBRARIES}
# )

# ceres_curve_fitting.cc 
# find_package(glog REQUIRED)
add_executable(ceres_curve_fitting ceres_curve_fitting.cc)
target_link_libraries(ceres_curve_fitting 
    ${CERES_LIBRARIES}
    ${OpenCV_LIBRARIES}
    # glog::glog
)