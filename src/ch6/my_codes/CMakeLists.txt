project(my_ch6)
# find_package(Eigen3 REQUIRED)
# find_package(OpenCV REQUIRED)
# include_directories(BEFORE
#     ${EIGEN_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIRS}
# )
add_executable(handwriting_gauss_newton handwriting_gauss_newton.cc)
target_link_libraries(handwriting_gauss_newton
    ${OpenCV_LIBRARIES}
)