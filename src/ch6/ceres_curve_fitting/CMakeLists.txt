project(CeresCurveFitting)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/ch6)
add_executable(ceres_curve_fitting ceres_curve_fitting.cc)
target_link_libraries(ceres_curve_fitting
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
)