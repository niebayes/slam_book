project(src)

add_executable(ceres_curve_fitting ceres_curve_fitting.cc)
target_link_libraries(ceres_curve_fitting 
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
  ${CMAKE_HOME_DIRECTORY}/lib/libplot2d_curve.dylib
)